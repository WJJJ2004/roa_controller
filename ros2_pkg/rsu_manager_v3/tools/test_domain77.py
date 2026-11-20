#!/usr/bin/env python3
"""Real DDS, existing v2 observation oracle and real HW merge/encoder, no CAN."""
import argparse
import gc
import json
import math
import os
from pathlib import Path
import signal
import struct
import subprocess
import time
import numpy as np
import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy
from roa_interfaces.msg import MotorState, MotorStateArray, MotorCommand, MotorCommandArray, RsuTarget, RsuStateArray, RsuTorqueCommand
from std_msgs.msg import Bool
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist, Vector3Stamped
from lifecycle_msgs.srv import ChangeState
from roa_interfaces.msg import SystemStatus


def ns(stamp): return stamp.sec * 10**9 + stamp.nanosec

def values(s):
    return np.array([s.left_rsu_roll, s.left_rsu_pitch, s.right_rsu_roll, s.right_rsu_pitch])


class Lut:
    def __init__(self, path):
        raw = Path(path).read_bytes()
        _, version, nr, nc, self.r0, self.r1, self.p0, self.p1, _ = struct.unpack_from('<8sIII5d', raw)
        assert version == 1
        self.data = np.frombuffer(raw, dtype='<f4', offset=60).reshape(nr, nc, 6)
    def query(self, q):
        nr, nc, _ = self.data.shape
        x = (q[0] - self.r0) / (self.r1 - self.r0) * (nr - 1)
        y = (q[1] - self.p0) / (self.p1 - self.p0) * (nc - 1)
        i, j = min(int(x), nr - 2), min(int(y), nc - 2)
        u, v = x-i, y-j
        return ((1-u)*(1-v)*self.data[i,j]+u*(1-v)*self.data[i+1,j]+(1-u)*v*self.data[i,j+1]+u*v*self.data[i+1,j+1]).astype(float)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--install', required=True)
    ap.add_argument('--output', required=True)
    ap.add_argument('--scenario', default='normal', choices=['normal','target_stop','feedback_stop','nan','missing','duplicate','reorder','old_target','upper_stop','node_death','walking'])
    ap.add_argument('--duration', type=float, default=12)
    ap.add_argument('--load-workers', type=int, default=0)
    args=ap.parse_args()
    assert os.environ.get('ROS_DOMAIN_ID') == '77'
    assert os.environ.get('ROS_LOCALHOST_ONLY') == '1'
    out=Path(args.output);out.mkdir(parents=True,exist_ok=True)
    install=Path(args.install).resolve()
    lut=Lut(install/'rsu_manager_v2/share/rsu_manager_v2/data/rsu_lut_v2.bin')
    rclpy.init(); node=rclpy.create_node('rsu_v3_test_driver')
    # Domain graph audit before any command publication; reject every unknown node.
    until=time.monotonic()+1.0
    while time.monotonic()<until: rclpy.spin_once(node,timeout_sec=.05)
    foreign=[n for n in node.get_node_names() if n!='rsu_v3_test_driver' and not n.startswith('_ros2cli_daemon')]
    if foreign: raise RuntimeError(f'domain77 is not empty: {foreign}')
    print('ISOLATION: ROS_DOMAIN_ID=77 ROS_LOCALHOST_ONLY=1; graph empty; only numerical managers and socket-free mock will be started',flush=True)
    qos=QoSProfile(depth=1,reliability=ReliabilityPolicy.BEST_EFFORT)
    feedback=node.create_publisher(MotorStateArray,'/hardware_interface/state',qos)
    target=node.create_publisher(RsuTarget,'/rsu/target',qos)
    upper=node.create_publisher(MotorCommandArray,'/hardware_interface/command',10)
    states={}; refs={}; targets={}; commands=[]; applied=[]; freezes=[]; upper_packets=[]; statuses=[]
    walking=args.scenario=='walking'
    lifecycle=node.create_client(ChangeState,'/roa_main_controller/change_state') if walking else None
    imu=node.create_publisher(Imu,'/imu/data',qos)
    gravity=node.create_publisher(Vector3Stamped,'/imu/gravity',qos)
    velocity_cmd=node.create_publisher(Twist,'/ctrl/policy_cmd',10)
    node.create_subscription(RsuTarget,'/rsu/target',lambda m: targets.update({m.seq:np.array([m.left_roll,m.left_pitch,m.right_roll,m.right_pitch])}),qos)
    if walking: node.create_subscription(MotorCommandArray,'/hardware_interface/command',lambda m: upper_packets.append((time.monotonic(),m)),10)
    node.create_subscription(SystemStatus,'/controller/status',lambda m: statuses.append(m),10)
    node.create_subscription(RsuStateArray,'/rsu/state',lambda m: states.update({ns(m.header.stamp):m}) if m.feasible else None,qos)
    node.create_subscription(RsuStateArray,'/reference/state',lambda m: refs.update({ns(m.header.stamp):m}) if m.feasible else None,qos)
    node.create_subscription(RsuTorqueCommand,'/rsu/torque_command',lambda m: commands.append((time.monotonic(),m)),QoSProfile(depth=2048,reliability=ReliabilityPolicy.BEST_EFFORT))
    node.create_subscription(MotorCommandArray,'/replay/applied_command',lambda m: applied.append((time.monotonic(),m)),10)
    node.create_subscription(Bool,'/replay/frozen',lambda m: freezes.append((time.monotonic(),m.data)),10)
    procs=[]; logs=[]
    def spawn(pkg,exe,extra=()):
        f=(out/f'{pkg}.log').open('w');logs.append(f)
        p=subprocess.Popen([str(install/pkg/'lib'/pkg/exe),*extra],stdout=f,stderr=subprocess.STDOUT)
        procs.append(p);return p
    try:
        v3=spawn('rsu_manager_v3','rsu_manager_v3_node')
        spawn('rsu_manager_v2','rsu_manager_v2_node',['--ros-args','-r','/rsu/state:=/reference/state','-r','/rsu/imp_solution:=/reference/imp_solution'])
        spawn('robstride_hardware_interface','rsu_torque_mock')
        if walking:
            spawn('roa_main_controller','roa_main_controller',['--ros-args','-p','rsu_control_mode:=torque','-p','REALTIME_CONTROL_MODE:=true','-p','topics.walk_policy_cmd:=/ctrl/policy_cmd'])
        # ROS message buffers here are acyclic. Avoid Python full-GC pauses in
        # this synthetic 200Hz feedback producer; production feedback is C++.
        # The watchdog thresholds remain unchanged and still fail closed.
        for _ in range(args.load_workers):
            procs.append(subprocess.Popen(['/usr/bin/python3','-c',
                'import time,sys\nend=time.monotonic()+float(sys.argv[1])\nx=1.0\nwhile time.monotonic()<end:x=(x+1.0)*.999999',
                str(args.duration+2)],stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL))
        gc.collect();gc.disable()
        producer_times=[]
        transition=None; lifecycle_stage=0
        start=time.monotonic(); deadline=start; tick=0; fault_at=None; killed=False
        while time.monotonic()-start<args.duration:
            producer_times.append(time.monotonic())
            t=producer_times[-1]-start
            stamp=node.get_clock().now().to_msg()
            if walking:
                if transition is not None and transition.done():
                    if not transition.result().success: raise RuntimeError('main lifecycle transition failed')
                    lifecycle_stage+=1;transition=None
                if transition is None and lifecycle_stage<2 and lifecycle.service_is_ready():
                    req=ChangeState.Request();req.transition.id=1 if lifecycle_stage==0 else 3
                    transition=lifecycle.call_async(req)
                im=Imu();im.header.stamp=stamp;im.orientation.w=1.;im.linear_acceleration.z=9.81;imu.publish(im)
                g=Vector3Stamped();g.header.stamp=stamp;g.vector.z=-1.;gravity.publish(g)
                vcmd=Twist();vcmd.linear.x=.1;velocity_cmd.publish(vcmd)
            q=np.array([.025*math.sin(.7*t),-.4+.02*math.sin(.4*t)])
            v=np.array([.0175*math.cos(.7*t),.008*math.cos(.4*t)])
            a=lut.query(q); alpha=a[:2]; vel=a[2:].reshape(2,2)@v
            f=MotorStateArray();f.header.stamp=stamp
            for i,id in enumerate((18,20,19,21)):
                s=MotorState();s.motor_id=id;sgn=1 if i<2 else -1
                s.position=float(sgn*alpha[i%2]);s.velocity=float(sgn*vel[i%2]);f.states.append(s)
            if walking:
                for id in range(24):
                    if id in (8,18,20,19,21):continue
                    motor=MotorState();motor.motor_id=id;f.states.append(motor)
            fault=t>=args.duration-1.0 and args.scenario not in ('normal','walking')
            if fault and fault_at is None: fault_at=time.monotonic()
            if fault and args.scenario=='nan': f.states[0].position=float('nan')
            if fault and args.scenario=='missing': f.states.pop()
            if fault and args.scenario=='duplicate': f.states.append(f.states[0])
            if fault and args.scenario=='reorder': f.header.stamp.sec-=1
            if not (fault and args.scenario=='feedback_stop'): feedback.publish(f)
            if not walking and tick%4==0 and not(fault and args.scenario=='target_stop'):
                m=RsuTarget();m.header.stamp=stamp;m.seq=tick//4
                # Slowly varying target at 50Hz, feedback still updates at200Hz.
                offset=.04+.01*math.sin(.3*t)
                m.left_roll=float(q[0]+offset);m.left_pitch=float(q[1]-.04)
                m.right_roll=-m.left_roll;m.right_pitch=-m.left_pitch
                if fault and args.scenario=='old_target':m.header.stamp.sec-=2
                targets[m.seq]=np.array([m.left_roll,m.left_pitch,m.right_roll,m.right_pitch])
                target.publish(m)
            if not walking and not(fault and args.scenario=='upper_stop'):
                p=MotorCommandArray();p.header.stamp=stamp
                p.header.frame_id="/CTRL/RT_CONTROL" if t>1.5 else "INIT POS HOLD"
                for id in range(24):
                    if id==8:continue
                    c=MotorCommand();c.motor_id=id;c.position=.2;c.kp=20.;c.kd=2.;p.commands.append(c)
                upper.publish(p)
            if fault and args.scenario=='node_death' and not killed:
                v3.send_signal(signal.SIGKILL);killed=True
            while time.monotonic()<deadline+.005:
                rclpy.spin_once(node,timeout_sec=max(0.,min(.001,deadline+.005-time.monotonic())))
            tick+=1;deadline+=.005
            if time.monotonic()-deadline>.05:deadline=time.monotonic()
        # Drain before stopping producers; do not mix normal post-test timeout in metrics.
        end=time.monotonic()
        all_healthy=[(t,m)for t,m in commands if t>start+2 and t<end-1.2]
        healthy=[(t,m)for t,m in all_healthy if m.valid]
        dt=np.diff([ns(m.header.stamp) for _,m in healthy])*1e-6
        errors=[]; power_errors=[]
        for _,m in healthy:
            s=states.get(ns(m.feedback_header.stamp));tar=targets.get(m.target_seq)
            if s is None or tar is None:continue
            q=values(s.q);v=values(s.q_dot)
            expected=np.clip(np.array([34.25,25.,34.25,25.])*(tar-q)-np.array([1.644,1.2,1.644,1.2])*v,-11.9,11.9)
            errors.append(float(np.max(np.abs(expected-np.array(m.virtual_torque)))))
            for foot in range(2):
                i=2*foot;internal=q[i:i+2]*(1 if foot==0 else -1)
                a=lut.query(internal)[2:].reshape(2,2)
                power_errors.append(float(np.max(np.abs(a.T@np.array(m.requested_torque[i:i+2])-expected[i:i+2]))))
        common=sorted(set(states)&set(refs))
        stateerr=max([float(np.max(np.abs(values(states[k].q)-values(refs[k].q)))) for k in common]or[float('inf')])
        velerr=max([float(np.max(np.abs(values(states[k].q_dot)-values(refs[k].q_dot)))) for k in common]or[float('inf')])
        merge_ok=True
        for t,p in applied:
            for c in p.commands:
                if c.motor_id in (18,20,19,21):merge_ok &= c.kp==0 and c.kd==0 and c.velocity==0 and abs(c.torque)<=11.90001
                elif not walking:merge_ok &= c.kp==20 and c.kd==2 and abs(c.position-.2)<1e-6
                else:merge_ok &= c.kp>0 and c.kd>0 and math.isfinite(c.position) and c.torque==0.
        prior_freezes=[t for t,f in freezes if f and t<end-1.2]
        post_freezes=[t for t,f in freezes if f and fault_at and t>=fault_at]
        after_fault=[(t,m)for t,m in commands if fault_at and t>fault_at+.2]
        valid_zeros=all(not m.valid and all(x==0 for x in m.torque) for _,m in after_fault)
        checks={'sufficient_commands':len(healthy)>100,'virtual_pd':bool(errors)and max(errors)<2e-5,
            'inverse_transpose':bool(power_errors)and max(power_errors)<2e-5,
            'observation_parity':len(common)>100 and stateerr<1e-6 and velerr<1e-6,
            'motor_ownership':merge_ok and len(applied)>100,'no_early_freeze':not prior_freezes,
            'mean_200hz':len(dt)>0 and abs(float(np.mean(dt))-5)<.15}
        if walking:
            checks['actual_policy_lifecycle']=lifecycle_stage==2
            checks['actual_policy_targets']=len(targets)>100
            checks['complete_23_motor_packets']=len(upper_packets)>100 and all(len(m.commands)==23 for _,m in upper_packets)
            checks['controller_ready_healthy_rt']=any(m.ready and m.healthy and m.rt_ok for m in statuses)
        if fault_at:
            checks['fault_latched']=bool(post_freezes) and all(f for t,f in freezes if t>fault_at+.2)
            checks['applied_stopped']=not any(t>fault_at+.2 for t,_ in applied)
            if args.scenario not in ('upper_stop','node_death'):checks['invalid_zero_output']=bool(after_fault) and valid_zeros
        report={'scenario':args.scenario,'checks':checks,'pass':all(checks.values()),'healthy_commands':len(healthy),
            'applied_packets':len(applied),'policy_targets':len(targets),'full_motor_packets':len(upper_packets),'common_state_epochs':len(common),'state_max_rad':stateerr,'qd_max_rad_s':velerr,
            'virtual_torque_max_error':max(errors)if errors else None,'mapping_max_residual':max(power_errors)if power_errors else None,
            'period_ms':{'mean':float(np.mean(dt)),'p99':float(np.percentile(dt,99)),'max':float(max(dt)), 'above_7_5ms':int(np.sum(dt>7.5))} if len(dt) else {},
            'fault_detection_ms':1000*(post_freezes[0]-fault_at)if post_freezes else None,
            'pre_fault_freezes':len(prior_freezes),
            'producer_gap_ms_max':float(max(np.diff(producer_times))*1000),
            'measurement_torque_subscription_depth':2048,
            'received_invalid_in_healthy_window':sum(not m.valid for _,m in all_healthy),
            'received_sequence_gaps':int(sum(max(0,int(b.seq)-int(a.seq)-1)for (_,a),(_,b)in zip(all_healthy,all_healthy[1:]))),
            'python_gc_during_production':False,'load_workers':args.load_workers,'domain':77,'hardware':'NOT RUN; socket-free HW gate/encoder mock only'}
        (out/'report.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report,indent=2),flush=True)
        if not report['pass']:raise SystemExit(1)
    finally:
        for p in procs:
            if p.poll() is None:p.send_signal(signal.SIGINT)
        for p in procs:
            try:p.wait(timeout=5)
            except subprocess.TimeoutExpired:p.kill();p.wait()
        for f in logs:f.close()
        node.destroy_node();rclpy.shutdown();gc.enable()
if __name__=='__main__':main()
