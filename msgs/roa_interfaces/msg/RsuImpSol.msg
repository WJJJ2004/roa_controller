# /rsu/imp_solution
# this message is for impedance control solution, which is the output of the impedance controller
std_msgs/Header header
uint32 seq

RsuActuator left_actuator_1
RsuActuator left_actuator_2

RsuActuator right_actuator_1
RsuActuator right_actuator_2

bool feasible