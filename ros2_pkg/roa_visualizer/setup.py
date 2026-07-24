from glob import glob
import os

from setuptools import find_packages, setup

package_name = "roa_visualizer"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            ["resource/" + package_name],
        ),
        ("share/" + package_name, ["package.xml"]),
        (
            os.path.join("share", package_name, "launch"),
            glob("launch/*.launch.py"),
        ),
        (
            os.path.join("share", package_name, "config"),
            glob("config/*.rviz"),
        ),
        (
            os.path.join("share", package_name, "urdf"),
            glob("urdf/*"),
        ),
        (
            os.path.join("share", package_name, "meshes"),
            glob("meshes/*"),
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="ROA Developer",
    maintainer_email="maintainer@example.com",
    description=(
        "RViz visualization bridge for ROA PACE commands "
        "and RSU virtual targets."
    ),
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "command_joint_state_publisher = "
            "roa_visualizer.command_joint_state_publisher:main",
        ],
    },
)
