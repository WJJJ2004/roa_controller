from glob import glob
import os

from setuptools import find_packages, setup


package_name = "roa_hw_joint_test"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            ["resource/" + package_name],
        ),
        ("share/" + package_name, ["package.xml", "README.md"]),
        (
            os.path.join("share", package_name, "config"),
            glob("config/*.yaml"),
        ),
        (
            os.path.join("share", package_name, "launch"),
            glob("launch/*.launch.py"),
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="lwj",
    maintainer_email="dldnjswns7412@gmail.com",
    description="Slow mirrored joint-pair direction test for ROA hardware",
    license="MIT",
    entry_points={
        "console_scripts": [
            "mirrored_joint_test_node = "
            "roa_hw_joint_test.mirrored_joint_test_node:main",
        ],
    },
)
