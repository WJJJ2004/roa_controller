from glob import glob
import os

from setuptools import find_packages, setup


package_name = "rsu_lut_rt"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="lwj",
    maintainer_email="dldnjswns7412@gmail.com",
    description="Two-process real-time RSU LUT pipeline",
    license="MIT",
    entry_points={
        "console_scripts": [
            "state_node = rsu_lut_rt.state_node:main",
            "solution_node = rsu_lut_rt.solution_node:main",
        ]
    },
)
