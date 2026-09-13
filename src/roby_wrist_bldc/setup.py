import os
from glob import glob

from setuptools import find_packages, setup

package_name = "roby_wrist_bldc"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml", "README.md"]),
        (os.path.join("share", package_name, "config"), glob("config/*")),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "udev"), glob("udev/*")),
    ],
    install_requires=["setuptools", "pyserial"],
    zip_safe=True,
    maintainer="Sam",
    maintainer_email="rmurawka@4cad.fr",
    description=(
        "Axe 5 (poignet) BLDC : driver serie de la carte B-G431B-ESC1 (SimpleFOC) "
        "et noeud pont ros2_control."
    ),
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "wrist_bldc_node = roby_wrist_bldc.node:main",
            "wrist_cli = roby_wrist_bldc.cli:main",
            "wrist_bench = roby_wrist_bldc.bench:main",
        ],
    },
)
