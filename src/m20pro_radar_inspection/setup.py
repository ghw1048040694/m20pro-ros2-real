import os
from glob import glob

from setuptools import setup

package_name = "m20pro_radar_inspection"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="M20Pro Developer",
    maintainer_email="user@example.com",
    description="Radar and LiDAR inspection nodes for task-point scanning on the M20 Pro.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "radar_inspection = m20pro_radar_inspection.radar_inspection_node:main",
        ],
    },
)
