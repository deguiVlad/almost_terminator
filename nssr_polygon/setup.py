# Python package setup of nssr_polygon.
# Author: Nikolay Semenov <nssr.nikolay@gmail.com>
from setuptools import setup

package_name = "nssr_polygon"

setup(
    name=package_name,
    version="1.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", ["config/cameras.yaml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Nikolay Semenov",
    maintainer_email="nssr.nikolay@gmail.com",
    author="Nikolay Semenov",
    author_email="nssr.nikolay@gmail.com",
    description="ROS 2 side of the nssr-polygon manipulation simulator.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "sim_node = nssr_polygon.sim_node:main",
            "camera_bridge = nssr_polygon.camera_bridge:main",
        ],
    },
)
