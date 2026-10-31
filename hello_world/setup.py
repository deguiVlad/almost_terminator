# Python package setup of hello_world.
# Author: Nikolay Semenov <nssr.nikolay@gmail.com>
from setuptools import setup

package_name = "hello_world"

setup(
    name=package_name,
    version="1.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Nikolay Semenov",
    maintainer_email="nssr.nikolay@gmail.com",
    author="Nikolay Semenov",
    author_email="nssr.nikolay@gmail.com",
    description="Starter package for the nssr-polygon manipulation track.",
    license="Apache-2.0",
    entry_points={"console_scripts": ["hello = hello_world.hello:main"]},
)
