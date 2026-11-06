import glob

from setuptools import setup

package_name = "arm_ik"

setup(
    name=package_name,
    version="1.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob.glob("launch/*.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="hackathon team",
    maintainer_email="dev@example.com",
    description="IK обеих рук TienKung 2.0 Pro (7 DOF) по URDF из /robot_description",
    license="Apache-2.0",
    entry_points={"console_scripts": ["arm_ik_node = arm_ik.node:main"]},
)
