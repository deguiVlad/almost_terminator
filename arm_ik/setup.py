import os
from setuptools import setup  # glob нужен при включении srv: , glob

package_name = "arm_ik"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/config", ["config/arms_template.json"]),
        (f"share/{package_name}/launch", ["launch/arm_ik.launch.py"]),
        (f"share/{package_name}/scripts", ["scripts/export_kinematics_from_urdf.py"]),
    ],
    install_requires=["setuptools", "numpy", "scipy"],
    zip_safe=True,
    maintainer="hackathon team",
    maintainer_email="dev@example.com",
    description="IK обеих рук TienKung 2.0 Pro (7 DOF) c limits и защитой стола",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "arm_ik_node = arm_ik.arm_ik_node:main",
            "demo_goal_publisher = arm_ik.demo_goal_publisher:main",
        ],
    },
)
