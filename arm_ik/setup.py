import glob
import os
from setuptools import setup

package_name = "arm_ik"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/config", glob.glob("config/*.json")),
        (f"share/{package_name}/launch", glob.glob("launch/*.py")),
        (os.path.join("lib", package_name), glob.glob("scripts/*.py")),
        # standalone-ноды: ros2 run arm_ik arm_ik_node.py находит их в lib/arm_ik
        (os.path.join("lib", package_name), [p for p in glob.glob("bin/*.py") if not p.endswith("_test.py")]),
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
            "demo_delta_publisher = arm_ik.demo_delta_publisher:main",
            "export_kinematics_from_urdf = scripts.export_kinematics_from_urdf:main",
        ],
    },
)
