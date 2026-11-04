import glob
import os
from setuptools import setup

package_name = "arm_ik"

# Важно: имена исполняемых в bin/ НЕ совпадают с именами модулей в arm_ik/nodes/
# (run_arm_ik.py vs arm_ik_node.py). Это исключает классическую ошибку
#   ImportError: cannot import name 'main' from partially initialized module
# которая возникала, когда wrapper в lib/arm_ik назывался так же, как модуль,
# и Python принимал wrapper за сам модуль (circular import).
setup(
    name=package_name,
    version="0.2.2",
    packages=[package_name, f"{package_name}.nodes"],
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml", "README.md", "CHANGELOG.md"]),
        (f"share/{package_name}/config", glob.glob("config/*.json")),
        (f"share/{package_name}/launch", glob.glob("launch/*.py")),
        # standalone-скрипты + bootstrap едут в libexec одним куском:
        (os.path.join("lib", package_name),
         ["bin/_bootstrap.py", "bin/run_arm_ik.py",
          "bin/run_demo_goal.py", "bin/run_demo_delta.py"]),
    ],
    install_requires=["setuptools", "numpy", "scipy"],
    zip_safe=True,
    maintainer="hackathon team",
    maintainer_email="dev@example.com",
    description="IK обеих рук TienKung 2.0 Pro (7 DOF) c limits и защитой стола",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            # ros2 run arm_ik arm_ik / demo_goal / demo_delta — без коллизий имён
            "arm_ik = arm_ik.nodes.arm_ik_node:main",
            "demo_goal = arm_ik.nodes.demo_goal_publisher:main",
            "demo_delta = arm_ik.nodes.demo_delta_publisher:main",
        ],
    },
)
