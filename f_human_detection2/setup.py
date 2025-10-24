from setuptools import setup, find_packages
from glob import glob
import os

package_name = "f_human_detection2"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(include=[package_name, f"{package_name}.*"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
        (os.path.join("share", package_name, "rviz"), glob("rviz/*.rviz")),
        (os.path.join("share", package_name, "models"), glob("models/*")),
    ],
    zip_safe=True,
    install_requires=["setuptools"],
    entry_points={
        "console_scripts": [
            "pose_node = f_human_detection2.pose_node:main",
        ],
    },
)
