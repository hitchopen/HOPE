import os
from glob import glob

from setuptools import find_packages, setup

package_name = "hope_planner"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="HOPE Maintainers",
    maintainer_email="maintainer@example.com",
    description="HOPE no-spin model-based racket planner.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "hope_planner_node = hope_planner.node:main",
            "hope_bag_to_csv = hope_planner.bag_to_csv:main",
        ],
    },
)
