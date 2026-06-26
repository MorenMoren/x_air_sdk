from setuptools import find_packages, setup

package_name = "io_teleop_bridge"

setup(
    name=package_name,
    version="1.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name + "/launch", ["launch/io_teleop_bridge.launch.py"]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="vlai",
    maintainer_email="user@example.com",
    description=(
        "Bridge /io_teleop/joint_cmd (JointState) to xArm ROS2 controllers."
    ),
    license="Apache License 2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "bridge_node = io_teleop_bridge.bridge_node:main",
        ],
    },
)
