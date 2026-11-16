from glob import glob

from setuptools import setup

package_name = "robotic_arm_control"
setup(
    name=package_name,
    version="1.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="engineers over engineering",
    maintainer_email="team@todo.invalid",
    description="Control of the 6-axis arm: homing, IK, pick-and-place state machine, E-stop, logger.",
    license="BSD-3-Clause",
    entry_points={"console_scripts": [
        "pick_place_node = robotic_arm_control.pick_place_node:main",
        "estop_node = robotic_arm_control.estop_node:main",
        "logger_node = robotic_arm_control.logger_node:main",
        "camera_capture = robotic_arm_control.camera_capture:main",
    ]},
)
