from setuptools import setup

package_name = "robotic_arm_analysis"
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
    maintainer="engineers over engineering",
    maintainer_email="team@todo.invalid",
    description="Static torque, workspace, clearance, offline simulation, figures.",
    license="BSD-3-Clause",
    entry_points={"console_scripts": [
        "make_figures = robotic_arm_analysis.make_figures:main",
        "static_torque = robotic_arm_analysis.static_torque:main",
        "offline_sim = robotic_arm_analysis.offline_sim:main",
        "cross_check = robotic_arm_analysis.cross_check:main",
    ]},
)
