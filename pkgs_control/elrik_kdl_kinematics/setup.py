from setuptools import setup

package_name = "elrik_kdl_kinematics"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Nicoline",
    maintainer_email="xnlth@energinet.dk",
    description="ROS2 Humble package for Elrik kinematics (URDF, arms FK/IK). ",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "elrik_kdl_kinematics_node = elrik_kdl_kinematics.elrik_kdl_kinematics_node:main",
            "joint_state_plotter = elrik_kdl_kinematics.joint_state_plotter:main",
        ],
    },
)
