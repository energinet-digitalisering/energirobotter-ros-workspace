from setuptools import find_packages, setup

package_name = "control_utils"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Nicoline",
    maintainer_email="xnlth@energinet.dk",
    description="Utilities for control-related tasks.",
    license="Energinet",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            f"pointcloud_publisher = {package_name}.pointcloud_publisher:main",
            f"target_pose_marker = {package_name}.target_pose_marker:main",
        ],
    },
)
