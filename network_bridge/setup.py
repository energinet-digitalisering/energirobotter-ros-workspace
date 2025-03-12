from setuptools import find_packages, setup

package_name = "network_bridge"

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
    description="ROS alternative for network communication, to send and recieve data from Windows computers.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            f"send_camera_node = {package_name}.send_camera_node:main",
            f"receive_tracking_node = {package_name}.receive_tracking_node:main",
        ],
    },
)
