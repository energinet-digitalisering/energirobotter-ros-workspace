from setuptools import find_packages, setup

package_name = "webrtc_server_camera"

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
    description="WebRTC server for the ZED cameras from StereoLabs.",
    license="Energinet",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            f"webrtc_server_camera_node = {package_name}.webrtc_server_camera_node:main",
        ],
    },
)
