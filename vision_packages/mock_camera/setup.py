import os
from glob import glob
from setuptools import find_packages, setup

package_name = "mock_camera"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "images"), glob("images/*.jpg")),
    ],
    install_requires=["setuptools", "opencv-python"],
    zip_safe=True,
    maintainer="Nicoline",
    maintainer_email="xnlth@energinet.dk",
    description="Periodically publish an image to camera topic, for testing purposes.",
    license="Energinet",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "photo_pub_node = mock_camera.photo_pub_node:main",
            "webcam_pub_node = mock_camera.webcam_pub_node:main",
        ],
    },
)
