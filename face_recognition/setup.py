from setuptools import find_packages, setup

package_name = "face_recognition"

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
    description="Face recognition with YOLOv8 from camera topic.",
    license="Energinet",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "face_recognition_node = face_recognition.face_recognition_node:main"
        ],
    },
)
