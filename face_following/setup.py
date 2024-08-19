from setuptools import find_packages, setup

package_name = "face_following"

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
    description="Given a bounding box of a detected face, this package will create movement commands for servos in a pan-til configuration, to move the camera to look at the face.",
    license="Energinet",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "face_following_node = face_following.face_following_node:main"
        ],
    },
)
