import os
from glob import glob
from setuptools import find_packages, setup

package_name = "object_detection"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        (
            os.path.join("share", "ament_index", "resource_index", "packages"),
            ["resource/" + package_name],
        ),
        (os.path.join("share", package_name), ["package.xml"]),
        (os.path.join("share", package_name, "models"), glob("models/*.pt")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Nicoline",
    maintainer_email="xnlth@energinet.dk",
    description="Object detection with YOLO.",
    license="Energinet",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            f"object_detection_node = {package_name}.object_detection_node:main"
        ],
    },
)
