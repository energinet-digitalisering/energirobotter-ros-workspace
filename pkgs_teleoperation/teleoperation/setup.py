from setuptools import find_packages, setup

package_name = "teleoperation"

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
    description="Teleoperation capabilities for the robot Elrik, with Vuer.",
    license="Energinet",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            f"teleoperation_vuer_node = {package_name}.teleoperation_vuer_node:main",
            f"teleoperation_zeromq_node = {package_name}.teleoperation_zeromq_node:main",
        ],
    },
)
