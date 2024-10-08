from setuptools import find_packages, setup

package_name = "servo_control"

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
    description="Package implementing PID control and communication of/with servo motors, with Serial or I2C protocol.",
    license="Energinet",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "servo_control_node = servo_control.servo_control_node:main"
        ],
    },
)
