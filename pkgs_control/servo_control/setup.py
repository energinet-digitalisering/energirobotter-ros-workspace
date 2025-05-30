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
            "elrik_servo_manager_node = servo_control.elrik_servo_manager_node:main",
            "wattson_servo_manager_node = servo_control.wattson_servo_manager_node:main",
            "servo_reset_node = servo_control.servo_reset_node:main",
            "servo_control_node = servo_control.servo_control_node:main",
            "servo_driver_arduino = servo_control.test_nodes.servo_driver_arduino:main",
            "servo_driver_pca9685 = servo_control.test_nodes.servo_driver_pca9685:main",
            "servo_driver_waveshare = servo_control.test_nodes.servo_driver_waveshare:main",
            "servo_acc_test_node = servo_control.test_nodes.servo_acc_test_node:main",
            "servo_acc_test2_node = servo_control.test_nodes.servo_acc_test2_node:main",
        ],
    },
)
