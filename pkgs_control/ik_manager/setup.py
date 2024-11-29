from setuptools import find_packages, setup

package_name = "ik_manager"

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
    maintainer="drone",
    maintainer_email="xnlth@energinet.dk",
    description="TODO: Package description",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "ik_manager_node = ik_manager.ik_manager_node:main",
            "ik_visualizer_node = ik_manager.ik_visualizer_node:main",
        ],
    },
)
