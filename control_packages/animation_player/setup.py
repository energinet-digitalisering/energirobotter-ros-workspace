from setuptools import find_packages, setup

package_name = 'animation_player'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Nicoline',
    maintainer_email='xnlth@energinet.dk',
    description='Publishes servo angles from CSV file to animate robot.',
    license='Energinet',
    tests_require=['pytest'],
    entry_points={
        "console_scripts": [
            "animation_player_node = animation_player.animation_player_node:main"
        ],
    },
)
