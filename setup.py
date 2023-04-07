from setuptools import setup

package_name = 'x3_simulation'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='david',
    maintainer_email='david@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "x3_controller = x3_simulation.x3_controller:main",
            "x3_localizer = x3_simulation.x3_localizer:main",
            "x3_planner = x3_simulation.x3_planner:main",
            "x3_camera_task = x3_simulation.x3_camera_task:main"
        ],
    },
)
