from setuptools import setup

package_name = 'my_package'
submodules = "my_package/submodules"

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name, submodules],
    data_files=[('share/ament_index/resource_index/packages', ['resource/' + package_name]), ('share/' + package_name, ['package.xml']),],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ubuntu',
    maintainer_email='ubuntu@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'xbox_one_controller_node = my_package.xbox_one_controller_node:main',
            'md25_drive_node = my_package.md25_drive_node:main',
            'new_md25_drive_node = my_package.new_md25_drive_node:main',
            'new_xbox_one_controller_node = my_package.new_xbox_one_controller_node:main'
        ],
    },
)
