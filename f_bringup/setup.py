from setuptools import setup

package_name = 'f_bringup'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/bringup.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Evgenii Shlomov',
    maintainer_email='e.shlomov@innopolis.university',
    description='Launch files and bringup for Unitree A1',
    license='MIT',
    entry_points={
        'console_scripts': [],
    },
)
