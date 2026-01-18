from setuptools import find_packages, setup

package_name = 'ch3'

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
    maintainer='ros_user',
    maintainer_email='adpoitras@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'diff_drive_odometry = ch3.diff_drive_odometry:main',
            'open_loop = ch3.open_loop:main',
            'closed_loop = ch3.closed_loop:main',
            'closed_loop_pid = ch3.closed_loop_pid:main',
        ],
    },
)
