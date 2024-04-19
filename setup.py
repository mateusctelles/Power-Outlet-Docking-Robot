from setuptools import find_packages, setup
from glob import glob

package_name = 'gpg_test'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mateus',
    maintainer_email='mateus@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "socket_follower = gpg_test.socket_follower:main",
            "socket_controller = gpg_test.gpg_control_socket:main"
        ],
    },
)
