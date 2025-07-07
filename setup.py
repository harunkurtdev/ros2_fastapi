from setuptools import find_packages, setup

package_name = 'ros2_fastapi'

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
    maintainer='harunkurtdev',
    maintainer_email='harunkurt.dev@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ros2_fastapi = ros2_fastapi.ros2_fastapi:main'
        ],
    },
)
