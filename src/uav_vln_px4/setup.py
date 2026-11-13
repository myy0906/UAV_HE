from setuptools import find_packages, setup

package_name = 'uav_vln_px4'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name],
        ),
        (
            'share/' + package_name,
            ['package.xml'],
        ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='uavhe2',
    maintainer_email='uavhe2@localhost',
    description='PX4 execution layer for the UAV-VLN system.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'px4_executor = uav_vln_px4.px4_executor:main',
        ],
    },
)
