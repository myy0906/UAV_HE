from setuptools import find_packages, setup

package_name = 'uav_vln_perception'

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
    maintainer_email='uavhe2@example.com',
    description='Vision perception layer for UAV-VLN.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'red_target_detector = '
            'uav_vln_perception.red_target_detector:main',
        ],
    },
)
