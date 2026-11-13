from setuptools import find_packages, setup


package_name = 'uav_vln_mapping'


setup(
    name=package_name,

    version='0.1.0',

    packages=find_packages(
        exclude=['test']
    ),

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

    install_requires=[
        'setuptools',
    ],

    zip_safe=True,

    maintainer='uavhe2',

    maintainer_email='uavhe2@example.com',

    description=(
        'RGB-D semantic target localization '
        'and known static obstacle mapping '
        'for UAV-VLN.'
    ),

    license='Apache-2.0',

    entry_points={
        'console_scripts': [

            'target_mapper = '
            'uav_vln_mapping.target_mapper:main',

            'known_map = '
            'uav_vln_mapping.known_map:main',

        ],
    },
)
