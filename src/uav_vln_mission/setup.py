from setuptools import (
    find_packages,
    setup,
)


package_name = 'uav_vln_mission'


setup(

    name=package_name,

    version='0.2.0',

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
        'Validated semantic mission execution '
        'and command-line console.'
    ),

    license='Apache-2.0',

    entry_points={

        'console_scripts': [

            'mission_manager = '
            'uav_vln_mission.mission_manager:main',

            'cli_console = '
            'uav_vln_mission.cli_console:main',
        ],
    },
)
