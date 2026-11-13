from glob import glob
import os

from setuptools import (
    find_packages,
    setup,
)


package_name = (
    'uav_vln_bringup'
)


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

        (
            os.path.join(

                'share',

                package_name,

                'launch',
            ),

            glob(
                'launch/*.launch.py'
            ),
        ),

        (
            os.path.join(

                'share',

                package_name,

                'config',
            ),

            glob('config/*.yaml')
            + glob('config/*.json'),
        ),

        (
            os.path.join(

                'share',

                package_name,

                'models',

                'vln_scene',
            ),

            glob(
                'models/vln_scene/*'
            ),
        ),
    ],

    install_requires=[
        'setuptools'
    ],

    zip_safe=True,

    maintainer='uavhe2',

    maintainer_email='uavhe2@example.com',

    description=(
        'Known-map visual semantic '
        'navigation bringup.'
    ),

    license='Apache-2.0',
)
