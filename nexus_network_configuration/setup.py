import os
from glob import glob
from setuptools import setup

package_name = 'nexus_network_configuration'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            [os.path.join('resource', package_name)]),
        (os.path.join('share', package_name), ['package.xml']),
        (os.path.join('share', package_name, 'schemas'),
         glob('schemas/*.json')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='john',
    maintainer_email='johntan@openrobotics.org',
    description='This package generates zenoh bridge configurations for the \
       NEXUS Network',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'nexus_network_configuration = \
              nexus_network_configuration.nexus_network_configuration:main'
        ],
    },
)
