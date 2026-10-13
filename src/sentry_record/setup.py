from setuptools import find_packages, setup
from glob import glob

package_name = 'sentry_record'
setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/'+package_name]),
        ('share/'+package_name, ['package.xml']),
        ('share/'+package_name+'/launch', glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@example.com',
    description='Optional sentry topic and telemetry recording',
    license='Apache-2.0',
    entry_points={'console_scripts': ['telemetry_node = sentry_record.telemetry_node:main']},
)
