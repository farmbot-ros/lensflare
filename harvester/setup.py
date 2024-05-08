from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'harvester'

data_files = []
#resources
data_files.append(('share/' + package_name + '/resource', glob('resource/*')))
data_files.append(('share/' + package_name, ['package.xml']))

#launch files
data_files.append((os.path.join('share', package_name), glob('launch/*.py')))
#params
data_files.append(('share/' + package_name + '/configs', glob('configs/*')))


setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='bresilla',
    maintainer_email='trim.bresilla@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'camera_node = harvester.camera_node:main',
            'camera_node2 = harvester.camera_node2:main',
            'camera_info = harvester.camera_info:main',
            'image_saver = harvester.image_saver:main',
            'timer_trigger = harvester.trigger_node:main',
        ],
    },
)
