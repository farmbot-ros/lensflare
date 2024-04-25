from setuptools import find_packages, setup

package_name = 'yolo8_inference'

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
    maintainer='bresilla',
    maintainer_email='anouk.leunissen@wur.nl',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'yolo8_inference = yolo8_inference.yolo8_inference:main',
            'yolo8_inference2 = yolo8_inference.yolo8_inference2:main'
        ],
    },
)
