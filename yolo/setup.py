from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'yolo'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name, 'models'), glob(os.path.join('models', '*.onnx')))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='lior',
    maintainer_email='lior.falach@caesarealabs.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'image_node = yolo.image_saver:main'
        ],
    },
)
