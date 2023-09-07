import os
from glob import glob
from setuptools import setup


package_name = 'alphabet_manip'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'data'), glob('data/mnist.pt')),
        (os.path.join('share', package_name, 'data/MNIST.v8i.yolov5pytorch'), glob('data/MNIST.v8i.yolov5pytorch/data.yaml')),
        (os.path.join('share', package_name, 'data/MNIST.v8i.yolov5pytorch/valid/images'), glob('data/MNIST.v8i.yolov5pytorch/valid/images/*jpg')),
        (os.path.join('share', package_name, 'data/MNIST.v8i.yolov5pytorch/valid/labels'), glob('data/MNIST.v8i.yolov5pytorch/valid/labels/*txt')),

        ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Denis',
    maintainer_email='denis.teknetas@alumni.fh-aachen.de',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pipe_detect = alphabet_manip.pipe_detect:main',
            'wood_detect = alphabet_manip.wood_detect:main',
            'nav2_action = alphabet_manip.nav2_action:main',
            'number_detect = alphabet_manip.number_detect:main',
            'moveit_action = alphabet_manip.moveit_action:main',
            'moveit_ik = alphabet_manip.moveit_ik:main',
            'ObstacleAvoider = alphabet_manip.ObstacleAvoider:main',
            'mover = alphabet_manip.mover:main',
        ],
    },
)
