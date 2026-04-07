from setuptools import setup
import os
from glob import glob

package_name = 'armor1_pose_solver'

config_files = glob(os.path.join(os.path.dirname(__file__), 'config', '*'))
# 提取文件名，适配data_files的路径格式
config_data = [(os.path.join('share', package_name, 'config'), config_files)]

setup(    
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        *config_data,
    ],
    
    install_requires=['setuptools', 'numpy', 'opencv-python', 'ultralytics', 'pyyaml'],
    zip_safe=True,
    maintainer='hh',
    maintainer_email='hh@example.com', 
    description='ROS2 Python装甲板检测节点+实现YOLO检测+PnP位姿解算+自定义msg发布',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'armor_detect_node = armor1_pose_solver.armor_detect_node:main',
        ],
    },
)
