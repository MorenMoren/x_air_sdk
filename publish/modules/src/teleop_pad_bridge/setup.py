from setuptools import find_packages, setup

package_name = 'teleop_pad_bridge'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='wangzhangming',
    maintainer_email='wangzhangming@example.com',
    description='桥接节点 — 订阅 /pad_control，拆分发布到各执行话题',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'teleop_pad_bridge = teleop_pad_bridge.teleop_bridge_node:main',
        ],
    },
)
