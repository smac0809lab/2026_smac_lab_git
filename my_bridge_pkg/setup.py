import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'my_bridge_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # 🔴 런치 파일을 복사하도록 추가 (launch 폴더 내의 모든 .py 파일)
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sanguk',
    maintainer_email='lsy7771014@naver.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': ['pytest'],
    },
    entry_points={
        'console_scripts': [
            'bridge_node = my_bridge_pkg.bridge_node:main'
        ],
    },
)