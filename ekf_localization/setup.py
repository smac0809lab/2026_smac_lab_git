import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'ekf_localization'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # 런치 파일 설치 설정
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        # 설정 파일(yaml) 설치 설정
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    description='GPS Reliability Filtered and EKF Launch',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # 실행파일명 = 패키지명.파일명:메인함수

        ],
    },
)