from setuptools import setup
import os

package_name = 'python'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # 🟢 glob을 쓰지 않고 파일 이름을 직접 명시하여 확실하게 복사합니다.
        (os.path.join('share', package_name, 'launch'), ['launch/control26_launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'odom_parser = python.gps_imu_to_odom:main',
            'path_pub = python.path_publisher:main',
            'gps_imu_stanley = python.control26:main',
        ],
    },
)
