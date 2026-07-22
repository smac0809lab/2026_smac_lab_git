from setuptools import setup
import os
from glob import glob

package_name = "iahrs_driver"

setup(
    name=package_name,
    version="0.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        # (선택 사항) 만약 launch 파일을 만든다면 아래 줄이 필요합니다.
        # (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="varofla",
    maintainer_email="dhksrl0508@naver.com",
    description="ROS 2 Driver for iAHRS RB-SDA-v1",
    license="Apache 2.0",
    tests_require=["pytest"],
    entry_points={
        # ros2 run <패키지이름> <실행이름> 형식입니다.
        "console_scripts": [
            "driver = iahrs_driver.driver:main"
        ],
    },
)