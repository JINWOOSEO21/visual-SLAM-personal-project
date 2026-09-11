from pathlib import Path

from setuptools import find_packages, setup

package_name = "mpu6050_driver"
share_dir = Path("share") / package_name

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (str(share_dir / "launch"), [str(p) for p in sorted(Path("launch").glob("*.launch.py"))]),
        (str(share_dir / "config"), [str(p) for p in sorted(Path("config").glob("*.yaml"))]),
    ],
    scripts=["scripts/calibrate_accel.py"],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="jinwooseo",
    maintainer_email="jinwooseo@todo.todo",
    description="MPU6050 IMU driver for ROS2",
    license="MIT",
    extras_require={
        "test": [
            "pytest",
        ],
    },
    entry_points={
        "console_scripts": [
            "mpu6050_node = mpu6050_driver.mpu6050_node:main",
            "imu_kalman_node = mpu6050_driver.imu_kalman_node:main",
        ],
    },
)
