from setuptools import setup

package_name = 'ros2_bridge'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ren',
    maintainer_email='md23062@shibaura-it.ac.jp',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ros2_bridge_2to1= ros2_bridge.ros2_bridge_2to1:main',
            'ros2_bridge_1to2= ros2_bridge.ros2_bridge_1to2:main',
        ],
    },
)
