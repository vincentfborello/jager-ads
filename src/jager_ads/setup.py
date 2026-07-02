from setuptools import find_packages, setup

package_name = 'jager_ads'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ubuntu',
    maintainer_email='ubuntu@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'test_node = jager_ads.test_node:main',
            'vision_node = jager_ads.vision_node:main',
            'turret_controller_node = jager_ads.turret_controller_node:main',
            'camera_node = jager_ads.camera_node:main',
        ],
    },
)
