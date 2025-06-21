from setuptools import find_packages, setup

package_name = 'bv_core'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        # ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/mission_params.yaml']),
        ('share/' + package_name + '/config', ['config/vision_params.yaml']),
        ('share/' + package_name + '/config', ['config/filtering_params.yaml']),
        ('share/' + package_name + '/launch', ['launch/mission.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='eashan',
    maintainer_email='Eashan.Vytla@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mission = bv_core.mission:main',
            'vision_node = bv_core.vision_node:main',
            'filtering_node = bv_core.filtering_node:main',
            'stitching = bv_core.stitching:main'
        ],
    },
)
