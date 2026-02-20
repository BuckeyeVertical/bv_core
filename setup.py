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
        ('share/' + package_name + '/config',
         ['config/filtering_params.yaml']),
        ('share/' + package_name + '/config', ['config/test_obj_loc.yaml']),
        ('share/' + package_name + '/launch', ['launch/mission.launch.py']),
        ('share/' + package_name + '/meshes', ['meshes/Render_CAD.STL']),
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
            'mission_node = bv_core.mission:main',
            'vision_node = bv_core.vision_node:main',
            'filtering_node = bv_core.filtering_node:main',
            'stitching_node = bv_core.stitching:main',
            'bv_viz_node = bv_core.bv_viz_node:main',
            'test_servo = bv_core.test_servo:main',
            'camera_pipeline_test_node = bv_core.camera_pipeline_test_node:main',
            'test_obj_loc = bv_core.test_obj_loc:main',
        ],
    },
)
