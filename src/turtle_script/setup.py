from setuptools import find_packages, setup

package_name = 'turtle_script'

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
    maintainer='Utkarsh',
    maintainer_email='no_reply@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'circle_drawer = turtle_script.circle_drawer:main',
            'polygon_drawer = turtle_script.polygon_drawer:main',
            'turtle_controller = turtle_script.turtle_controller:main',
            'turtle_spawner = turtle_script.turtle_spawner:main',
            'area_service_server = turtle_script.area_service_server:main',
            'area_service_client = turtle_script.area_service_client:main'
        ],
    },
)
