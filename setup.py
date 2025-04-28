from setuptools import find_packages, setup

package_name = 'robot_butler'

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
    maintainer='kevell',
    maintainer_email='shanmugasundharam2610@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
                'navigate_task_1 = robot_butler.butler_task1:main',
                'navigate_task_2 = robot_butler.butler_task2:main',
                'navigate_task_3 = robot_butler.butler_task3:main', 
                'navigate_task_4 = robot_butler.butler_task4:main',
        ],
    },
)
