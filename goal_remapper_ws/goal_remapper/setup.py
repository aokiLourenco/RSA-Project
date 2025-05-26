from setuptools import setup

package_name = 'goal_remapper'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/goal_remapper_launch.py']),
    ],
    install_requires=[
        'setuptools',
        'paho-mqtt',
    ],
    zip_safe=True,
    maintainer='Aoki',
    maintainer_email='ander5loure@gmail.com',
    description='MQTT-based goal remapper for Autoware',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'goal_remapper_node = goal_remapper.goal_remapper_node:main',
        ],
    },
)
