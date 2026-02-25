from setuptools import find_packages, setup

package_name = 'node_templates'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/template_demo.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='student',
    maintainer_email='student@todo.todo',
    description='Template nodes demonstrating action and service patterns',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'service_server_node = node_templates.service_server_node:main',
            'action_server_node = node_templates.action_server_node:main',
            'action_client_node = node_templates.action_client_node:main',
        ],
    },
)
