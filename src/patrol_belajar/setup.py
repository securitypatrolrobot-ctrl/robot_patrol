from setuptools import setup

package_name = 'patrol_belajar'

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
    maintainer='patrol',
    maintainer_email='patrol@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
    'console_scripts': [
        'publisher_node = patrol_belajar.publisher_node:main',
        'subscriber_node = patrol_belajar.subscriber_node:main',
     ],
    },
)
