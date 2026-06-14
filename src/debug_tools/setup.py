from setuptools import find_packages, setup

setup(
    name='debug_tools',
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/debug_tools']),
        ('share/debug_tools', ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ubuntu',
    maintainer_email='todo@todo.todo',
    description='Debug tools for laser battle car',
    license='MIT',
    entry_points={
        'console_scripts': [
            'hsv_sampler = debug_tools.hsv_sampler:main',
            'attack_monitor = debug_tools.attack_monitor:main',
        ],
    },
)
