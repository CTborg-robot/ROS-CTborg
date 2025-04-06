from setuptools import setup

package_name = 'object_tracker'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/rspec', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'opencv-python', 'cv-bridge'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='Object tracking using OpenCV',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'object_tracker = object_tracker.object_tracker:main',
        ],
    },
)

