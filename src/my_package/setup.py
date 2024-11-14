from setuptools import setup

package_name = 'my_package'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your_email@example.com',
    description='My custom package for ROS 2 nodes',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'initial_data_processing_node = my_package.initial_data_processing_node:main',
            'yolo_inference_node = my_package.yolo_inference_node:main',
            'segment_node = my_package.segment_node:main',
            'dummy_image_node = my_package.dummy_image_publisher_node:main',
            
            # Add other nodes as needed
        ],
    },
)
