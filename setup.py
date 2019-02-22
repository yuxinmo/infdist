from setuptools import find_packages
from setuptools import setup

package_name = 'comm_evaluation'

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
    author='Michal Barcis',
    author_email='michal.barcis@aau.at',
    maintainer='Michal Barcis',
    maintainer_email='michal.barcis@aau.at',
    keywords=['ROS'],
    classifiers=[
        'Intended Audience :: Developers',
        'License :: OSI Approved :: Apache Software License',
        'Programming Language :: Python',
        'Topic :: Software Development',
    ],
    description='TODO',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'comm_evaluation = comm_evaluation.main:main',
        ],
    },
)
