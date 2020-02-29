from setuptools import setup, find_packages, Extension
from Cython.Build import cythonize


package_name = 'infdist'


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
            'infdist = infdist.main:main',
        ],
    },
    ext_modules=cythonize(
        [
            Extension(
                'infdist.optimization.aggregations',
                ["infdist/optimization/aggregations.pyx"],
            ),
            Extension(
                'infdist.optimization.dynamic_models',
                ["infdist/optimization/dynamic_models.pyx"],
            ),
        ]
    )
)
