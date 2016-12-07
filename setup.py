from setuptools import setup

def readme():
    with open("README.rst") as f:
        return f.read()

setup(name='bmp180-raspberrypi',
    version='1.0',
    description='A package to handle the i2c communication between a Raspberry Pi and a BMP180',
    classifiers=[
        'License :: OSI Approved :: MIT License',
        'Topic :: Software Development :: Libraries',
        'Programming Language :: Python :: 2.7',
    ],
    keywords='bmp180 raspberrypi raspberry pi',
    url='https://github.com/Tijndagamer/BMP180-Python',
    author='MrTijn/Tijndagamer',
    license='MIT',
    packages=['bmp180'],
    install_requires=[
        'smbus',
    ],
    scripts=['bin/bmp180-example'],
    zip_safe=False)
