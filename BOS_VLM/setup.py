#!/usr/bin/env python

from setuptools import setup, find_packages

setup(
    name='BOS_VLM',
    version='0.1.0',
    packages=find_packages(),
    install_requires=[
        'numpy>=1.19.0',
        'opencv-python>=4.5.0',
        'transformers>=4.20.0',
        'torch>=1.10.0',
        'pillow>=8.0.0',
        'pyyaml>=5.1',
    ],
    author='Homebrew Robotics Club',
    author_email='contact@homebrewrobotics.org',
    description='Visual Language Models for Brew Operating System',
    license='MIT',
    keywords='vision, language, robotics, BOS',
    url='https://github.com/homebrewroboticsclub/BOS_VLM',
    classifiers=[
        'Development Status :: 3 - Alpha',
        'Intended Audience :: Developers',
        'License :: OSI Approved :: MIT License',
        'Programming Language :: Python :: 3',
        'Topic :: Scientific/Engineering :: Artificial Intelligence',
    ],
)
