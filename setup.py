#!/usr/bin/env python

from setuptools import setup

with open("README.md", "r", encoding="utf-8") as f:
    long_description: str = f.read()


with open("requirements.txt", "r", encoding="utf-8") as f:
    requirements: list[str] = f.read().splitlines()


with open("requirements-dev.txt", "r", encoding="utf-8") as f:
    requirements_dev: list[str] = f.read().splitlines()


with open("stretch/__version__.txt", "r", encoding="utf-8") as fh:
    version = fh.read().strip()


setup(
    name="super-simple-stretch-api",
    version=version,
    description="Super simple Python API for controlling the Stretch RE1 robot",
    author="Benjamin Bolte",
    url="https://github.com/codekansas/super-simple-stretch-api",
    long_description=long_description,
    long_description_content_type="text/markdown",
    classifiers=[
        "Development Status :: 4 - Beta",
        "Intended Audience :: Developers",
        "License :: OSI Approved :: MIT License",
        "Programming Language :: Python :: 3",
    ],
    python_requires=">=3.10",
    install_requires=requirements,
    tests_require=requirements_dev,
    extras_require={"dev": requirements_dev},
    package_data={"stretch": ["py.typed"]},
)
