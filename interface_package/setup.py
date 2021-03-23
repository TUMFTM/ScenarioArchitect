import setuptools

"""
NOTE: This script is only used for package generation! Do not execute, unless intended package changes.

Package on pypi.org can be updated with the following commands:
python3 setup.py sdist bdist_wheel
sudo python3 -m twine upload --skip-existing dist/*
"""

with open("README.md", "r") as fh:
    long_description = fh.read()

setuptools.setup(
    name='scenario-testing-tools',
    version='0.82',
    author="Tim Stahl",
    author_email="stahl@ftm.mw.tum.de",
    description="Useful functions used together with a (currently private) scenario generation tool.",
    long_description=long_description,
    long_description_content_type="text/markdown",
    packages=setuptools.find_packages(),
    install_requires=['pandas==0.25.3',
                      'Shapely==1.7.0'],
    classifiers=[
        "Programming Language :: Python :: 3",
        "Programming Language :: Python :: 3.5",
        "Programming Language :: Python :: 3.6",
        "Programming Language :: Python :: 3.7",
        "License :: OSI Approved :: GNU Lesser General Public License v3 (LGPLv3)",
        "Operating System :: OS Independent",
    ])
