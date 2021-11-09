import setuptools

with open("README.md", "r", encoding="utf-8") as fh:
    long_description = fh.read()

setuptools.setup(
    name="pyedo",
    version="0.6",
    author="COMAU",
    author_email="info@edo.cloud",
    description="This package contains the SDK to program e.DO robot with Python",
    long_description=long_description,
    long_description_content_type="text/markdown",
    url="https://github.com/comau/pyedo",
    project_urls={
        "Bug Tracker": "https://github.com/Comau/pyedo/issues",
    },
    classifiers=[
        "Programming Language :: Python",
        "License :: OSI Approved :: GNU General Public License v3 (GPLv3)",
        "Operating System :: OS Independent",
    ],
    packages=setuptools.find_packages(where="src"),
    package_dir={"": "src"},
    package_data={
        # And include any *.dat files found in the "data" subdirectory
        # of the "mypkg" package, also:
        "pyedo": ["examples/*.py"],
    }
)