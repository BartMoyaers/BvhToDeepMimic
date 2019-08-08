import setuptools

with open("README.md", "r") as fh:
    long_description = fh.read()

setuptools.setup(
    name="bvhtodeepmimic",
    version="0.0.5",
    author="Bart Moyaers",
    author_email="bart.moyaers@gmail.com",
    description="Convert .bvh files (Biovision Hierarchy) to DeepMimic format.",
    long_description=long_description,
    long_description_content_type="text/markdown",
    url="https://github.com/BartMoyaers/BvhToDeepMimic",
    packages=["bvhtodeepmimic"],
    py_modules=["bvhtomimic"],
    classifiers=[
        "Programming Language :: Python :: 3",
        "Programming Language :: Python :: 3.6",
        "Programming Language :: Python :: 3.7",
        "License :: OSI Approved :: MIT License",
        "Operating System :: OS Independent",
    ],
    install_requires=["pyquaternion", "numpy", "bvh", "tqdm"],
    python_requires='>=3.6.*'
)