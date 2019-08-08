# BvhToMimic [WIP] [![Build Status](https://travis-ci.org/BartMoyaers/BvhToDeepMimic.svg?branch=master)](https://travis-ci.org/BartMoyaers/BvhToDeepMimic) [![PyPI version](https://badge.fury.io/py/bvhtodeepmimic.svg)](https://badge.fury.io/py/bvhtodeepmimic) ![Codecov](https://img.shields.io/codecov/c/github/BartMoyaers/BvhToDeepMimic) ![PyPI - Python Version](https://img.shields.io/pypi/pyversions/bvhtodeepmimic)

## Goal

The [DeepMimic project](https://github.com/xbpeng/DeepMimic) currently offers no way to import custom reference motions. This is shown in [DeepMimic issue #23](https://github.com/xbpeng/DeepMimic/issues/23). This project aims to transfer animation data from .BVH files into DeepMimic motion files. Motion files can then be used to train DeepMimic skills. Many thanks to user [SleepingFox88](https://github.com/SleepingFox88) for starting off the project. Original repository can be found [here](https://github.com/SleepingFox88/BvhToMimic). This repository currently works well with [this database](http://mocap.cs.sfu.ca/).

## Installation
`pip install bvhtodeepmimic`    
or  
`pip3 install bvhtodeepmimic`

Will install the library together with the requirements. Currently works with python 3.6 or 3.7

## Usage
Create a BvhConverter object:
```python
from bvhtomimic import BvhConverter
converter = BvhConverter("./Settings/settings.json")
```

Generate the DeepMimic text from a .bvh file:
```python
converter.convertBvhFile("pathToBvhFile", loop=False)
```

Or write directly to file:
```python
converter.writeDeepMimicFile(pathToBvhFile, outputPath)
```

Or use [the example script](./example_script.py) that will convert all .bvh files located in ./InputBvh/ into Mimic Motion files, located in ./OutputMimic/ .

## Progress

![Walking_example](./Assets/walking_example.gif)

(GIF has been generated using [this](http://mocap.cs.sfu.ca/nusmocap/0005_Walking001.bvh) mocap file from [this database](http://mocap.cs.sfu.ca/).)

![SpeedVault_example](./Assets/SpeedVault_example.gif)

Watch more videos of recorded conversions [here](https://www.youtube.com/playlist?list=PLd8lridYo1jPV26RsWZIGSJJew9nu4XSF).

## Creating a settings file

Currently joints in .bvh files have to be manually assigned by name to the corresponding joints in the DeepMimic humanoid model. This is done by assigning the .bvh model's bone names to the corresponding joint properties in [./Settings/settings.json](./Settings/settings.json). On top of the joint assignments, this file should also include settings to change the scale by which the .bvh file should be transformed, and the joints used to identify the **root** rotation of the model.

## Related Projects

[List of related projects](https://github.com/SleepingFox88/DeepMimic-Animation-Conversion)

### Notes

This code has been developed within the [ACRO research group](https://iiw.kuleuven.be/onderzoek/acro).