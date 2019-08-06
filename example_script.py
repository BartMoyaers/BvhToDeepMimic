# Imports
# ===========================================================================

import json
import os
from os import listdir
from os.path import isfile, join
from bvhtodeepmimic.bvhtomimic import BvhConverter

# Function declarations
# ===========================================================================

def removeAllFilesInDirectory(directory):
    onlyfiles = [f for f in listdir(directory) if isfile(join(directory, f))]
    for i in range(0, len(onlyfiles)):
        os.remove(f"{directory}{onlyfiles[i]}")


# Initialization
# ===========================================================================
mypath = "./inputBvh/"
dirnames = ["./OutputMimic/", mypath]
for dirname in dirnames:
    if not os.path.exists(dirname):
        os.makedirs(dirname)

removeAllFilesInDirectory(dirnames[0])

# Locks root position and rotation for dev testing
posLocked = False

# sets onlyfiles to a list of files founds in the "mypath" directory
onlyfiles = [f for f in listdir(mypath) if isfile(join(mypath, f))]


# Start of main program
# ===========================================================================

# for all files to convert
for j in range(0, len(onlyfiles)):

    outputPath = f"./OutputMimic/{onlyfiles[j]}.txt"

    # list containing all the frames
    frames = []

    # open file to convert
    inputPath = "./inputBvh/" + onlyfiles[j]

    # Convert file
    converter = BvhConverter("./Settings/settings.json")

    print("Converting:\t\"" + onlyfiles[j] + "\"")
    converter.writeDeepMimicFile(inputPath, outputPath)
