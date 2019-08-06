# Imports
# ===========================================================================

from bvhtodeepmimic.bvh_extended import BvhExtended
import json
import os
from os import listdir
from os.path import isfile, join
from bvhtodeepmimic.bvh_joint_handler import BvhJointHandler

# Function declarations
# ===========================================================================


def removeAllFilesInDirectory(directory):
    onlyfiles = [f for f in listdir(directory) if isfile(join(directory, f))]
    for i in range(0, len(onlyfiles)):
        os.remove(f"{directory}{onlyfiles[i]}")


# Initialization
# ===========================================================================
dirname = "./OutputMimic/"
if not os.path.exists(dirname):
    os.makedirs(dirname)

removeAllFilesInDirectory(dirname)

# Locks root position and rotation for dev testing
posLocked = False

# sets onlyfiles to a list of files founds in the "mypath" directory
mypath = "./inputBvh/"
onlyfiles = [f for f in listdir(mypath) if isfile(join(mypath, f))]


# Start of main program
# ===========================================================================

# for all files to convert
for j in range(0, len(onlyfiles)):

    with open(f"./OutputMimic/{onlyfiles[j]}.txt", "w") as output:

        # list containing all the frames
        frames = []

        # open file to convert
        with open("./inputBvh/" + onlyfiles[j]) as f:

            # Convert file
            mocap = BvhExtended(f.read())

            jointHandler = BvhJointHandler(mocap, posLocked=posLocked)

            print("Converting:\t\"" + onlyfiles[j] + "\"")
            frames = jointHandler.generateKeyFrames()

            # Output in dictionary format for easy json dump
            outputDict = {
                "Loop": "none",  # "none" or "wrap"
                "Frames": frames
            }

            json.dump(outputDict, output, indent=4)
