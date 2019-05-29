# Imports
# ===========================================================================


import numpy as np
import math
import json
import os
from os import listdir
from os.path import isfile, join


# Function declarations
# ===========================================================================


def euler_to_quaternion(heading, attitude, bank):
    c1 = np.cos(heading/2)
    s1 = np.sin(heading/2)
    c2 = np.cos(attitude/2)
    s2 = np.sin(attitude/2)
    c3 = np.cos(bank/2)
    s3 = np.sin(bank/2)
    c1c2 = c1 * c2
    s1s2 = s1 * s2
    w = c1c2*c3 - s1s2*s3
    x = c1c2*s3 + s1s2*c3
    y = s1*c2*c3 + c1*s2*s3
    z = c1*s2*c3 - s1*c2*s3

    return [w, x, y, z]


def bvhBoneName(mimicBone):
    return humanoidRig[f"{mimicBone}"]

# Start of main program
# ===========================================================================


# get json of humanoidRig
with open(f"./Rigs/humanoidRig.json") as json_data:
    humanoidRig = json.load(json_data)

# Order is important
deepMimicHumanoidJoints = ["seconds", "hip", "hip", "chest", "neck", "right hip", "right knee", "right ankle",
                           "right shoulder", "right elbow", "left hip", "left knee", "left ankle", "left shoulder", "left elbow"]
dimensions = [1, 3, 4, 4, 4, 4, 1, 4, 4, 1, 4, 1, 4, 4, 1]
posLocked = True

mypath = "./Utils/Temp/"
onlyfiles = [f for f in listdir(mypath) if isfile(join(mypath, f))]

for j in range(0, len(onlyfiles)):
    # Open files, Start of main program
    with open(f"./Utils/Temp/{onlyfiles[j]}") as json_data:
        with open(f"./OutputMimic/{onlyfiles[j]}.txt", "w") as output:
            d = json.load(json_data)

            # print(d)
            Frames = int(d["Frames:"])

            # For every unique time, create a keyFrame
            keyFrame = []
            for i in range(0, Frames):

                print(f"Keyframe: {i}/{Frames}", end="\r")
                oldKeyframe = keyFrame
                keyFrame = []
