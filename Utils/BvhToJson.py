# Imports
# ===========================================================================


import os
from os import listdir
from os.path import isfile, join

# Function declarations
# ===========================================================================


def depthOf(str):
    depth = 0
    for z in range(0, len(str)):
        if str[z] == "\t":
            depth += 1
        else:
            return depth
    return depth


def getTabs(str):
    tabs = ""
    for i in range(0, depthOf(str)):
        tabs = tabs + "\t"
    return tabs + "\t"


# Remove all files in given directory
def removeAllFilesInDirectory(directory):
    onlyfiles = [f for f in listdir(directory) if isfile(join(directory, f))]
    for i in range(0, len(onlyfiles)):
        os.remove(f"{directory}{onlyfiles[i]}")

# Start of main program
# ===========================================================================


removeAllFilesInDirectory("./Utils/Temp/")

mypath = "./InputBvh"
onlyfiles = [f for f in listdir(mypath) if isfile(join(mypath, f))]

print(f"Files to Convert: {onlyfiles}")
for j in range(0, len(onlyfiles)):
    with open(f"./InputBvh/{onlyfiles[j]}") as input:
        with open(f"./Utils/Temp/{onlyfiles[j]}.json", "w") as output:
            print(f"opened file:    {onlyfiles[j]}")

            content = input.readlines()

            # Start JSON object
            print(f"{{", file=output)

            # End JSON object
            print(f"}}", file=output)
