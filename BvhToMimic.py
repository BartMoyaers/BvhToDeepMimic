# Imports
# ===========================================================================

import os
from os import listdir
from os.path import isfile, join

# Function declarations
# ===========================================================================


# Remove all files in given directory
def removeAllFilesInDirectory(directory):
    onlyfiles = [f for f in listdir(directory) if isfile(join(directory, f))]
    for i in range(0, len(onlyfiles)):
        os.remove(f"{directory}{onlyfiles[i]}")


removeAllFilesInDirectory("./OutputMimic/")

os.system("python ./Utils/BvhToJson.py")
# os.system("python ./Utils/JsonToMimic.py")
