# Imports
# ===========================================================================


import os
from os import listdir
from os.path import isfile, join
import time

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

            # Start of file conversion
            hitAnimation = False
            for x in range(0, len(content)):
                tokens = str.split(content[x])
                tabDepth = getTabs(content[x])
                strToPrint = ""
                curToken = 0
                stringLeft = ""

                if hitAnimation == False:
                    if x < len(content) - 1:
                        nextTokens = str.split(content[x + 1])
                    else:
                        nextTokens = ""

                    if len(tokens) > 0:

                        # Property is "ReferenceTime"
                        if tokens[0] == "HIERARCHY":
                            strToPrint = "\"HIERARCHY\": \"\","

                        # Line after this has last character "{"
                        elif nextTokens != "" and nextTokens[len(nextTokens) - 1] == "{":
                            strToPrint += "\""
                            for o in range(0, len(tokens)):
                                strToPrint = strToPrint + tokens[o]
                                if o < len(tokens) - 1:
                                    strToPrint += "_"
                            strToPrint += "\":"

                        else:

                            # last char is "{"
                            if tokens[len(tokens) - 1] == "{":
                                strToPrint = "{"

                            # last char is "}"
                            elif tokens[len(tokens) - 1] == "}":
                                strToPrint = "}"
                            else:
                                strToPrint += "\""
                                strToPrint += tokens[0]
                                strToPrint += "\""
                                strToPrint += ": "
                                strToPrint += "\""
                                for o in range(0, len(tokens)):
                                    if o > 0:
                                        strToPrint += tokens[o]
                                        if o < len(tokens) - 1:
                                            strToPrint += " "

                                strToPrint += "\""
                                # strToPrint = "\"test\": \"\""

                            # add comma if next depth == curDepth && not already have comma
                            if x < len(content) - 1:
                                if depthOf(content[x]) == depthOf(content[x + 1]):
                                    if strToPrint[len(strToPrint) - 1] != ",":
                                        strToPrint += ","

                # hitAnimation == true
                else:
                    print("")

                # print(strToPrint)
                # If something to print
                if strToPrint != "":
                    strToPrint = f"{tabDepth}{strToPrint}"
                    print(f"{strToPrint}", file=output)

            # End JSON object
            print(f"}}", file=output)
