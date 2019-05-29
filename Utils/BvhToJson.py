# Imports
# ===========================================================================
from os import listdir
from os.path import isfile, join

# Start of main program
# ===========================================================================

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
