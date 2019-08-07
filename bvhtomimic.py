import json
from bvhtodeepmimic.bvh_extended import BvhExtended
from bvhtodeepmimic.bvh_joint_handler import BvhJointHandler

class BvhConverter:
    def __init__(self, setting_path: str):
        self.setting_path = setting_path

    def convertBvhFile(self, filePath: str, loop=False):
        with open(filePath) as bvhFile:
            mocap = BvhExtended(bvhFile.read())
        
        jointHandler = BvhJointHandler(mocap, settingsPath=self.setting_path)
        frames = jointHandler.generateKeyFrames()

        loopText = "none"
        if loop:
            loopText = "wrap"

        # Output in dictionary format for easy json dump
        outputDict = {
            "Loop": loopText,  # "none" or "wrap"
            "Frames": frames
        }

        return json.dumps(outputDict, indent=4)

    def writeDeepMimicFile(self, bvhPath, outputPath):
        with open(outputPath, "w") as output:
            output.write(self.convertBvhFile(bvhPath))