import json
import numpy as np
import math
from pyquaternion import Quaternion
from typing import List
from bvh import Bvh
from tqdm import tqdm
from JointInfo import JointInfo

class BvhJointHandler:
    """ Handles conversion of BVH files to DeepMimic format.
    """

    def __init__(self, mocap: Bvh, rigPath="./Rigs/humanoidRig.json", posLocked=False):
        self.mocap = mocap
        self.posLocked = posLocked

        # get json of humanoidRig
        with open(rigPath) as json_data:
            self.humanoidRig = json.load(json_data)

        # Sets up list of bones used by DeepMimic humanoid
        # Order is important
        self.deepMimicHumanoidJoints = ["seconds", "hip", "hip", "chest", "neck", "right hip", "right knee", "right ankle",
                                        "right shoulder", "right elbow", "left hip", "left knee", "left ankle", "left shoulder", "left elbow"]

        self.jointDimensions = [1, 3, 4, 4, 4, 4, 1, 4, 4, 1, 4, 1, 4, 4, 1]

        # Looking directly at the front of the model, X-axis points at you, Y-axis points straight up, Z-axis points left.
        # Image of correct deepMimic humanoid bind pose: https://user-images.githubusercontent.com/43953552/61379957-cb485c80-a8a8-11e9-8b78-24f4bf581900.PNG
        self.rotVecDict = {
            "seconds": [],
            "hip": [0, 0, 0],
            "hip": [0, 0, 0],
            "chest": [0, 1, 0],
            "neck": [0, 1, 0],
            "right hip": [0, -1, 0],
            "right knee": [0, 0, 0],
            "right ankle": [0, 0, 0],
            "right shoulder": [0, -1, 0],
            "right elbow": [0, 0, 0],
            "left hip": [0, -1, 0],
            "left knee": [0, 0, 0],
            "left ankle": [0, 0, 0],
            "left shoulder": [0, -1, 0],
            "left elbow": [0, 0, 0]
        }

        self.jointChildDict = {
            # TODO: expand BVH parser to allow to get child joints
            # Child joints are necessary to be able to compute the offset quaternions correctly.
            "Hips": "Hips",
            "Spine": "Spine1",
            "Neck": "Head",
            "RightUpLeg": "RightLeg",
            "RightLeg": "RightFoot",
            "RightFoot": "RightToeBase",
            "RightArm": "RightForeArm",
            "RightForeArm": "RightHand",
            "LeftUpLeg": "LeftLeg",
            "LeftLeg": "LeftFoot",
            "LeftFoot": "LeftToeBase",
            "LeftArm": "LeftForeArm",
            "LeftForeArm": "LeftHand"
        }

        self.positionChannelNames = ["Xposition", "Yposition", "Zposition"]
        self.rotationChannelNames = ["Xrotation", "Yrotation", "Zrotation"]
        self.generateJointData()

    def generateJointData(self):
        assert len(self.deepMimicHumanoidJoints) == len(self.jointDimensions)

        self.jointData = []

        for i in range(2, len(self.deepMimicHumanoidJoints)):
            deepMimicBoneName = self.deepMimicHumanoidJoints[i]
            bvhBoneName = self.bvhBoneName(deepMimicBoneName)
            jointInfo = JointInfo(
                deepMimicBoneName,
                bvhBoneName,
                self.jointDimensions[i],
                self.rotVecDict[deepMimicBoneName],
                self.getJointOffset(self.jointChildDict[bvhBoneName])
            )
            self.jointData.append(jointInfo)

    def generateKeyFrame(self, frameNumber: int):
        result = []

        # Append Time
        result.append(self.mocap.frame_time)

        # Append hip root pos
        if self.posLocked:
            result.extend([2, 2, 2])
        else:
            result.extend(
                self.getJointTranslation(frameNumber, self.jointData[0])
            )

        # Append hip rotation
        result.extend(
            self.getJointRotation(frameNumber, self.jointData[0])
        )

        # Append other rotations
        for joint in self.jointData[1:]:
            result.extend(self.getJointRotation(frameNumber, joint))

        return result

    def generateKeyFrames(self):
        keyFrames = []
        for i in tqdm(range(0, self.mocap.nframes)):
            keyFrames.append(self.generateKeyFrame(i))

        return keyFrames

    def bvhBoneName(self, deepMimicBoneName):
        return self.humanoidRig[deepMimicBoneName]

    def getJointOffset(self, bvhJointName):
        return list(self.mocap.joint_offset(bvhJointName))

    def getJointTranslation(self, frameNumber: int, jointInfo: JointInfo):
        # TODO: get channel names from file or params
        channels = self.mocap.joint_channels(jointInfo.bvhName)
        result = []

        if len(channels) > 3:
            for channel in self.positionChannelNames:
                result.append(self.mocap.frame_joint_channel(
                    frameNumber, jointInfo.bvhName, channel))
        return result

    def getJointRotation(self, frameNumber: int, jointInfo: JointInfo) -> List[float]:
        # Get the order of channel names as desribed in BVH file for this joint.
        channels = self.mocap.joint_channels(jointInfo.bvhName)
        # Only keep the rotation channels
        channels = list(
            filter(lambda x: x in self.rotationChannelNames, channels))

        # Get the X, Y, Z euler angles
        eulerAngles = []
        for channel in self.rotationChannelNames:
            eulerAngles.append(self.mocap.frame_joint_channel(
                frameNumber, jointInfo.bvhName, channel))

        # Calculate joint rotations
        if jointInfo.dimensions > 1:
            # 4D (quaternion) DeepMimic joint
            rotation = self.eulerToQuat(eulerAngles, channels)
            rotation = JointInfo.quatBvhToDM(rotation)

            offset = jointInfo.offsetQuat

            result = offset * rotation
            return result.elements
        else:
            # 1D DeepMimic joint
            assert jointInfo.dimensions == 1
            # TODO: calculate this angle correctly (keep all DOF's into account)
            return [-math.radians(eulerAngles[0])]

    def eulerToQuat(self, angles: List[float], channels: List[str]):
        # Convert angles to radians
        Xangle = math.radians(angles[0])
        Yangle = math.radians(angles[1])
        Zangle = math.radians(angles[2])

        # Create the rotation matrix for every euler angle
        # See: https://en.wikipedia.org/wiki/Rotation_matrix#In_three_dimensions
        Xrot = np.array(
            [
                [1, 0, 0],
                [0, math.cos(Xangle), -math.sin(Xangle)],
                [0, math.sin(Xangle), math.cos(Xangle)]
            ]
        )
        Yrot = np.array(
            [
                [math.cos(Yangle), 0, math.sin(Yangle)],
                [0, 1, 0],
                [-math.sin(Yangle), 0, math.cos(Yangle)]
            ]
        )
        Zrot = np.array(
            [
                [math.cos(Zangle), -math.sin(Zangle), 0],
                [math.sin(Zangle), math.cos(Zangle), 0],
                [0, 0, 1]
            ]
        )

        # Connect the rotation channel names to corresponding matrices
        rotationDict = {
            self.rotationChannelNames[0]: Xrot,
            self.rotationChannelNames[1]: Yrot,
            self.rotationChannelNames[2]: Zrot
        }

        # Compute the final rotation matrix in the order as the BVH file describes.
        rotationMatrix = rotationDict[channels[2]].dot(
            rotationDict[channels[1]].dot(
                rotationDict[channels[0]]
            )
        )

        # Return the corresponding quaternion
        return Quaternion(matrix=rotationMatrix)
