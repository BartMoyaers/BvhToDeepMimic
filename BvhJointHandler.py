import json
import numpy as np
import math
from pyquaternion import Quaternion
from typing import List
from bvh import Bvh
from tqdm import tqdm


class JointInfo:
    def __init__(self, deepMimicName: str, bvhName: str, dimensions: int,
                 zeroRotVector: List[float], childOffset: List[float]):
        self.deepMimicName = deepMimicName
        self.bvhName = bvhName
        self.dimensions = dimensions
        self.zeroRotVector = zeroRotVector
        self.childOffset = childOffset

        self.offsetQuat = self.calculateOffsetQuat()

    def calculateOffsetQuat(self) -> Quaternion:
        v1 = np.array(self.zeroRotVector)
        v2 = np.array(self.childOffset)

        # Normalize vectors
        norm1 = np.linalg.norm(v1)
        norm2 = np.linalg.norm(v2)
        if norm1 > 0:
            v1 = v1 / norm1
        if norm2 > 0:
            v2 = v2 / norm2

        # Calculate perpendicular vector
        perpvec = np.cross(v1, v2)
        perpnorm = np.linalg.norm(perpvec)
        if perpnorm > 0:
            perpvec = perpvec / perpnorm
            angle = math.acos(np.dot(v1, v2))
        else:
            perpvec = np.array([1,0,0])
            angle = 0

        result = Quaternion(axis=perpvec, radians=angle)

        # Note that this quaternion is defined in the frame of the BVH file.
        return self.quatBvhToDM(result)

    @staticmethod
    def quatBvhToDM(quaternion: Quaternion) -> Quaternion:
        # transform x -> z and z -> -x
        return Quaternion(
            quaternion[0],
            quaternion[3],
            quaternion[2],
            -quaternion[1]
        )

    @staticmethod
    def EulerXYZToQuaternion(Xangle, Yangle, Zangle) -> Quaternion:
        X = math.radians(Xangle)
        Y = math.radians(Yangle)
        Z = math.radians(Zangle)

        cy = math.cos(X * 0.5)
        sy = math.sin(X * 0.5)
        cp = math.cos(Y * 0.5)
        sp = math.sin(Y * 0.5)
        cr = math.cos(Z * 0.5)
        sr = math.sin(Z * 0.5)

        q = Quaternion()
        q[0] = cy * cp * cr + sy * sp * sr
        q[1] = cy * cp * sr - sy * sp * cr
        q[2] = sy * cp * sr + cy * sp * cr
        q[3] = sy * cp * cr - cy * sp * sr

        return q

class BvhJointHandler:
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

        self.rotVecDict = {
            "seconds": [],
            "hip": [0,0,0],
            "hip": [0,0,0],
            "chest": [0,1,0],
            "neck": [0,1,0],
            "right hip": [0,-1,0],
            "right knee": [0,0,0],
            "right ankle": [0,0,1],
            "right shoulder": [0,-1,0],
            "right elbow": [0,0,0],
            "left hip": [0,-1,0],
            "left knee": [0,0,0],
            "left ankle": [0,0,0],
            "left shoulder": [0,-1,0],
            "left elbow": [0,0,0]
        }

        self.jointChildDict = {
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
            result.extend([2,2,2])
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
                result.append(self.mocap.frame_joint_channel(frameNumber, jointInfo.bvhName, channel))
        return result

    def getJointRotation(self, frameNumber: int, jointInfo: JointInfo) -> List[float]:
        eulerAngles = []
        for channel in self.rotationChannelNames:
            eulerAngles.append(self.mocap.frame_joint_channel(frameNumber, jointInfo.bvhName, channel))

        if jointInfo.dimensions > 1:
            rotation = JointInfo.EulerXYZToQuaternion(eulerAngles[0], eulerAngles[1], eulerAngles[2])
            offset = jointInfo.offsetQuat
            yRot = JointInfo.EulerXYZToQuaternion(0,180,0)
            
            result = yRot * (offset * rotation)
            return result.elements
        else:
            assert jointInfo.dimensions == 1
            # TODO: check if this is really always the X rotation
            # (Maybe needs some kind of projection)
            return [math.radians(eulerAngles[0])]
