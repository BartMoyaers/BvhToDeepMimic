import json
import numpy as np
import math
from pyquaternion import Quaternion
from typing import List
from BvhExtended import BvhExtended
from tqdm import tqdm
from JointInfo import JointInfo
from BvhJoint import BvhJoint

class BvhJointHandler:
    """ Handles conversion of BVH files to DeepMimic format.
    """

    def __init__(self, mocap: BvhExtended, settingsPath="./Settings/settings.json", posLocked=False):
        self.mocap = mocap
        self.posLocked = posLocked

        # Get settings json
        with open(settingsPath) as json_data:
            self.settings = json.load(json_data)

        self.scaleFactor = self.settings["scale"]
        self.deepMimicHumanoidJoints = self.settings["joints"]
        self.jointDimensions = self.settings["jointDimensions"]
        self.rotVecDict = self.settings["zeroRotationVectors"]
        self.rootUp = self.settings["rootRotJoints"]["root rot up"]
        self.rootLeft = self.settings["rootRotJoints"]["root rot left"]
        positionChannelNames = self.settings["positionChannelNames"]
        rotationChannelNames = self.settings["rotationChannelNames"]

        self.jointData = []
        self.generateJointData()

        # Joint tree starting at root
        self.root = BvhJoint(
                            self.mocap,
                            self.settings["jointAssignments"][self.deepMimicHumanoidJoints[1]],
                            positionChannelNames,
                            rotationChannelNames, 
        )

    def generateJointData(self):
        assert len(self.deepMimicHumanoidJoints) == len(self.jointDimensions)

        for i in range(2, len(self.deepMimicHumanoidJoints)):
            deepMimicBoneName = self.deepMimicHumanoidJoints[i]
            bvhBoneName = self.bvhBoneName(deepMimicBoneName)
            jointInfo = JointInfo(
                deepMimicBoneName,
                bvhBoneName,
                self.jointDimensions[i],
                self.rotVecDict[deepMimicBoneName],
            )
            self.jointData.append(jointInfo)

    def generateKeyFrame(self, frameNumber: int):
        result = []
        # Update positions and transformation
        self.root.update(frameNumber)
        self.current_hip_rotation = self.getRootQuat()

        # Append Time
        result.append(self.mocap.frame_time)

        # Append hip root pos
        if self.posLocked:
            result.extend([2, 2, 2])
        else:
            result.extend(
                self.getJointTranslation(self.jointData[0])
            )

        # Append hip rotation
        result.extend(
            BvhJointHandler.quatBvhToDM(
                self.current_hip_rotation.elements
            )
        )

        # Append other rotations
        for joint in self.jointData[1:]:
            result.extend(self.getJointRotation(joint))

        return result

    def generateKeyFrames(self):
        keyFrames = []
        for i in tqdm(range(0, self.mocap.nframes)):
            keyFrames.append(self.generateKeyFrame(i))

        return keyFrames

    def bvhBoneName(self, deepMimicBoneName):
        return self.settings["jointAssignments"][deepMimicBoneName]

    def getJointOffset(self, bvhJointName):
        return list(self.mocap.joint_offset(bvhJointName))

    def getJointTranslation(self, jointInfo: JointInfo):
        return BvhJointHandler.posBvhToDM(
            self.scaleFactor * self.root.getJointPosition(jointInfo.bvhName)
        )

    def getJointRotation(self, jointInfo: JointInfo) -> List[float]:
        joint = self.root.searchJoint(jointInfo.bvhName)
        if jointInfo.dimensions > 1:
            return self.calcRotation(joint, jointInfo)
        else:
            # 1D DeepMimic joint
            assert jointInfo.dimensions == 1
            # Get positions
            childPos = self.normalize(joint.getRelativeChildPosition())
            jointPos = self.normalize(joint.getRelativeJointTranslation())

            angle = math.acos(np.dot(jointPos, childPos))
            return [angle]

    def calcRotation(self, joint: BvhJoint, jointInfo: JointInfo):
        # Get vector from joint to child
        childPos = self.normalize(joint.getRelativeChildPosition())

        child = joint.children[0]
        if jointInfo.deepMimicName not in ["chest", "neck", "left ankle", "right ankle"]:
            # get child's child position
            childsChildPos = child.getRelativeChildPosition()

            y = -1 * childPos
            # TODO: check if vectors coincide
            x = self.normalize(np.cross(y, childsChildPos))
            z = self.normalize(np.cross(x, y))

            # Create rotation matrix from frame
            rot_mat = np.array([x, y, z]).T
            # Take base rotation into account
            zero_rot_mat = self.root.getTotalRotationMatrix()
            result = zero_rot_mat.T @ rot_mat
            return BvhJointHandler.quatBvhToDM(
                Quaternion(matrix=result)
            ).elements
        elif jointInfo.deepMimicName in ["left ankle", "right ankle"]:
            # get child's child position
            childsChildPos = child.getRelativeChildPosition()

            # Feet are pointed in Z direction TODO: find out why minus sign is needed
            z = -childPos
            x = self.normalize(np.cross(childsChildPos, z))
            y = self.normalize(np.cross(z, x))
            # Create rotation matrix from frame
            rot_mat = np.array([x, y, z]).T
            # Take base rotation into account
            zero_rot_mat = self.root.getTotalRotationMatrix()
            result = zero_rot_mat.T @ rot_mat
            return BvhJointHandler.quatBvhToDM(
                Quaternion(matrix=result)
            ).elements
        else:
            # rotate zeroRotVec with rootquat
            zeroRotVec = np.array(jointInfo.zeroRotVector)
            zeroVec = self.current_hip_rotation.rotate(zeroRotVec)

            # Calculate quaternion
            result = BvhJointHandler.calcQuatFromVecs(zeroVec, childPos)
            return BvhJointHandler.quatBvhToDM(result).elements

    def getRelativeJointTranslation(self, bvhJointName):
        joint = self.root.searchJoint(bvhJointName)
        return joint.getRelativeJointTranslation()

    def getRootQuat(self):
        # get left hip position
        root_left = BvhJointHandler.normalize(
            self.getRelativeJointTranslation(
                self.rootLeft
            )
        )

        # get spine "up" position (y axis in local root frame)
        y = BvhJointHandler.normalize(
            self.getRelativeJointTranslation(
                self.rootUp
            )
        )

        # Create orthonormal frame
        z = BvhJointHandler.normalize(np.cross(root_left, y))
        x = BvhJointHandler.normalize(np.cross(y, z))

        # Create rotation matrix from frame
        rot_mat = np.array([x, y, z]).T

        # Create quaternion
        return Quaternion(matrix=rot_mat)

    def calcScale(self):
        # Calculate the scaling factor to transform bvh translations to DM translations
        # From: http://mocap.cs.cmu.edu/faqs.php
        # For 02_01.bvh:
        # self.scaleFactor = (1.0 / 0.45) * 2.54 / 100.0 # = 0.056444
        # For bvh files from http://mocap.cs.sfu.ca/
        self.scaleFactor = 2.54 / 100.0


    @staticmethod
    def calcQuatFromVecs(v1, v2) -> Quaternion:
        v1 = BvhJointHandler.normalize(v1)
        v2 = BvhJointHandler.normalize(v2)

        # Calculate perpendicular vector
        perpvec = np.cross(v1, v2)
        perpnorm = np.linalg.norm(perpvec)
        if perpnorm > 0:
            perpvec = perpvec / perpnorm
            # Check for float slightly larger than 1
            temp = np.dot(v1, v2)
            if temp > 1:
                temp = 1.0
            angle = math.acos(temp)
        else:
            perpvec = np.array([1, 0, 0])
            angle = 0

        # Calculate quaternion from angle axis form.
        result = Quaternion(axis=perpvec, radians=angle)

        return result

    @staticmethod
    def normalize(vector):
        # Normalize a vector (create unit vector)
        norm = np.linalg.norm(vector)
        if norm != 1 and norm > 0:
            vector = vector / norm
        return vector

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
    def posBvhToDM(translation: List[float]) -> List[float]:
        # transform x -> z and z -> -x
        return [
            translation[2],
            translation[1],
            -translation[0]
        ]