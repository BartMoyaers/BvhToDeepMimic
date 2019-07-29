from JointInfo import JointInfo
from typing import List
from BvhChildren import BvhExtended
import numpy as np
import math

class BvhJoint:
    # TODO: get channel names from file for easy configuration
    positionChannelNames = ["Xposition", "Yposition", "Zposition"]
    rotationChannelNames = ["Xrotation", "Yrotation", "Zrotation"]

    def __init__(self, mocap: BvhExtended, name: str, parent=None, isRoot=False):
        self.name = name
        self.parent = parent
        self.mocap = mocap
        self.is_root = isRoot
        self.offset = np.array(mocap.joint_offset(self.name))
        self.channels = self.mocap.joint_channels(self.name)
        self.rotation_channels = list(
            filter(lambda x: x in self.rotationChannelNames, self.channels)
        )
        self.children: List[BvhJoint] = self._createChildJoints()
        # Rotation matrix of the joint.
        self.rotation_matrix = np.eye(3)
        self.translation_vector = np.zeros(3)

        # Affine transformation matrix, combining rotation and translation.
        self.tf_matrix = np.eye(4)
        # Transformation matrix reflecting the total transformation from the
        # root joint up to this joint.
        self.total_tf_matrix = np.eye(4)
        # Position of this joint in 3D space.
        self.position: np.array = np.zeros(3)

    def update(self, frameNumber, parent_transform=np.eye(4)):
        self._updateJointTranslation(frameNumber)
        self._updateRotationMatrix(frameNumber)
        self._updateTransformationMatrix(parent_transform)
        self._updatePosition()

        for child in self.children:
            child.update(frameNumber, self.total_tf_matrix)

    def _createChildJoints(self):
        result = []
        for childName in self.mocap.getDirectChildrenNames(self.name):
            result.append(BvhJoint(self.mocap, childName, parent=self))
        return result

    def _updateJointTranslation(self, frameNumber: int):
        # Only update translation when there are more than 3 rotation channels.
        if len(self.channels) <= 3:
            return

        result = []
        for channel in self.positionChannelNames:
            result.append(self.mocap.frame_joint_channel(
                frameNumber, self.name, channel))

        self.translation_vector = np.array(result)

    def _getXYZEulerAngles(self, frameNumber) -> List[float]:
        # Get the X, Y, Z euler angles
        eulerAngles = []
        for channel in self.rotationChannelNames:
            eulerAngles.append(
                self.mocap.frame_joint_channel(
                    frameNumber, self.name, channel)
            )

        return eulerAngles

    def _updateRotationMatrix(self, frameNumber: int):
        angles = self._getXYZEulerAngles(frameNumber)
        # Convert angles to radians
        Xangle = math.radians(angles[0])
        Yangle = math.radians(angles[1])
        Zangle = math.radians(angles[2])

        # Create the rotation matrix for every euler angle
        # See: https://en.wikipedia.org/wiki/Rotation_matrix#In_three_dimensions
        xCos = math.cos(Xangle)
        xSin = math.sin(Xangle)
        Xrot = np.array(
            [
                [1, 0, 0],
                [0, xCos, -xSin],
                [0, xSin, xCos]
            ]
        )

        yCos = math.cos(Yangle)
        ySin = math.sin(Yangle)
        Yrot = np.array(
            [
                [yCos, 0, ySin],
                [0, 1, 0],
                [-ySin, 0, yCos]
            ]
        )
        zCos = math.cos(Zangle)
        zSin = math.sin(Zangle)
        Zrot = np.array(
            [
                [zCos, -zSin, 0],
                [zSin, zCos, 0],
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
        channels = self.rotation_channels
        rotationMatrix = rotationDict[channels[0]] @ (rotationDict[channels[1]] @ rotationDict[channels[2]])

        self.rotation_matrix = rotationMatrix
    
    def _updateTransformationMatrix(self, parent_transform: np.array):
        total_translation = self.offset + self.translation_vector
        result = np.concatenate((self.rotation_matrix, np.array([total_translation]).T), axis=1)
        result = np.concatenate((result, np.array([[0, 0, 0, 1]])), axis=0)
        self.tf_matrix = result
        result = parent_transform @ result
        self.total_tf_matrix = result

    def _updatePosition(self):
        vector = np.array([0,0,0,1]) @ self.total_tf_matrix.T
        self.position = vector[:-1]

    def _search(self, name: str):
        if self.name == name:
            return self
        else:
            for child in self.children:
                found = child._search(name)
                if found is not None:
                    return found
        return

    def searchJoint(self, name:str):
        found = self._search(name)
        if found is None:
            raise LookupError("Joint name not found.")
        else:
            return found

    def getJointPosition(self, name):
        joint = self.searchJoint(name)
        return joint.position

    def getRelativeChildPosition(self):
        if len(self.children) > 0:
            childPos = self.children[0].getRelativeJointTranslation()
        else:
            # get end site position
            childPos = self.getRelativeEndSitePosition()
        return childPos

    def getRelativeJointTranslation(self):
        jointPos = self.position
        parentPos = self.parent.position
        return jointPos - parentPos

    def hasEndSite(self):
        return self.mocap.joint_has_end_site(self.name)

    def getEndSiteOffset(self):
        return self.mocap.joint_get_end_site_offset(self.name)

    def getEndSitePosition(self):
        vector = self.getEndSiteOffset() + [1]
        result = (self.total_tf_matrix @ np.array([vector]).T).T
        return result[0][:-1]

    def getRelativeEndSitePosition(self):
        endSitePosition = self.getEndSitePosition()
        return endSitePosition - self.position