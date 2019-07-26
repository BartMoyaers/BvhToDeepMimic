import numpy as np
import math
from typing import List
from pyquaternion import Quaternion

class JointInfo:
    """ Contains information on a certain joint in the DeepMimic human model as 
    well as some functions in order to convert joint data from BVH to DeepMimic.
    """

    def __init__(self, deepMimicName: str, bvhName: str, 
                 dimensions: int, zeroRotVector: List[float]):
        self.deepMimicName = deepMimicName
        self.bvhName = bvhName
        # self.bvhChildName = bvhChildName
        self.dimensions = dimensions
        self.zeroRotVector = zeroRotVector

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
            perpvec = np.array([1, 0, 0])
            angle = 0

        # Calculate quaternion from angle axis form.
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
    def posBvhToDM(translation: List[float]) -> List[float]:
        # transform x -> z and z -> -x
        return [
            translation[2],
            translation[1],
            -translation[0]
        ]
