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
