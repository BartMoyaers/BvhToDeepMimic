from typing import List

class JointInfo:
    """ Contains information on a certain joint in the DeepMimic human model.
    """

    def __init__(self, deepMimicName: str, bvhName: str, 
                 dimensions: int, zeroRotVector: List[float]):
        self.deepMimicName = deepMimicName
        self.bvhName = bvhName
        # self.bvhChildName = bvhChildName
        self.dimensions = dimensions
        self.zeroRotVector = zeroRotVector
