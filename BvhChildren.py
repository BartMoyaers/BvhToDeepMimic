from bvh import Bvh

class BvhExtended(Bvh):
    """Class extending the bvh-python class "Bvh", so that a joint's
    children can be looked up.
    """
    def __init__(self, data):
        super().__init__(data)

    def getDirectChildrenNames(self, name):
        joint = super().get_joint(name)
        return [child.name for child in joint.filter('JOINT')]