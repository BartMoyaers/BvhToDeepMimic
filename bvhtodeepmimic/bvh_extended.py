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

    def joint_has_end_site(self, joint):
        return any(True for _ in joint.filter('End'))

    def joint_name_has_end_site(self, name):
        joint = super().get_joint(name)
        return self.joint_has_end_site(joint)

    def joint_get_end_site_offset(self, name):
        joint = super().get_joint(name)
        if self.joint_has_end_site(joint):
            end_site = next(joint.filter('End'))
            offset = end_site['OFFSET']
            return [float(offset[0]), float(offset[1]), float(offset[2])]
        raise LookupError('No end site found.')