import unittest
import json
from bvhtodeepmimic.bvh_extended import BvhExtended
from bvhtodeepmimic.bvh_joint_handler import BvhJointHandler
import numpy as np

class TestBvhJointHandler(unittest.TestCase):

    def __init__(self, *args, **kwargs):
        super(TestBvhJointHandler, self).__init__(*args, **kwargs)

        self.mocap = self.createMocap("./bvhtodeepmimic/tests/0005_Walking001.bvh")
        self.jointHandler = BvhJointHandler(self.mocap, settingsPath="./bvhtodeepmimic/tests/0005_Walking001.json")

    def createMocap(self, bvhPath):
        with open(bvhPath, "r") as myFile:
            mocap = BvhExtended(myFile.read())

        return mocap

    def test_BvhBoneName(self):
        hips = self.jointHandler.bvhBoneName("hip")
        self.assertEqual("Hips", hips)

    def test_normalize(self):
        testvec = np.array([1,2,3])
        normvec = BvhJointHandler.normalize(testvec)
        self.assertAlmostEqual(np.linalg.norm(normvec), 1.0)

    def test_calcQuatFromVecs(self):
        vec1 = np.array([4,5,6])
        vec2 = np.array([6,5,4])
        quat = BvhJointHandler.calcQuatFromVecs(vec1, vec2)
        vec3 = quat.rotate(vec1)
        for i in range(3):
            self.assertAlmostEqual(vec3[i], vec2[i])

if __name__ == '__main__':
    unittest.main()