import unittest
import json
from bvhtodeepmimic.bvh_extended import BvhExtended
from bvhtodeepmimic.bvh_joint import BvhJoint

class TestBvhJoint(unittest.TestCase):

    def __init__(self, *args, **kwargs):
        super(TestBvhJoint, self).__init__(*args, **kwargs)

        self.mocap = self.createMocap("./bvhtodeepmimic/tests/0005_Walking001.bvh")
        self.settings = self.readSettings("./bvhtodeepmimic/tests/0005_Walking001.json")
        self.root = self.createRoot()

    def createMocap(self, bvhPath):
        with open(bvhPath, "r") as myFile:
            mocap = BvhExtended(myFile.read())

        return mocap

    def readSettings(self, settingsPath):
        with open(settingsPath) as json_data:
            settings = json.load(json_data)
        return settings

    def createRoot(self):
        root = BvhJoint(self.mocap,
                        "Hips",
                        self.settings["positionChannelNames"],
                        self.settings["rotationChannelNames"]
        )
        return root

    def test_updatePosition(self):
        self.root.update(0)
        child = self.root.children[0] #leftupleg
        self.assertEqual(child.position[0], 4.803989001169213)
        self.assertEqual(child.position[1], 32.631972506422834)
        self.assertEqual(child.position[2], -20.776430497304112)

    def test_updateJointTranslation(self):
        self.root.update(0)
        # 5.1427 32.5308 -24.4088
        self.assertEqual(self.root.translation_vector[0], 5.1427)
        self.assertEqual(self.root.translation_vector[1], 32.5308)
        self.assertEqual(self.root.translation_vector[2], -24.4088)

    def test_updateRotationMatrix(self):
        self.root.update(0)
        child = self.root.children[0] #leftupleg
        self.assertEqual(round(child.rotation_matrix[0,0], 7), 0.9996688)

    def test_updateTransformationMatrix(self):
        self.root.update(0)
        child = self.root.children[0] #leftupleg
        self.assertEqual(round(child.tf_matrix[0,0], 9), 9.99668797e-01)

    def test_search(self):
        test = self.root._search("LeftUpLeg")
        self.assertIsNotNone(test)
        test = self.root._search("NonExistantName")
        self.assertEqual(test, None)

    def test_searchJoint(self):
        test = self.root.searchJoint("LeftUpLeg")
        self.assertIsNotNone(test)
        test = self.root._search("NonExistantName")
        self.assertRaises(LookupError)

    def test_hasEndSite(self):
        test = self.root.searchJoint("LeftToeBase")
        self.assertTrue(test.hasEndSite())

if __name__ == '__main__':
    unittest.main()
