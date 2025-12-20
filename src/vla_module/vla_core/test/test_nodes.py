import rclpy
from vla_core.nodes.speech_to_text_node import SpeechToTextNode
from vla_core.nodes.robot_controller_node import RobotControllerNode
from vla_core.nodes.llm_planner_node import LLMPlannerNode
from vla_core.nodes.perception_node import PerceptionNode
import unittest

class TestVLACoreNodes(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = None

    def tearDown(self):
        if self.node:
            self.node.destroy_node()

    def test_speech_to_text_node_creation(self):
        self.node = SpeechToTextNode()
        self.assertTrue(self.node is not None)

    def test_robot_controller_node_creation(self):
        self.node = RobotControllerNode()
        self.assertTrue(self.node is not None)
    
    def test_llm_planner_node_creation(self):
        self.node = LLMPlannerNode()
        self.assertTrue(self.node is not None)

    def test_perception_node_creation(self):
        self.node = PerceptionNode()
        self.assertTrue(self.node is not None)

if __name__ == '__main__':
    unittest.main()
