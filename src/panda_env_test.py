import unittest
import panda_env as env

class PandaEnvTest(unittest.TestCase):

    """This unit test currently tests the methods that
    do not cause the robot to execute any real actions.
    """

    def setUp(self):
        self.panda_robot_test = env.FrankaPegInHole()  

    def test_valid_act_pose_based(self):
        action = [0.8, 0.0, 0.0]
        self.panda_robot_test.cur_pos = [0.0, 0.0, 0.0]
        # Make sure this is enabled
        self.panda_robot_test.enable_safe_zone = True

        action_validity = self.panda_robot_test.valid_act_pose_based()
        self.assertTrue(action_validity)

        action = [0.8, 0.0, 0.0]
        action_validity = self.panda_robot_test.valid_act_pose_based()
        self.assertTrue(action_validity)

        action_validity = self.panda_robot_test.valid_act_pose_based()
        self.assertTrue(action_validity)



if __name__ == '__main__':
    unittest.main()