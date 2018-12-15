#!/usr/bin/env python

import rospy

from duckietown_msgs.msg import Pose2D, BoolStamped, FSMState
from std_msgs.msg import


#from feature_extractor import something

class MotionEstimationNode(object):
    def __init__(self):
        self.node_name = "Motion Estimation"

        # Set parameters
        self.active = True


        # Set subscribers
        self.sub_image = rospy.Subscriber("~feature_pairs")
        self.sub_switch = rospy.Subscriber("~switch",BoolStamped, self.cbSwitch, queue_size=1)
        self.sub_fsm_mode = rospy.Subscriber("~fsm_mode", FSMState, self.cbMode, queue_size=1)

        # Set publishers
        self.pub_pose = rospy.Publisher(
            "~pose",Pose2D, queue_size = 1)

    def cbSwitch(self, switch_msg):
        self.active = switch_msg.data

    def estimateMotion(self, feature_pairs):
        # Get actual timestamp
        timestamp_now = rospy.Time.now()

        if not self.active:
            return

        pose = Pose2D()

        #TODO logic goes here

        self.pub_pose(pose)


    def onShutdown(self):
        rospy.loginfo("[MotionEstimationNode] Shutdown.")

    def loginfo(self, s):
        rospy.loginfo('[%s] %s' % (self.node_name, s))


if __name__ == '__main__':
    rospy.init_node('motion_estimation', anonymous=False)
    motion_estimation_node = MotionEstimationNode()
    rospy.on_shutdown(motion_estimation_node.onShutdown)
    rospy.spin()
