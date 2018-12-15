#!/usr/bin/env python

import rospy

from sensor_msgs.msg import Image, PointCloud2
from duckietown_msgs.msg import BoolStamped, FSMState

#from feature_tracking import something

class FeatureTrackingNode(object):
    def __init__(self):
        self.node_name = "Feature Tracking"

        # Set parameters
        self.active = True


        # Set subscribers
        self.sub_image = rospy.Subscriber("~features")
        self.sub_switch = rospy.Subscriber("~switch",BoolStamped, self.cbSwitch, queue_size=1)
        self.sub_fsm_mode = rospy.Subscriber("~fsm_mode", FSMState, self.cbMode, queue_size=1)

        # Set publishers
        self.pub_features_pairs = rospy.Publisher(
            "~feature_pairs",Vector2D, queue_size = 1)

    def cbSwitch(self, switch_msg):
        self.active = switch_msg.data

    def trackFeatures(self, features):
        # Get actual timestamp
        timestamp_now = rospy.Time.now()

        if not self.active:
            return

        feature_pairs = Vector2D()

        #TODO logic goes here

        self.pub_features(feature_pairs)




    def onShutdown(self):
        rospy.loginfo("[FeatureTrackingNode] Shutdown.")

    def loginfo(self, s):
        rospy.loginfo('[%s] %s' % (self.node_name, s))


if __name__ == '__main__':
    rospy.init_node('feature_tracking', anonymous=False)
    feature_tracking_node = FeatureTrackingNode()
    rospy.on_shutdown(feature_tracking_node.onShutdown)
    rospy.spin()
