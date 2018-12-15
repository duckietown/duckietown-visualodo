#!/usr/bin/env python

import rospy

from sensor_msgs.msg import Image, PointCloud2
from duckietown_msgs.msg import BoolStamped, FSMState


#from feature_extractor import something

class FeatureExtractorNode(object):
    def __init__(self):
        self.node_name = "Feature Extractor"

        # Set parameters
        self.active = True


        # Set subscribers
        self.sub_image = rospy.Subscriber("~image/compressed")
        self.sub_switch = rospy.Subscriber("~switch",BoolStamped, self.cbSwitch, queue_size=1)
        self.sub_fsm_mode = rospy.Subscriber("~fsm_mode", FSMState, self.cbMode, queue_size=1)

        # Set publishers
        self.pub_features = rospy.Publisher(
            "~features",PointCloud2, queue_size = 1)

    def cbSwitch(self, switch_msg):
        self.active = switch_msg.data

    def extractFeatures(self, image_msg):
        # Get actual timestamp
        timestamp_now = rospy.Time.now()

        if not self.active:
            return

        features = PointCloud2()

        #TODO logic goes here

        self.pub_features(features)


    def onShutdown(self):
        rospy.loginfo("[FeatureExtractorNode] Shutdown.")

    def loginfo(self, s):
        rospy.loginfo('[%s] %s' % (self.node_name, s))

if __name__ == '__main__':
    rospy.init_node('feature_extractor', anonymous=False)
    feature_extractor_node = FeatureExtractorNode()
    rospy.on_shutdown(feature_extractor_node.onShutdown)
    rospy.spin()
