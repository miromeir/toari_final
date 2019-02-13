#!/usr/bin/env python

# import roslib; roslib.load_manifest('rbx1_nav')
import rospy
import actionlib
from actionlib_msgs.msg import *
from geometry_msgs.msg import Pose, Point, Quaternion, Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from tf.transformations import quaternion_from_euler
from math import radians, pi
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, LaserScan, CameraInfo
import numpy as np
import tf
# from matplotlib import pyplot as plt

class MoveBase():

    def move(self, goal):
        # Send the goal pose to the MoveBaseAction server
        self.move_base.send_goal(goal)


        #Allow 1 minute to get there
        finished_within_time = self.move_base.wait_for_result(rospy.Duration(60))

        # If we don't get there in time, abort the goal
        if not finished_within_time:
            self.move_base.cancel_goal()
            rospy.loginfo("Timed out achieving goal")
        else:
            # We made it!
            state = self.move_base.get_state()
            if state == GoalStatus.SUCCEEDED:
                rospy.loginfo("Goal succeeded!")


    def __init__(self):
        rospy.init_node('nav_test', anonymous=False)
        self.bridge = CvBridge()
        self.OPENNING_ANGLE = np.deg2rad(60)
        self.has_box = False

        #rospy.on_shutdown(self.shutdown)

        # Publisher to manually control the robot (e.g. to stop it)
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist)

        # Subscribe to the move_base action server
        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        rospy.loginfo("Waiting for move_base action server...")

        # Wait 60 seconds for the action server to become available
        self.move_base.wait_for_server(rospy.Duration(60))
        rospy.loginfo("Connected to move base server")

        if not rospy.is_shutdown():
            print("setting goal")
            self.goal = MoveBaseGoal()

            # Use the map frame to define goal poses
            self.goal.target_pose.header.frame_id = 'base_link'

            # Set the time stamp to "now"
            self.goal.target_pose.header.stamp = rospy.Time.now()
            q = tf.transformations.quaternion_from_euler(0,0,0)
            self.goal.target_pose.pose = Pose(Point(1.0, 0, 0), Quaternion(*q))

            # Start the robot moving toward the goal
            self.move(self.goal)

        rospy.spin()

if __name__ == '__main__':
    try:
        MoveBase()
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")
