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
from numpy.linalg import norm
from operator import add, sub
import tf
from tf.transformations import quaternion_from_euler
from laser_geometry import LaserProjection
import sensor_msgs.point_cloud2 as pc2
from matplotlib import pyplot as plt

def state_navigate(cloud, has_box, box_x, cmdvel_pub):
    msg = Twist()
    if not has_box:
        msg.linear.x = 0.1
        cmdvel_pub.publish(msg)
        return state_navigate
    else:
        msg.linear.x = 0
        cmdvel_pub.publish(msg)
        return state_center_box

def of_angles(cloud, minang, maxang):
    return cloud[(cloud[:, -1] >= minang) & (cloud[:,-1] <= maxang)]

def is_box_in_center(has_box, box_x):
    return has_box and (260 <= box_x <= 360)

def state_center_box(cloud, has_box, box_x, cmdvel_pub):
    msg = Twist()
    if is_box_in_center(has_box, box_x):
        msg.angular.z = 0
        cmdvel_pub.publish(msg)
        return state_move_to_box
    elif box_x < 260:
        msg.angular.z = 0.1
        cmdvel_pub.publish(msg)
        return state_center_box
    else:
        # rotate left
        msg.angular.z = -0.1
        cmdvel_pub.publish(msg)
        return state_center_box

def reached_box_front(cloud_in_openning):
    print("max is: {}".format(np.max(cloud_in_openning[:,2])))
    return np.max(cloud_in_openning[:,2]) < 0.3

def mark_points(cloud, **kwargs):
    plt.plot(cloud[:,0], cloud[:,1],"*", **kwargs)

def state_move_to_box(cloud, has_box, box_x, cmdvel_pub):
    openning_angle = 10
    in_opening = of_angles(cloud, -openning_angle, openning_angle)
    mark_points(in_opening, color="red")

    msg = Twist()
    if not is_box_in_center(has_box, box_x):
        msg.linear.x = 0
        cmdvel_pub.publish(msg)
        return state_center_box
    elif not reached_box_front(in_opening):
        msg.linear.x = 0.05
        cmdvel_pub.publish(msg)
        return state_move_to_box
    else:
        msg.linear.x = 0
        cmdvel_pub.publish(msg)
        return state_box_to_right

def is_box_on_right(cloud):
    if len(cloud) == 0:
        return False
    return np.max(cloud[:,2]) < 0.3

def state_box_to_right(cloud, cmdvel_pub, **kwargs):
    cloud_right = of_angles(cloud, -110, -85)
    mark_points(cloud_right, color="red")
    msg = Twist()
    if not is_box_on_right(cloud_right):
        msg.angular.z = 0.1
        cmdvel_pub.publish(msg)
        return state_box_to_right
    else:
        msg.angular.z = 0
        cmdvel_pub.publish(msg)
        return state_5

def state_5(**kwargs):
    return state_5

def dist_for_angle(ranges, angle):
    result = ranges[(angle  + 360) % 360]
    if result == 0:
        return 999
    return result

def calc_vec(angle, dist):
   angle = np.deg2rad(angle)
   return dist * np.array([np.sin(angle), np.cos(angle)])

def box_vector(ranges, theta_center):
    p_center = calv_vec(theta_center, dist_for_angle(angle))

def calc_goal(dist, angle):
    goal = MoveBaseGoal()

    goal.target_pose.header.frame_id = 'base_link'
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose = Pose(Point(dist, 0, 0),
                                 Quaternion(*quaternion_from_euler(0,0,  np.deg2rad(angle))))
    return goal

class MoveBase():

    def img_callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            hsv_img = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)[200:, :]
            color = (255,0,0)
            color_hsv = np.array(color,dtype="uint8")
            color_hsv = np.array([[color_hsv]],dtype="uint8")
            color_hsv = cv2.cvtColor(color_hsv,cv2.COLOR_BGR2HSV)
            color_hsv = color_hsv[0][0]

            lower = (max(color_hsv[0] - 35, 0),
                        20,
                        20)
            upper = (color_hsv[0]+35 if color_hsv[0]+35 < 180 else 255,
                        255,
                        255)

            lower = np.array(lower, dtype = "uint8")
            upper = np.array(upper, dtype = "uint8")
            mask = cv2.inRange(hsv_img, lower, upper)
            im2, contours, hierarchy = cv2.findContours(mask,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
            sorted_by_perimiter = sorted(contours,key = lambda x: cv2.arcLength(x,False))

            self.has_box = False
            self.box_x = 0
            if(len(sorted_by_perimiter)>0):
                selected_contour = sorted_by_perimiter[-1] #largest contour
                (x,y),radius = cv2.minEnclosingCircle(selected_contour)
		x, y, w, h = cv2.boundingRect(selected_contour)
                cv2.rectangle(hsv_img,(x,y),(x+w, y+h),(0,255,0),2)
                self.angle = int(np.rad2deg(self.find_angle(x + (w/2), hsv_img.shape[1], self.OPENNING_ANGLE)))

                self.right_angle = int(np.rad2deg(self.find_angle(x, hsv_img.shape[1], self.OPENNING_ANGLE)))
                self.left_angle = int(np.rad2deg(self.find_angle(x + w, hsv_img.shape[1], self.OPENNING_ANGLE)))
                if w > 70 and (len(mask.nonzero()[1]) > 5000):
                    self.has_box = True
                    self.box_x = (x + w/2)

            cv2.imshow('title', cv2.cvtColor(hsv_img, cv2.COLOR_HSV2BGR))
            cv2.waitKey(100)

        except CvBridgeError as e:
            print(e)

    def print_dists(self,
                    left_out_box_average,
                    left_box_average,
                    right_box_average,
                    right_out_box_average):
        print("--------------------------------------")
	print(left_out_box_average - left_box_average,
              right_out_box_average - right_box_average)

        print(left_out_box_average,
              left_box_average,
              right_box_average,
              right_out_box_average)


    def scan_callback(self, data):
        if self.moving:
            return
        cloud = self.proj.projectLaser(data)
        cloud = np.array(list(pc2.read_points(cloud, skip_nans=True, field_names=("x", "y", "z"))))
        cloud_norms = norm(cloud[:,:2], axis=1)
        cloud_angles = np.rad2deg(np.arctan2(cloud[:, 1], cloud[:, 0]))
        cloud = np.c_[cloud[:, :2], cloud_norms, cloud_angles]
        plt.cla()
        plt.plot(cloud[:,0], cloud[:, 1], '*')
        plt.ylim(-5, 5)
        plt.xlim(-5, 5)

        self.state = self.state(cloud=cloud,
                                has_box=self.has_box,
                                box_x=self.box_x,
                                cmdvel_pub=self.cmd_vel_pub)


        plt.title("state: " + self.state.__name__)
        plt.draw()
        plt.pause(0.000000001)


        

    def __init__(self):
        plt.ion()
        plt.show()
        self.proj = LaserProjection()
        rospy.init_node('nav_test', anonymous=False)
        self.bridge = CvBridge()
        self.OPENNING_ANGLE = np.deg2rad(60)
        self.has_box = False
        self.box_x = 0
        self.moving = False
        rospy.Subscriber("/usb_cam/image_raw", Image, self.img_callback)
        rospy.Subscriber('scan',LaserScan,self.scan_callback)
	

        rospy.on_shutdown(self.shutdown)

        self.state = state_navigate
        # Publisher to manually control the robot (e.g. to stop it)
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist)

        # Subscribe to the move_base action server
        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)

        rospy.loginfo("Waiting for move_base action server...")

        # Wait 60 seconds for the action server to become available
        self.move_base.wait_for_server(rospy.Duration(60))

        rospy.loginfo("Connected to move base server")
        rospy.loginfo("Starting navigation test")

        if not rospy.is_shutdown():
            rospy.spin()

    def done_cb(self, terminal_state, result):
        print("done_cb")
        print(terminal_state, result)
    def active_cb(self):
        print("active_cb")
    def feedback_cb(self, feedback):
        print("feedback_cb")
        # print(feedback)

    def move(self, goal):
        # Send the goal pose to the MoveBaseAction server
        self.move_base.send_goal(goal , self.done_cb, self.active_cb, self.feedback_cb)

    def move2(self, goal):
        # Send the goal pose to the MoveBaseAction server
        self.move_base.send_goal(goal)
        finished = self.move_base.wait_for_result(rospy.Duration(120))

        if not finished:
             print("not finished within time")
        else:
            state = self.move_base.get_state()
            if state == GoalStatus.SUCCEEDED:
                print("goal succeeded")

    def shutdown(self):
        rospy.loginfo("Stopping the robot...")
        # Cancel any active goals
        self.move_base.cancel_goal()
        rospy.sleep(2)
        # Stop the robot
        self.cmd_vel_pub.publish(Twist())
        rospy.sleep(1)


    def identify_box(self, box_img):
      hsv = cv2.cvtColor(box_img, cv2.COLOR_RGB2HSV)
      hue = hsv[:, :, 0]
      sat = hsv[:, :, 1]
      return ((185 <= hue) & (hue <= 200) &
              ((100 <= sat) & (sat <= 120) |
               (65 <= sat) & (sat <= 75))).nonzero()

    def mark_box(self, ax, identified_box):
      ys, xs = identified_box
      ax.scatter(xs, ys, color='pink')

    def box_xcenter(self, identified_box):
      _, xs = identified_box
      return np.average(xs)

    def find_angle(self, box_x, total_width, openning_angle):
      fix_constant = np.deg2rad(-5)
      half_width = 0.5 * total_width
      img_plane_z = half_width / np.tan(0.5 * openning_angle)
      x_from_center = box_x - half_width
      return -np.arctan2(x_from_center, img_plane_z) + fix_constant

if __name__ == '__main__':
    try:
        MoveBase()
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")
