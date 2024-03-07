#! /usr/bin/env python3
import math
import rospy
import unittest
import rostest
import actionlib

from tortoisebot_waypoints.msg import (
    WaypointActionGoal,
    WaypointActionAction,
)
from nav_msgs.msg import Odometry


class TestWaypointsActionServer(unittest.TestCase):
    def setUp(self):
        # setup node
        rospy.init_node("tortoisebot_as_test")

        # setup ros object
        self.client = actionlib.SimpleActionClient(
            "/tortoisebot_as", WaypointActionAction
        )
        self.subscriber = rospy.Subscriber(
            "/odom", Odometry, self.subscriber_odom_callback
        )

        # member variable
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        
        # error
        self.error_x = 0.0
        self.error_y = 0.0

        # send goal
        self.timeout = rospy.Duration(30)
        self.client.wait_for_server(self.timeout)
        self.send_action_goal(0.25, 0.25)

    def tearDown(self):
        # self.subscriber.unregister()
        pass

    def subscriber_odom_callback(self, msg):
        # reading current position from /odom topic
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

        # reading current orientation from /odom topic
        x = msg.pose.pose.orientation.x
        y = msg.pose.pose.orientation.y
        z = msg.pose.pose.orientation.z
        w = msg.pose.pose.orientation.w

        # convert quaternion into euler angles
        self.yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))

    def send_action_goal(self, x, y):
        # define goal
        self.goal: WaypointActionGoal = WaypointActionGoal()
        self.goal.position.x = x
        self.goal.position.y = y
        
        # send goal to server
        self.client.send_goal(self.goal)
        self.client.wait_for_result(self.timeout)
        
        # collect result
        _ = self.client.get_result()
        
        # calculate error
        self.error_x = self.goal.position.x - self.x
        self.error_y = self.goal.position.y - self.y

    def test_orientation(self):
        # Test Case 1 : validate robot orientation error
        error_yaw = math.atan2(self.error_y, self.error_x) - self.yaw
        self.assertTrue(abs(error_yaw) <= 2.50, "Test Case 1, Failed")

    def test_position(self):
        # Test Case 2 : validate robot position error
        self.assertTrue(abs(self.error_x) <= 0.25 and abs(self.error_y) <= 0.25, "Test Case 2, Failed")


if __name__ == "__main__":
    rostest.rosrun(
        "tortoisebot_waypoints", "tortoisebot_as_test", TestWaypointsActionServer
    )
