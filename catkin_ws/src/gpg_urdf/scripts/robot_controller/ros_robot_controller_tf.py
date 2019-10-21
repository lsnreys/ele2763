#!/usr/bin/env python
# BSD 3-Clause License
#
# Copyright (c) 2019, Matheus Nascimento, Luciana Reys
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice, this
#    list of conditions and the following disclaimer.
#
# 2. Redistributions in binary form must reproduce the above copyright notice,
#    this list of conditions and the following disclaimer in the documentation
#    and/or other materials provided with the distribution.
#
# 3. Neither the name of the copyright holder nor the names of its
#    contributors may be used to endorse or promote products derived from
#    this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

# Launch file created to attend the requirements established on the Ex4 by the discipline of Intelligent Control \
#      of Robotics Systems
# Professor: Wouter Caarls
# Students: Matheus do Nascimento Santos 1920858  (@matheusns)
#           Luciana Reys 1920856 (@lsnreys)

import angles as ros_angles
import tf
import tf2_ros
from geometry_msgs.msg import Twist
from tf2_geometry_msgs import PointStamped
from geometry_msgs.msg import Pose2D
import rospy
from nav_msgs.msg import Odometry

from math import atan2, hypot


class RobotController:
    def __init__(self):
        rospy.init_node('robot_pose_controller_tf')
        self.initMemberVariables()
        self.initROSChannels()
        rospy.spin()

    def initROSChannels(self):
        # The following lines are respective to exercise 2 item 'c'
        self.pose_listener = rospy.Subscriber("mobile_base_controller/odom", Odometry, self.callbackPose)
        self.goal_listener = rospy.Subscriber("goal", Pose2D, self.goalcallback)
        self.stamped_goal_listener = rospy.Subscriber("stamped_goal", PointStamped, self.stampedGoalCallback)
        self.velocity_publisher = rospy.Publisher('mobile_base_controller/cmd_vel', Twist, queue_size=10)

    def initMemberVariables(self):
        self.current_pose = Odometry()

    def callbackPose(self, msg):
        self.current_pose = msg

    def goalcallback(self, msg):
        self.current_goal = msg
        self.goToGoal(self.current_goal)

    def stampedGoalCallback(self, msg):
        try:
            tf_buffer = tf2_ros.Buffer()
            listener = tf2_ros.TransformListener(tf_buffer)
            self.odom_trans = tf_buffer.lookup_transform(('odom', 'base_link', rospy.Time()))

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            print "Transformation error"

    def goToGoal(self, goal_position):
        stamped_goal = PointStamped()
        stamped_goal.header.frame_id = "odom"
        stamped_goal.point.x = goal_position.x
        stamped_goal.point.y = goal_position.y

        # Attend exercise requirements
        tf_buffer = tf2_ros.Buffer()
        listener = tf2_ros.TransformListener(tf_buffer)

        rate = rospy.Rate(10.0)
        while not self.reachTheGoal():
            try:
                trans = tf_buffer.transform(stamped_goal, "base_link")
            except:
                rate.sleep()
                print "Error getting transform"
                continue

            vel_msg = Twist()

            print "vel_msg.linear.x = " + str(self.linearVelocity(trans)) + " vel_msg.angular.z = " + str(self.angularVelocity(trans))

            vel_msg.linear.x = self.linearVelocity(trans)
            vel_msg.angular.z = self.angularVelocity(trans)
            self.velocity_publisher.publish(vel_msg)

            rate.sleep()

    def reachTheGoal(self):
        import math
        distance = math.sqrt(pow((self.current_goal.x - self.current_pose.pose.pose.position.x), 2) + pow(
            (self.current_goal.y - self.current_pose.pose.pose.position.y), 2))
        if distance <= 0.2:
            return True

        print "Distance = " + str(distance)

        return False

    def linearVelocity(self, trans_point, gain=0.5):
        return gain * hypot(trans_point.point.x, trans_point.point.y)

    def angularVelocity(self, trans_point, gain=4):
        return gain * atan2(trans_point.point.x, trans_point.point.y)