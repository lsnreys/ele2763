#!/usr/bin/python

from __future__ import print_function

import rospy, sys
from geometry_msgs.msg import PoseStamped
from gazebo_msgs.msg import ModelStates

model_ = ""
model_pose_ = PoseStamped()
model_pose_.header.frame_id = "odom"

def stateCallback(msg):
    for idx, name in enumerate(msg.name):
        if name == model_:
            model_pose_.pose = msg.pose[idx]

def modelStateToPose():
    rospy.init_node(model_ + "_pose_publisher")

    state_subscriber = rospy.Subscriber('/gazebo/model_states', ModelStates, stateCallback)
    pose_publisher = rospy.Publisher(model_ + "_pose", PoseStamped, queue_size=10)
    
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        model_pose_.header.stamp = rospy.Time.now()
        pose_publisher.publish(model_pose_)
        rate.sleep()

if __name__ == '__main__':
    if len(sys.argv) < 2:
        print("Usage: ", sys.argv[0], " <model> [frame]")
        sys.exit(1)
        
    model_ = sys.argv[1]
    if len(sys.argv) > 2:
        model_pose_.header.frame_id = sys.argv[2]

    try:
        modelStateToPose()
    except rospy.ROSInterruptException:
        pass

