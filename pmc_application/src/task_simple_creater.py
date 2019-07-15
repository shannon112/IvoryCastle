#!/usr/bin/env python

import random
import rospy
import rosbag
import string
import yaml
from geometry_msgs.msg import PoseStamped
from ira_factory_msgs.msg import Task, TaskData
import tf

def saveBag(fileName):
    pose = PoseStamped()
    pose.header.stamp = rospy.Time.now()
    pose.header.frame_id = "map"
    pose.pose.position.x = 0.0
    pose.pose.position.y = 0.0
    pose.pose.position.z = 0.0

    quaternion = tf.transformations.quaternion_from_euler(0, 0, 1.57)
    pose.pose.orientation.x = quaternion[0]
    pose.pose.orientation.y = quaternion[1]
    pose.pose.orientation.z = quaternion[2]
    pose.pose.orientation.w = quaternion[3]

    tk = Task()
    tk.task_id = fileName
    tk.dynamic = False
    tk.amr_num = 0
    tk.mmp_num = 1
    tk.task_type = 6
    tk.task_data = []
    tk.amr_task_pose = []
    tk.amr_leave_pose = []
    tk.mmp_task_pose = [pose.pose]
    tk.mmp_leave_pose = []
    bag = rosbag.Bag(fileName + '.bag', 'w')
    bag.write('task', tk)
    bag.close()


if __name__ == '__main__':
    rospy.init_node('task_creater')
    saveBag("test")
