#!/usr/bin/env python

import os
import random
import rospy
import rospkg
import rosbag
import string
import sys
import yaml
from ira_factory_msgs.msg import Task, TaskList

projectName = 'test'

if __name__ == '__main__':
  rospy.init_node('project_broadcaster')

  #projectName = rospy.get_param('~project_name', 'test')
  projectName = sys.argv[1]
  taskListPub = rospy.Publisher('/task_list', TaskList)
  rospack = rospkg.RosPack()
  taskDir = os.path.join(rospack.get_path('pmc_application'), 'projects', projectName)

  # Read task order
  orderFile = os.path.join(taskDir, 'task_list.yaml')
  bagList = None
  with open(orderFile, 'r') as stream:
    bagList = yaml.safe_load(stream)['list']

  # Read tasks
  taskList = TaskList()
  taskList.task_list_id = 'task_list_' + ''.join(random.choice(string.ascii_uppercase + string.digits) for _ in range(5))
  for taskName in bagList:
    taskFile = os.path.join(taskDir, taskName + '.bag')
    bag = rosbag.Bag(taskFile)
    for topic, msg, t in bag.read_messages(topics=['task']):
      tk = Task()
      tk = msg
      tk.task_list_id = taskList.task_list_id
      taskList.task_list.append(tk)
    bag.close()


  amr_num_list = []
  mmp_num_list = []
  for task in taskList.task_list:
    amr_num_list.append(task.amr_num)
    mmp_num_list.append(task.mmp_num)
  taskList.amr_num = max(amr_num_list)
  taskList.mmp_num = max(mmp_num_list)

  r = rospy.Rate(10)
  #for _ in range(10):
  while not rospy.is_shutdown():
    taskListPub.publish(taskList)
    r.sleep()
