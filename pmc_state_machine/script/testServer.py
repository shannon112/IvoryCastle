#!/usr/bin/env python

import rospy
from std_srvs.srv import Trigger, TriggerResponse
from pmc_msgs.srv import detection_PMC_half, detection_PMC_halfResponse
from pmc_msgs.srv import GraspPoseEst_direct, GraspPoseEst_directResponse
from pmc_msgs.srv import PickPlace, PickPlaceResponse
from pmc_msgs.srv import PoseSrv, PoseSrvResponse

from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import Point, Pose

def trigger(req):
  rospy.sleep(2)
  print('[Trigger] Triggered.')
  return TriggerResponse(True, "")

def alignment(req):
  rospy.sleep(2)
  print('[Alignment] Alignment completed.')
  return AlignmentResponse(True)

def objDetectionPMC(req):
  print('[Object Detection]')
  rep = detection_PMC_halfResponse()
  rep.data = [1., 1., 1., 1., 2., 2., 2., 2., 3., 3., 3., 3., 4., 4., 4., 4.]

  return rep

def objDetection(req):
  print('[Object Detection] Receive object name: ' + req.obj_name)
  rep = ObjDetectionWithNameResponse()
  rep.input_pc = PointCloud2()
  pt1 = Point()
  pt1.x = 0; pt1.y = 0; pt1.z = 0

  pt2 = Point()
  pt2.x = 50; pt2.y = 50; pt2.z = 0
  rep.bbox_corner1 = pt1
  rep.bbox_corner2 = pt2

  pc = PointCloud2()
  pc.height = 480; pc.width = 640
  rep.input_pc = pc
  rospy.sleep(1)
  print('[Object Detection] Return pointcloud and bounding boxes:')
  print('    pointcloud = (width, height) = ({}, {})'.format(
    rep.input_pc.width, rep.input_pc.height))
  print('    bbox_corner1 = (x, y, z) = ({}, {}, {})'.format(
    rep.bbox_corner1.x, rep.bbox_corner1.y, rep.bbox_corner1.z))
  print('    bbox_corner2 = (x, y, z) = ({}, {}, {})'.format(
    rep.bbox_corner2.x, rep.bbox_corner2.y, rep.bbox_corner2.z))
  return rep

def graspEst(req):
  print('[Grasping Pose Estimation] Receive poindcloud and bounding boxes:')
  #print('    pointcloud = (width, height) = ({}, {})'.format(
  #  req.input_pc.width, req.input_pc.height))
  print('    bbox_corner1 = (x, y, z) = ({}, {}, {})'.format(
    req.bbox_corner1.x, req.bbox_corner1.y, req.bbox_corner1.z))
  print('    bbox_corner2 = (x, y, z) = ({}, {}, {})'.format(
    req.bbox_corner2.x, req.bbox_corner2.y, req.bbox_corner2.z))
  rep = GraspPoseEst_directResponse()
  ps = Pose()
  ps.position.x = 100.0; ps.position.y = 100.0; ps.position.z = 100.0
  rep.grasp_pose = ps
  rospy.sleep(1)
  print('[Grasping Pose Estimation] Return grasping pose:')
  print('    grasping_pose = (x, y, z) = ({}, {}, {})'.format(
    rep.grasp_pose.position.x, rep.grasp_pose.position.y, rep.grasp_pose.position.z))
  return rep

def stackEst(req):
  print('[Stacking Pose Estimation] Receive poindcloud and bounding boxes:')
  #print('    pointcloud = (width, height) = ({}, {})'.format(
  #  req.input_pc.width, req.input_pc.height))
  print('    bbox_corner1 = (x, y, z) = ({}, {}, {})'.format(
    req.bbox_corner1.x, req.bbox_corner1.y, req.bbox_corner1.z))
  print('    bbox_corner2 = (x, y, z) = ({}, {}, {})'.format(
    req.bbox_corner2.x, req.bbox_corner2.y, req.bbox_corner2.z))
  rep = GraspPoseEst_directResponse()
  ps = Pose()
  ps.position.x = 1.0; ps.position.y = 1.0; ps.position.z = 1.0
  rep.grasp_pose = ps
  rospy.sleep(1)
  print('[Stacking Pose Estimation] Return stacking pose:')
  print('    stacking_pose = (x, y, z) = ({}, {}, {})'.format(
    rep.grasp_pose.position.x, rep.grasp_pose.position.y, rep.grasp_pose.position.z))
  return rep


def attack(req):
  print('[Attack] Receive attack_pose:')
  print('    attack_pose = (x, y, z) = ({}, {}, {}), index = {}'.format(
    req.pose.position.x, req.pose.position.y, req.pose.position.z, req.str_box_ind))
  rospy.sleep(1)
  print('[Attack] Attack completed.')
  return PoseSrvResponse(True)


def pickPlace(req):
  print('[Pick & Place] Receive pick_pose and place_pose:')
  print('    pick_pose = (x, y, z) = ({}, {}, {})'.format(
    req.pick_pose.position.x, req.pick_pose.position.y, req.pick_pose.position.z))
  print('    place_pose = (x, y, z) = ({}, {}, {})'.format(
    req.place_pose.position.x, req.place_pose.position.y, req.place_pose.position.z))
  print('    at station {}'.format(req.str_box_ind))
  rospy.sleep(1)
  print('[Pick & Place] Pick and place completed.')
  print('============ Above Scenario Completed ==========\n')
  return PickPlaceResponse(True)

if __name__ == '__main__':
  rospy.init_node('neuronbot_test_server')
  #alignment_srv = rospy.Service('alignment_trigger', Alignment, alignment)
  #trigger1_srv = rospy.Service('triggerPlacing', Trigger, trigger)
  #trigger2_srv = rospy.Service('triggerFetching', Trigger, trigger)
  #trigger3_srv = rospy.Service('triggerGrasping', Trigger, trigger)
  #trigger4_srv = rospy.Service('triggerStacking', Trigger, trigger)
  #attack_srv = rospy.Service('/attacking_pose', PoseSrv, attack)
  #obj_srv = rospy.Service('object_detection_willie', detection_PMC_half, objDetectionPMC)
  #grasp_est_srv = rospy.Service('grasping_pose_estimation', GraspPoseEst_direct, graspEst)
  #pick_place_srv = rospy.Service('pick_and_place', PickPlace, pickPlace)
  rospy.spin()
