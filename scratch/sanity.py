
import copy
import numpy as np
import rospy
from std_msgs.msg import Header
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import Image

import cv_bridge

import pymsg
import pypcd

#ts = pymsg.make_time()
#print ts
#
#tsp = rospy.Time()
#pymsg.print_time(tsp)
#
#hdr = pymsg.make_header(32)
#print hdr
#
#pymsg.print_header_seq(hdr)
#
#print
#cloud = pymsg.make_pc2(32)
#print cloud
#
#print

pc_data = np.random.random((4, 3)).astype('float32')
pc = pypcd.make_xyz_point_cloud(pc_data)
print 'pc.width:', pc.width
pc_msg = pc.to_msg()
print type(pc_msg)

print 'pc_msg.width:', pc_msg.width

pymsg.print_centroid(pc_msg)
print pc_data.mean(0)

#
#print 'ci'
#
#ci = CameraInfo()
#print ci
#pymsg.print_cam_info(ci)
#
#print 'img'
#print

#
#bridge = cv_bridge.CvBridge()
#cv_img = (np.random.random((8, 8))*255).astype('u1')
#img_msg = bridge.cv2_to_imgmsg(cv_img, encoding='passthrough')
#print img_msg
#print '---'
#pymsg.print_img(img_msg)
#
#print '***'
#img_msg2 = pymsg.make_img(8, 8)
#print img_msg2
#cv_img2 = bridge.imgmsg_to_cv2(img_msg2, desired_encoding='passthrough')
