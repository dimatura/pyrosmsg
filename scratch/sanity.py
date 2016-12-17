
import numpy as np
import rospy
from std_msgs.msg import Header

import pymsg

ts = pymsg.make_time()
print ts

tsp = rospy.Time()
pymsg.print_time(tsp)

hdr = pymsg.make_header(32)
print hdr

pymsg.print_header_seq(hdr)

print
cloud = pymsg.make_pc2(32)
print cloud

import pypcd

print

pc_data = np.random.random((4, 3)).astype('float32')
pc = pypcd.make_xyz_point_cloud(pc_data)
print 'pc.width:', pc.width
pc_msg = pc.to_msg()

print 'pc_msg.width:', pc_msg.width

pymsg.print_centroid(pc_msg)
print pc_data.mean(0)
