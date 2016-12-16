
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

