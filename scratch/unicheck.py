
import copy
import numpy as np
import rospy
from std_msgs.msg import Header
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import Image

import cv_bridge

import pymsg

hdr = pyrosmsg.make_header(32, "header")
print hdr

