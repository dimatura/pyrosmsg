
import cStringIO as StringIO
import numpy as np
from pymsg import libpymsg

from sensor_msgs.msg import PointCloud2

#from rospy.numpy_msg import numpy_msg
#PointCloud2Np = numpy_msg(PointCloud2)

import pypcd

import pymsg.libpymsg

pc = pypcd.PointCloud.from_path('./tmp.pcd')
msg = pc.to_msg()

#msg2 = PointCloud2()
#msg2.deserialize(smsg)

def with_caster(m):
    # 38.4 us per loop,
    pymsg.libpymsg.print_centroid(m)

def with_serial(m):
    # 117 us
    buf = StringIO.StringIO()
    m.serialize(buf)
    smsg = buf.getvalue()
    pymsg.libpymsg.print_centroid2(smsg)
