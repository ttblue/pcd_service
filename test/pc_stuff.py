import numpy as np, numpy.linalg as nlg
import rospy
import cyni
import roslib; roslib.load_manifest('ar_track_service')
from ar_track_service.srv import *

from hd_utils import ros_utils as ru, conversions

np.set_printoptions(precision=5, suppress=True)
rospy.init_node('test_pcd')

xyzrgb1 = cyni.readPCD('/home/sibi/sandbox/pcd_service/clouds/cloud1.pcd')
pc1 = ru.xyzrgb2pc(xyzrgb1[:,:,0:3],xyzrgb1[:,:,3:6],'')
xyzrgb2 = cyni.readPCD('/home/sibi/sandbox/pcd_service/clouds/cloud2.pcd')
pc2 = ru.xyzrgb2pc(xyzrgb2[:,:,0:3],xyzrgb2[:,:,3:6],'')

mserv = rospy.ServiceProxy('getMarkers', MarkerPositions)

req = MarkerPositionsRequest()
req.pc = pc1
res1 = mserv(req)
req.pc = pc2
res2 = mserv(req)

m1 = res1.markers.markers[1]
m2 = res2.markers.markers[1]

tfm1 = conversions.pose_to_hmat(m1.pose.pose)
tfm2 = conversions.pose_to_hmat(m2.pose.pose)

bb = np.array([[4.807, -1.239, -4.11,3.495], 
	       [3.869,0.2728,-3.501,3.155],
	       [5.462,-0.8972,-6.1051,4.372],
	       [0,0,0,1]])

