import rospy 
from kinematics.srv import IK, IKRequest 
from geometry_msgs.msg import Pose
from tf.transformations import quaternion_from_euler
import tf
from math import pi

rospy.init_node("test_ik_node")
ik_client = rospy.ServiceProxy("/inverse_kinematics", IK)

req = IKRequest()
pose = Pose()
pose.position.x = 2.078
pose.position.y = 0
pose.position.z = 0.499

roll = 0.0
pitch = 0.0
yaw = 0.0
quaternion = quaternion_from_euler(roll, pitch, yaw)
pose.orientation.x = quaternion[0]
pose.orientation.y = quaternion[1]
pose.orientation.z = quaternion[2]
pose.orientation.w = quaternion[3]

req.pose = pose
result = ik_client.call(req)
print result

