import moveit_commander 
from moveit_commander import MoveGroupCommander, PlanningSceneInterface
import sys
import rospy 
from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import PoseStamped, Pose 

rospy.init_node("testing_move_to")

moveit_commander.roscpp_initialize(sys.argv)
scene = PlanningSceneInterface()
arm = MoveGroupCommander("arm_group")

q = quaternion_from_euler(0, 0, 0)
pose = PoseStamped()
pose.header.frame_id = "base_link"
pose.pose.position.x = 1.5
pose.pose.position.y = 0.0
pose.pose.position.z = 1.4
pose.pose.orientation.x = q[0]
pose.pose.orientation.y = q[1]
pose.pose.orientation.z = q[2]
pose.pose.orientation.w = q[3]

arm.set_pose_target(pose, "gripper_link")
plan = arm.plan()

result = arm.go(wait=True)
print result
