from std_msgs.msg import Float64
from motion_planning.srv import RRTPath, RRTPathRequest, RRTPathResponse
from geometry_msgs.msg import Pose
import rospy 
from tf.transformations import quaternion_from_euler
from math import pi, sqrt
import time
import numpy as np

def interpolate(s0, st, sd0, sdt, sdd0, sddt, T, t): 
  mat = np.matrix([[0, 0, 0, 0, 0, 1], 
            [T**5, T**4, T**3, T**2, T, 1], 
            [0, 0, 0, 0, 1, 0], 
            [5*T**4, 4 * T**3, 3 * T**2, 2 * T, 1, 0], 
            [0, 0, 0, 2, 0, 0], 
            [20 * T**3, 12 * T**2, 6 * T, 2, 0, 0]])
  vec = np.matrix([s0, st, sd0, sdt, sdd0, sddt])
  coeff = np.dot(np.linalg.inv(mat), vec.T)
  A = coeff[0, 0]
  B = coeff[1, 0]
  C = coeff[2, 0]
  D = coeff[3, 0]
  E = coeff[4, 0]
  F = coeff[5, 0]

  pos = A * t**5 + B * t**4 + C * t**3 + D * t**2 + E * t + F 
  vel = 5 * A * t**4 + 4 * B * t**3 + 3 * C * t**2 + 2 * D * t**1 + E 
  acc = 20 * A * t **3 + 12 * B * t**2 + 6 * C * t + 2 * D 
  return pos

rospy.init_node("rrt_test")

controllers = []

for i in range(6):
  controllers.append(rospy.Publisher("/kuka_arm/joint{0}_controller/command".format(i+1), Float64, queue_size=1))

rrt_client = rospy.ServiceProxy("plan_rrt", RRTPath)
print("Wait for plan_rrt service")
rrt_client.wait_for_service()
print("Connected to plan_rrt")


q = quaternion_from_euler(0, 0, pi)
goal_pose = Pose()
goal_pose.position.x = -2.153
goal_pose.position.y = 0
goal_pose.position.z = 0.5
goal_pose.orientation.x = q[0]
goal_pose.orientation.y = q[1]
goal_pose.orientation.z = q[2]
goal_pose.orientation.w = q[3]

req = RRTPathRequest()
req.goal_pose = goal_pose 

begin_time =  time.time()
try:
  result = rrt_client.call(req)
except Exception as e:
  print e
  print("No path found")
  exit()

print "Time:", time.time() - begin_time

trajectory = result.trajectory.points
print(len(trajectory), " points")

for i in range(len(trajectory) - 1):
  p0 = trajectory[i]
  p1 = trajectory[i + 1]

  print p0.positions
  print p0.velocities

  tf = p1.time_from_start.to_sec() - p0.time_from_start.to_sec()
  time_step = 0.01 
  
  for t in np.arange(0, tf, time_step):
    begin_time = rospy.Time.now()
    q = [0] * 6
    qd = [0] * 6
    for j in range(6):
      q0 = p0.positions[j]
      q1 = p1.positions[j]

      qd0 = p0.velocities[j]
      qd[j] = qd0
      qd1 = p1.velocities[j]

      qdd0 = p0.accelerations[j]
      qdd1 = p1.accelerations[j]

      q[j] = interpolate(q0, q1, qd0, qd1, qdd0, qdd1, tf, t)

    for j in range(6): 
      controllers[j].publish(Float64(q[j]))
    
    total_time = (rospy.Time.now() - begin_time).to_sec()
    wait_time = time_step - total_time

    if wait_time > 0:
      rospy.sleep(wait_time)
