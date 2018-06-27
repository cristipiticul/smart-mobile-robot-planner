using RobotOS
@rosimport geometry_msgs.msg.Twist
rostypegen()

using RobotOS
import geometry_msgs.msg: Twist
init_node("test_ros_julia")
print("Topic created\n")
pub = Publisher{Twist}("/cmd_vel_mux/input/navi", queue_size = 10)
rossleep(1.0)
command = Twist()
command.linear.x = -1.0
command.angular.z = 0.0
print("Publishing\n")
publish(pub, command)
rossleep(5.0)
print("Done\n")
