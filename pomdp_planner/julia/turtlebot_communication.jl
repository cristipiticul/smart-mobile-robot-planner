# =================================
# ROS communication with TurtleBot:
# =================================
using RobotOS
@rosimport geometry_msgs.msg.Twist
rostypegen()
import geometry_msgs.msg: Twist

ROBOT_VELOCITY = 1.0
ROBOT_ANGULAR_VELOCITY = 1.5707

init_node("test_ros_julia")
pub = Publisher{Twist}("/cmd_vel_mux/input/navi", queue_size = 10)

function robotPublishCommand(velocity::Float64, rotation::Float64)
    command = Twist()
    command.linear.x = velocity
    command.angular.z = rotation
    publish(pub, command)
end

function robotStop()
    robotPublishCommand(0.0, 0.0)
end

function robotMoveForward()
    robotPublishCommand(ROBOT_VELOCITY, 0.0)
    rossleep(1.0)
    robotStop()
end

function robotRotateClockwise()
    robotPublishCommand(0.0, -ROBOT_ANGULAR_VELOCITY)
    rossleep(1.0)
    robotStop()
end

function robotRotateCounterClockwise()
    robotPublishCommand(0.0, ROBOT_ANGULAR_VELOCITY)
    rossleep(1.0)
    robotStop()
end

function actionToRobotFunction(action::Symbol)
    mapping = Dict(
        :forward => robotMoveForward,
        :rotate_clockwise => robotRotateClockwise,
        :rotate_counterclockwise => robotRotateCounterClockwise
    )
    return mapping[action]
end
