# =================================
# ROS communication with TurtleBot:
# =================================
using RobotOS
@rosimport geometry_msgs.msg.Twist
@rosimport turtlebot_controller.msg.TurtleBotMovementAction
@rosimport turtlebot_controller.msg.TurtleBotMovementActionGoal
@rosimport turtlebot_controller.msg.TurtleBotMovementActionResult
rostypegen()
import geometry_msgs.msg: Twist
import turtlebot_controller.msg: TurtleBotMovementActionGoal, TurtleBotMovementActionResult
using Dates # for time-out checks

init_node("pomdp_planner")

POLL_RATE = Rate(10.0)
TIMEOUT = Dates.Millisecond(3000)

while get_param("turtlebot_controller_action_server_ready", false) != true
    println("turtlebot_communication: Waiting for turtlebot_controller")
    rossleep(Duration(1.0))
end

mutable struct ActionStatus
    last_status::Bool # True if the last action was successfully completed
    status::Symbol # one of [:idle, :running, :done]
end
struct TurtleBotActionClient
    status::ActionStatus
    goal_publisher::Publisher{TurtleBotMovementActionGoal}
    result_subscriber::Subscriber{TurtleBotMovementActionResult}
end

function TurtleBotActionClient(action_name::String)
    status = ActionStatus(false, :idle)
    TurtleBotActionClient(
        status,
        Publisher{TurtleBotMovementActionGoal}(string(action_name, "/goal"), queue_size=10),
        Subscriber{TurtleBotMovementActionResult}(string(action_name, "/result"),
            function (result)
                status.last_status = result.result.success
                status.status = :done
            end, queue_size=10)
    )
end

action_client = TurtleBotActionClient("turtlebot_controller")
#pub = Publisher{Twist}("/cmd_vel_mux/input/navi", queue_size = 10)

function sendGoal(action_client::TurtleBotActionClient, goal::Symbol)
    if action_client.status.status != :idle
        println("turtlebot_communication.jl: Shouldn't send goal, action not idle")
    end
    action_goal = TurtleBotMovementActionGoal()
    action_goal.goal.command = string(goal)
    publish(action_client.goal_publisher, action_goal)
    #should call rossleep after (see waitForResult)
end

function waitForResult(action_client::TurtleBotActionClient)
    start_time = Dates.now()
    timeout = false
    while action_client.status.status != :done
        if Dates.now() - start_time >= TIMEOUT
            timeout = true
            break
        end
        rossleep(POLL_RATE)
    end
    action_client.status.status = :idle
    if timeout
        return false
    end
    return action_client.status.last_status
end

function robotPublishCommand(velocity::Float64, rotation::Float64)
    command = Twist()
    command.linear.x = velocity
    command.angular.z = rotation
    publish(pub, command)
end

function robotDoAction(action::Symbol)
    global action_client
    sendGoal(action_client, action)
    return waitForResult(action_client)
end
