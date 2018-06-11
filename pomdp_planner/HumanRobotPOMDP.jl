# Human-Robot Interaction: collision avoidance
# Robot state: x:known, y:known, dir:known
# Human state: x:known, y:known, dir:observable, (?)willing_to_hit_robot:observable
# Robot actions: go forward, turn +-45 deg
# Human actions: go forward, stop
# Robot reward: reaching goal (decreases with time), do not hit human

struct RobotState
    x::Int64
    y::Int64
    dir::Int32
end

struct HumanState
    x::Int64
    y::Int64
    dir::Int64
    willing_to_hit_robot::Bool
end

mutable struct HumanRobotPOMDP <: POMDP{}
