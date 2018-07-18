using POMDPs
importall POMDPs
using Distributions
using POMDPToolbox
#include("turtlebot_communication.jl")
#resetGazeboWorld()

# =================================
# (PO)MDP structures
# =================================
struct GridCell
    x::Int64
    y::Int64
end

# abstract type AgentState end

struct RobotState # <: AgentState
    position::GridCell
    orientation::Symbol # one of [:north, :south, :east, :west]
end

struct ObstacleState # <: AgentState
    position::GridCell
    orientation::Symbol # one of [:north, :south]
end

struct GridWorldState
    robot::RobotState
    obstacles::Vector{ObstacleState}
end

type GridWorld <: MDP{GridWorldState, Symbol} # Note that our MDP is parametarized by the state and the action
    size_x::Int64 # x size of the grid
    size_y::Int64 # y size of the grid
    goal::GridCell
    obstacle_1_x::Int64
    obstacle_2_x::Int64
    goal_reward::Float64      # positive, received when robot reaches goal
    collision_reward::Float64 # negative
    time_reward::Float64      # negative, to encourage fastest route
    discount_factor::Float64  # disocunt factor
end

# =================================
# Useful functions
# =================================

function inbounds(mdp::GridWorld, pos::GridCell)
    return 1 <= pos.x <= mdp.size_x && 1 <= pos.y <= mdp.size_y
end

function rotateClockwise(orientation::Symbol)
    if orientation == :north
        return :east
    elseif orientation == :east
        return :south
    elseif orientation == :south
        return :west
    elseif orientation == :west
        return :north
    else
        println("Error! Invalid orientation!")
    end
end

function rotateCounterClockwise(orientation::Symbol)
    if orientation == :north
        return :west
    elseif orientation == :west
        return :south
    elseif orientation == :south
        return :east
    elseif orientation == :east
        return :north
    else
        println("Error! Unknown orientation: $orientation")
    end
end

function forwardPosition(pos::GridCell, orientation::Symbol)
    if orientation == :north
        return GridCell(pos.x, pos.y + 1)
    elseif orientation == :east
        return GridCell(pos.x + 1, pos.y)
    elseif orientation == :south
        return GridCell(pos.x, pos.y - 1)
    elseif orientation == :west
        return GridCell(pos.x - 1, pos.y)
    else
        println("Error! Unknown orientation: $orientation")
    end
end

posequal(s1::GridCell, s2::GridCell) = s1.x == s2.x && s1.y == s2.y

function stateInCollision(state::GridWorldState)
    for obstacle = state.obstacles
        if posequal(state.robot.position, obstacle.position)
            return true
        end
    end
    return false
end

function goalIsReached(mdp::GridWorld, state::GridWorldState)
    return posequal(state.robot.position, mdp.goal)
end

# =================================
# (PO)MDP description
# =================================

function POMDPs.states(mdp::GridWorld)
    s = GridWorldState[]
    for robot_y = 1:mdp.size_y, robot_x = 1:mdp.size_x, robot_orientation = [:north, :south, :east, :west],
        obstacle_1_y = 1:mdp.size_y, obstacle_1_orientation = [:east, :west],
        obstacle_2_y = 1:mdp.size_y, obstacle_2_orientation = [:east, :west]

        robot_state = RobotState(GridCell(robot_x, robot_y), robot_orientation)
        obstacle_1_state = ObstacleState(GridCell(mdp.obstacle_1_x, obstacle_1_y), obstacle_1_orientation)
        obstacle_2_state = ObstacleState(GridCell(mdp.obstacle_2_x, obstacle_2_y), obstacle_2_orientation)

        push!(s, GridWorldState(robot_state, [obstacle_1_state, obstacle_2_state]))
    end
    return s
end

POMDPs.actions(mdp::GridWorld) = [:forward, :rotate_clockwise, :rotate_counterclockwise, :wait]

function POMDPs.transition(mdp::GridWorld, state::GridWorldState, action::Symbol)
    next_obstacles_states = Vector{ObstacleState}()
    for obstacle = state.obstacles
        # TODO: the obstacle turns at the end of the way
        new_position = forwardPosition(obstacle.position, obstacle.orientation)
        new_orientation = obstacle.orientation
        push!(next_obstacles_states, ObstacleState(new_position, new_orientation))
    end

    next_robot_position::GridCell = state.robot.position
    next_robot_orientation::Symbol = state.robot.orientation
    if action == :rotate_clockwise
        next_robot_orientation = rotateClockwise(state.robot.orientation)
    elseif action == :rotate_counterclockwise
        next_robot_orientation = rotateCounterClockwise(state.robot.orientation)
    elseif action == :forward
        forward_robot_position = forwardPosition(state.robot.position, state.robot.orientation)
        if inbounds(mdp, forward_robot_position)
            next_robot_position = forward_robot_position
        end
    elseif action == :wait
        # do nothing, they are already initialized
    else
        println("Error! Unrecognized action: $action")
    end
    next_robot_state = RobotState(next_robot_position, next_robot_orientation)
    return SparseCat([GridWorldState(next_robot_state, next_obstacles_states)], [1.0])
end

function POMDPs.reward(mdp::GridWorld, state::GridWorldState, action::Symbol, statep::GridWorldState)
    if stateInCollision(state)
        return mdp.collision_reward
    elseif goalIsReached(mdp, state)
        return mdp.goal_reward
    else
        return mdp.time_reward
    end
end


# orientation: 4 possibilities
# done: 0 or 1
# x between 1, mdp.size_x
# y between 1, mdp.size_y
POMDPs.n_states(mdp::GridWorld) = mdp.size_x * mdp.size_y * 4 *
                                  mdp.size_y * 2 *
                                  mdp.size_y * 2
# forward, rotate_clockwise, rotate_counterclockwise, wait
POMDPs.n_actions(mdp::GridWorld) = 4
POMDPs.discount(mdp::GridWorld) = mdp.discount_factor;

function POMDPs.state_index(mdp::GridWorld, state::GridWorldState)
    orientations_all = Dict(:north=>1, :south=>2, :east=>3, :west=>4)
    orientations_two = Dict(:north=>1, :south=>2)
    robot_orientation = orientations_all[state.robot.orientation]
    obstacle_1_orientation = orientations_two[state.obstacle_1.orientation]
    obstacle_2_orientation = orientations_two[state.obstacle_2.orientation]
    return sub2ind(
        (mdp.size_x, mdp.size_y, 4,
        mdp.size_y, 2,
        mdp.size_y, 2),
        (state.robot.position.x, state.robot.position.y, robot_orientation,
        state.obstacle_1.position.y, obstacle_1_orientation,
        state.obstacle_2.position.y, obstacle_2_orientation)
    )
end

function POMDPs.action_index(mdp::GridWorld, act::Symbol)
    if act==:forward
        return 1
    elseif act==:rotate_clockwise
        return 2
    elseif act==:rotate_counterclockwise
        return 3
    elseif act==:wait
        return 4
    end
    error("Invalid GridWorld action: $act")
end

POMDPs.isterminal(mdp::GridWorld, s::GridWorldState) = stateInCollision(s) || goalIsReached(mdp, s)

# =================================
# (PO)MDP solving
# =================================
using MCTS

mdp = GridWorld(
    10,              #size_x::Int64
    10,              #size_y::Int64
    GridCell(9, 5),  #goal::GridCell
    4,               #obstacle_1_x::Int64
    6,               #obstacle_2_x::Int64
    100,             #goal_reward::Float64
    -400,            #collision_reward::Float64
    -5,              #time_reward::Float64
    0.9             #discount_factor::Float64
)

# initialize the solver with hyper parameters
# n_iterations: the number of iterations that each search runs for
# depth: the depth of the tree (how far away from the current state the algorithm explores)
# exploration constant: this is how much weight to put into exploratory actions.
# A good rule of thumb is to set the exploration constant to what you expect the upper bound on your average expected reward to be.
solver = MCTSSolver(n_iterations=1000,
                    depth=30,
                    exploration_constant=10.0,
                    enable_tree_vis=true)

policy = solve(solver, mdp)

# =================================
# Test the policy
# =================================
start_state = GridWorldState(
    RobotState(GridCell(3, 6), :south),
    [
        ObstacleState(GridCell(mdp.obstacle_1_x, 1), :north),
        ObstacleState(GridCell(mdp.obstacle_2_x, 6), :south)
    ]
)

sim(mdp, start_state, max_steps=20) do s
    println("State: $s")
    a = action(policy, s)
    println("Action: $a")
    #if !robotDoAction(a)
    #    println("Robot failed to do action: $a")
    #    return :wait
    #end
    return a
end
