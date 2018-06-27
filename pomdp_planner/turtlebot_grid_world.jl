using POMDPs
importall POMDPs
using Distributions
using POMDPToolbox
include("turtlebot_communication.jl")

# =================================
# (PO)MDP structures
# =================================
struct GridCell
    x::Int64
    y::Int64
end

struct GridWorldState
    position::GridCell
    orientation::Symbol # one of [:north, :south, :east, :west]
    done::Bool # are we in a terminal state?
end

type GridWorld <: MDP{GridWorldState, Symbol} # Note that our MDP is parametarized by the state and the action
    size_x::Int64 # x size of the grid
    size_y::Int64 # y size of the grid
    reward_states::Vector{GridCell} # the states in which agent recieves reward
    reward_values::Vector{Float64} # reward values for those states
    discount_factor::Float64 # disocunt factor
end

# =================================
# Useful functions
# =================================

GridWorldState(x::Int64, y::Int64, orientation::Symbol) = GridWorldState(GridCell(x, y), orientation, false)

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

# =================================
# (PO)MDP description
# =================================

# we use key worded arguments so we can change any of the values we pass in
function GridWorld(;sx::Int64=10, # size_x
                    sy::Int64=10, # size_y
                    rs::Vector{GridCell}=[GridCell(4,3), GridCell(4,6), GridCell(9,3), GridCell(8,8)], # reward states
                    rv::Vector{Float64}=rv = [-10.,-5,10,3], # reward values
                    discount_factor::Float64=0.9)
    return GridWorld(sx, sy, rs, rv, discount_factor)
end

function POMDPs.states(mdp::GridWorld)
    s = GridWorldState[]
    for d = 0:1, y = 1:mdp.size_y, x = 1:mdp.size_x, orientation = [:north, :south, :east, :west]
        push!(s, GridWorldState(GridCell(x,y),orientation,d))
    end
    return s
end

POMDPs.actions(mdp::GridWorld) = [:forward, :rotate_clockwise, :rotate_counterclockwise]

function POMDPs.transition(mdp::GridWorld, state::GridWorldState, action::Symbol)
    if state.done
        return SparseCat([GridWorldState(state.position, state.orientation, true)], [1.0])
    elseif state.position in mdp.reward_states
        return SparseCat([GridWorldState(state.position, state.orientation, true)], [1.0])
    end

    next_position::GridCell = state.position
    next_orientation::Symbol = state.orientation
    if action == :rotate_clockwise
        next_orientation = rotateClockwise(state.orientation)
    elseif action == :rotate_counterclockwise
        next_orientation = rotateCounterClockwise(state.orientation)
    elseif action == :forward
        new_position = forwardPosition(state.position, state.orientation)
        if inbounds(mdp, new_position)
            next_position = new_position
        end
    else
        println("Error! Unrecognized action: $action")
    end
    return SparseCat([GridWorldState(next_position, next_orientation, false)], [1.0])
end

function POMDPs.reward(mdp::GridWorld, state::GridWorldState, action::Symbol, statep::GridWorldState) #deleted action
    if state.done
        return 0.0
    end
    r = 0.0
    n = length(mdp.reward_states)
    for i = 1:n
        if posequal(state.position, mdp.reward_states[i])
            r += mdp.reward_values[i]
        end
    end
    return r
end


# orientation: 4 possibilities
# done: 0 or 1
# x between 1, mdp.size_x
# y between 1, mdp.size_y
POMDPs.n_states(mdp::GridWorld) = 4*2*mdp.size_x*mdp.size_y
# forward, rotate_clockwise, rotate_counterclockwise
POMDPs.n_actions(mdp::GridWorld) = 3
POMDPs.discount(mdp::GridWorld) = mdp.discount_factor;

function POMDPs.state_index(mdp::GridWorld, state::GridWorldState)
    sd = Int(state.done + 1)
    orientations = Dict(:north=>1, :south=>2, :east=>3, :west=>4)
    orientation = orientations[state.orientation]
    return sub2ind((mdp.size_x, mdp.size_y, 4, 2), state.position.x, state.position.y, orientation, sd)
end

function POMDPs.action_index(mdp::GridWorld, act::Symbol)
    if act==:forward
        return 1
    elseif act==:rotate_clockwise
        return 2
    elseif act==:rotate_counterclockwise
        return 3
    end
    error("Invalid GridWorld action: $act")
end

POMDPs.isterminal(mdp::GridWorld, s::GridWorldState) = s.done

# =================================
# (PO)MDP solving
# =================================
using MCTS

mdp = GridWorld()

# initialize the solver with hyper parameters
# n_iterations: the number of iterations that each search runs for
# depth: the depth of the tree (how far away from the current state the algorithm explores)
# exploration constant: this is how much weight to put into exploratory actions.
# A good rule of thumb is to set the exploration constant to what you expect the upper bound on your average expected reward to be.
solver = MCTSSolver(n_iterations=1000,
                    depth=20,
                    exploration_constant=10.0,
                    enable_tree_vis=true)

policy = solve(solver, mdp)

# =================================
# Test the policy
# =================================
start_state = GridWorldState(9, 4, :north)

sim(mdp, start_state, max_steps=10) do s
    println("State: $s")
    a = action(policy, s)
    println("Action: $a\n")
    actionToRobotFunction(a)()
    return a
end
