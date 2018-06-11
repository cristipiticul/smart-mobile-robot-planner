using POMDPs
using Distributions
using POMDPToolbox

struct GridCell
    x::Int64
    y::Int64
end

struct GridWorldState
    position::GridCell
    orientation::Symbol # one of [:north, :south, :east, :west]
    done::Bool # are we in a terminal state?
end



# initial state constructor
GridWorldState(x::Int64, y::Int64, orientation::Symbol) = GridWorldState(GridCell(x, y), orientation, false)
# checks if the position of two states are the same
posequal(s1::GridCell, s2::GridCell) = s1.x == s2.x && s1.y == s2.y

# the grid world mdp type
type GridWorld <: MDP{GridWorldState, Symbol} # Note that our MDP is parametarized by the state and the action
    size_x::Int64 # x size of the grid
    size_y::Int64 # y size of the grid
    reward_states::Vector{GridCell} # the states in which agent recieves reward
    reward_values::Vector{Float64} # reward values for those states
    tprob::Float64 # probability of transitioning to the desired state
    discount_factor::Float64 # disocunt factor
end

# we use key worded arguments so we can change any of the values we pass in
function GridWorld(;sx::Int64=10, # size_x
                    sy::Int64=10, # size_y
                    rs::Vector{GridCell}=[GridCell(4,3), GridCell(4,6), GridCell(9,3), GridCell(8,8)], # reward states
                    rv::Vector{Float64}=rv = [-10.,-5,10,3], # reward values
                    tp::Float64=0.7, # tprob
                    discount_factor::Float64=0.9)
    return GridWorld(sx, sy, rs, rv, tp, discount_factor)
end


# we can now create a GridWorld mdp instance like this:
mdp = GridWorld()

function POMDPs.states(mdp::GridWorld)
    s = GridWorldState[] # initialize an array of GridWorldStates
    # loop over all our states, remeber there are two binary variables:
    # done (d)
    for d = 0:1, y = 1:mdp.size_y, x = 1:mdp.size_x, orientation = [:north, :south, :east, :west]
        push!(s, GridWorldState(x,y,orientation,d))
    end
    return s
end

POMDPs.actions(mdp::GridWorld) = [:forward, :rotate_clockwise, :rotate_counterclockwise]

# transition helpers
function inbounds(mdp::GridWorld,pos::GridCell)
    if 1 <= pos.x <= mdp.size_x && 1 <= pos.y <= mdp.size_y
        return true
    else
        return false
    end
end

inbounds(mdp::GridWorld, position::GridCell) = inbounds(mdp, position);

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
        println("Error! Invalid orientation!")
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

function POMDPs.transition(mdp::GridWorld, state::GridWorldState, action::Symbol)
    if state.done
        return SparseCat([GridWorldState(state.position, state.orientation, true)], [1.0])
    elseif state in mdp.reward_states
        return SparseCat([GridWorldState(state.position, state.orientation, true)], [1.0])
    end

    if action == :rotate_clockwise
        return SparseCat([GridWorldState(state.position, rotateClockwise(state.orientation))], [1.0])
    elseif action == :rotate_counterclockwise
        return SparseCat([GridWorldState(state.position, rotateCounterClockwise(state.orientation))], [1.0])
    elseif action == :forward
        new_position = forwardPosition(state.position, state.orientation)
        if inbounds(mdp, new_position)
            return SparseCat([GridWorldState(new_position, state.orientation)], [1.0])
        else
            return SparseCat([GridWorldState(state.position, state.orientation)], [1.0])
        end
    else
        println("Error! Unrecognized action: $action")
    end

    if !inbounds(mdp, neighbors[target])
        # If would transition out of bounds, stay in
        # same cell with probability 1
        return SparseCat([GridWorldState(x, y)], [1.0])
    else
        probability[target] = mdp.tprob

        oob_count = sum(!inbounds(mdp, n) for n in neighbors) # number of out of bounds neighbors

        new_probability = (1.0 - mdp.tprob)/(3-oob_count)

        for i = 1:4 # do not include neighbor 5
            if inbounds(mdp, neighbors[i]) && i != target
                probability[i] = new_probability
            end
        end
    end

    return SparseCat(neighbors, probability)
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



POMDPs.n_states(mdp::GridWorld) = 4*2*mdp.size_x*mdp.size_y
POMDPs.n_actions(mdp::GridWorld) = 3
POMDPs.discount(mdp::GridWorld) = mdp.discount_factor;

function POMDPs.state_index(mdp::GridWorld, state::GridWorldState)
    sd = Int(state.done + 1)
    orientations = Dict(:north=>1, :south=>2, :east=>3, :west=>4) # See Performance Note below
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

mdp = GridWorld()
#mdp.tprob=1.0
sim(mdp, GridWorldState(GridCell(4,1),:north), max_steps=10) do s
    println("state is: $s")
    a = :forward
    println("moving $a")
    return a
end


# # first let's load the value iteration module
# using DiscreteValueIteration
#
# # initialize the problem
# mdp = GridWorld()
#
# # initialize the solver
# # max_iterations: maximum number of iterations value iteration runs for (default is 100)
# # belres: the value of Bellman residual used in the solver (defualt is 1e-3)
# solver = ValueIterationSolver(max_iterations=100, belres=1e-3)
#
# # initialize the policy by passing in your problem
# policy = ValueIterationPolicy(mdp)
#
# # solve for an optimal policy
# # if verbose=false, the text output will be supressed (false by default)
# solve(solver, mdp, policy, verbose=true);
#


using MCTS

# initialize the problem
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

# initialize the planner by calling the `solve` function. For online solvers, the
planner = solve(solver, mdp)

# to get the action:
s = GridWorldState(9,2)
a = action(planner, s)



# we'll use POMDPToolbox for simulation
using POMDPToolbox # if you don't have this module install it by running POMDPs.add("POMDPToolbox")

s = GridWorldState(4,1) # this is our starting state
hist = HistoryRecorder(max_steps=1000)

hist = simulate(hist, mdp, policy, s)

println("Total discounted reward: $(discounted_reward(hist))")
