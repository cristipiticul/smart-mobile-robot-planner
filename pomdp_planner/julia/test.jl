using POMDPs, POMDPToolbox, AEMS, POMDPModels

pomdp = BabyPOMDP()
solver = AEMSSolver()
policy = solve(solver, pomdp)      # planner is of type AEMSPlanner

belief_updater = updater(policy)
history = simulate(HistoryRecorder(max_steps=10), pomdp, policy, belief_updater)

# look at what happened
for (s, b, a, o) in eachstep(history, "sbao")
    println("State was $s,")
    println("belief was $b,")
    println("action $a was taken,")
    println("and observation $o was received.\n")
end

# once you have a belief b
#a = action(planner, true)
