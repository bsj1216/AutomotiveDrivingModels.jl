using Random
using JuMP
using Ipopt
using MosekTools

function run()
    ind_others = [1,2]
    h = 4/2
    w = 2/2

    prob = Model(with_optimizer(Ipopt.Optimizer, print_level=0))
    # prob = Model(with_optimizer(Mosek.Optimizer))
    ego_x,ego_y,ego_θ = 0, 0, 0

    oth_xs = zeros(length(ind_others))
    oth_ys = zeros(length(ind_others))
    oth_θs = zeros(length(ind_others))
    # for (i,ind) in enumerate(ind_others)
    #     oth_xs[i],oth_ys[i],oth_θs[i] = scene[ind].state.posG.x, scene[ind].state.posG.y, scene[ind].state.posG.θ
    # end

    # TEMP
    oth_xs = [0 6 6 0];
    oth_ys = [3 0 3 -3];
    oth_θs = [0 0 0 0];

    @variable(prob, ego_p); @variable(prob, ego_q) # oth_p: x-coord oth_q: y-coord
    @variable(prob, oth_p); @variable(prob, oth_q)
    @objective(prob, MOI.MIN_SENSE, (ego_p-oth_p)^2+(ego_q-oth_q)^2)
    # @objective(prob, Min, (ego_p-oth_p)^2+(ego_q-oth_q)^2)

    # constraints
    @constraint(prob, ((ego_p-ego_x)*cos(ego_θ) + (ego_q-ego_y)*sin(ego_θ))^2/h^2
                    + ((ego_p-ego_x)*sin(ego_θ) + (ego_q-ego_y)*cos(ego_θ))^2/w^2
                    <= 1.0
                )
    for (i,ind) in enumerate(ind_others)
        @constraint(prob, ((oth_p-oth_xs[i])*cos(oth_θs[i]) + (oth_q-oth_ys[i])*sin(oth_θs[i]))^2/h^2
                        + ((oth_p-oth_xs[i])*sin(oth_θs[i]) + (oth_q-oth_ys[i])*cos(oth_θs[i]))^2/w^2
                        <= 1.0
                    )
    end

    # @variable(prob, xᵢ′); @variable(prob, yᵢ′)
    # @constraint(prob, ((x′-x)*cos(θ) + (y′-y)*sin(θ))^2/(h^2) + ((x′-x)*sin(θ) + (y′-y)*cos(θ))^2/(w^2) <= 1.0)
    # @constraint(prob, ((xᵢ′-xᵢ)*cos(θᵢ) + (yᵢ′-yᵢ)*sin(θᵢ))^2/(h^2) + ((xᵢ′-xᵢ)*sin(θᵢ) + (yᵢ′-yᵢ)*cos(θᵢ))^2/(w^2) <= 1.0)
    optimize!(prob)

    return prob
end

prob = run()
println("solution:", objective_value(prob))
