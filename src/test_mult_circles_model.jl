
ind_others = [1,2]
h = 4/2
w = 2/2
r = w

ego_x,ego_y,ego_θ = 0, 0, 0
oth_x,oth_y,oth_θ = 6, 0, 0

# oth_xs = zeros(length(ind_others))
# oth_ys = zeros(length(ind_others))
# oth_θs = zeros(length(ind_others))
# # for (i,ind) in enumerate(ind_others)
# #     oth_xs[i],oth_ys[i],oth_θs[i] = scene[ind].state.posG.x, scene[ind].state.posG.y, scene[ind].state.posG.θ
# # end
#
# # TEMP
# oth_xs = [0 6 6 0];
# oth_ys = [3 0 3 -3];
# oth_θs = [0 0 0 0];

# @time begin
min_dis = Inf
for i in [-1,0,1]
    global min_dis
    for j in [-1,0,1]
        dis = sqrt(
                  ( (ego_x + i*r*cos(ego_θ)) - (oth_x+ j*r*cos(oth_θ)) )^2
                 +( (ego_y + i*r*sin(ego_θ)) - (oth_y+ j*r*sin(oth_θ)) )^2
                 )     - 2*r
        min_dis = min(dis, min_dis)
    end
end
# end

println("min distance: ", min_dis)
