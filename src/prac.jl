using AutomotiveDrivingModels
using AutoViz
using Distributions
using Interact

roadway = gen_stadium_roadway(3)
num_vehs = 11
timestep = 0.2

scene = Scene()
for i in 1:num_vehs
    if i&3 == 0
        push!(scene, Vehicle(VehicleState(VecSE2(0.0+10.0*(i-1),-DEFAULT_LANE_WIDTH,0.0),roadway,30.0), VehicleDef(),i))
    elseif i&3 == 1
        push!(scene, Vehicle(VehicleState(VecSE2(0.0+10.0*(i-1),0.0,0.0),roadway,30.0), VehicleDef(),i))
    else
        push!(scene, Vehicle(VehicleState(VecSE2(0.0+10.0*(i-1),-2*DEFAULT_LANE_WIDTH,0.0),roadway,30.0), VehicleDef(),i))
    end
end
car_colors = get_pastel_car_colors(scene)
cam = FitToContentCamera()

models = Dict{Int, DriverModel}()
for i in 1:num_vehs
    models[i] = BafflingDriver(timestep)
    set_desired_speed!(models[i], 40.0)
end

nticks = 200
rec = SceneRecord(nticks+1, timestep)
simulate!(rec, scene, roadway, models, nticks)
render(rec[0], roadway, cam=cam, car_colors=car_colors)

@manipulate for frame_index in 1:nframes(rec)
    render(rec[frame_index-nframes(rec)],roadway,cam=cam,car_colors=car_colors)
end
