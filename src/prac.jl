using AutomotiveDrivingModels
using AutoViz
using Distributions
using Interact

roadway = gen_stadium_roadway(3)

scene = Scene()
push!(scene, Vehicle(VehicleState(VecSE2(0.0,-DEFAULT_LANE_WIDTH,0.0),roadway,30.0), VehicleDef(),1))
push!(scene, Vehicle(VehicleState(VecSE2(43.0,-DEFAULT_LANE_WIDTH,0.0),roadway,20.0), VehicleDef(),2))
push!(scene, Vehicle(VehicleState(VecSE2(80.0,-DEFAULT_LANE_WIDTH,0.0),roadway,20.0), VehicleDef(),3))
car_colors = get_pastel_car_colors(scene)
cam = FitToContentCamera()
# render(scene, roadway, cam=cam, car_colors=car_colors)

timestep = 0.2

models = Dict{Int, DriverModel}()
# models[1] = IntelligentDriverModel()
# models[2] = IntelligentDriverModel()
# models[3] = IntelligentDriverModel()
models[1] = BafflingDriver(timestep)
models[2] = BafflingDriver(timestep)
models[3] = BafflingDriver(timestep)

set_desired_speed!(models[1], 50.0)
set_desired_speed!(models[2], 40.0)
set_desired_speed!(models[3], 20.0)

nticks = 200
rec = SceneRecord(nticks+1, timestep)
simulate!(rec, scene, roadway, models, nticks)
render(rec[0], roadway, cam=cam, car_colors=car_colors)

@manipulate for frame_index in 1:nframes(rec)
    render(rec[frame_index-nframes(rec)],roadway,cam=cam,car_colors=car_colors)
end
