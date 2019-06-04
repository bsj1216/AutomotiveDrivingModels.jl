"""
	BafflingLaneChanger
A ramdom lane change behavior that changes lanes:
 - whenever the lead car is going slower than our desired speed.
 - whenever randomly decided (based on a uniform distribution).

[For collision avoidance] Lane changes are made when:
 - there is an available lane and
 - fore/rear gaps exceed our thresholds.
[For practicality] However, lane changes can be also made when:
 - we are slower than a rear vehicle in the target lane,
 - any lead vehicle in the target lane is slower than we can currently go.

# Constructors
	BafflingLaneChanger(timestep::Float64,
                    v_des::Float64=29.0,
                    rec::SceneRecord=SceneRecord(2,timestep),
                    threshold_fore::Float64 = 50.0,
                    threshold_lane_change_gap_fore::Float64 = 10.0,
                    threshold_lane_change_gap_rear::Float64 = 10.0,
                    dir::Int=DIR_MIDDLE)

# Fields
- `dir::Int = DIR_MIDDLE` the desired lane to go to eg: left,middle (i.e. stay in same lane) or right
- `rec::SceneRecord` TODO
- `v_des::Float64 = 29.0` desired velocity
- `threshold_fore::Float64 = 50.0` Distance from lead vehicle
- `threshold_lane_change_gap_fore::Float64 = 10.0` Space in frontLaneChangeChoice
- `threshold_lane_change_gap_rear::Float64 = 10.0` Space rear
"""
mutable struct BafflingLaneChanger <: LaneChangeModel
    dir::Int
    rec::SceneRecord

    v_des::Float64
    threshold_fore::Float64
    threshold_lane_change_gap_fore::Float64
    threshold_lane_change_gap_rear::Float64

    function BafflingLaneChanger(
        timestep::Float64;
        v_des::Float64=29.0,
        rec::SceneRecord=SceneRecord(2,timestep),
        threshold_fore::Float64 = 50.0,
        threshold_lane_change_gap_fore::Float64 = 10.0,
        threshold_lane_change_gap_rear::Float64 = 10.0,
        dir::Int=DIR_MIDDLE,
        )

        retval = new()
        retval.dir = dir
        retval.rec = rec
        retval.v_des = v_des
        retval.threshold_fore = threshold_fore
        retval.threshold_lane_change_gap_fore = threshold_lane_change_gap_fore
        retval.threshold_lane_change_gap_rear = threshold_lane_change_gap_rear
        retval
    end
end
get_name(::BafflingLaneChanger) = "BafflingLaneChanger"
function set_desired_speed!(model::BafflingLaneChanger, v_des::Float64)
    model.v_des = v_des
    model
end
function observe!(model::BafflingLaneChanger, scene::Scene, roadway::Roadway, egoid::Int)

    rec = model.rec
    update!(rec, scene)
    vehicle_index = findfirst(egoid, scene)

    veh_ego = scene[vehicle_index]
    v = veh_ego.state.v

    left_lane_exists = convert(Float64, get(N_LANE_LEFT, rec, roadway, vehicle_index)) > 0
    right_lane_exists = convert(Float64, get(N_LANE_RIGHT, rec, roadway, vehicle_index)) > 0
    fore_M = get_neighbor_fore_along_lane(scene, vehicle_index, roadway, VehicleTargetPointFront(), VehicleTargetPointRear(), VehicleTargetPointFront(), max_distance_fore=model.threshold_fore)
    fore_L = get_neighbor_fore_along_left_lane(scene, vehicle_index, roadway, VehicleTargetPointRear(), VehicleTargetPointRear(), VehicleTargetPointFront())
    fore_R = get_neighbor_fore_along_right_lane(scene, vehicle_index, roadway, VehicleTargetPointRear(), VehicleTargetPointRear(), VehicleTargetPointFront())
    rear_L = get_neighbor_rear_along_left_lane(scene, vehicle_index, roadway, VehicleTargetPointFront(), VehicleTargetPointFront(), VehicleTargetPointRear())
    rear_R = get_neighbor_rear_along_right_lane(scene, vehicle_index, roadway, VehicleTargetPointFront(), VehicleTargetPointFront(), VehicleTargetPointRear())

    model.dir = DIR_MIDDLE
    if fore_M.Δs < model.threshold_fore # there is a lead vehicle
        veh_M = scene[fore_M.ind]
        speed_M = veh_M.state.v
        r = Base.rand(Float64)
        lane_change_triggered = speed_M ≤ min(model.v_des, v) || # they are driving slower than we want
                                r <= 0.1                           # just randomly triggered with the probability of 0.1

        if lane_change_triggered
            speed_ahead = speed_M
            if right_lane_exists && left_lane_exists &&
                isEnoughSpace(model,fore_R,rear_R) && isEnoughSpace(model,fore_L,rear_L)
                # (rear_R.ind == nothing || scene[rear_R.ind].state.v ≤ v) && # we are faster than any follower
                # (fore_R.ind == nothing || scene[fore_R.ind].state.v > speed_ahead) # lead is faster than current speed

                r = Base.rand([-1,1])
                model.dir = r # randomly choose one if both lanes exist
                ind = r == DIR_RIGHT ? fore_R.ind : fore_L.ind
            elseif right_lane_exists && isEnoughSpace(model,fore_R,rear_R)
                model.dir, ind = DIR_RIGHT, fore_R.ind
            elseif left_lane_exists && isEnoughSpace(model,fore_L,rear_L)
                model.dir, ind = DIR_LEFT, fore_L.ind
            end
            speed_ahead = ind == nothing ? Inf : scene[ind].state.v
        end
    end

    model
end

function isEnoughSpace(model::BafflingLaneChanger, fore::NeighborLongitudinalResult, rear::NeighborLongitudinalResult)
    return fore.Δs > model.threshold_lane_change_gap_rear &&
        rear.Δs > model.threshold_lane_change_gap_fore
end

Base.rand(model::BafflingLaneChanger) = LaneChangeChoice(model.dir)
