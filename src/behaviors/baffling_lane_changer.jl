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
    threshold_lane_change_rand::Float64

    function BafflingLaneChanger(
        timestep::Float64;
        v_des::Float64=29.0,
        rec::SceneRecord=SceneRecord(2,timestep),
        threshold_fore::Float64 = 50.0,
        threshold_lane_change_gap_fore::Float64 = 5.0,
        threshold_lane_change_gap_rear::Float64 = 5.0,
        threshold_lane_change_rand::Float64 = 0.00,
        dir::Int=DIR_MIDDLE,
        )

        retval = new()
        retval.dir = dir
        retval.rec = rec
        retval.v_des = v_des
        retval.threshold_fore = threshold_fore
        retval.threshold_lane_change_gap_fore = threshold_lane_change_gap_fore
        retval.threshold_lane_change_rand = threshold_lane_change_rand
        retval.threshold_lane_change_gap_rear = threshold_lane_change_gap_rear
        retval
    end
end
get_name(::BafflingLaneChanger) = "BafflingLaneChanger"
function AutomotiveDrivingModels.set_desired_speed!(model::BafflingLaneChanger, v_des::Float64)
    model.v_des = v_des
    model
end
function AutomotiveDrivingModels.observe!(model::BafflingLaneChanger, scene::Scene, roadway::Roadway, egoid::Int)

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
    r = Base.rand(Float64)
    if fore_M.Δs < model.threshold_fore # there is a lead vehicle
        veh_M = scene[fore_M.ind]
        speed_M = veh_M.state.v
        if speed_M ≤ min(model.v_des, v) # they are driving slower than we want

            speed_ahead = speed_M

            # consider changing to a different lane
            if right_lane_exists &&
               fore_R.Δs > model.threshold_lane_change_gap_rear && # there is space rear
               rear_R.Δs > model.threshold_lane_change_gap_fore && # there is space fore
               (rear_R.ind == nothing || scene[rear_R.ind].state.v ≤ v) && # we are faster than any follower
               (fore_R.ind == nothing || scene[fore_R.ind].state.v > speed_ahead) # lead is faster than current speed

                speed_ahead = fore_R.ind != nothing ? scene[fore_R.ind].state.v : Inf
                model.dir = DIR_RIGHT
            end
            if left_lane_exists &&
               fore_L.Δs > model.threshold_lane_change_gap_rear && # there is space rear
               rear_L.Δs > model.threshold_lane_change_gap_fore && # there is space fore
               (rear_L.ind == nothing || scene[rear_L.ind].state.v ≤ v) && # we are faster than any follower
               (fore_L.ind == nothing || scene[fore_L.ind].state.v > speed_ahead) # lead is faster than current speed

                speed_ahead = fore_L.ind != nothing ? scene[fore_L.ind].state.v : Inf
                model.dir = DIR_LEFT
            end
        end
    elseif r <= model.threshold_lane_change_rand
        # consider changing to a different lane
        if right_lane_exists &&
           fore_R.Δs > model.threshold_lane_change_gap_rear && # there is space rear
           rear_R.Δs > model.threshold_lane_change_gap_fore # there is space fore
            model.dir = DIR_RIGHT
        end
        if left_lane_exists &&
           fore_L.Δs > model.threshold_lane_change_gap_rear && # there is space rear
           rear_L.Δs > model.threshold_lane_change_gap_fore # there is space fore
            model.dir = DIR_LEFT
        end
    end

    model
end

Base.rand(model::BafflingLaneChanger) = LaneChangeChoice(model.dir)
