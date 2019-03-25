@testset "action interface" begin
    roadway = get_test_roadway()
    trajdata = get_test_trajdata(roadway)
    veh = get(trajdata, 1, 1)
    s = VehicleState()
    @test VehicleState() == propagate(veh, s, roadway, NaN)
end

@testset "AccelTurnrate" begin
    roadway = get_test_roadway()
    trajdata = get_test_trajdata(roadway)
    veh = get(trajdata, 1, 1)
    a = AccelTurnrate(0.1,0.2)
    @test a == convert(AccelTurnrate, [0.1,0.2])
    @test copyto!([NaN, NaN], AccelTurnrate(0.1,0.2)) == [0.1,0.2]
    @test length(AccelTurnrate) == 2

    s = propagate(veh, AccelTurnrate(0.0,0.0), roadway, 1.0)
    @test isapprox(s.posG.x, veh.state.v)
    @test isapprox(s.posG.y, 0.0)
    @test isapprox(s.posG.θ, 0.0)

    # TODO Test the get method
end

@testset "AccelDesang" begin
    roadway = get_test_roadway()
    trajdata = get_test_trajdata(roadway)
    veh = get(trajdata, 1, 1)
    a = AccelDesang(0.1,0.2)
    @test a == convert(AccelDesang, [0.1,0.2])
    @test copyto!([NaN, NaN], AccelDesang(0.1,0.2)) == [0.1,0.2]
    @test length(AccelDesang) == 2

    s = propagate(veh, AccelDesang(0.0,0.0), roadway, 1.0)
    @test isapprox(s.posG.x, veh.state.v*1.0)
    @test isapprox(s.posG.y, 0.0)
    @test isapprox(s.posG.θ, 0.0)
    @test isapprox(s.v, veh.state.v)
end

@testset "AccelSteeringAngle" begin
    roadway = get_test_roadway()
    trajdata = get_test_trajdata(roadway)
    veh = get(trajdata, 1, 1)
    a = AccelSteeringAngle(0.1,0.2)
    @test a == convert(AccelSteeringAngle, [0.1,0.2])
    @test copyto!([NaN, NaN], AccelSteeringAngle(0.1,0.2)) == [0.1,0.2]
    @test length(AccelSteeringAngle) == 2

    # Check that propagate
    # won't work with the veh type loaded from test_tra
    @test_throws ErrorException propagate(veh, AccelSteeringAngle(0.0,0.0), roadway, 1.0)

    
    # TODO Test the branch condition within propagate
    # s = propagate(veh, AccelSteeringAngle(0.0,0.0), roadway, 1.0)
    # @test isapprox(s.posG.x, veh.state.v*1.0)
    # @test isapprox(s.posG.y, 0.0)
    # @test isapprox(s.posG.θ, 0.0)
    # @test isapprox(s.v, veh.state.v)
end

@testset "LatLonAccel" begin
    roadway = get_test_roadway()
    trajdata = get_test_trajdata(roadway)
    veh = get(trajdata, 1, 1)
    a = LatLonAccel(0.1,0.2)
    @test a == convert(LatLonAccel, [0.1,0.2])
    @test copyto!([NaN, NaN], LatLonAccel(0.1,0.2)) == [0.1,0.2]
    @test length(LatLonAccel) == 2

    Δt = 0.1
    s = propagate(veh, LatLonAccel(0.0,0.0), roadway, Δt)
    @test isapprox(s.posG.x, veh.state.v*Δt)
    @test isapprox(s.posG.y, 0.0)
    @test isapprox(s.posG.θ, 0.0)
    @test isapprox(s.v, veh.state.v)
end
