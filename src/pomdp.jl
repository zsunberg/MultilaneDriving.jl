struct MLDState{DM, DS}
    vehicle_states::Vector{SVector{3,Float64}} # last one is own state
    driver_models::Vector{DM}
    driver_states::Vector{DS}
    terminal::Nullable{Symbol}
end

MLDState(vss, dms, dss) = MLDState(vss, dms, dss, Nullable{Symbol}())

# const MLDPhysicalState = Vector{SVector{3, Float64}}
struct MLDPhysicalState
    vehicle_states::Vector{SVector{3, Float64}}
end

my_idx(s::Union{MLDState, MLDPhysicalState}) = length(s.vehicle_states)

struct MLDPOMDP{DM, DS} <: POMDP{MLDState{DM, DS}, AccelSpeed, MLDPhysicalState}
    reward::MLDRewardModel
    roadway::Highway
    vehicle_model::HighwayVehicle
    dt::Float64
    radius::Float64
    safety::MaxSafeAccel
    dm_dist::IDMGaussianCopula
    _normal_actions::Vector{AccelSpeed}
end

function MLDPOMDP(;reward=MLDRewardModel()
                  roadway::Highway=Highway(4, 4.0, 100.0),
                  vehicle_model::HighwayVehicle=HighwayVehicle(),
                  adjustment_accel::Float64=1.0,
                  dt::Float64=1.0,
                  radius::Float64=50.0,
                  safety=MaxSafeAccel(dt),
                  dm_dist::IDMGaussianCopula=IDMGaussianCopula(0.75)
                 )

    lr_speed = roadway.lanewidth/2.0 # half a lane per second
    accels = (-adjustment_accel, 0.0, adjustment_accel)
    na = vec(collect(AccelSpeed(a, s) for a in accels, s in (-lr_speed, 0.0, lr_speed)))
    DM = NoCrashMLDriver{IDM, Void, MaxSafeAccel}
    C = HighwayDrivingContext{typeof(vehicle_model), SVector{3,Float64}}
    DS = MLDriverState{C, nothing, C}
    return MLDPOMDP{DM, DS}(reward, roadway, vehicle_model, dt, safety, dm_dist, na)
end

isterminal(p::MLPOMDP, s::MLDState) = !isnull(s.terminal)

function generate_sr(p::MLDPOMDP, s::MLDState, a::AccelSpeed, rng::AbstractRNG)
    r = 0.0

    context = HighwayDrivingContext(p.vehicle_model, s.vehicle_states, p.roadway)

    # ego vehicle
    if a.long_accel < -p.reward.brake_thresh
        r -= p.reward.hard_brake_penalty
    end
    my_sp = propagate(p.vehicle_model, s.vehicle_states[end], a, context, p.dt)
   
    # others
    n_other = length(s.driver_models)
    vss = similar(s.vehicle_states, 0)
    dms = similar(s.driver_models, 0)
    old_is = Int[]
    own_xp = pos(my_sp)[1]
    for i in 1:n_other
        m = s.driver_models[i]
        ds = s.driver_states[i]
        da = sample_action(m, ds, i, AccelSpeed, rng)
        if da.long_accel < -p.reward.brake_thresh
            r -= p.reward.hard_brake_penalty
        end
        vs = s.vehicle_states[i]
        vsp = propagate(p.vehicle_model, vs, da, context, p.dt)
        if abs(pos(vsp)[1]-own_xp) > p.radius
            push!(vss, vsp)
            push!(dms, s.driver_models[i])
            push!(old_is, i)
        end
    end

    # insert ego vehicle
    push!(vss, my_sp)
    holdovers = HighwayDrivingContext(p.vehicle_model, vss, p.roadway)

    all_vss = vss[1:end-1] # all except ego
    newf = push_new_cars_front!(dms, all_vss, holdovers, p.dm_dist, own_xp + p.radius)
    newb = push_new_cars_back!(dms, all_vss, holdovers, p.dm_dist, own_xp - p.radius)

    push!(all_vss, my_sp)

    new_ctx = HighwayDrivingContext(p.vehicle_model, all_vss, p.roadway)::HighwayDrivingContext{typeof(p.vehicle_model), eltype(vss)}

    # see if terminal
    if pos(my_sp)[1] > p.roadway.len
        terminal = Nullable(:reached_end)
    elseif all(occupied_lanes(new_ctx, length(vss)) .== p.reward.target_lane) 
        terminal = Nullable(:success)
        r += p.reward.success_reward
    else
        terminal = Nullable{Symbol}()
    end

    # update driver states
    dss = similar(s.driver_states, 0)
    for i in 1:length(vss)-1
        if i <= length(old_is)
            ds = observe(dms[i],
                         s.driver_states[old_is[i]],
                         new_ctx, p.dt)
            push!(dss, ds)
        else
            ds = initial_state(dms[i], new_ctx)
            push!(dss, ds)
        end
    end

    return MLDState(vss, s.driver_models, dss, terminal)::typeof(s), r
end

generate_o(p::MLPOMDP, sp::MLDState, rng::AbstractRNG) = MLDPhysicalState(s.vehicle_states)

function initial_state(p::MLPOMDP, rng::AbstractRNG)
    init = seed_state(p)
    burn_in_steps = 100
    my_model = NORMAL_IDM
    policy = ModelPolicy(p, my_model)
    hr = HistoryRecorder(max_steps=burn_in_steps)
    up = PrimedPreviousObservationUpdater(init)
    hist = simulate(hr, p, policy, up, init, init)
    return state_hist(hist)[end]
end

function seed_state(p::MLPOMDP{DM, DS})
    cars_per_100_m = 3
    xs = linspace(0.0, 2.0*p.radius, cars_per_100_m)
    car_inits = SVector{3, Float64}[]
    for l in 1:p.roadway.n_lanes
        y = (l-1)*p.roadway.lanewidth
        for x in xs
            if l > 1 || abs(x-p.radius) # stay away from ego at (radius,0)
                push!(car_inits, SVector(x, y, 20.0))
            end
        end
    end
    dms = DM[]
    dss = DS[]
    for 1:length(car_inits)
        dm = rand(rng, p.dm_dist)
        push!(dms, dm)
        push!(dss, initial_state(dm))
    end
    push!(car_inits, SVector(p.radius, 0.0, 20.0)) # ego
    return MLDState(vss, dms, dss)
end

# ACTIONS

function actions(p::MLDPOMDP, s::Union{MLDState, MLDPhysicalState})
    na = p._normal_actions
    c = HighwayDrivingContext(p.vehicle_model, s.vehicle_states, p.roadway)
    brake_acc = min(max_safe_acc(p.safety, c, my_idx(s)), -p.reward.brake_thresh/2.0)
    brake = AccelSpeed(brake_acc, 0.0)
    acceptable = [brake]
    for a in na
        # leaving road is always unsafe 
        if !is_safe(RoadEdges(p.dt), c, my_idx(s), a)
            continue
        end

        # prevent running into the person in front or to the side
        if is_safe(p.safety, c, my_idx(s), a)
            push!(acceptable, a)
        end
    end
    return acceptable
end

struct ModelPolicy{M} <: Policy
    p::MLDPOMDP
    model::M
end

function action(policy::ModelPolicy, s::Union{MLDState, MLDPhysicalState})
    c = HighwayDrivingContext(policy.p.vehicle_model, s.vehicle_states, policy.p.roadway)
    accel = action(policy.model, c, my_idx(s), LongAccel)
    return AccelSpeed(accel, 0.0)
end

# REWARD
@with_kw struct MLDRewardModel
    brake_thresh::Float64    = 4.0 # always positive
    brake_penalty::Float64   = 1.0 # always positive
    target_lane::Int         = 
    success_reward::Float64
end

# ASSUME IDM MODELS for access to sstar
function push_new_cars_front!(models, states, c::Context, dm_dist, front_x::Float64, rng::AbstractRNG)
    # the models MUST correspond to the same indices as the states in c
    for l in 1:c.roadway.n_lanes
        head = lane_head(c, l)
        if head !=0
            head_x = pos(state(c, head))[1]
            delta_x = front_x - head_x
            head_xdot = vel(state(c, head))
            delta_xdot = head_xdot - NORMAL_IDM.v0
        end
        if head == 0 || delta_x >= sstar(NORMAL_IDM, head_xdot, delta_xdot)
            push!(models, rand(rng, p.dm_dist))
            push!(states, SVector(front_x, (l-1)*c.roadway.lanewidth, NORMAL_IDM.v0))
        end
    end
end

function push_new_cars_back!(models, states, c::Context, dm_dist, back_x::Float64)
    for l in 1:c.roadway.n_lanes
        tail = lane_tail(c, l)
        if tail != 0
            tail_x = pos(state(c, tail))[1]
            delta_x = tail_x - back_x
            tail_xdot = vel(state(c, tail))
            delta_xdot = NORMAL_IDM.v0 - tail_xdot
        end
        if tail == 0 || delta_x >= sstar(NORMAL_IDM, NORMAL_IDM.v0, delta_xdot)
            push!(models, rand(rng, p.dm_dist))
            push!(states, SVector(front_x, (l-1)*c.roadway.lanewidth, NORMAL_IDM.v0))
        end
    end

    return models, states
end
