struct MLDState{DM, DS}
    vehicle_states::Vector{SVector{3,Float64}} # last one is own state
    driver_models::Vector{DM}
    driver_states::Vector{DS}
end

struct MLDObs
    vehicle_states::Vector{SVector{3, Float64}}
end

struct MLDPOMDP{DM, DS} <: POMDP{MLDState{DM, DS}, AccelSpeed, MLDObs}
    roadway::Highway
    vehicle_model::HighwayVehicle
    dt::Float64
end

function generate_s(p::MLDPOMDP, s::MLDState, a::AccelSpeed, rng::AbstractRNG)

    context = HighwayDrivingContext(p.vehicle_model, s.vehicle_states, p.roadway)
    
    n_other = length(s.driver_models)
    vss = Array{SVector{3,Float64}}(n_other)
    for i in 1:n_other
        m = s.driver_models[i]
        ds = s.driver_states[i]
        da = sample_action(m, ds, i, AccelSpeed, rng)
        vs = s.vehicle_states[i]
        vss[i] = propagate(p.vehicle_model, vs, da, context, p.dt)
    end

    # remove and add new vehicles here.

    # ego vehicle
    push!(vss, propagate(p.vehicle_model, s.vehicle_states[end], a, p.dt))

    new_context = HighwayDrivingContext(p.vehicle_model, vss, p.roadway)
    dss = Array{SVector{3,Float64}}(n_other)
    for i in 1:n_other
        dss[i] = observe(m, ds, new_context, dt)
    end

    return MLDState(vss, s.driver_models, dss)
end
