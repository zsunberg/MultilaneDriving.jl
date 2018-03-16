# INTERFACE
sample_action(m, ds, idx, A, rng::AbstractRNG) = rand(rng, action_distribution(m, ds, idx, A))

observe(m, ds, new_context, dt) = new_context
observe(m::Void, ds, new_context, dt) = nothing

# statetype(t::Type, c::Type) = throw(MethodError(statetype, (t,c)))
# statetype(x::Any, c) = statetype(typeof(x))

# NO CRASH DRIVER

@with_kw struct NoCrashMLDriver{M1, M2, S}
    long::M1  = NORMAL_IDM
    lat::M2   = nothing
    safety::S = nothing
end
NoCrashMLDriver(dt::Float64) = NoCrashMLDriver(NORMAL_IDM, nothing, MaxSafeAccel(dt))

initial_state(d::NoCrashMLDriver, c::Context) = MLDriverState(initial_state(d.long), initial_state(d.lat))

struct MLDriverState{S1, S2, H<:HighwayDrivingContext}
    long::S1
    lat::S2
    c::H
end

# function statetype(::Type{NoCrashMLDriver{M1,M2,S}}) where {M1, M2, S}
#     return MLDriverState{statetype(M1), statetype(M2), }
# end

function observe(m::NoCrashMLDriver, ds, new_context, dt)
    long = observe(m.long, ds.long, new_context, dt)
    lat = observe(m.lat, ds.lat, new_context, dt)
    return MLDriverState(long, lat, new_context)
end

function action_distribution(m::NoCrashMLDriver, state::MLDriverState, idx, A::Type{AccelSpeed})
    c = state.c
    max_safe = max_safe_acc(m.safety, state.long, idx)
    a::LongAccel = action(m.long, state.long, idx, LongAccel)
    mid = min(a.a, max_safe)
    lower = min(-1e-5, max(-max_acc(m.long)/2, -brake_limit(vehicle(c, idx))-mid))
    upper = min(max_acc(m.long)/2, max(max_safe-mid, 1e-5))
    long_dist = TriangularDist(mid+lower, mid+upper, mid)

    lat_dist = DeltaDist(0.0)

    return AccelSpeedDist(long_dist, lat_dist)
end

struct AccelSpeedDist{D1, D2}
    long::D1
    lat::D2
end

rand(rng::AbstractRNG, d::AccelSpeedDist) = AccelSpeed(rand(rng, d.long), rand(rng, d.lat))
function pdf(d::AccelSpeedDist, val::AbstractVector{Float64})
    @assert length(val) == 2
    return pdf(d.long, val[1])*pdf(d.lat, val[2])
end
sampletype(::Type{AccelSpeedDist}) = AccelSpeed

struct DeltaDist val::Float64 end

rand(rng::AbstractRNG, d::DeltaDist) = d.val
pdf(d::DeltaDist, val::Float64) = convert(Float64, val == d.val)
sampletype(::Type{DeltaDist}) = Float64
