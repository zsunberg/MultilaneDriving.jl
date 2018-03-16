struct MaxSafeAccel
    dt::Float64
end

"""
Calculate the maximum safe acceleration that will allow the car to avoid a collision if the car in front slams on its brakes
"""
function max_safe_acc(safety::MaxSafeAccel, c::Context, idx::Int)
    dt = safety.dt

    fidx = fore_index(c, idx)
    if fidx == 0
        return Inf
    else
        n_brake_acc = nullable_max_safe_acc(headway(c, idx),
                                            vel(state(c, idx)),
                                            vel(state(c, fidx)),
                                            brake_limit(vehicle(c, idx)), dt)
        return get(n_brake_acc, -brake_limit(vehicle(c, idx)))
    end
end

"""
Return max_safe_acc or an empty Nullable if the discriminant is negative.
"""
function nullable_max_safe_acc(gap, v_behind, v_ahead, braking_limit, dt)
    bp = braking_limit
    v = v_behind
    vo = v_ahead
    g = gap
    # VVV see mathematica notebook
    discriminant = 8.*g*bp + bp^2*dt^2 - 4.*bp*dt*v + 4.*vo^2
    if discriminant >= 0.0
        return Nullable{Float64}(- (bp*dt + 2.*v - sqrt(discriminant)) / (2.*dt))
    else
        return Nullable{Float64}()
    end
end

function is_safe(safety::MaxSafeAccel, c::Context, idx::Int, a::AccelSpeed)
    return a.long_accel <= max_safe_acc(safety, c, idx)
end

struct RoadEdges
    dt::Float64
end

function is_safe(safety::RoadEdges, c::Context, idx::Int, a::AccelSpeed)
    y = pos(state(c, idx))[2]
    w = width(vehicle(c, idx))
    rw = c.roadway
    next_y = y + a.lat_speed*safety.dt
    return next_y - w/2 >= -rw.lanewidth/2 && next_y + w/2 <= (rw.n_lanes-0.5)*rw.lanewidth
end
