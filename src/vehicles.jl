"Action consisting of longitudinal acceleration and lateral speed"
struct AccelSpeed <: FieldVector{2, Float64}
    long_accel::Float64
    lat_speed::Float64
end

"Action consisting of a longitudinal acceleration"
struct LongAccel
    a::Float64
end


# state: [x, y, xdot]; y = 0 is middle of car in middle of rightmost lane

struct HighwayVehicle end

function propagate(m::HighwayVehicle, s::AbstractVector{Float64}, a::AccelSpeed, context, dt)
    xddot = a[1]
    ydot = a[2]
    x = s[1]
    xdot = s[3]
    xp = x + xdot*dt + 0.5*xddot*dt^2
    xdotp = xdot + xddot*dt
    y = s[2]
    yp = y + ydot*dt
    return SVector(xp, yp, xdotp)
end

brake_limit(v::HighwayVehicle) = 8.0
Base.length(v::HighwayVehicle) = 4.8
width(v::HighwayVehicle) = 1.8

pos(v::AbstractVector{Float64}) = v[1:2]
# XXX :/
vel(v::AbstractVector{Float64}) = v[3]
