abstract type Context end

struct HighwayDrivingContext{VM, VS} <: Context
    vehicle_model::VM
    vehicle_states::Vector{VS}
    roadway::Highway

    _fore_indices::Cached{Vector{Int}}
end

HighwayDrivingContext(vm, vss, rw) = HighwayDrivingContext(vm, vss, rw, Cached{Vector{Int}}())

# public interface
function headway(c::HighwayDrivingContext, idx)
    fidx = fore_index(c, idx)
    if fidx == 0
        return Inf
    else
        s = c.vehicle_states[idx]
        v = c.vehicle_model
        fs = c.vehicle_states[fidx]
        fv = c.vehicle_model
        return pos(fs)[1] - pos(s)[1] - length(v)/2 - length(fv)/2
    end
end

fore_index(c::HighwayDrivingContext, idx) = fore_indices(c)[idx]

vehicle(c::HighwayDrivingContext, idx) = c.vehicle_model
state(c::HighwayDrivingContext, idx) = c.vehicle_states[idx]
n_vehicles(c::HighwayDrivingContext) = length(c.vehicle_states)

"Return a 2-tuple containing at most two lanes that the car is in."
function occupied_lanes(c::HighwayDrivingContext, idx)
    vs = c.vehicle_states[idx]
    vm = c.vehicle_model
    h = c.roadway
    y = pos(vs)[2]
    l = y + width(vm)/2
    r = y - width(vm)/2
    lo = clamp(ceil(Int, r/h.lanewidth), 1, h.n_lanes)
    hi = clamp(ceil(Int, l/h.lanewidth), 1, h.n_lanes)
    return (lo, hi)
end

"Iterator over each car index."
indexes(c::HighwayDrivingContext) = 1:n_vehicles(c)

# less-public interface
# function neighborhood(c::HighwayDrivingContext, idx)
#     ns = get(_calc_neighborhoods, c.neighborhoods, c)
#     return ns[idx]
# end

function fore_index(c::HighwayDrivingContext, idx)
    fis = get(c._fore_indices, c) do c
        # indices for cars in each lane sorted by x
        carsin = [Int[] for i in 1:c.roadway.n_lanes]

        # for each car, insert it into the lane list while maintaining order
        for i in indexes(c)
            l1, l2 = occupied_lanes(c, i)
            spot = searchsortedfirst(carsin[l1], i, by=ind->pos(c.vehicle_states[ind])[1])
            insert!(carsin[l1], spot, i)
            if l2 != l1
                spot = searchsortedfirst(carsin[l2], i, by=ind->pos(c.vehicle_states[ind])[1])
                insert!(carsin[l2], spot, i)
            end
        end

        # sweep through the lanes and construct the fore indices
        fore_indices = zeros(Int, n_vehicles(c))
        for l in 1:c.roadway.n_lanes
            for j in 1:length(carsin[l])-1
                fore_indices[carsin[l][j]] = carsin[l][j+1]
            end
        end
        return fore_indices
    end
    return fis[idx]
end


# internal
function _calc_neighborhoods

end
