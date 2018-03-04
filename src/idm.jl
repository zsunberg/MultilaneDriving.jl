@with_kw struct IDM{R<:Real} <: FieldVector{6, R}
   	a::R       # max comfy acceleration
	b::R       # max comfy brake speed
	T::R       # desired safety time headway
	v0::R      # desired speed
	s0::R      # minimum headway (e.g. if x is less than this then you crashed)
	del::R     # accel exponent
end

const NORMAL_IDM =     IDM(1.4, 2.0, 1.5, 33.3, 2.0, 4.0)
const AGGRESSIVE_IDM = IDM(2.0, 3.0, 1.0, 38.9, 0.0, 4.0)
const TIMID_IDM =      IDM(0.8, 1.0, 2.0, 27.8, 4.0, 4.0)

function action(idm::IDM, c, idx, ::Type{LongAccel})
    Δx = headway(c, idx) # positive distance to car in front
    fidx = fore_index(c, idx)
    xdot = vel(state(c, idx))
    if fidx == 0
        xddot = idm.a*(1.0 - (xdot/idm.v0))
    else
        Δxdot = xdot - vel(state(c, fidx))
        xddot = idm.a*(1.0 - (xdot/idm.v0)^idm.delta - (sstar(idm, xdot, Δxdot)/Δx)^2)
    end
    return LongAccel(xddot)
end

sstar(idm::IDM, xdot, Δ̇xdot) = idm.s0 + idm.T*xdot + xdot*Δxdot/(2*sqrt(idm.a*idm.b))
max_acc(idm::IDM) = idm.a
