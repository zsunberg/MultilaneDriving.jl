@with_kw struct IDM{R<:Real} <: FieldVector{6, R}
   	a::R       # max comfy acceleration
	b::R       # max comfy brake speed
	T::R       # desired safety time headway
	v0::R      # desired speed
	s0::R      # minimum headway (e.g. if x is less than this then you crashed)
	delta::R     # accel exponent
end

const NORMAL_IDM =     IDM(1.4, 2.0, 1.5, 33.3, 2.0, 4.0)
const AGGRESSIVE_IDM = IDM(2.0, 3.0, 1.0, 38.9, 0.0, 4.0)
const TIMID_IDM =      IDM(0.8, 1.0, 2.0, 27.8, 4.0, 4.0)

function action(idm::IDM, c, idx, ::Type{LongAccel})
    fidx = fore_index(c, idx)
    xdot = vel(state(c, idx))
    if fidx == 0
        xddot = idm.a*(1.0 - (xdot/idm.v0))
    else
        delta_x = headway(c, idx) # positive distance to car in front
        delta_xdot = xdot - vel(state(c, fidx))
        ss = sstar(idm, xdot, delta_xdot)
        xddot = idm.a*(1.0 - (xdot/idm.v0)^idm.delta - (ss/delta_x)^2)
    end
    return LongAccel(xddot)
end

sstar(idm::IDM, xdot, delta_xdot) = idm.s0 + idm.T*xdot + xdot*delta_xdot/(2*sqrt(idm.a*idm.b))
max_acc(idm::IDM) = idm.a

struct IDMGaussianCopula
    gc::GaussianCopula
    timid::IDM
    aggressive::IDM
end

IDMGaussianCopula(rho::Float64) = IDMGaussianCopula(GaussianCopula(length(IDM), rho), TIMID_IDM, AGGRESSIVE_IDM)

function rand(rng::AbstractRNG, d::IDMGaussianCopula)
    return d.timid + (d.aggressive-d.timid).*rand(rng(d.gc))
end
