module MultilaneDriving

using StaticArrays
using Distributions
using Parameters
using AutomotiveDrivingModels
using Base.Test

importall POMDPs

import Distributions: rand, pdf

export
    NoCrashMLDriver,
    MLDriverState,
    HighwayVehicle,
    HighwayDrivingContext,
    Highway,
    AccelSpeed,
    LongAccel,

    MLDPOMDP,
    MLDState,
    MLDObs,

    action_distribution,
    propagate

# package code goes here
include("cached.jl")
include("copula.jl")

include("roadway.jl")

include("vehicles.jl")
include("contexts.jl")
include("safety.jl")
include("idm.jl")
include("drivers.jl")

include("pomdp.jl")

end # module
