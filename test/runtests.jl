using MultilaneDriving
using Base.Test
using POMDPToolbox

using StaticArrays
using POMDPs

using Gallium
using Traceur

rng = MersenneTwister(7)

dt = 0.1

driver = NoCrashMLDriver(dt)

car = HighwayVehicle()
carstate = SVector(0.0, 0.0, 20.0)
carstate2 = SVector(100.0, 0.0, 20.0)

road = Highway(3, 3.0, 100.0)

context = HighwayDrivingContext(car, [carstate, carstate2], road)

# @trace action_distribution(driver, MLDriverState(context, nothing, context), 1, AccelSpeed)
# @code_warntype action(driver.long, context, 1, LongAccel)
ad = @inferred action_distribution(driver, MLDriverState(context, nothing, context), 1, AccelSpeed)

a = rand(MersenneTwister(4), ad)

propagate(car, carstate, a, context, dt)

s = MLDState([carstate, SVector(50.0, 0.0, 20.0)], [driver], [MLDriverState(context, nothing, context)])
p = MLDPOMDP()
sp, r = @inferred generate_sr(p, s, AccelSpeed(1.0, 0.1), MersenneTwister(5))

as = @inferred actions(p, sp)
a = @inferred rand(rng, as)

# @show [rand(rng, as) for i in 1:10]
# hr = HistoryRecorder(max_steps=10, rng=MersenneTwister(4))
# simulate(hr, p, RandomPolicy(p, rng=MersenneTwister(5)))
