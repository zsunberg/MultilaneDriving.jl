using MultilaneDriving
using Base.Test

using StaticArrays
using POMDPs

using Gallium

dt = 0.1

driver = NoCrashMLDriver(dt)

car = HighwayVehicle()
carstate = SVector(0.0, 0.0, 20.0)

road = Highway(3, 3.0, 100.0)

context = HighwayDrivingContext(car, [carstate], road)

ad = action_distribution(driver, MLDriverState(context, nothing, context), 1, AccelSpeed)

a = rand(MersenneTwister(4), ad)

propagate(car, carstate, a, context, dt)

s = MLDState([carstate, SVector(50.0, 0.0, 20.0)], [driver], [MLDriverState(context, nothing, context)])
p = MLDPOMDP{typeof(s), typeof(carstate)}(road, car, dt)
sp = generate_s(p, s, AccelSpeed(1.0, 0.1), MersenneTwister(5))
