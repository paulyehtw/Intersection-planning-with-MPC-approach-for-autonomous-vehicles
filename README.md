# Intersection-planning-with-MPC-approach-for-autonomous-vehicles
![alt text](https://github.com/paulyehtw/Intersection-planning-with-MPC-approach-for-autonomous-vehicles/blob/master/Simulation_GIF.gif)


The intersection planning problem can be divided into two parts : priority assignment problem and  vehicle control problem. This project assumes that it the priority is governed by the European right hand driving traffic rules, which means vehicles must defer to vehicles arriving on the road to its right. So here only the vehicle control problem is tackled and the Model Predictive Control approach is implemented.

Intersection scenario setting : 
An ego will show up at the left lane of the intersection at random position and with random velocity between 5 m/s and 20 m/s. 
A prior car (the car arriving from the right) will show up also at the right lane to ego car at random position and with random velocity.

Constraints subjected to the MPC cost function : 
1. The ego car must drive with velocity in range of [0, 20] m/s
2. The ego car must drive with acceleration in range of [-2, 2] m/s^2
3. The change of acceleration must be within [-0.2, 0.2] m/s^2 between each time step for the sake of driving comfort
4. A quadratic cost function is chosen for the MPC as follows : Cv(vel_target - vel_next)^2 + Ca(acc^2), where vel_target is set to 20 m/s, Cv and Ca are factors for tuning

Simulation settings : 
1. Time step is 0.1 second
2. Prediction horizon for the MPC is 50 steps (5 seconds)
3 Timeout threshold is 150 steps if the ego car does not reach goal.

Simulation results over 10000 episodes:
1. The ego car doesnt yield to the prior car 2531 times in 10000 episodes(25%),  it is because the initial position is too close to the intersection or initial velocity is too fast, the controller cannot stop the car if constraints are obeyed.
2. The ego car timeouted 103 times in 10000 episodes(1%) because the prior car is too slow and ego needs to wait for it to pass.
3. The efficiency of the algorithm is evaluated, on average the ego car needs 1.8 steps to travel 1 meter.
4. The comfort index is evaluated, on average the acceleration is 1.6 m/s^2
