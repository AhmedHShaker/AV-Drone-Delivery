# AV-Drone-Delivery
This project simulates a drone delivery system that uses an underlying car-sharing Autonomous Vehicle (AV) infrastructure as intermediate transport to perform out-of-range deliveries.

## Model
The car-sharing AV system performs trips that are generated randomly and operates in an 80-by-80 kilometer grid, where each square cell has a length of 1 kilometer. The AVs are assumed to travel at a constant speed and require one time step to move from one cell to another.

The drone starts its journey from the depot and flies to delivery points to complete orders. Since the drone is limited by a maximum flight time, it can temporarily ride on top of AVs to move closer to out-of-range delivery locations. After the order is delivered, the drone returns to the depot to swap its depleted battery.

## Simulation
The drone's route from origin to destination is made up of a combination of multiple moves. There are three possible moves:
1) Move to the destination (delivery point or depot)
2) Move to an AV
3) Stay idle

While the drone is not flying, it is required to perform a move during every time step. For example, if the drone is on top of an AV it might remain idle for a couple of time steps until it is close enough to the destination.

The objective is to minimize the number of time steps required to complete all deliveries. Hence, a stochastic hill climbing algorithm is implemented to iteratively make changes to the moves in a solution until a near-optimal solution is achieved.

## Getting Started
1) Download the repository
2) OPTIONAL: Change the default values in ```Constants.py```
3) Run ```sim.py``` to start the simulation
