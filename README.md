# AV-Drone-Delivery
This project is all about simulating a drone delivery system that partners up with car-sharing Autonomous Vehicles (AVs) to tackle out-of-range deliveries.

## The Setup
* **Grid Structure:** In this project, we operate within an 80x80 kilometer grid, where each square corresponds to a distance of 1 kilometer. 
* **Autonomous Vehicles (AVs):** Our fleet of AVs traverses this grid, maintaining a constant speed, advancing one cell per time step, and executing trips generated randomly.
* **Drones:** Now, the exciting part is the drones. They start their journey from a base depot, fly to delivery spots, and drop off packages. But here's the twist: drones can only fly for a limited time before needing to recharge. To get around this, they can hop on an AV to get closer to deliveries that are far away. After each delivery, they return to the depot for a recharge.

## The Simulation
Let's talk strategy. The drone's route consists of three types of moves:
* Flying to the destination (delivery point or depot)
* Catching a ride on an AV
* Stay idle

When the drone isn't airborne, it has to make a move during each time step. For instance, if it's riding on an AV, it might stay idle for a bit until it gets closer to its destination.

Our goal is simple: minimize the number of time steps needed to wrap up all deliveries. That's where our stochastic hill climbing algorithm comes into play. It tweaks our delivery plan to get us as close as possible to an optimal solution.

## Getting Started
Ready to dive in? Follow these steps:
1) Clone the repository.
2) If you're feeling adventurous, tweak the default settings in ```Constants.py```.
3) Start the simulation by running ```simulation.py```.

Have fun exploring and experimenting with the AV-Drone-Delivery system! 📦🚗🚁
