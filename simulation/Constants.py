import os

ITERATIONS = 500                                    # Number of iterations to run hill climbing
SEED = 56                                           # Random seed
MAX_FLIGHT_TIME = 30                                # Maximum drone flight time in minutes
DRONE_SPEED = 80.5                                  # Drone speed in kilometers per hour
MAX_SIM_TIME = 1000                                 # Maximum number of time steps allowed for simulation to run
NUMBER_OF_DELIVERY_LOCATIONS = 5                    # Number of random delivery locations generated in simulation
TIMESTEP = 60/50                                    # Time step duration in minutes
DEPOT_LOCATION = (40,40)                            # Location of the depot on the grid
file_path = os.path.realpath('results.txt')         # Path of results file