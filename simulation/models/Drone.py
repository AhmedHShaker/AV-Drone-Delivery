from Constants import DEPOT_LOCATION

# Drone to perform deliveries
class Drone:
    def __init__(self):
        self.status = 'at depot or dest'
        self.dir = 'deliver'
        self.flighttime = 0
        self.location = {0:DEPOT_LOCATION}
        self.lastCar = 0