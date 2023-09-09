# Autonomous Vehicles (AV) make up the underlying car-sharing infrastructure.
# The drone uses AVs as intermediate transport for long-range deliveries
class AV:
    def __init__(self, initTime, route, name):
        self.initTime = initTime
        self.route = route
        self.name = name