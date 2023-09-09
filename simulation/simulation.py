import Constants as Constants
from models import Drone
from utils import *

# Runs the simulation
def simulate():
    time = 0
    reachTime = 0
    startTimeBeforeRoute = 0
    AVcounter = 0
    locAtStart = Constants.DEPOT_LOCATION
    drone = Drone.Drone()
    inRange = []
    outOfRange = []
    best_solution = []
    AVlist = []
    
    newRoutes, newTripTimes, initialRoutes, deliverTo = generateTripsAndTime()
    printStart(deliverTo)
    
    # Create initial trips and AVs
    for r in initialRoutes:
        AVlist.append(AV.AV(0,r,'AV'+str(AVcounter)))
        AVcounter += 1
    
    # Simulate each time step
    while time < Constants.MAX_SIM_TIME:
        # Vehicles that finished their trips are removed
        for av in AVlist:
            if time >= av.initTime + len(av.route) - 1:
                AVlist.remove(av)
    
        if time in newTripTimes:
            # Adds a new vehicle with a trip
            AVlist.append(AV.AV(time,newRoutes.pop(0),'AV'+str(AVcounter)))
            AVcounter += 1
        
        # time == reachTime is when the drone reaches its destination
        # reachTime == -1 is if the drone failed to make a move during the last time step
        if time == reachTime or reachTime == -1:
            if reachTime == -1:
                drone.location = {time: locAtStart}
            elif time > 0:
                drone.location[time] = best_solution[len(best_solution)-1]
            else:
                drone.location = {time:Constants.DEPOT_LOCATION}
                drone.dir = 'depot'
    
            inRange = []
            outOfRange = []
            
            # Either the drone is in the 'depot' and it switches to 'deliver' and gets the out of range and within range delivery points
            # Or the drone has failed to deliver in the previous time step and it tries another time
            if drone.dir == 'depot' or (reachTime == -1 and drone.dir == 'deliver'):
                if reachTime != -1:
                    drone.dir = 'deliver'
                drone.flighttime = 0
                
                for dest in deliverTo:
                    if withinRange(drone,dest,time):
                        inRange.append(dest)
                    else:
                        outOfRange.append(dest)
            
            # Either the drone is in the 'deliver' and it switches to 'depot' and determines if depot is out of range or within range
            # Or the drone has failed to move towards the depot in the previous time step and it tries another time
            elif drone.dir == 'deliver' or (reachTime == -1 and drone.dir == 'depot'):
                dest = Constants.DEPOT_LOCATION
                if reachTime != -1:
                    drone.dir = 'depot'
                    updateLocation(drone, drone.location[startTimeBeforeRoute], startTimeBeforeRoute, best_solution)
                    ftime = updateFlightTime(drone, best_solution, startTimeBeforeRoute)
                
                drone.flighttime = ftime
                if withinRange(drone,dest,time):
                    inRange.append(dest)
                else:
                    outOfRange.append(dest)
            
            origin = drone.location[time]
            best_solution = []
    
            # Hill climbing is run for out of range delivery points
            if len(outOfRange) > 0:
                best_solution = hillClimbing(drone, origin, outOfRange, AVlist, time)
    
            # Solution is found for out of range delivery point
            if len(best_solution) > 0:
                printPreTripOutput(False, time, len(AVlist), drone.dir, best_solution[len(best_solution)-1])
            
            # Drone travels to the closest within range delivery point
            else:
                best_dest = min(inRange,key=lambda dest:eucDist(origin,dest),default=[])
                best_solution = [best_dest]
                if len(best_dest) > 0:
                    printPreTripOutput(True, time, len(AVlist), drone.dir, best_solution[len(best_solution)-1])
                        
            # Drone is going to a delivery point
            if best_solution[len(best_solution)-1] in deliverTo: 
                deliverTo.remove(best_solution[len(best_solution)-1])
                startTimeBeforeRoute = time
                printOutput(drone, origin, time, best_solution, deliverTo, False)
                reachTime = timeStepAfterRoute(drone,best_solution, time)
             
            # Drone is going to depot    
            elif best_solution[len(best_solution)-1] == Constants.DEPOT_LOCATION:
                printOutput(drone, origin, time, best_solution, deliverTo, True)
                startTimeBeforeRoute = time
                reachTime = timeStepAfterRoute(drone,best_solution, time)                
                if len(deliverTo) == 0:
                    printEnd(startTimeBeforeRoute)
                    break
            
            # Drone has failed to make a move in this time step
            else:
                reachTime = -1
                locAtStart = drone.location[time]        
        time += 1

simulate()