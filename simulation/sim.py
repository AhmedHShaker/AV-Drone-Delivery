import random
import math
import simulation.Constants as Constants


#####################################    CLASSES    #####################################

# Drone to perform deliveries
class Drone:
    def __init__(self):
        self.status = 'at depot or dest'
        self.dir = 'deliver'
        self.flighttime = 0
        self.location = {0:Constants.DEPOT_LOCATION}
        self.lastCar = 0
              
# Autonomous Vehicles (AV) make up the underlying car-sharing infrastructure.
# The drone uses AVs as intermediate transport for long-range deliveries
class AV:
    def __init__(self, initTime, route, name):
        self.initTime = initTime
        self.route = route
        self.name = name


#####################################    FUNCTIONS    #####################################

# Calculates the time required for the drone to travel between 2 locations
def eucDistTime(loc1,loc2):
    return (math.sqrt(((loc1[0]-loc2[0])**2)+((loc1[1]-loc2[1])**2)) / Constants.DRONE_SPEED) * 60

# Calculates the euclidean distance between 2 locations
def eucDist(loc1,loc2):
    return (math.sqrt(((loc1[0]-loc2[0])**2)+((loc1[1]-loc2[1])**2)))

# Returns time to destination and time step reached respectively
def getTimeTaken(d, time, av):
    loc1 = d.location[time]
    i=1
    while (av.initTime <= time+i and time+i < (av.initTime + len(av.route)-1)):
        if eucDistTime(loc1, av.route[time+i-av.initTime]) > i*(Constants.TIMESTEP):
            i+=1
            continue
        else:
            return (2*eucDistTime(loc1, av.route[time+i-av.initTime]) - i*(Constants.TIMESTEP), time+i)
    
    return (10000000,time+i)

# Checks if the drone has enough flight time left to execute the given solution i.e. checks if the solution is feasible
def isFeasible(d, solution, time):
    flighttime = d.flighttime
    for move in solution:
        if move == 'idle':
            time += 1
            
        elif type(move) is tuple:
            distTime = eucDistTime(d.location[time],move)
            if (d.dir == 'deliver' and flighttime + distTime > (Constants.MAX_FLIGHT_TIME/2)) or (d.dir == 'depot' and flighttime + distTime > Constants.MAX_FLIGHT_TIME):
                return False 
            flighttime += distTime
            time += math.ceil(distTime / (Constants.TIMESTEP))
        
        else:
            timeToDest,timeStepReached = getTimeTaken(d, time, move)
            if (d.dir == 'deliver' and flighttime + timeToDest > (Constants.MAX_FLIGHT_TIME/2)) or (d.dir == 'depot' and flighttime + timeToDest > Constants.MAX_FLIGHT_TIME):
                return False
            flighttime += timeToDest
            time = timeStepReached
        
    return True

# Updates the time step reached after the route was modified
def updateTime(d, route, actionAt, time):
    for move in route[0:actionAt]:
        if move == 'idle':
            time += 1
        else:
            _,timeStepReached = getTimeTaken(d, time, move)
            time = timeStepReached    
    
    return time

# Gets the time step reached after the entire route
def timeStepAfterRoute(d, route, time):
    for move in route:
        if move == 'idle':
            time += 1
        elif type(move) is tuple:
            time += math.ceil(eucDistTime(d.location[time],move) / (Constants.TIMESTEP))
        else:
            _,timeStepReached = getTimeTaken(d, time, move)
            time = timeStepReached    
    
    return time

# Returns the flight time required for the solution 
def updateFlightTime(d, best_solution, time):
    flighttime = 0
    for move in best_solution:
        if move == 'idle':
            time += 1
            continue
            
        elif type(move) is tuple:
            flighttime += eucDistTime(d.location[time],move)
            time += math.ceil(eucDistTime(d.location[time],move) / (Constants.TIMESTEP))
            
        else:
            timeToDest,timeStepReached = getTimeTaken(d, time, move)
            flighttime += timeToDest
            time = timeStepReached
    
    return flighttime

# Gets all the available AVs that are currently completing a trip
def getAvailableVehicles(AVlist, time):
    available = []
    for av in AVlist:
        if av.initTime <= time and time < (av.initTime + len(av.route)-1):
            available.append(av)
    
    return available

# Returns the time to distance ratio to evaluate the solution
def timeToDistRatio(d, route, time, origin):
    timetaken = 0
    for move in route:
        if move == 'idle':
            timetaken += Constants.TIMESTEP
            time += 1
            
        elif type(move) is tuple:
            timetaken += eucDistTime(d.location[time],move)
            time += math.ceil(eucDistTime(d.location[time],move) / (Constants.TIMESTEP))
            
        else:  
            timeToDest,timeStepReached = getTimeTaken(d, time, move)
            timetaken += timeToDest
            time = timeStepReached
    
    if len(route) > 0:
        score = (timetaken / eucDist(origin,route[len(route)-1]))
        return score
    else:
        return math.inf

# Applies the move to the solution
def applyMove(route, action, move, actionAt):
    if action == 'add':
        route.insert(actionAt, move)
    elif action == 'swap':
        route[actionAt] = move
    elif action == 'swap dest':
        route[len(route)-1] = move
    else:
        route.pop(actionAt)
    
    return route

# Records the time step and corresponding location of the drone for every move
def updateLocation(d, origin, time, route):
    d.location = {time: origin}
    d.status = 'at depot or dest'
    for move in route:
        if move == 'idle' and d.status == 'at depot or dest':
            d.location[time+1] = d.location[time]
            time += 1
        
        elif move == 'idle' and d.status == 'on car':
            if time - d.lastCar.initTime + 1 < len(d.lastCar.route):
                d.location[time+1] = d.lastCar.route[time-d.lastCar.initTime+1]
            else:
                d.location[time+1] = d.lastCar.route[len(d.lastCar.route)-1]
            time += 1
            
        elif type(move) is tuple:
            timeToDest = math.ceil(eucDistTime(d.location[time],move) / (Constants.TIMESTEP))
            d.location[time + timeToDest] = move
            time += timeToDest
            d.status = 'at depot or dest'
            
        elif type(move) is not tuple:
            _,timeStepReached = getTimeTaken(d, time, move)
            if timeStepReached-move.initTime < len(move.route):
                d.location[timeStepReached] = move.route[timeStepReached-move.initTime]
            else:
                d.location[time+1] = d.lastCar.route[len(d.lastCar.route)-1]
                
            time = timeStepReached
            d.lastCar = move
            d.status = 'on car'
            
# Checks if the location is within range of the drone to travel to and back to the depot within the drone's flight time
def withinRange(d,loc,starttime):
    
    distTime = eucDistTime(d.location[starttime],loc)
    if (d.dir == 'deliver' and distTime > (Constants.MAX_FLIGHT_TIME/2)) or (d.dir == 'depot' and d.flighttime + distTime > Constants.MAX_FLIGHT_TIME):
        return False
    return True

# Swaps two random moves in the solution
def randomSwap(route):
    
    a = random.randint(0,len(route)-2)
    i = random.randint(0,len(route)-2)
    its = 10
    
    while its > 0:
        if type(route[i]) is AV:
            b = route[i]
            route[i] = route[a]
            route[a] = b
            break
        else:
            its-=1
            i = random.randint(0,len(route)-2)
            continue
    
    return route

# Returns move to AV or Idle move
# Halves probability of returning a move to AV for every move to AV that already exists in the solution
def moveProb(route, AVlist, time):
    prob = 0.5
    for move in route:
        if type(move) is AV:
            prob = prob/2.0
    
    r = random.random()
    if r < prob:
        return random.choice(getAvailableVehicles(AVlist, time))
    else:
        return 'idle'

# Runs a modified hill climbing algorithm to find the best solution
def hillClimbing(d, origin, dests, AVlist, time):
    route = [dests[0]]
    for _ in range(random.randint(7,15)):
        route.insert(0,'idle')
    best = []
    startTime = time
    iterations = 0
    best_ratio = math.inf

    while iterations < Constants.ITERATIONS:
        
        if len(route) > 1 and len(dests) > 1:
            action = random.choice(['add','swap','swap dest','remove'])
            actionAt = random.randint(0,len(route)-2)
            
        elif len(route) > 1 and len(dests) == 1:
            action = random.choice(['add','swap','remove'])
            actionAt = random.randint(0,len(route)-2)
        else:
            action = 'add'
            actionAt = 0
        
        if action == 'swap dest':
            move = random.choice(dests)
        
        elif action == 'add' or action == 'swap':
            time = updateTime(d, route, actionAt, startTime)
            
            if len(getAvailableVehicles(AVlist, time)) == 0:
                move = 'idle'
            else:
                move = moveProb(route, AVlist, time)
            if type(move) is AV and type(route[actionAt-1]) is AV and move is route[actionAt-1]:
                move = 'idle'
            
            if move == 'idle' and actionAt > 0 and type(route[actionAt-1]) is AV and len(route[actionAt-1].route) + route[actionAt-1].initTime - 2 <= time:
                continue
        else:
            move = 0
        
        if move == 'idle' and action == 'swap' and route[actionAt] == 'idle':
            continue
        route = applyMove(route, action, move, actionAt)
            
        if len(route) > 3:
            for _ in range(len(route)):
                if _ > 0:
                    route = randomSwap(route)
                updateLocation(d, origin, startTime, route)
                
                if isFeasible(d, route, startTime):
                    if len(best) == 0 or timeToDistRatio(d, route, startTime, origin) < best_ratio:
                        best = route
                        best_ratio = timeToDistRatio(d, best, startTime, origin)
                        return best
        iterations += 1
    return best

# Creates a route from origin to destination
def createRoute(origin, dest):

    route = [origin]
    if origin[0] < dest[0] and origin[1] < dest[1]:
        first = random.choice(['right','up'])
        if first == 'right':
            second = 'up'
        else:
            second = 'right'
    elif origin[0] < dest[0] and origin[1] > dest[1]:
        first = random.choice(['right','down']) 
        if first == 'right':
            second = 'down'
        else:
            second = 'right'
    elif origin[0] < dest[0] and origin[1] == dest[1]:
        second = 'right'

    elif origin[0] > dest[0] and origin[1] > dest[1]:
        first = random.choice(['left','down'])
        if first == 'left':
            second = 'down'
        else:
            second = 'left'
    elif origin[0] > dest[0] and origin[1] < dest[1]:
        first = random.choice(['left','up'])
        if first == 'left':
            second = 'up'
        else:
            second = 'left'
    elif origin[0] > dest[0] and origin[1] == dest[1]:
        second = 'left'

    elif origin[0] == dest[0] and origin[1] > dest[1]:
        second = 'down'
    elif origin[0] == dest[0] and origin[1] == dest[1]:
        return []
    elif origin[0] == dest[0] and origin[1] < dest[1]:
        second = 'up'
    
    while origin[0] != dest[0] and origin[1] != dest[1]:
        
        if first == 'up':
            y = list(origin)
            y[1] += 1
            origin = tuple(y)
            route.append(origin)
        elif first == 'down':
            y = list(origin)
            y[1] -= 1
            origin = tuple(y)
            route.append(origin)
        elif first == 'right':
            y = list(origin)
            y[0] += 1
            origin = tuple(y)
            route.append(origin)
        elif first == 'left':
            y = list(origin)
            y[0] -= 1
            origin = tuple(y)
            route.append(origin)
    
    while origin[0] != dest[0] or origin[1] != dest[1]:
        
        if second == 'up':
            y = list(origin)
            y[1] += 1
            origin = tuple(y)
            route.append(origin)
        elif second == 'down':
            y = list(origin)
            y[1] -= 1
            origin = tuple(y)
            route.append(origin)
        elif second == 'right':
            y = list(origin)
            y[0] += 1
            origin = tuple(y)
            route.append(origin)
        elif second == 'left':
            y = list(origin)
            y[0] -= 1
            origin = tuple(y)
            route.append(origin)
    return route

# Generates AV trips, their routes, their initial start times, and the drone delivery points
def generateTripsAndTime():
    random.seed(Constants.SEED)
    trips = []
    for _ in range(10000):
        trips.append(( (random.randint(0,80),random.randint(0,80)) , (random.randint(0,80),random.randint(0,80)) ))
    
    routes = []
    for i in range(10000):
        route = createRoute(trips[i][0], trips[i][1])
        if len(route) != 0:
            routes.append(route)
    
    newTripTimes = []
    
    for _ in range(1000):
        newTripTimes.append(random.randint(0,1000))
    
    newTripTimes = list(set(newTripTimes))
    initialTrips = []
    initialRoutes = []
    for i in range(50):
        initialTrips.append(( (random.randint(0,80),random.randint(0,80)) , (random.randint(0,80),random.randint(0,80)) ))
        initialRoutes.append(createRoute(initialTrips[i][0], initialTrips[i][1]))
    
    deliverTo = []
    for _ in range(Constants.NUMBER_OF_DELIVERY_LOCATIONS):
        deliverTo.append((random.randint(0,80),random.randint(0,80)))
    
    return (routes, newTripTimes, initialRoutes,deliverTo)

# Prints the actions taken by the drone
def printOutput(drone, origin, time, best_solution, deliverTo, toDepot):
    updateLocation(drone, origin, time, best_solution)
    locKeys = list(drone.location.keys())
    locVals = list(drone.location.values())
    print('MOVES:')
    print('Time step %s: Origin %s.' % (locKeys[0],locVals[0]))
    for (t,loc,move) in zip(locKeys[1:],locVals[1:],best_solution):
        if type(move) is tuple:
            print('Time step %s: Arrived at %s.' % (t,loc))
        elif type(move) is AV:
            print('Time step %s: Arrived on vehicle %s.' % (t,move.name))
        else:
            print('Time step %s: Remained %s in location %s.' % (t,move,loc))
    print()
    if toDepot:
        print("Arrived at depot successfully")
    else:
        print("Delivery completed successfully for",best_solution[len(best_solution)-1])
        print('Delivery locations remaining',deliverTo)


#####################################    SIMULATION    #####################################


# Runs the simulation
def sim():
    time = 0
    reachTime = 0
    startTimeBeforeRoute = 0
    AVcounter = 0
    locAtStart = Constants.DEPOT_LOCATION
    drone = Drone()
    inRange = []
    outOfRange = []
    best_solution = []
    AVlist = []
    
    newRoutes, newTripTimes, initialRoutes, deliverTo = generateTripsAndTime()
    print('Delivery locations:', deliverTo)
    
    # Create initial trips and AVs
    for r in initialRoutes:
        AVlist.append(AV(0,r,'AV'+str(AVcounter)))
        AVcounter += 1
    
    while time < Constants.MAX_SIM_TIME:
        
        # Vehicles that finished their trips are removed
        for av in AVlist:
            if time >= av.initTime + len(av.route) - 1:
                AVlist.remove(av)
    
        if time in newTripTimes:
            # Adds a new vehicle with a trip
            AVlist.append(AV(time,newRoutes.pop(0),'AV'+str(AVcounter)))
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
                print()
                print()
                print('Time step',time)
                print('Cars available',len(AVlist))
                print('Location is out of drone range. Drone will move to',drone.dir,'at',best_solution[len(best_solution)-1])
            
            # Drone travels to the closest within range delivery point
            else:
                best_dest = min(inRange,key=lambda dest:eucDist(origin,dest),default=[])
                best_solution = [best_dest]
                if len(best_dest) > 0:
                    print()
                    print()
                    print('Time step',time)
                    print('Cars available',len(AVlist))
                    print('Location is within drone range. Drone will move to',drone.dir,best_solution[len(best_solution)-1])
                        
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
                    print()
                    print('###############################################################################')
                    print()
                    print('ALL DELIVERIES COMPLETED AT TIME STEP', startTimeBeforeRoute)
                    break
            
            # Drone has failed to make a move in this time step
            else:
                reachTime = -1
                locAtStart = drone.location[time]        
        time += 1
sim()
