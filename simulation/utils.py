import random
import math
import Constants as Constants
from models import AV

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
        if type(route[i]) is AV.AV:
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
        if type(move) is AV.AV:
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
            if type(move) is AV.AV and type(route[actionAt-1]) is AV.AV and move is route[actionAt-1]:
                move = 'idle'
            
            if move == 'idle' and actionAt > 0 and type(route[actionAt-1]) is AV.AV and len(route[actionAt-1].route) + route[actionAt-1].initTime - 2 <= time:
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
def printPreTripOutput(withinRange, time, carsAvailable, droneDirection, loc):
    with open(Constants.file_path, mode="a") as file:
        if withinRange:
            file.write('\n')
            file.write('Starting time step: %d\n' % time)
            file.write('Cars available: %d\n' % carsAvailable)
            file.write('Location is within drone range. Drone will start move to %s %s\n'% (droneDirection,loc))
        else:
            file.write('\n')
            file.write('Starting time step: %d\n' % time)
            file.write('Cars available: %d\n'% carsAvailable)
            file.write('Location is out of drone range. Drone will start move to %s at %s\n' % (droneDirection,loc))


# Prints the actions taken by the drone
def printOutput(drone, origin, time, best_solution, deliverTo, toDepot):
    updateLocation(drone, origin, time, best_solution)
    locKeys = list(drone.location.keys())
    locVals = list(drone.location.values())
    with open(Constants.file_path, mode="a") as file:
        file.write('Moves taken at each time step:\n')
        file.write('Time step %s: Origin %s.\n' % (locKeys[0],locVals[0]))
        for (t,loc,move) in zip(locKeys[1:],locVals[1:],best_solution):
            if type(move) is tuple:
                file.write('Time step %s: Arrived at %s.\n' % (t,loc))
            elif type(move) is AV.AV:
                file.write('Time step %s: Arrived on vehicle %s.\n' % (t,move.name))
            else:
                file.write('Time step %s: Remained %s in location %s.\n' % (t,move,loc))
        file.write('\n')
        if toDepot:
            file.write("Arrived at depot successfully\n")
            if len(deliverTo) > 0:
                file.write('Delivery locations remaining %s\n' % deliverTo)
            file.write('_____________________________________________________________\n')
        else:
            file.write('Delivery completed successfully for %s\n' % str(best_solution[len(best_solution)-1]))
            file.write('Starting trip back to depot...\n')

# Prints starting text
def printStart(deliverTo):
    print('Printing to results.txt...')
    with open(Constants.file_path, mode="w") as file:
        file.write('Delivery locations: %s\n'% deliverTo)

# Prints ending text
def printEnd(time):
    with open(Constants.file_path, mode="a") as file:
        file.write('\n')
        file.write('ALL DELIVERIES COMPLETED AT TIME STEP %d\n' % time)