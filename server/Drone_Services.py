import sys, ast, math
import time
import threading
import dronekit

altitude = 10+5.3

"""
	Connect to the number of vehicles spesified as command line arguments
"""
def initialize(vehicles, UAV_BASE_PORT, server):
	t = threading.Thread(target=lambda:initialize_in_new_thread(vehicles, UAV_BASE_PORT, server))
	t.daemon = True
	t.start()


def initialize_in_new_thread(vehicles, UAV_BASE_PORT, server):

	# Number of simulated drones available at startup given as a command line argument
	instance_count = 0
	if (len(sys.argv) > 1 ):
		instance_count = int(sys.argv[1])
	
	for instance_index in range(instance_count):
		
		port = UAV_BASE_PORT + 10 * instance_index
		connection_string = "127.0.0.1:%i" % port #connect to port on localhost

		# Connect to the Vehicles
		print('Connecting to vehicle %i on: %s' % (instance_index, connection_string))
		while True:
			try:
				vehicle = dronekit.connect(connection_string, wait_ready=True)
			except Exception as e:
				print(e)
				time.sleep(1)
			else:
				vehicle.id = instance_index
				vehicle.max_speed = 5
				vehicle.current_battery = 600
				vehicle.max_battery_time = 600
				vehicle.max_carry_weight = 10
				vehicle.start_up_time = time.time()
				vehicle.nextlocations = []
				vehicle.groundspeed = 50
				vehicle.airspeed = 40
				vehicles.append(vehicle)
				break

	start_data_updating(vehicles, server)
	do_for_all(vehicles, lambda v:arm_and_takeoff(v, 10, server))


"""
	Start a new simulated drone
"""
def start_new_simulated_drone(vehicles, UAV_BASE_PORT, server):
	t = threading.Thread(target=lambda:start_new_simulated_drone_in_new_thread(vehicles, UAV_BASE_PORT, server))
	t.daemon = True
	t.start()

def start_new_simulated_drone_in_new_thread(vehicles, UAV_BASE_PORT, server):

		port = UAV_BASE_PORT + 10 * len(vehicles)
		connection_string = "127.0.0.1:%i" % port #connect to port on localhost
		
		# Connect to the Vehicles
		print('Connecting to vehicle %i on: %s' % (len(vehicles), connection_string))
		server.send_message_to_all("[\"-1\", \"Connecting to vehicle...\"]")
		
		while True:
			try:
				vehicle = dronekit.connect(connection_string, wait_ready=True)
			except Exception as e:
				print(e)
				time.sleep(1)
			else:
				vehicle.id = len(vehicles)
				vehicle.max_speed = 5
				vehicle.current_battery = 600
				vehicle.max_battery_time = 600
				vehicle.max_carry_weight = 10
				vehicle.start_up_time = time.time()
				vehicle.nextlocations = []
				vehicles.append(vehicle)
				t = threading.Thread(target=lambda:start_data_updating_thread(vehicle, server))
				t.daemon = True
				t.start()
				arm_and_takeoff(vehicle, 10, server)
				break
				

"""
	Arms vehicle and fly to aTargetAltitude.
"""
def arm_and_takeoff(vehicle, aTargetAltitude, server):

	print("Vehicle %i: Basic pre-arm checks" % vehicle.id)
	# Don't try to arm until autopilot is ready
	print("Vehicle %i: Waiting for vehicle to initialise..."  % vehicle.id)
	server.send_message_to_all("[\"-1\", \"Waiting for vehicle to initialise...\"]")
	while not vehicle.is_armable:
		time.sleep(1)

	print("Vehicle %i: Arming motors" % vehicle.id)
	# Copter should arm in GUIDED mode
	vehicle.mode = dronekit.VehicleMode("GUIDED")
	vehicle.armed = True    

	# Confirm vehicle armed before attempting to take off
	print("Vehicle %i: Waiting for arming..." % vehicle.id)
	server.send_message_to_all("[\"-1\", \"Waiting for arming...\"]")
	while not vehicle.armed:      
		time.sleep(1)

	print("Vehicle %i: Taking off!" % vehicle.id)
	server.send_message_to_all("[\"-1\", \"Taking off!\"]")
	vehicle.simple_takeoff(aTargetAltitude) # Take off to target altitude

	# Wait until the vehicle reaches a safe height before processing the goto (otherwise the command 
	#  after Vehicle.simple_takeoff will execute immediately).
	while True:
		print("Vehicle %i: Altitude: " % vehicle.id, vehicle.location.global_relative_frame.alt)
		#Break and return from function just below target altitude.        
		if vehicle.location.global_relative_frame.alt>=aTargetAltitude*0.95: 
			print("Vehicle %i: Reached target altitude" % vehicle.id)
			server.send_message_to_all("[\"-1\", \"Reached target altitude\"]")
			break
		time.sleep(1)


"""
Start to send vehicle information to client
"""
def start_data_updating(vehicles, server):
	for vehicle in vehicles:
		t = threading.Thread(target=lambda:start_data_updating_thread(vehicle, server))
		t.daemon = True
		t.start()


def start_data_updating_thread(vehicle, server):
	while True:
		vehicle_id = vehicle.id
		vehicle_position = [vehicle.location.global_frame.lat, vehicle.location.global_frame.lon]
		
		# Flight data
		vehicle_altitude = vehicle.location.global_relative_frame.alt
		vehicle_speed = vehicle.groundspeed
		vehicle_heading = vehicle.heading
		vehicle_current_battery = vehicle.current_battery - (time.time() - vehicle.start_up_time)

		#if(vehicle_current_battery <= 0):
			#vehicle.close()

		# Parameters
		vehicle_max_speed = vehicle.max_speed
		vehicle_max_battery_time = vehicle.max_battery_time
		vehicle_max_carry_weight = vehicle.max_carry_weight

		server.send_message_to_all(repr([
			vehicle_id,
			vehicle_position,
			vehicle_altitude,
			vehicle_speed,
			vehicle_heading,
			vehicle_current_battery,
			vehicle_max_speed,
			vehicle_max_battery_time,
			vehicle_max_carry_weight
			]))
		time.sleep(0.1)


def do_for_all(vehicles, function):
	threads = [threading.Thread(target=lambda v=v:function(v)) for v in vehicles]
	for t in threads:
		t.start()
	for t in threads:
		t.join()


def find_vehicle_by_id(vehicles, id):
	for vehicle in vehicles:
		if vehicle.id == id:
			return vehicle


"""
	Calculate the distance between 2 points
"""
def distance(location1, location2):
	(x1,y1) = location1.lat, location1.lon
	(x2,y2) = location2.lat, location2.lon
	try:
		d = math.acos(math.sin(math.radians(x1))*math.sin(math.radians(x2)) + math.cos(math.radians(x1))*math.cos(math.radians(x2))*math.cos(math.radians(y2-y1)))*6378.137
	except ValueError:
		print("ValueError in distance calculation for ", x1,y1, "to", x2,y2)
		d = 0.0
	return d


"""
	Functions to check if two lines intersect
"""

#Given three colinear points (x1, y1), (x2, y2), (x3, y3), the function checks if 
#point (x2, y2) lies on line segment (x1,y1)(x3,y3)
def onSegment(x1, y1, x2, y2, x3, y3):
	if x2 <= max(x1, x3) and x2 >= min(x1, x3) and y2 <= max(y1, y3) and y2 >= min(y1, y3):
		return True
	return False

def orientation(x1, y1, x2, y2, x3, y3):
	value = (y2 - y1) * (x3 - x2) - (x2 - x1) * (y3 - y2)

	if value == 0:
		return 0
	
	return 1 if value > 0 else 2

def doIntersect(p1, q1, p2, q2):
	o1 = orientation(p1.lat, p1.lon, q2.lat, q2.lon, p2.lat, p2.lon)
	o2 = orientation(p1.lat, p1.lon, q1.lat, q1.lon, q2.lat, q2.lon)
	o3 = orientation(p2.lat, p2.lon, q2.lat, q2.lon, p1.lat, p1.lon)
	o4 = orientation(p2.lat, p2.lon, q2.lat, q2.lon, q1.lat, q1.lon)

	if o1 != o2 and o3 != o4:
		return True

	return False

def is_on_line(drone, location1, location2): 
	return distance(location1, drone) + distance(location2, drone) == distance(location1, location2)


"""
	Check if there is an obstacle between 2 points and returns the closest
"""
def is_obstacle(p1, p2, obstacles):
	closest_obstacle = None
	for obstacle in obstacles:
		line1 = doIntersect(obstacle[0], obstacle[1], p1, p2)
		line2 = doIntersect(obstacle[1], obstacle[2], p1, p2)
		line3 = doIntersect(obstacle[2], obstacle[3], p1, p2)
		line4 = doIntersect(obstacle[3], obstacle[0], p1, p2)

		if line1 or line2 or line3 or line4:
			if closest_obstacle == None:
				closest_obstacle = obstacle
			else:
				closest_obstacle = calculate_closest_obstacle(p1, closest_obstacle, obstacle)

	return closest_obstacle


"""
	Returns the obstacle closest to p
"""
def calculate_closest_obstacle(p, obstacle_1, obstacle_2):
	shortest_distance = 9999999
	closest_obstacle = None
	for i in range(4):
		if distance(p, obstacle_1[i]) < shortest_distance:
			shortest_distance = distance(p, obstacle_1[i])
			closest_obstacle = obstacle_1
	for i in range(4):
		if distance(p, obstacle_2[i]) < shortest_distance:
			shortest_distance = distance(p, obstacle_2[i])
			closest_obstacle = obstacle_2
	
	return closest_obstacle


"""
	Check if the spesific obstacle is between 2 points
"""
def is_spesific_obstacle_between(p1, p2, obstacle):
	line1 = doIntersect(obstacle[0], obstacle[1], p1, p2)
	line2 = doIntersect(obstacle[1], obstacle[2], p1, p2)
	line3 = doIntersect(obstacle[2], obstacle[3], p1, p2)
	line4 = doIntersect(obstacle[3], obstacle[0], p1, p2)

	if line1 or line2 or line3 or line4:
		return True
	return False



"""
	Creates a bigger obstacle, so corner is outside the obstacle, so drone don't crash
"""
def make_obstacle_with_offset(obstacle):
	obstacle_with_offset = []
	obstacle_with_offset.append(makeOffset(obstacle[0], 0))
	obstacle_with_offset.append(makeOffset(obstacle[1], 1))
	obstacle_with_offset.append(makeOffset(obstacle[2], 2))
	obstacle_with_offset.append(makeOffset(obstacle[3], 3))
	return obstacle_with_offset


"""
	Creates a point outside the corner of a square
"""
def makeOffset(p, corner_nr):
	if corner_nr == 0:
		return dronekit.LocationGlobal(p.lat+0.0001, p.lon-0.0001, altitude)
	if corner_nr == 1:
		return dronekit.LocationGlobal(p.lat+0.0001, p.lon+0.0001, altitude)
	if corner_nr == 2:
		return dronekit.LocationGlobal(p.lat-0.0001, p.lon+0.0001, altitude)
	if corner_nr == 3:
		return dronekit.LocationGlobal(p.lat-0.0001, p.lon-0.0001, altitude)



"""
	Calculates the distance and path from a spesific vehicle to a spesific task
	If an obstacle is in the way, try to calculate path around, not optimized.
"""
def calculate_distance_path(task, current_location, obstacles):
	obstacle = is_obstacle(current_location, task[0], obstacles)
	distance_ = None
	path = []

	if obstacle == None:
		distance_ = distance(current_location, task[0])
	else:
		shortest_distance_around_obstacle, shortest_path_around_obstacle = calculate_distance_path_around_obstacle(current_location, task, obstacle, obstacles)
		distance_ = shortest_distance_around_obstacle
		path.extend(shortest_path_around_obstacle)

	return distance_, path



"""
	Try to calculate the shortest distance and path around the obstacle
	This function is recursiv and will try to calculate the shortest path around additional
	obstacles, it is not completly optimized, and will result in pythons max recursive depth error if
	there are many obstacles on the map
"""
def calculate_distance_path_around_obstacle(drone_location, task, obstacle, obstacles):
	obstacle_with_offset = make_obstacle_with_offset(obstacle)
	shortest_distance = 999999999999999
	shortest_path = []

	# For each corner of the obstacle
	for i in range(4):
	
		# If corner is in line of sight from drone
		if not is_spesific_obstacle_between(drone_location, obstacle_with_offset[i], obstacle):

			# Calculate distance from corner to task
			distance_from_corner_to_task, path_from_corner = calculate_distance_path_from_corner_to_task(i, task, obstacle, obstacles)
			
			distance_ = distance(drone_location, obstacle_with_offset[i]) + distance_from_corner_to_task
			if distance_ < shortest_distance:
				shortest_distance = distance_
				shortest_path = [obstacle_with_offset[i]]
				shortest_path.extend(path_from_corner)
	
	return shortest_distance, shortest_path


"""
	Calculate the distance from the spesific corner around the obstacle and to the task
"""
def calculate_distance_path_from_corner_to_task(corner_nr, task, obstacle, obstacles):
	distance_around_obstacle_clockwise, path_around_clockwise = calculate_distance_path_from_corner_clockwise(corner_nr, task, obstacle, obstacles)
	distance_around_obstacle_anti_clockwise, path_around_anti_clockwise = calculate_distance_path_from_corner_anti_clockwise(corner_nr, task, obstacle, obstacles)

	if(distance_around_obstacle_clockwise < distance_around_obstacle_anti_clockwise):
		return distance_around_obstacle_clockwise, path_around_clockwise
	else:
		return distance_around_obstacle_anti_clockwise, path_around_anti_clockwise


"""
	Calculate the shortest distance around the obstacle clockwise recursivly
"""
def calculate_distance_path_from_corner_clockwise(corner_nr, task, obstacle, obstacles):
	obstacle_with_offset = make_obstacle_with_offset(obstacle)
	corner = obstacle_with_offset[corner_nr]
	
	if not is_spesific_obstacle_between(corner, task[0], obstacle):
		new_obstacle = is_obstacle(corner, task[0], obstacles)
		if new_obstacle == None:
			return distance(corner, task[0]), []
		else:
			return calculate_distance_path_around_obstacle(corner, task, new_obstacle, obstacles)
	else:
		distance_from_next_corner, path_from_next_corner = calculate_distance_path_from_corner_clockwise((corner_nr+1) % 4, task, obstacle_with_offset, obstacles)
		distance_around_obstacle_clockwise = distance(corner, obstacle_with_offset[(corner_nr+1) % 4]) + distance_from_next_corner
		path_from_corner = [obstacle_with_offset[(corner_nr+1) % 4]]
		if(path_from_next_corner):
			path_from_corner.extend(path_from_next_corner)
		
		return distance_around_obstacle_clockwise, path_from_corner


"""
	Calculate the shortest distance around the obstacle counter clockwise recursivly
"""
def calculate_distance_path_from_corner_anti_clockwise(corner_nr, task, obstacle, obstacles):
	obstacle_with_offset = make_obstacle_with_offset(obstacle)
	corner = obstacle_with_offset[corner_nr]

	if not is_spesific_obstacle_between(corner, task[0], obstacle):
		new_obstacle = is_obstacle(corner, task[0], obstacles)
		if new_obstacle == None:
			return distance(corner, task[0]), []
		else:
			return calculate_distance_path_around_obstacle(corner, task, new_obstacle)
	else:
		distance_from_next_corner, path_from_next_corner = calculate_distance_path_from_corner_anti_clockwise((corner_nr-1) % 4, task, obstacle_with_offset, obstacles)
		distance_around_obstacle_anti_clockwise = distance(corner, obstacle_with_offset[(corner_nr-1) % 4]) + distance_from_next_corner
		path_from_corner = [obstacle_with_offset[(corner_nr-1) % 4]]
		if(path_from_next_corner):
			path_from_corner.extend(path_from_next_corner)
		
		return distance_around_obstacle_anti_clockwise, path_from_corner




"""
	Creates a sequence of points to follow when searching over an area
"""
def generate_search_coordinates(search_tasks):

	points = search_tasks[0]

	'''
	Indexes of "points":
	0 = North West, 1 = North East, 2 = South East, 3 = South West
	0---------------1
	|				|
	|				|
	|				|
	3---------------2
	'''
	#This is the North West corner of the search area
	starting_point = points[0]
	#How far south the drone shall search
	lowest_lat = points[3].lat
	#How far west the drone shall go 
	lowest_lon = points[0].lon
	#How far east the drone shall go
	highest_lon = points[1].lon

	paths = []

	current_location = starting_point
	next_location = None
	moveLatDirection = False

	while current_location.lat > lowest_lat:
		if next_location == None:
			#Start in northwest corner
			next_location = starting_point
		else:
			if moveLatDirection:
				next_location = current_location
				next_location = dronekit.LocationGlobal(next_location.lat - 0.0001,next_location.lon,10)
				moveLatDirection = False
			else:
				#If we are max east, move left
				if current_location.lon >= highest_lon:
					next_location = dronekit.LocationGlobal(current_location.lat,lowest_lon,10)
				#Move right
				else:
					next_location = dronekit.LocationGlobal(current_location.lat,highest_lon, 10)
				moveLatDirection = True

		paths.append(next_location)
		current_location = next_location
	
	return paths