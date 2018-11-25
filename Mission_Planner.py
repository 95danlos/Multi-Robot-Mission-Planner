import sys, ast, math
import time
import subprocess
import threading
import dronekit
from server import combiserver
from server import Drone_Services

HTTP_PORT = 8000
WEBSOCKET_PORT = 9001
UAV_BASE_PORT = 14550
server = combiserver.CombiServer(HTTP_PORT,WEBSOCKET_PORT)

altitude = 10+5.3 #Don't ask why, it just works
vehicles = []
free_vehicles = []

obstacles = []
u_tasks = []

def main():
	Drone_Services.initialize(vehicles, UAV_BASE_PORT, server)
	Drone_Services.start_data_updating(vehicles, server)
	server.set_fn_message_received(message_received)


	# Start web interface
	with server:
		while True:

			free_vehicles.clear()

			# For each vehicle
			for vehicle in vehicles:
				# If the vehicle does not have a next task, add it to free vehicles'
				if not vehicle.nextlocations:
					free_vehicles.append(vehicle)
				# Else if next task is reached remove it from the vehicles's task lis
				elif Drone_Services.distance(vehicle.nextlocations[0], vehicle.location.global_frame) < 0.001:
					vehicle.nextlocations.pop(0)
					# If the vehicle has more tasks, start the next

					if vehicle.nextlocations:
						vehicle.simple_goto(vehicle.nextlocations[0])

					# Else add it to free vehicles
					else:
						free_vehicles.append(vehicle)


			# Distribute new tasks	
			distribute_tasks(free_vehicles)
			#distribute_search_task(free_vehicles)
			time.sleep(1)

def distribute_tasks(free_vehicles):

	while free_vehicles and is_more_tasks():
		task, vehicle = best_vehicle_task(free_vehicles)
		if (task == None) or vehicle == None:
			raise RuntimeError("Error in task distribution: There should always be an optimal task")
		
		free_vehicles.remove(vehicle)

		u_tasks.remove(task) 


		vehicle.nextlocations.extend(task)
		vehicle.simple_goto(vehicle.nextlocations[0])

def best_vehicle_task(free_vehicles):

	best_vehicle = None
	best_task = None
	best_distance = None
	for vehicle in free_vehicles:
		print("Free ve")
		for task in u_tasks:
			print("IN TASK!!!!!!!!!!!!!!!!!!!!!!!")
			#The first point in the task is the starting point
			point = task[0]


			#FINAL DESIGN LETSGO
			distance, path = calculate_distance_path(task, vehicle.location.global_frame)
			print("Path:", path)
			print(distance)
			if (best_distance == None) or (distance < best_distance):
				best_vehicle = vehicle
				best_task = task
				best_distance = distance
	
	if path:
		best_vehicle.nextlocations.extend(path)
	return best_task, best_vehicle


def calculate_distance_path(task, location):
	print("CALCULATING!!!!!!!!")
	current_location = location
	obstacle = is_obstacle(current_location, task)

	if obstacle == None:
		return Drone_Services.distance(current_location, task[0])
	else:
		return calculate_obstacle_distance(current_location, task, obstacle)


def is_obstacle(p, task):
	print("In is obstacle")
	i = 0
	for obstacle in obstacles:
		print("Inside loop obstacle:", obstacle)
		print("Obstacle 0:", obstacle[0])
		line1 = Drone_Services.doIntersect(obstacle[0], obstacle[1], p, task[0])
		line2 = Drone_Services.doIntersect(obstacle[1], obstacle[2], p, task[0])
		line3 = Drone_Services.doIntersect(obstacle[2], obstacle[3], p, task[0])
		line4 = Drone_Services.doIntersect(obstacle[3], obstacle[0], p, task[0])

		if line1 or line2 or line3 or line4:
			print(i)
			return obstacle
		i=i+1
	return None

def calculate_obstacle_distance(location, task, obstacle):
	counter = 0
	distance = 0
	corner = 0
	path = []
	closest_corner_distance = Drone_Services.distance(location, obstacle[0])
	closest_corner = obstacle[0]

	for point in obstacle:
		if Drone_Services.distance(point, location) < closest_corner_distance:
			closest_corner = point
			corner = counter
		counter+=1

	offset = makeOffset(closest_corner, corner)
	path.append(offset)
	distance = Drone_Services.distance(location, offset)

	#If there is another obstacle from the first obstacle corner to the task then we need to go to another corner and then we are safe.. we assume atleast
	if is_obstacle(offset, task) == None:
		path.extend(task)
		distance = distance +  Drone_Services.distance(offset,task[0])
	else: 
		corner = (corner+1) % 4
		if is_obstacle(makeOffset(obstacle[corner], corner), task) == None:
			path.append(makeOffset(obstacle[corner], corner))

			#:))
			distance = distance + Drone_Services.distance(makeOffset(obstacle[corner], corner),task[0]) + Drone_Services.distance(offset,makeOffset(obstacle[corner], corner))


	return distance, path

def makeOffset(p, corner):
	if corner == 0:
		return dronekit.LocationGlobal(p.lat+0.0001, p.lon-0.0001, altitude)
	if corner == 1:
		return dronekit.LocationGlobal(p.lat+0.0001, p.lon+0.0001, altitude)
	if corner == 2:
		return dronekit.LocationGlobal(p.lat-0.0001, p.lon+0.0001, altitude)
	if corner == 3:
		return dronekit.LocationGlobal(p.lat-0.0001, p.lon-0.0001, altitude)


def is_more_tasks():
	return len(u_tasks) > 0

# Called when a client clicks on fly button
def message_received(client, server, message):


	# First position indicates message type
	message_type = ast.literal_eval(message)[0]

	if(message_type == 0):

		""" adding new lines that user draw on map """
		tasks = ast.literal_eval(message)[1]
		lines = tasks['line']
		lines = [[dronekit.LocationGlobal(point["lat"],point["lng"],altitude) for point in line] for line in lines]
		u_tasks.extend(lines)

		#Search task
		search_tasks = tasks['search']
		search_tasks_temp = []


		#Obstacles 
		obstacles_from_client = tasks['obstacles']
		obstacles_temp = []



		for obstacle in obstacles_from_client:
			point_temp = []
			for point in obstacle:
				point = dronekit.LocationGlobal(point[0], point[1], altitude)
				point_temp.append(point)
			obstacles_temp.append(point_temp)

		obstacles.extend(obstacles_temp)

		#Convert to DroneKit's Global Position Object
		for task in search_tasks:
			task_temp = []
			for point in task:
				point = dronekit.LocationGlobal(point[0], point[1], altitude)
				task_temp.append(point)
			search_tasks_temp.append(task_temp)
		search_tasks = search_tasks_temp

		if len(search_tasks) > 0:
			search_tasks = generate_search_coordinates(search_tasks)
			u_tasks.append(search_tasks)

	
	# New Parameter recieved
	if(message_type == 1):

		""" Change Parameters """
		vehicle_id = ast.literal_eval(message)[1]
		vehicle = Drone_Services.find_vehicle_by_id(vehicles, vehicle_id)

		new_max_speed = ast.literal_eval(message)[2]
		vehicle.max_speed = float(new_max_speed)
		vehicle.groundspeed = float(new_max_speed)

		new_max_battery_time = ast.literal_eval(message)[3]
		vehicle.max_battery_time = float(new_max_battery_time)
		vehicle.current_battery = float(new_max_battery_time)
		vehicle.start_up_time = time.time()

		new_max_carry_weight = ast.literal_eval(message)[4]
		vehicle.max_carry_weight = float(new_max_carry_weight)
	

	# Reacharge battery
	if(message_type == 2):
		vehicle_id = ast.literal_eval(message)[1]
		vehicle = Drone_Services.find_vehicle_by_id(vehicles, vehicle_id)

		vehicle.current_battery = vehicle.max_battery_time
		vehicle.start_up_time = time.time()

	
	# Open new cygwin terminal
	if(message_type == 3):
		subprocess.call('start cd C:\cygwin ^& cygwin', shell=True)


	# Start new simulated drone
	if(message_type == 4):
		Drone_Services.start_new_simulated_drone(vehicles, UAV_BASE_PORT, server)


	# Delete mission and stop vehicles
	if(message_type == 5):
		for vehicle in vehicles:
			vehicle.nextlocations = []
			vehicle.simple_goto(dronekit.LocationGlobal(vehicle.location.global_frame.lat, vehicle.location.global_frame.lon, altitude))

		
	
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
main()