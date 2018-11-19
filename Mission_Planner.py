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

vehicles = []
free_vehicles = []

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
		
				


def distribute_taskssss(free_vehicles, line_task):
	
	# While there are free vehicles and undistributed tasks
	while free_vehicles and line_task:
		v_best = None #vehicle
		l_best = None #line
		d_best = None
		swap = None

		# Find the shortes distance between a free vehicle and an undistributed task
		for vehicle in free_vehicles:
			for line in line_task:
				d_s = Drone_Services.distance(vehicle.location.global_frame, line[0])
				d_e = Drone_Services.distance(vehicle.location.global_frame, line[1])
				d = min(d_e, d_s)
				if (d_best == None) or (d < d_best):
					d_best = d
					v_best = vehicle
					l_best = line
					swap = (d_e < d_s)
		if d_best == None:
			raise RuntimeError("Unexpected Error in distribute tasks: if there are tasks then there should be an optimal one")
		
		# Remove the vehicle from free vehicles and remove the task from undistributed tasks
		free_vehicles.remove(v_best)
		line_task.remove(l_best)

		# Swap the line start and end position if the end position is closest
		if swap:
			l_best = reversed(l_best)

		# Add task to the vehicle's task list and start the task
		v_best.nextlocations.extend(l_best)
		v_best.simple_goto(v_best.nextlocations[0])

def distribute_tasks(free_vehicles):

	while free_vehicles and is_more_tasks():
		print("free vehicles and more tasks")
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
		for task in u_tasks:
			#The first point in the task is the starting point
			point = task[0]
			distance = Drone_Services.distance(vehicle.location.global_frame, point)
			if (best_distance == None) or (distance < best_distance):
				best_vehicle = vehicle
				best_task = task
				best_distance = distance
	
	return best_task, best_vehicle

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
		lines = [[dronekit.LocationGlobal(point["lat"],point["lng"],20) for point in line] for line in lines]
		u_tasks.extend(lines)

		#Search task
		search_tasks = tasks['search']
		search_tasks_temp = []

		#Convert to DroneKit's Global Position Object
		for task in search_tasks:
			task_temp = []
			for point in task:
				point = dronekit.LocationGlobal(point[0], point[1], 20)
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
			print("Start location: ", starting_point)
		else:
			if moveLatDirection:
				next_location = current_location
				next_location = dronekit.LocationGlobal(next_location.lat - 0.0001,next_location.lon,20)
				moveLatDirection = False
			else:
				#If we are max east, move left
				if current_location.lon >= highest_lon:
					next_location = dronekit.LocationGlobal(current_location.lat,lowest_lon,20)
				#Move right
				else:
					next_location = dronekit.LocationGlobal(current_location.lat,highest_lon, 20)
				moveLatDirection = True

		print("Appending " , next_location)
		paths.append(next_location)
		current_location = next_location
	
	for path in paths:
		print("Path: " , path)
	return paths
main()