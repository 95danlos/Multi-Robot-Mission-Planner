import sys, ast, math
import time
import threading
import dronekit
from server import combiserver
from server import Drone_Services

HTTP_PORT = 8000
WEBSOCKET_PORT = 9001
UAV_BASE_PORT = 14550
server = combiserver.CombiServer(HTTP_PORT,WEBSOCKET_PORT)

#Cnstatsotnjas
LINE_INDEX = 0
POLY_INDEX = 1
undistributed_tasks = [None]*3
vehicles = []
line_task = []
polygon_task = []
free_vehicles = []

def main():
	Drone_Services.initialize(vehicles, UAV_BASE_PORT)
	Drone_Services.start_data_updating(vehicles, server)
	server.set_fn_message_received(message_received)

	try:
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
	except Exception as e:
		print(e)			
				


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

	i = 0
	while free_vehicles and is_more_tasks():
		i = i + 1
		time.sleep(3)
		best_task = find_best_vehicle_task()

		if best_task[0] == "line":

			best_vehicle = best_task[1]

			free_vehicles.remove(best_vehicle)
			remove_task(best_task[2])

			if best_task[4]:
				best_task[2] = reversed(best_task[2])

			best_vehicle.nextlocations.extend(best_task[2])
			best_vehicle.simple_goto(best_vehicle.nextlocations[0])

		elif best_task[0] == "poly":
			best_vehicle = best_task[1]

def distribute_search_task(free_vehicles):

	while free_vehicles and is_more_search_tasks():
		vehicle = free_vehicles[0]

		vehicle.nextlocations.extend(undistributed_tasks[2])
		vehicle.simple_goto(vehicle.nextlocations[0])
		undistributed_tasks[2] = None
		free_vehicles.remove(vehicle)

def is_more_search_tasks():
	if undistributed_tasks[2] == None:
		return False
	return True


def remove_task(task):
	for linetask in undistributed_tasks[0]:
		if linetask == task:
			undistributed_tasks[0].remove(task)

def is_more_tasks():
	num_tasks = 0
	if undistributed_tasks[0] == None:
		return False
	for t in undistributed_tasks[0]:
		num_tasks = num_tasks + 1
	
	return num_tasks > 0


def find_best_vehicle_task():
	best_linetask = find_best_vehicle_linetask(undistributed_tasks[0])
	best_polytask = find_best_vehicle_polytask(undistributed_tasks[1]) 
	return best_task(best_linetask, best_polytask)



def best_task(best_linetask, best_polytask):
	
	if best_linetask[1] == None:
		return best_polytask
	if best_polytask[1] == None:
		return best_linetask

	if best_linetask[2] < best_polytask[2]:
		return best_linetask
	else:
		return best_polytask



def find_best_vehicle_linetask(line_tasks):
	# While there are free vehicles and undistributed tasks
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

	return ["line", v_best, l_best, d_best, swap]

def find_best_vehicle_polytask(poly_tasks):
	v_best = None #vehicle
	l_best = None #line
	p_best = None #polygon
	d_best = None
	swap = None
	for vehicle in free_vehicles:
		for polygon in poly_tasks:
			for line in polygon: 
				d_s = Drone_Services.distance(vehicle.location.global_frame, line[0])
				d_e = Drone_Services.distance(vehicle.location.global_frame, line[1])
				d = min(d_e, d_s)
				if (d_best == None) or (d < d_best):
					d_best = d
					v_best = vehicle
					p_best = polygon
					l_best = line
					swap = (d_e < d_s)
	if swap:
		l_best = reversed(l_best)

	return ["poly", v_best, l_best, d_best, swap]

# Called when a client clicks on fly button
def message_received(client, server, message):

	# First position indicates message type
	message_type = ast.literal_eval(message)[0]

	if(message_type == 0):

		""" adding new lines that user draw on map """
		tasks = ast.literal_eval(message)[1]
		lines = tasks['line']
		polygons = []
		lines = [[dronekit.LocationGlobal(point["lat"],point["lng"],20) for point in line] for line in lines]

		line_task.extend(lines)
		undistributed_tasks[0] = line_task


		#Polytask
		poly_tasks = []
		for polygon in polygons:
			polygon = [[dronekit.LocationGlobal(point["lat"],point["lng"],20) for point in line] for line in polygon]
			closing_line = [polygon[len(polygon)-1][1], polygon[0][0]]
			polygon.append(closing_line)
			poly_tasks.append(polygon)

		polygon_task.extend(poly_tasks)
		undistributed_tasks[1] = poly_tasks

		#Search task
		search_tasks = tasks['search']
		search_tasks_temp = []; 

		#Convert to DroneKit's Global Position Object
		for task in search_tasks:
			task_temp = []
			for point in task:
				point = dronekit.LocationGlobal(point[0], point[1], 20)
				task_temp.append(point)
			search_tasks_temp.append(task_temp)
		search_tasks = search_tasks_temp

		if len(search_tasks) > 0:
			undistributed_tasks[2] = generate_search_coordinates(search_tasks)
	
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
	
def generate_search_coordinates(search_tasks):
	points = search_tasks[0]

	#This is the North West corner of the search area
	starting_point = points[3]
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
				next_location.lat = next_location.lat - 0.00001
				moveLatDirection = False
			else:
				#If we are max east, move left
				if current_location.lon >= highest_lon:
					next_location.lat = current_location.lat
					next_location.lon = lowest_lon
				#Move right
				else:
					next_location.lat = current_location.lat
					next_location.lon = highest_lon
				moveLatDirection = True
			
		#paths.append(next_location)
		current_location = next_location

	paths.append(starting_point)
	print(len(paths))
	return paths
main()

