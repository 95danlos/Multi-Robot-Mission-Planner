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

vehicles = []
undistributed_lines = []


def main():
	Drone_Services.initialize(vehicles, UAV_BASE_PORT)
	Drone_Services.start_position_tracking(vehicles, server)
	server.set_fn_message_received(message_received)

	try:
		# Start web interface
		with server:
			while True:
				free_vehicles = []

				# For each vehicle
				for vehicle in vehicles:
					# If the vehicle does not have a next task, add it to free vehicles
					if not vehicle.nextlocations:
						free_vehicles.append(vehicle)

					# Else if next task is reached remove it from the vehicles's task list
					elif Drone_Services.distance(vehicle.nextlocations[0], vehicle.location.global_frame) < 0.001:
						vehicle.nextlocations.pop(0)

						# If the vehicle has more tasks, start the next
						if vehicle.nextlocations:
							vehicle.simple_goto(vehicle.nextlocations[0])

						# Else add it to free vehicles
						else:
							free_vehicles.append(vehicle)

				# Distribute new tasks	
				distribute_tasks(free_vehicles, undistributed_lines)
				time.sleep(1)
	except:
  		print("An exception occurred")			
				


def distribute_tasks(free_vehicles, undistributed_lines):
	
	# While there are free vehicles and undistributed tasks
	while free_vehicles and undistributed_lines:
		v_best = None #vehicle
		l_best = None #line
		d_best = None
		swap = None

		# Find the shortes distance between a free vehicle and an undistributed task
		for vehicle in free_vehicles:
			for line in undistributed_lines:
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
		undistributed_lines.remove(l_best)

		# Swap the line start and end position if the end position is closest
		if swap:
			l_best = reversed(l_best)

		# Add task to the vehicle's task list and start the task
		v_best.nextlocations.extend(l_best)
		print("3")
		v_best.simple_goto(v_best.nextlocations[0])



# Called when a client clicks on fly button
def message_received(client, server, message):
	
	""" adding new lines that user draw on map """
	lines = ast.literal_eval(message)
	lines = [[dronekit.LocationGlobal(point["lat"],point["lng"],20) for point in line] for line in lines]
	undistributed_lines.extend(lines)





main()

