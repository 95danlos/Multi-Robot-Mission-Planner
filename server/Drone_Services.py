import sys, ast, math
import time
import threading
import dronekit


def initialize(vehicles, UAV_BASE_PORT, server):

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

	do_for_all(vehicles, lambda v:arm_and_takeoff(v, 10, server))


def arm_and_takeoff(vehicle, aTargetAltitude, server):
	"""
	Arms vehicle and fly to aTargetAltitude.
	"""

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


def distance(location1, location2):
	(x1,y1) = location1.lat, location1.lon
	(x2,y2) = location2.lat, location2.lon
	try:
		d = math.acos(math.sin(math.radians(x1))*math.sin(math.radians(x2)) + math.cos(math.radians(x1))*math.cos(math.radians(x2))*math.cos(math.radians(y2-y1)))*6378.137
	except ValueError:
		print("ValueError in distance calculation for ", x1,y1, "to", x2,y2)
		d = 0.0
	return d


def is_on_line(drone, location1, location2): 
	return distance(location1, drone) + distance(location2, drone) == distance(location1, location2)

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
				arm_and_takeoff(vehicle, 10, server)
				t = threading.Thread(target=lambda:start_data_updating_thread(vehicle, server))
				t.daemon = True
				t.start()
				break
				

