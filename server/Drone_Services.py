import sys, ast, math
import time
import threading
import dronekit

"""
Connects and arms all available vehicles based on how many SITL instances is running.
Each instance needs to be started at forehand using

sim_vehicle.py -L KSFO -I 0 	// vehicle 1
sim_vehicle.py -L KSFO -I 1		// vehicle 2
etc..

"""
def initialize(vehicles, UAV_BASE_PORT):

	# Number of simulated drones available at startup given as a command line argument
	instance_count = 0
	if (len(sys.argv) > 1 ):
		instance_count = int(sys.argv[1])
	
	for instance_index in range(instance_count):
		
		port = UAV_BASE_PORT + 10 * instance_index
		connection_string = "127.0.0.1:%i" % port #connect to port on localhost

		# Connect to the Vehicles
		print('Connecting to vehicle %i on: %s' % (instance_index, connection_string))
		for _ in range(60):
			try:
				vehicle = dronekit.connect(connection_string, wait_ready=True)
			except Exception as e:
				print(e)
				time.sleep(1)
			else:
				vehicle.instance_index = instance_index
				vehicle.nextlocations = []
				vehicles.append(vehicle)
				break

	do_for_all(vehicles, lambda v:arm_and_takeoff(v, 10))


def arm_and_takeoff(vehicle, aTargetAltitude):
	"""
	Arms vehicle and fly to aTargetAltitude.
	"""

	print("Vehicle %i: Basic pre-arm checks" % vehicle.instance_index)
	# Don't try to arm until autopilot is ready
	print("Vehicle %i: Waiting for vehicle to initialise..."  % vehicle.instance_index)
	while not vehicle.is_armable:
		time.sleep(1)

	print("Vehicle %i: Arming motors" % vehicle.instance_index)
	# Copter should arm in GUIDED mode
	vehicle.mode = dronekit.VehicleMode("GUIDED")
	vehicle.armed = True    

	# Confirm vehicle armed before attempting to take off
	print("Vehicle %i: Waiting for arming..." % vehicle.instance_index)
	while not vehicle.armed:      
		time.sleep(1)

	print("Vehicle %i: Taking off!" % vehicle.instance_index)
	vehicle.simple_takeoff(aTargetAltitude) # Take off to target altitude

	# Wait until the vehicle reaches a safe height before processing the goto (otherwise the command 
	#  after Vehicle.simple_takeoff will execute immediately).
	while True:
		print("Vehicle %i: Altitude: " % vehicle.instance_index, vehicle.location.global_relative_frame.alt)
		#Break and return from function just below target altitude.        
		if vehicle.location.global_relative_frame.alt>=aTargetAltitude*0.95: 
			print("Vehicle %i: Reached target altitude" % vehicle.instance_index)
			break
		time.sleep(1)

	#print("Vehicle %i: Set default/target airspeed to 5" % vehicle.instance_index)
	#vehicle.airspeed = 5


def start_position_tracking(vehicles, server):
	for vehicle in vehicles:
		t = threading.Thread(target=lambda:position_tracker_thread(vehicle, server))
		t.daemon = True
		t.start()


def position_tracker_thread(vehicle, server):
	while True:
		position = [vehicle.location.global_frame.lat, vehicle.location.global_frame.lon]
		vehicle_id = vehicle.instance_index
		#print (vehicle, position)
		server.send_message_to_all(repr([vehicle_id,position]))
		time.sleep(0.1)


def do_for_all(vehicles, function):
	threads = [threading.Thread(target=lambda v=v:function(v)) for v in vehicles]
	for t in threads:
		t.start()
	for t in threads:
		t.join()


def distance(location1, location2):
	(x1,y1) = location1.lat, location1.lon
	(x2,y2) = location2.lat, location2.lon
	try:
		d = math.acos(math.sin(math.radians(x1))*math.sin(math.radians(x2)) + math.cos(math.radians(x1))*math.cos(math.radians(x2))*math.cos(math.radians(y2-y1)))*6378.137
	except ValueError:
		print("ValueError in distance calculation for ", x1,y1, "to", x2,y2)
		d = 0.0
	return d