import yaml
import random
import os
import math

# Settings
# FILE_PATH = os.getcwd() + "/IncNodes"
NUM_PLOTS = 50
NODE_START = 5
NODE_END = 100
NODE_INC = 5
TEAM_START = 1
TEAM_END = 10
DRONE_START = 1
DRONE_END = 10
OBSTACLES_START = 0
OBSTACLES_END = 200
OBSTACLES_INC = 10
DEFAULT_UAV_NUM = 4
DEFAULT_UGV_NUM = 2
DEFAULT_NODES = 50
DEFAULT_OBSTACLES = 100
PAD_FLAG = True  # Whether to assign 2 pads per UGV or just 1 per UAV (True for 2 pads)

# Bounds
MAX_X = 10000
MAX_Y = 10000
OBSTACLE_SIZE_LOWER_BOUND = 2
OBSTACLE_SIZE_UPPER_BOUND = 300

# Pick obstacle coordinates/radius that are not on depot
def pick_obstacle_xyr():
	valid_obstacle = False
	while not valid_obstacle:
		# Pick new x,y,r
		x = round(random.uniform(-MAX_X/2, MAX_X/2), 1)
		y = round(random.uniform(-MAX_Y/2, MAX_Y/2), 1)
		r = round(random.uniform(OBSTACLE_SIZE_LOWER_BOUND, OBSTACLE_SIZE_UPPER_BOUND), 1)

		# Does this overlap with the depot?
		dist = (x**2 + y**2)**(0.5)
		if dist <= r:
			valid_obstacle = False
		else:
			valid_obstacle = True
	return x,y,r

def gen_file(file_path, nodes, file_id, num_ugv, num_drone, num_obst=DEFAULT_OBSTACLES):
	# Build the filename
	file_name = f"plot_{num_ugv}_{num_drone}_{nodes}_{file_id}.yaml"

	# Create the data
	data = {
		'ID': f'state_every_UAV_action_{file_name}',
		'time': 0.0,
		'description': 'Generated scenario for UAV and UGV planning problem',
		'agents': [],
		'scenario': {
			'description': 'Generated scenario',
			'type': 'persistent_surveillance',
			'subtype': 'standard',
			'nodes': [],
			'connections': [],
			'obstacles': []  
		}
	}

	# Generate UAVs
	for i in range(1, num_drone + 1):
		uav = {
			'ID': f'UAV_{i:02}',
			'type': 'UAV',
			'subtype': 'standard',
			'location': {'x': 0.0, 'y': 0.0},
			'battery_state': {
				'max_battery_energy': 360381.0,
				'current_battery_energy': 360381.0
			},
			'stratum': 'docked',
			'charging_pad_ID': f'pad_{i:02}'
		}
		data['agents'].append(uav)

	# Number of drones per UGV
	drone_per_ugv = math.ceil(num_drone/num_ugv)
	# Generate UGVs
	curr_pad = 1
	for ugv_index in range(1, num_ugv + 1):
		ugv = {
			'ID': f'UGV_{ugv_index:02}',
			'type': 'UGV',
			'subtype': 'standard',
			'location': {'x': 0.0, 'y': 0.0},
			'battery_state': {
				'max_battery_energy': 25010000.0,
				'current_battery_energy': 25010000.0
			},
			'charging_pads': []
		}
		for j in range(curr_pad, curr_pad + drone_per_ugv):
			if(curr_pad <= num_drone):
				pad = {
					'ID': f'pad_{j:02}',
					'mode': 'occupied',
					'UAV_ID': f'UAV_{j:02}',
					'is_charging': True
				}
			ugv['charging_pads'].append(pad)
		curr_pad += drone_per_ugv

		# if PAD_FLAG:
		# 	# 2 pads per UGV
		# 	for j in range(curr_pad, curr_pad + 2):
		# 		pad = {
		# 			'ID': f'pad_{j:02}',
		# 			'mode': 'occupied',
		# 			'UAV_ID': f'UAV_{j:02}',
		# 			'is_charging': True
		# 		}
		# 		ugv['charging_pads'].append(pad)
		# 	curr_pad += 2
		# else:
		# 	# 1 pad per UAV
		# 	for j in range(1, num_uav + 1):
		# 		pad = {
		# 			'ID': f'pad_{j:02}',
		# 			'mode': 'occupied',
		# 			'UAV_ID': f'UAV_{j:02}',
		# 			'is_charging': True
		# 		}
		# 		ugv['charging_pads'].append(pad)

		data['agents'].append(ugv)

	# Generate nodes
	for i in range(nodes):
		node = {
			'ID': f'n_{i:04}',
			'type': 'air_only',
			'location': {
				'x': round(random.uniform(-MAX_X/2, MAX_X/2), 1),
				'y': round(random.uniform(-MAX_Y/2, MAX_Y/2), 1)
			},
			'time_last_service': 0.0
		}
		data['scenario']['nodes'].append(node)

	# Generate obstacles
	for i in range(1, num_obst + 1):
		# Get an x/y/radius position
		x, y, r =pick_obstacle_xyr()
		obstacle = {
			'type': 'circle',
			'ID': f'o_{i:02}',
			'location': {
				'x': x,
				'y': y
			},
			'radius': r
		}
		data['scenario']['obstacles'].append(obstacle)

	# Write to file
	full_path = os.path.join(file_path, file_name)
	with open(full_path, 'w') as file:
		yaml.dump(data, file, sort_keys=False)


print("** Creating IncNodes Data Set")
# Generate increasing nodes
for n in range(NODE_START, NODE_END+1, NODE_INC):
	for i in range(NUM_PLOTS):
		gen_file(os.getcwd() + "/IncNodes", n, i, DEFAULT_UGV_NUM, DEFAULT_UAV_NUM)
	print(f"✅ Completed {NUM_PLOTS} files with {DEFAULT_UGV_NUM} UGVs, {DEFAULT_UAV_NUM} UAVs, {n} nodes, and {DEFAULT_OBSTACLES} obstacles.yaml")

print("** Creating IncTeams Data Set")
# Generate increasing teams
for teams in range(TEAM_START, TEAM_END+1):
	ugvs = teams
	uavs = teams*2
	for i in range(NUM_PLOTS):
		gen_file(os.getcwd() + "/IncTeams", DEFAULT_NODES, i, ugvs, uavs)
	print(f"✅ Completed {NUM_PLOTS} files with {ugvs} UGVs, {uavs} UAVs, {DEFAULT_NODES} nodes, and {DEFAULT_OBSTACLES} obstacles.yaml")

print("** Creating IncDrones Data Set")
# Generate increasing drones
for drones in range(DRONE_START, DRONE_END+1):
	for i in range(NUM_PLOTS):
		gen_file(os.getcwd() + "/IncDrones", DEFAULT_NODES, i, 1, drones)
	print(f"✅ Completed {NUM_PLOTS} files with {1} UGVs, {drones} UAVs, {DEFAULT_NODES} nodes, and {DEFAULT_OBSTACLES} obstacles.yaml")

print("** Creating IncObstacles Data Set")
# Generate increasing drones
for obst in range(OBSTACLES_START, OBSTACLES_END+1, OBSTACLES_INC):
	for i in range(NUM_PLOTS):
		gen_file(os.getcwd() + "/IncObstacles", DEFAULT_NODES, f"{obst}_{i}", DEFAULT_UGV_NUM, DEFAULT_UAV_NUM, obst)
	print(f"✅ Completed {NUM_PLOTS} files with {1} UGVs, {drones} UAVs, {DEFAULT_NODES} nodes, and {obst} obstacles.yaml")
