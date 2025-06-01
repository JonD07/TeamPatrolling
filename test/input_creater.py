import yaml
import random
import os
import re

# Settings
FILE_PATH = os.getcwd() + "/IncNodes"
NUM_PLOTS = 5
NODE_START = 10
NODE_END = 200
NODE_INC = 10
NUM_UAV = 2
NUM_UGV = 1
NUM_OBSTACLES = 100
PAD_FLAG = True  # Whether to assign 2 pads per UGV or just 1 per UAV (True for 2 pads)

# Bounds
MAX_X = 10000
MAX_Y = 10000
OBSTACLE_SIZE_LOWER_BOUND = 12
OBSTACLE_SIZE_UPPER_BOUND = 300


# # Find the next file number
# def get_next_file_number():
#     existing_files = os.listdir(FILE_PATH)
#     pattern = r"plot_\d+_\d+_\d+_\d+_(\d+)\.yaml"
#     numbers = []
#     for filename in existing_files:
#         match = re.match(pattern, filename)
#         if match:
#             numbers.append(int(match.group(1)))
#     return max(numbers, default=0) + 1

# next_file_number = get_next_file_number()

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

def gen_file(nodes, file_id):
	# Build the filename
	file_name = f"plot_{NUM_UGV}_{NUM_UAV}_{nodes}_{file_id}.yaml"

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
	for i in range(1, NUM_UAV + 1):
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

	# Generate UGVs
	curr_pad = 1
	for ugv_index in range(1, NUM_UGV + 1):
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
		if PAD_FLAG:
			# 2 pads per UGV
			for j in range(curr_pad, curr_pad + 2):
				pad = {
					'ID': f'pad_{j:02}',
					'mode': 'occupied',
					'UAV_ID': f'UAV_{j:02}',
					'is_charging': True
				}
				ugv['charging_pads'].append(pad)
			curr_pad += 2
		else:
			# 1 pad per UAV
			for j in range(1, NUM_UAV + 1):
				pad = {
					'ID': f'pad_{j:02}',
					'mode': 'occupied',
					'UAV_ID': f'UAV_{j:02}',
					'is_charging': True
				}
				ugv['charging_pads'].append(pad)
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
	for i in range(1, NUM_OBSTACLES + 1):
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
	full_path = os.path.join(FILE_PATH, file_name)
	with open(full_path, 'w') as file:
		yaml.dump(data, file, sort_keys=False)

for n in range(NODE_START, NODE_END+1, NODE_INC):
	for i in range(NUM_PLOTS):
		gen_file(n, i)
	print(f"✅ Completed {NUM_PLOTS} files with {NUM_UGV} UGVs, {NUM_UAV} UAVs, {n} nodes, and {NUM_OBSTACLES} obstacles.yaml")