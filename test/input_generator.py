import yaml
import random

# Setup
num_files = 1  # Number of YAML files to generate
m1 = 2  # Number of UAVs
m2 = 1  # Number of UGVs
n = 5   # Number of nodes

MAX_X = 10000
MAX_Y = 10000


def generate_yaml_file(file_name, m1, m2, n):
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
			'connections': []
		}
	}

	# Generate UAVs
	for i in range(1, m1 + 1):
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
	for i in range(1, m2 + 1):
		ugv = {
			'ID': f'UGV_{i:02}',
			'type': 'UGV',
			'subtype': 'standard',
			'location': {'x': 0.0, 'y': 0.0},
			'battery_state': {
				'max_battery_energy': 25010000.0,
				'current_battery_energy': 25010000.0
			},
			'charging_pads': [{
				'ID': f'pad_{i:02}',
				'mode': 'occupied',
				'UAV_ID': f'UAV_{i:02}',
				'is_charging': True
			}]
		}
		data['agents'].append(ugv)

	# Generate nodes with random locations
	for i in range(1, n + 1):
		node = {
			'ID': f'n_{i:04}',
			'type': 'standard',
			'location': {
				'x': round(random.uniform(MAX_X/-2.0, MAX_X/2.0), 1),
				'y': round(random.uniform(MAX_Y/-2.0, MAX_Y/2.0), 1)
			},
			'time_last_service': 0.0
		}
		data['scenario']['nodes'].append(node)

	# Write to YAML file
	with open(file_name, 'w') as file:
		yaml.dump(data, file, sort_keys=False)


def generate_multiple_yaml_files(num_files, m1, m2, n):
	for i in range(1, num_files + 1):
		file_name = f'plot_{m1:02}_{m2:02}_{n:02}_{i:02}.yaml'
		generate_yaml_file(file_name, m1, m2, n)

generate_multiple_yaml_files(num_files, m1, m2, n)
