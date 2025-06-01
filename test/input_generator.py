import yaml
import random

## Setup
# Ways in increment inputs
INC_N = 0
INC_ROBOTS = 1
# What are we doing here?
INC_TYPE = INC_N

# Basics
FIX_NUM_UAV = 2 # Number of UAVs
FIX_NUM_UGV = 1  # Number of UGVs
FIXED_N = 20   # Number of nodes

# Increasing sizes
NUM_FILES = 1 # Number of YAML files to generate at each iteration
# MIN_UAV = 1
# MIN_UGV = 1
# MAX_UAV = 2
# MAX_UGV = 1
MIN_N = 5
MAX_N = 10
INC_N_STEP = 1

# Where to store files
# FILE_PATH = "Exp_01/"
FILE_PATH = "obstacles_tests"


# Bounds on area
MAX_X = 10000
MAX_Y = 10000

EXP_01_Flag = False
EXP_02_Flag = False
EXP_03_Flag = False


def main():
	if EXP_01_Flag:
		Constant_Teams_Increasing_Nodes()
	if EXP_02_Flag:
		Increasing_Drones_Constant_Nodes()
	if EXP_03_Flag:
		Increasing_Teams_Constant_Nodes()

def generate_yaml_file(file_name, m1, m2, n, padFlag): #padFlag used for charging pads for exp 3
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
	currUGV = 1
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
			'charging_pads':[] 
		}
		if(padFlag): #want to have each ugv have 2 pads since there are 2x uavs
			for i in range(currUGV, currUGV+2):
				temp = {
					'ID': f'pad_{i:02}',
					'mode': 'occupied',
					'UAV_ID': f'UAV_{i:02}',
					'is_charging': True
				}
				ugv['charging_pads'].append(temp)
			currUGV+=2
		else:
			for i in range(1, m1+1):
				temp = {
					'ID': f'pad_{i:02}',
					'mode': 'occupied',
					'UAV_ID': f'UAV_{i:02}',
					'is_charging': True
				}
				ugv['charging_pads'].append(temp)
		data['agents'].append(ugv)

	# Generate nodes with random locations
	for i in range(1, n+1):
		node = {
			'ID': f'n_{i:04}',
			'type': 'air_only',
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

####create multiple yaml files for increasing nodes##############
def Constant_Teams_Increasing_Nodes_Set_Generator(num_files, ma, mg, n):
	FILE_PATH = "IncreaseNodeExps/"
	for i in range(1, num_files + 1):
		file_name = f'{FILE_PATH}plot_{mg}_{ma}_{n}_{i}.yaml'
		generate_yaml_file(file_name, ma, mg, n, False)

def Constant_Teams_Increasing_Nodes():
	FIX_NUM_UAV = 2
	FIX_NUM_UGV = 1
	MIN_N = 5
	MAX_N = 100
	INC_N_STEP = 5
	NUM_FILES = 50
	for n in range(MIN_N, MAX_N+1, INC_N_STEP):
		Constant_Teams_Increasing_Nodes_Set_Generator(NUM_FILES, FIX_NUM_UAV, FIX_NUM_UGV, n)

def Increasing_Drones_Constant_Nodes_Set_Generator(num_files, ma, mg, n): 
	FILE_PATH = "IncreaseUAVsExps/"
	for i in range(1, num_files + 1):
		file_name = f'{FILE_PATH}plot_{mg}_{ma}_{50}_{i}.yaml'
		generate_yaml_file(file_name, ma, mg, n, False)

def Increasing_Drones_Constant_Nodes():
	FIX_NUM_UAV = 1
	FIX_NUM_UGV = 1
	MIN_N = 1
	MAX_N = 10
	INC_N_STEP = 1
	NUM_FILES = 50
	for n in range(MIN_N, MAX_N+1, INC_N_STEP): 
		Increasing_Drones_Constant_Nodes_Set_Generator(NUM_FILES, FIX_NUM_UAV, FIX_NUM_UGV, 50)
		FIX_NUM_UAV+=1

def Increasing_Teams_Constant_Nodes_Set_Generator(num_files, ma, mg, n): 
	FILE_PATH = "IncreaseTeamsExps/"
	for i in range(1, num_files + 1):
		file_name = f'{FILE_PATH}plot_{mg}_{ma}_{50}_{i}.yaml'
		generate_yaml_file(file_name, ma, mg, n, True)

def Increasing_Teams_Constant_Nodes():
	FIX_NUM_UAV = 2
	FIX_NUM_UGV = 1
	MIN_N = 1
	MAX_N = 10
	INC_N_STEP = 1
	NUM_FILES = 50
	for n in range(MIN_N, MAX_N+1, INC_N_STEP): 
		Increasing_Teams_Constant_Nodes_Set_Generator(NUM_FILES, FIX_NUM_UAV, FIX_NUM_UGV, 50)
		FIX_NUM_UAV+=2
		FIX_NUM_UGV+=1


if __name__ == '__main__':
    main()