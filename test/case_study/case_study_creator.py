import yaml
import json
import os
import math
import argparse

def load_json_data(json_file_path):
    """Load coordinate data from JSON file"""
    with open(json_file_path, 'r') as file:
        return json.load(file)

def create_scenario_from_json(json_data, num_ugv=2, num_drone=4, output_file=None):
    """Create a YAML scenario file from JSON coordinate data"""
    
    # Extract data from JSON
    reference_point = json_data['reference_point']
    pois = json_data['pois']
    obstacles = json_data['obstacles']
    
    # Build the filename if not provided
    if output_file is None:
        output_file = f"case_study_{num_ugv}ugv_{num_drone}uav_{len(pois)}pois_{len(obstacles)}obst.yaml"
    
    # Create the scenario data structure
    data = {
        'ID': f'case_study_{output_file}',
        'time': 0.0,
        'description': f'Case study scenario generated from coordinate data with {len(pois)} POIs and {len(obstacles)} obstacles',
        'agents': [],
        'scenario': {
            'description': f'Case study scenario with depot at origin (0,0)',
            'type': 'persistent_surveillance',
            'subtype': 'case_study',
            'nodes': [],
            'connections': [],
            'obstacles': []  
        }
    }

    # Generate UAVs (drones) - all start at depot (0,0)
    for i in range(1, num_drone + 1):
        uav = {
            'ID': f'UAV_{i:02}',
            'type': 'UAV',
            'subtype': 'standard',
            'location': {'x': 0.0, 'y': 0.0},  # Depot location
            'battery_state': {
                'max_battery_energy': 360381.0,
                'current_battery_energy': 360381.0
            },
            'stratum': 'docked',
            'charging_pad_ID': f'pad_{i:02}'
        }
        data['agents'].append(uav)

    # Calculate drones per UGV
    drone_per_ugv = math.ceil(num_drone / num_ugv)
    
    # Generate UGVs - all start at depot (0,0)
    curr_pad = 1
    for ugv_index in range(1, num_ugv + 1):
        ugv = {
            'ID': f'UGV_{ugv_index:02}',
            'type': 'UGV',
            'subtype': 'standard',
            'location': {'x': 0.0, 'y': 0.0},  # Depot location
            'battery_state': {
                'max_battery_energy': 25010000.0,
                'current_battery_energy': 25010000.0
            },
            'charging_pads': []
        }
        
        # Assign charging pads to this UGV
        for j in range(curr_pad, curr_pad + drone_per_ugv):
            if curr_pad <= num_drone:
                pad = {
                    'ID': f'pad_{j:02}',
                    'mode': 'occupied',
                    'UAV_ID': f'UAV_{j:02}',
                    'is_charging': True
                }
                ugv['charging_pads'].append(pad)
            curr_pad += 1
        
        data['agents'].append(ugv)

    # Convert POIs to nodes
    for poi in pois:
        node = {
            'ID': f'n_{poi["id"]:04}',
            'type': 'air_only',
            'location': {
                'x': float(poi['x']),
                'y': float(poi['y'])
            },
            'time_last_service': 0.0
        }
        data['scenario']['nodes'].append(node)

    # Convert obstacles from JSON
    for obstacle in obstacles:
        obs = {
            'type': 'circle',
            'ID': f'o_{obstacle["id"]:02}',
            'location': {
                'x': float(obstacle['x']),
                'y': float(obstacle['y'])
            },
            'radius': float(obstacle['radius'])
        }
        data['scenario']['obstacles'].append(obs)

    return data, output_file

def save_scenario(data, output_path):
    """Save the scenario data to a YAML file"""
    with open(output_path, 'w') as file:
        yaml.dump(data, file, sort_keys=False, default_flow_style=False)

def main():
    parser = argparse.ArgumentParser(description='Create case study scenarios from JSON coordinate data')
    parser.add_argument('json_file', help='Path to JSON file with coordinate data')
    parser.add_argument('--ugvs', type=int, default=2, help='Number of UGVs (default: 2)')
    parser.add_argument('--drones', type=int, default=4, help='Number of drones/UAVs (default: 4)')
    parser.add_argument('--output', help='Output YAML filename (auto-generated if not provided)')
    parser.add_argument('--output-dir', default='.', help='Output directory (default: current directory)')
    
    args = parser.parse_args()
    
    try:
        # Load JSON data
        print(f"Loading coordinate data from {args.json_file}...")
        json_data = load_json_data(args.json_file)
        
        # Validate JSON structure
        if 'reference_point' not in json_data or 'pois' not in json_data or 'obstacles' not in json_data:
            raise ValueError("JSON file must contain 'reference_point', 'pois', and 'obstacles' keys")
        
        print(f"Found {len(json_data['pois'])} POIs and {len(json_data['obstacles'])} obstacles")
        print(f"Reference point (depot): {json_data['reference_point']['lat']:.6f}, {json_data['reference_point']['lng']:.6f}")
        
        # Create scenario
        print(f"Creating scenario with {args.ugvs} UGVs and {args.drones} drones...")
        scenario_data, filename = create_scenario_from_json(
            json_data, 
            num_ugv=args.ugvs, 
            num_drone=args.drones, 
            output_file=args.output
        )
        
        # Save to file
        output_path = os.path.join(args.output_dir, filename)
        save_scenario(scenario_data, output_path)
        
        print(f"✅ Case study scenario saved to: {output_path}")
        print(f"   - {args.ugvs} UGVs, {args.drones} UAVs")
        print(f"   - {len(json_data['pois'])} POIs (nodes)")
        print(f"   - {len(json_data['obstacles'])} obstacles")
        print(f"   - Depot at reference point (0,0)")
        
    except FileNotFoundError:
        print(f"❌ Error: JSON file '{args.json_file}' not found")
    except json.JSONDecodeError as e:
        print(f"❌ Error: Invalid JSON format - {e}")
    except ValueError as e:
        print(f"❌ Error: {e}")
    except Exception as e:
        print(f"❌ Unexpected error: {e}")

if __name__ == "__main__":
    main()