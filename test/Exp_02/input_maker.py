import yaml
import random
import matplotlib.pyplot as plt

# Configuration
NUM_POINTS_PER_CLUSTER = 4  # Number of points in each cluster
CLUSTER_SIZE = 2450          # Size of the cluster square (distance bounds, tightened)
CLUSTER_OFFSET = 1700       # Distance between cluster centers
MAX_X = 7000               # Maximum X coordinate for the graph
MAX_Y = 7000             # Maximum Y coordinate for the graph
FILE_NAME = "generated_input.yaml"  # Output YAML file name

# Possible cluster center configurations
CLUSTER_CONFIGURATIONS = [
    ((CLUSTER_OFFSET, CLUSTER_OFFSET), (MAX_X - CLUSTER_OFFSET, MAX_Y - CLUSTER_OFFSET)),
    ((CLUSTER_OFFSET, MAX_Y - CLUSTER_OFFSET), (MAX_X - CLUSTER_OFFSET, CLUSTER_OFFSET)),
    ((CLUSTER_OFFSET, CLUSTER_OFFSET), (CLUSTER_OFFSET, MAX_Y - CLUSTER_OFFSET)),
    ((MAX_X - CLUSTER_OFFSET, CLUSTER_OFFSET), (MAX_X - CLUSTER_OFFSET, MAX_Y - CLUSTER_OFFSET))
]

def generate_cluster(center_x, center_y, num_points, cluster_size, start_id):
    """Generates a cluster of points around a center."""
    points = []
    for i in range(num_points):
        x = random.uniform(center_x - cluster_size / 2, center_x + cluster_size / 2)
        y = random.uniform(center_y - cluster_size / 2, center_y + cluster_size / 2)
        points.append({
            "ID": f"n_{start_id + i:04}",
            "type": "air_only",
            "location": {"x": round(x, 2), "y": round(y, 2)},
            "time_last_service": 0.0
        })
    return points

def generate_input_file(file_name, cluster1, cluster2, ugv_location):
    """Generates a YAML input file with the given points and UGV location."""
    data = {
        'ID': f'state_every_UAV_action_{file_name}',
        'time': 0.0,
        'description': 'Generated scenario for UAV and UGV planning problem',
        'agents': [
            {
                'ID': 'UAV_01',
                'type': 'UAV',
                'subtype': 'standard',
                'location': {'x': 0.0, 'y': 0.0},
                'battery_state': {
                    'max_battery_energy': 360381.0,
                    'current_battery_energy': 360381.0
                },
                'stratum': 'docked',
                'charging_pad_ID': 'pad_01'
            },
            {
                'ID': 'UGV_01',
                'type': 'UGV',
                'subtype': 'standard',
                'location': ugv_location,
                'battery_state': {
                    'max_battery_energy': 25010000.0,
                    'current_battery_energy': 25010000.0
                },
                'charging_pads': [
                    {
                        'ID': 'pad_01',
                        'mode': 'occupied',
                        'UAV_ID': 'UAV_01',
                        'is_charging': True
                    }
                ]
            }
        ],
        'scenario': {
            'description': 'Generated scenario',
            'type': 'persistent_surveillance',
            'subtype': 'standard',
            'nodes': cluster1 + cluster2,
            'connections': []
        }
    }

    with open(file_name, 'w') as file:
        yaml.dump(data, file, sort_keys=False)

def visualize_clusters(cluster1_center, cluster2_center, cluster1, cluster2, ugv_location):
    """Visualizes the generated clusters and UGV location."""
    plt.figure(figsize=(10, 8))
    plt.scatter([p["location"]["x"] for p in cluster1], [p["location"]["y"] for p in cluster1], color="blue", label="Cluster 1")
    plt.scatter([p["location"]["x"] for p in cluster2], [p["location"]["y"] for p in cluster2], color="red", label="Cluster 2")
    plt.scatter(ugv_location["x"], ugv_location["y"], color="green", label="UGV Location", s=100)
    plt.title("Clusters of POI Points")
    plt.xlabel("X Coordinate")
    plt.ylabel("Y Coordinate")
    plt.grid(True)
    plt.legend()
    plt.show()

def main():
    # Select a random cluster configuration
    cluster1_center, cluster2_center = random.choice(CLUSTER_CONFIGURATIONS)

    # Generate clusters
    cluster1 = generate_cluster(cluster1_center[0], cluster1_center[1], NUM_POINTS_PER_CLUSTER, CLUSTER_SIZE, start_id=1)
    cluster2 = generate_cluster(cluster2_center[0], cluster2_center[1], NUM_POINTS_PER_CLUSTER, CLUSTER_SIZE, start_id=1 + NUM_POINTS_PER_CLUSTER)

    # Define UGV location (at the center of the graph)
    ugv_location = {'x': MAX_X / 2, 'y': MAX_Y / 2}

    # Generate YAML file
    generate_input_file(FILE_NAME, cluster1, cluster2, ugv_location)
    print(f"YAML input file '{FILE_NAME}' generated successfully.")

    # Visualize the clusters
    visualize_clusters(cluster1_center, cluster2_center, cluster1, cluster2, ugv_location)

if __name__ == "__main__":
    main()
