import yaml
import networkx as nx
import matplotlib.pyplot as plt

def read_yaml_file(file_name):
    with open(file_name, 'r') as file:
        data = yaml.safe_load(file)
    return data

def create_graph_from_yaml(data):
    G = nx.Graph()

    # Add nodes
    for i, node in enumerate(data['scenario']['nodes'], 1):
        node_id = f"N_{(i-1):02}"
        position = (node['location']['x'], node['location']['y'])
        G.add_node(node_id, pos=position, label=node_id)

    # Add UGV locations
    ugv_count = 1
    for agent in data['agents']:
        if agent['type'] == 'UGV':
            ugv_id = f"Depot_{ugv_count:02}"
            position = (agent['location']['x'], agent['location']['y'])
            G.add_node(ugv_id, pos=position, label=ugv_id)
            ugv_count += 1

    return G

def draw_graph(G):
    # Create a new figure with a specific size
    plt.figure(figsize=(10, 10))
    
    # Get node positions and labels
    pos = nx.get_node_attributes(G, 'pos')
    labels = nx.get_node_attributes(G, 'label')
    
    # Create axis
    ax = plt.gca()
    
    # Draw the graph
    nx.draw(G, pos, ax=ax, with_labels=True, labels=labels, 
            node_size=700, node_color='skyblue', 
            font_size=10, font_weight='bold')
    
    # Get the x and y coordinates to determine axis limits
    x_coords = [coord[0] for coord in pos.values()]
    y_coords = [coord[1] for coord in pos.values()]
    
    # Add some padding to the limits
    padding = 0.1  # 10% padding
    x_range = max(x_coords) - min(x_coords)
    y_range = max(y_coords) - min(y_coords)
    x_padding = x_range * padding
    y_padding = y_range * padding
    
    # Set axis limits with padding
    plt.xlim(min(x_coords) - x_padding, max(x_coords) + x_padding)
    plt.ylim(min(y_coords) - y_padding, max(y_coords) + y_padding)
    
    # Add grid
    plt.grid(True, linestyle='--', alpha=0.6)
    
    # Show axis lines
    ax.axhline(y=0, color='k', linestyle='-', alpha=0.3)
    ax.axvline(x=0, color='k', linestyle='-', alpha=0.3)
    
    # Add labels
    plt.xlabel('X Coordinate')
    plt.ylabel('Y Coordinate')
    plt.title('Node Network with Coordinates')
    
    plt.show()

# Example usage:
yaml_file = '/Users/jacksigler/Library/CloudStorage/OneDrive-ColoradoSchoolofMines/Mines/Spring_2025/PeCS/TeamPatrolling/test/Exp_01/plot_1_2_5_2.yaml'  # Specify your YAML file here
data = read_yaml_file(yaml_file)
graph = create_graph_from_yaml(data)
draw_graph(graph)