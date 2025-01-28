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
	pos = nx.get_node_attributes(G, 'pos')
	labels = nx.get_node_attributes(G, 'label')
	nx.draw(G, pos, with_labels=True, labels=labels, node_size=700, node_color='skyblue', font_size=10, font_weight='bold')
	plt.show()

# Example usage:
yaml_file = 'plot_02_01_05_01.yaml'  # Specify your YAML file here
data = read_yaml_file(yaml_file)
graph = create_graph_from_yaml(data)
draw_graph(graph)