import re
import matplotlib.pyplot as plt
import networkx as nx

# Enum mapping
E_DroneActionTypes = {
    0: "LaunchFromUGV",
    1: "LandOnUGV",
    2: "MoveToNode",
    3: "MoveToUGV",
    4: "AtUGV",
    5: "KernelEnd",
}

E_UGVActionTypes = {
    0: "LaunchDrone",
    1: "ReceiveDrone",
    2: "MoveToNode",
    3: "MoveToWaypoint",
    4: "MoveToDepot",
    5: "AtDepot",
    6: "KernelEnd",
}

# Sample input
input_text = """
Drone 0:
  [27] 4(0) : (0.000000, 0.000000) - 0.000000
  [28] 0(0) : (4705.100000, 1651.300000) - 3334.304953
  ...
UGV 0:
  [18] 5(-1) : (0.000000, 0.000000) - 0.000000
  [19] 3(0) : (4705.100000, 1651.300000) - 3324.304953
  ...
"""

def parse_actions(input_text):
    """Parses the input text into actions for drones and UGVs."""
    drones = {}
    ugvs = {}
    current_entity = None
    entity_id = None

    # Regex for actions
    action_regex = re.compile(
        r"\[(\d+)\] (\d+)\((-?\d+)\) : \((-?\d+\.\d+), (-?\d+\.\d+)\) - (-?\d+\.\d+)"
    )

    for line in input_text.splitlines():
        line = line.strip()
        if line.startswith("Drone"):
            current_entity = drones
            entity_id = int(line.split()[1][:-1])
            current_entity[entity_id] = []
        elif line.startswith("UGV"):
            current_entity = ugvs
            entity_id = int(line.split()[1][:-1])
            current_entity[entity_id] = []
        elif action_match := action_regex.match(line):
            index, action_type, details, x, y, time = action_match.groups()
            current_entity[entity_id].append({
                "index": int(index),
                "action_type": int(action_type),
                "details": int(details),
                "x": float(x),
                "y": float(y),
                "time": float(time),
            })

    return drones, ugvs

def visualize_movements(drones, ugvs):
    """Visualizes drone and UGV movements using a graph."""
    G = nx.DiGraph()

    # Add drone nodes and edges
    for drone_id, actions in drones.items():
        for i in range(1, len(actions)):
            prev = actions[i - 1]
            curr = actions[i]
            G.add_edge(
                f"Drone{drone_id}_{prev['index']}",
                f"Drone{drone_id}_{curr['index']}",
                weight=curr['time'],
                action=E_DroneActionTypes.get(curr['action_type'], "Unknown"),
            )

    # Add UGV nodes and edges
    for ugv_id, actions in ugvs.items():
        for i in range(1, len(actions)):
            prev = actions[i - 1]
            curr = actions[i]
            G.add_edge(
                f"UGV{ugv_id}_{prev['index']}",
                f"UGV{ugv_id}_{curr['index']}",
                weight=curr['time'],
                action=E_UGVActionTypes.get(curr['action_type'], "Unknown"),
            )

    # Plot graph
    pos = nx.spring_layout(G)
    labels = nx.get_edge_attributes(G, "action")
    plt.figure(figsize=(12, 8))
    nx.draw(G, pos, with_labels=True, node_color="lightblue", font_size=10, node_size=3000)
    nx.draw_networkx_edge_labels(G, pos, edge_labels=labels)
    plt.title("Drone and UGV Movements")
    plt.show()

# Parse actions
drones, ugvs = parse_actions(input_text)

# Visualize movements
visualize_movements(drones, ugvs)
