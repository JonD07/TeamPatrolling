import yaml
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import numpy as np


def plot_paths_from_yaml(yaml_file, obstacle_file=None):
    # Load the YAML file
    with open(yaml_file, 'r') as file:
        data = yaml.safe_load(file)

    # Colors for different agents
    colors = plt.cm.get_cmap('tab10', len(data['individual_plans']))

    # Create a plot
    plt.figure(figsize=(10, 10))
    plt.title('Agent Paths')
    plt.xlabel('X Coordinate')
    plt.ylabel('Y Coordinate')

    # Plot paths for each agent
    for i, plan in enumerate(data['individual_plans']):
        agent_id = plan['agent_ID']
        actions = plan['actions']

        # Collect all locations
        for action in actions:
            action_type = action.get('type', {})

            if action_type == 'move_to_location':
                path_x = []
                path_y = []
                task_params = action.get('task_parameters', {})
                origin = task_params['origin']
                path_x.append(origin['x'])
                path_y.append(origin['y'])
                dest = task_params['destination']
                path_x.append(dest['x'])
                path_y.append(dest['y'])
                # Plot the path
                plt.plot(path_x, path_y, marker='o', color=colors(i))
            elif action_type == 'service_node':
                task_params = action.get('task_parameters', {})
                local = task_params['location']
                # noxe_x = append(local['x'])
                # noxe_y = append(local['y'])
                # plt.plot([local['x']], [local['y']], 'ro', label=task_params['node_ID'], color='black')
                plt.annotate(text=task_params['node_ID'], xy=(local['x'], local['y']), textcoords="offset points", xytext=(0, 10), ha='center', label="")
            elif action_type == 'swap_battery':
                task_params = action.get('task_parameters', {})
                local = task_params['location']
                # noxe_x = append(local['x'])
                # noxe_y = append(local['y'])
                # plt.plot([local['x']], [local['y']], 'ro', label=task_params['node_ID'], color='black')
                plt.annotate(text="depot", xy=(local['x'], local['y']), textcoords="offset points", xytext=(0, 10), ha='center', label="")

    # Optional: Plot obstacles if a file is provided
    if obstacle_file:
        with open(obstacle_file, 'r') as file:
            obstacle_data = yaml.safe_load(file)

        obstacles = obstacle_data.get('scenario', {}).get('obstacles', [])
        for obs in obstacles:
            if obs.get('type') == 'circle':
                loc = obs['location']
                radius = obs['radius']
                circle = patches.Circle((loc['x'], loc['y']), radius, color='red', alpha=0.3, label=None)
                plt.gca().add_patch(circle)

    # Show legend
    plt.legend()
    plt.grid(True)
    plt.show()

plot_paths_from_yaml('obstacles_tests/midsolve_plan.yaml', 'obstacles_tests/plot_1_2_10_100_4.yaml')
plot_paths_from_yaml('obstacles_tests/output_plan.yaml', 'obstacles_tests/plot_1_2_10_100_4.yaml')
