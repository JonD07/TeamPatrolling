#!/usr/bin/env python3
"""
Plan Visualizer - Google Maps visualization tool for output_plan.yaml files
Generates standalone HTML files for visualization
"""

import yaml
import json
import argparse
import math
import os
import webbrowser

class PlanVisualizer:
    def __init__(self, plan_file, scenario_file, reference_point=None):
        self.plan_file = plan_file
        self.scenario_file = scenario_file
        self.plan_data = None
        self.scenario_data = None
        self.reference_point = reference_point  # Can be overridden via command line
        self.nodes = []
        self.obstacles = []
        self.agents = []
        self.timeline = []
        
    def load_data(self):
        """Load and parse both YAML files"""
        print(f"Loading plan from: {self.plan_file}")
        with open(self.plan_file, 'r') as f:
            self.plan_data = yaml.safe_load(f)
            
        print(f"Loading scenario from: {self.scenario_file}")
        with open(self.scenario_file, 'r') as f:
            self.scenario_data = yaml.safe_load(f)
            
        self._extract_scenario_data()
        self._extract_plan_data()
        
    def _extract_scenario_data(self):
        """Extract nodes, obstacles, and reference point from scenario"""
        scenario = self.scenario_data.get('scenario', {})
        
        # Extract nodes (POIs)
        self.nodes = []
        for node in scenario.get('nodes', []):
            self.nodes.append({
                'id': node['ID'],
                'type': node['type'],
                'x': node['location']['x'],
                'y': node['location']['y'],
                'last_service': node.get('time_last_service', 0.0)
            })
            
        # Extract obstacles
        self.obstacles = []
        for obstacle in scenario.get('obstacles', []):
            self.obstacles.append({
                'id': obstacle['ID'],
                'type': obstacle['type'],
                'x': obstacle['location']['x'],
                'y': obstacle['location']['y'],
                'radius': obstacle['radius']
            })
            
        # Set reference point (depot at origin for coordinate conversion)
        if not self.reference_point:  # Only set if not already provided
            # Try to extract from scenario data first
            if 'reference_point' in self.scenario_data:
                ref_point = self.scenario_data['reference_point']
                self.reference_point = {'lat': ref_point['lat'], 'lng': ref_point['lng']}
                print(f"Using reference point from scenario: {self.reference_point['lat']}, {self.reference_point['lng']}")
            else:
                # Fall back to default
                self.reference_point = {'lat': 38.125592, 'lng': -121.835519}  # Default from case study
                print(f"Using default reference point: {self.reference_point['lat']}, {self.reference_point['lng']}")
        
        print(f"Loaded {len(self.nodes)} nodes and {len(self.obstacles)} obstacles")
        
    def _extract_plan_data(self):
        """Extract agent plans and create timeline"""
        self.agents = []
        self.timeline = []
        
        # Process each agent's individual plan
        for agent_plan in self.plan_data.get('individual_plans', []):
            agent_id = agent_plan['agent_ID']
            actions = agent_plan['actions']
            
            agent = {
                'id': agent_id,
                'type': 'UAV' if 'UAV' in agent_id else 'UGV',
                'actions': []
            }
            
            # Process each action for this agent
            for action in actions:
                action_data = {
                    'type': action['type'],
                    'start_time': action['start_time'],
                    'end_time': action['end_time'],
                    'parameters': action.get('task_parameters', {})
                }
                
                # Extract location information
                location = action_data['parameters'].get('location', {})
                if location:
                    action_data['x'] = location.get('x', 0)
                    action_data['y'] = location.get('y', 0)
                    
                # Extract origin/destination for movement actions
                if 'origin' in action_data['parameters']:
                    origin = action_data['parameters']['origin']
                    action_data['origin_x'] = origin.get('x', 0)
                    action_data['origin_y'] = origin.get('y', 0)
                    
                if 'destination' in action_data['parameters']:
                    dest = action_data['parameters']['destination']
                    action_data['dest_x'] = dest.get('x', 0)
                    action_data['dest_y'] = dest.get('y', 0)
                
                agent['actions'].append(action_data)
                
                # Add to global timeline
                self.timeline.append({
                    'agent_id': agent_id,
                    'action': action_data,
                    'start_time': action['start_time'],
                    'end_time': action['end_time']
                })
                
            self.agents.append(agent)
            
        # Sort timeline by start time
        self.timeline.sort(key=lambda x: x['start_time'])
        
        print(f"Loaded plans for {len(self.agents)} agents with {len(self.timeline)} total actions")
        
    def convert_to_latlng(self, x, y):
        """Convert local coordinates to lat/lng using reference point"""
        # Earth radius in meters
        R = 6371000
        
        # Convert x,y to lat,lng offset from reference point
        lat_offset = y / R * 180 / math.pi
        lng_offset = x / (R * math.cos(self.reference_point['lat'] * math.pi / 180)) * 180 / math.pi
        
        return {
            'lat': self.reference_point['lat'] + lat_offset,
            'lng': self.reference_point['lng'] + lng_offset
        }
        
    def get_visualization_data(self):
        """Prepare all data for visualization"""
        # Convert all coordinates to lat/lng
        viz_nodes = []
        for node in self.nodes:
            coords = self.convert_to_latlng(node['x'], node['y'])
            viz_nodes.append({
                'id': node['id'],
                'type': node['type'],
                'lat': coords['lat'],
                'lng': coords['lng'],
                'x': node['x'],
                'y': node['y']
            })
            
        viz_obstacles = []
        for obstacle in self.obstacles:
            coords = self.convert_to_latlng(obstacle['x'], obstacle['y'])
            viz_obstacles.append({
                'id': obstacle['id'],
                'type': obstacle['type'],
                'lat': coords['lat'],
                'lng': coords['lng'],
                'radius': obstacle['radius'],
                'x': obstacle['x'],
                'y': obstacle['y']
            })
            
        viz_agents = []
        for agent in self.agents:
            agent_actions = []
            for action in agent['actions']:
                action_viz = action.copy()
                
                # Convert coordinates
                if 'x' in action and 'y' in action:
                    coords = self.convert_to_latlng(action['x'], action['y'])
                    action_viz['lat'] = coords['lat']
                    action_viz['lng'] = coords['lng']
                    
                if 'origin_x' in action and 'origin_y' in action:
                    coords = self.convert_to_latlng(action['origin_x'], action['origin_y'])
                    action_viz['origin_lat'] = coords['lat']
                    action_viz['origin_lng'] = coords['lng']
                    
                if 'dest_x' in action and 'dest_y' in action:
                    coords = self.convert_to_latlng(action['dest_x'], action['dest_y'])
                    action_viz['dest_lat'] = coords['lat']
                    action_viz['dest_lng'] = coords['lng']
                    
                agent_actions.append(action_viz)
                
            viz_agents.append({
                'id': agent['id'],
                'type': agent['type'],
                'actions': agent_actions
            })
            
        return {
            'reference_point': self.reference_point,
            'nodes': viz_nodes,
            'obstacles': viz_obstacles,
            'agents': viz_agents,
            'timeline': self.timeline,
            'total_time': self.plan_data.get('end_time', 0),
            'plan_id': self.plan_data.get('ID', 'Unknown Plan')
        }

    def generate_html(self, output_file):
        """Generate standalone HTML file with embedded data"""
        data = self.get_visualization_data()
        
        html_content = f'''<!DOCTYPE html>
<html>
<head>
    <title>Plan Visualizer - {data["plan_id"]}</title>
    <style>
        body {{ margin: 0; font-family: Arial, sans-serif; }}
        #map {{ height: 70vh; }}
        #controls {{ 
            height: 30vh; 
            padding: 20px; 
            background: #f5f5f5; 
            border-top: 1px solid #ddd;
        }}
        .control-row {{
            display: flex;
            align-items: center;
            gap: 20px;
            margin-bottom: 15px;
        }}
        #timeline {{
            flex: 1;
            margin: 0 10px;
        }}
        .info-panel {{
            display: grid;
            grid-template-columns: 1fr 1fr 1fr;
            gap: 20px;
            margin-top: 15px;
        }}
        .info-box {{
            background: white;
            padding: 10px;
            border-radius: 5px;
            border: 1px solid #ddd;
        }}
        .info-box h4 {{
            margin: 0 0 10px 0;
            color: #333;
        }}
        button {{
            padding: 8px 16px;
            border: none;
            border-radius: 4px;
            cursor: pointer;
            background: #007cba;
            color: white;
        }}
        button:hover {{ background: #005a87; }}
        button:disabled {{ background: #ccc; cursor: not-allowed; }}
        .legend {{
            position: absolute;
            top: 10px;
            right: 10px;
            background: white;
            padding: 10px;
            border-radius: 5px;
            box-shadow: 0 2px 10px rgba(0,0,0,0.3);
            z-index: 1000;
        }}
        .legend-item {{
            display: flex;
            align-items: center;
            gap: 8px;
            margin: 5px 0;
        }}
        .legend-color {{
            width: 16px;
            height: 16px;
            border-radius: 50%;
        }}
        .legend-line {{
            width: 20px;
            height: 3px;
            border-radius: 2px;
        }}
        .view-controls {{
            position: absolute;
            top: 10px;
            left: 10px;
            background: white;
            padding: 15px;
            border-radius: 5px;
            box-shadow: 0 2px 10px rgba(0,0,0,0.3);
            z-index: 1000;
            max-width: 250px;
        }}
        .control-group {{
            margin-bottom: 15px;
        }}
        .control-group h5 {{
            margin: 0 0 8px 0;
            color: #333;
            font-size: 14px;
            border-bottom: 1px solid #eee;
            padding-bottom: 5px;
        }}
        .control-group label {{
            display: block;
            margin: 5px 0;
            font-size: 13px;
            cursor: pointer;
        }}
        .control-group input[type="checkbox"] {{
            margin-right: 8px;
        }}
        .preset-buttons {{
            display: flex;
            gap: 5px;
            flex-wrap: wrap;
        }}
        .preset-btn {{
            padding: 4px 8px;
            font-size: 11px;
            border: 1px solid #ddd;
            background: #f8f9fa;
            border-radius: 3px;
            cursor: pointer;
        }}
        .preset-btn:hover {{
            background: #e9ecef;
        }}
    </style>
</head>
<body>
    <div id="map"></div>
    
    <div class="view-controls">
        <div class="control-group">
            <h5>Quick Presets</h5>
            <div class="preset-buttons">
                <button class="preset-btn" onclick="setPreset('all')">All</button>
                <button class="preset-btn" onclick="setPreset('uav-only')">UAVs Only</button>
                <button class="preset-btn" onclick="setPreset('ugv-only')">UGVs Only</button>
                <button class="preset-btn" onclick="setPreset('paths-only')">Paths Only</button>
                <button class="preset-btn" onclick="setPreset('static')">Static Elements</button>
            </div>
        </div>
        
        <div class="control-group">
            <h5>Map Elements</h5>
            <label><input type="checkbox" id="showDepot" checked> Depot</label>
            <label><input type="checkbox" id="showPOIs" checked> POIs (Nodes)</label>
            <label><input type="checkbox" id="showObstacles" checked> Obstacles</label>
        </div>
        
        <div class="control-group">
            <h5>Agents & Paths</h5>
            <label><input type="checkbox" id="showUAVs" checked> UAV Markers</label>
            <label><input type="checkbox" id="showUGVs" checked> UGV Markers</label>
            <label><input type="checkbox" id="showUAVPaths" checked> UAV Paths</label>
            <label><input type="checkbox" id="showUGVPaths" checked> UGV Paths</label>
        </div>
    </div>
    
    <div class="legend">
        <h4>Legend</h4>
        <div class="legend-item">
            <div class="legend-color" style="background: #4CAF50;"></div>
            <span>POIs (Nodes)</span>
        </div>
        <div class="legend-item">
            <div class="legend-color" style="background: #F44336;"></div>
            <span>Obstacles</span>
        </div>
        <div class="legend-item">
            <div class="legend-color" style="background: #2196F3;"></div>
            <span>UAVs</span>
        </div>
        <div class="legend-item">
            <div class="legend-color" style="background: #FF9800;"></div>
            <span>UGVs</span>
        </div>
        <div class="legend-item">
            <div class="legend-color" style="background: #000000;"></div>
            <span>Depot (0,0)</span>
        </div>
        <div class="legend-item">
            <div class="legend-line" style="background: #2196F3;"></div>
            <span>UAV Paths</span>
        </div>
        <div class="legend-item">
            <div class="legend-line" style="background: #FF9800;"></div>
            <span>UGV Paths</span>
        </div>
    </div>
    
    <div id="controls">
        <div class="control-row">
            <button id="playBtn">Play</button>
            <button id="pauseBtn" disabled>Pause</button>
            <button id="resetBtn">Reset</button>
            <span>Time: <span id="currentTime">0.0</span> / <span id="totalTime">0.0</span> seconds</span>
            <input type="range" id="timeline" min="0" max="100" value="0" step="0.1">
            <span>Speed: </span>
            <select id="speedSelect">
                <option value="0.5">0.5x</option>
                <option value="1" selected>1x</option>
                <option value="2">2x</option>
                <option value="5">5x</option>
                <option value="10">10x</option>
            </select>
        </div>
        
        <div class="info-panel">
            <div class="info-box">
                <h4>Current Actions</h4>
                <div id="currentActions">No active actions</div>
            </div>
            <div class="info-box">
                <h4>Agent Status</h4>
                <div id="agentStatus">Loading...</div>
            </div>
            <div class="info-box">
                <h4>Plan Info</h4>
                <div id="planInfo">Loading...</div>
            </div>
        </div>
    </div>

    <script>
        // Embedded data
        const data = {json.dumps(data, indent=2)};
        
        let map;
        let currentTime = 0;
        let totalTime = 0;
        let isPlaying = false;
        let animationSpeed = 1;
        let agentMarkers = {{}};
        let nodeMarkers = {{}};
        let obstacleCircles = {{}};
        let pathLines = {{}};
        let depotMarker = null;
        let uavMarkers = {{}};
        let ugvMarkers = {{}};
        let uavPaths = {{}};
        let ugvPaths = {{}};

        function initMap() {{
            // Initialize map
            map = new google.maps.Map(document.getElementById('map'), {{
                zoom: 16,
                center: data.reference_point,
                mapTypeId: 'satellite'
            }});
            
            totalTime = data.total_time;
            document.getElementById('totalTime').textContent = totalTime.toFixed(1);
            document.getElementById('timeline').max = totalTime;
            document.getElementById('planInfo').innerHTML = `
                <strong>${{data.plan_id}}</strong><br>
                Nodes: ${{data.nodes.length}}<br>
                Obstacles: ${{data.obstacles.length}}<br>
                Agents: ${{data.agents.length}}
            `;

            // Add depot marker at reference point
            depotMarker = new google.maps.Marker({{
                position: data.reference_point,
                map: map,
                title: 'Depot (0,0)',
                icon: {{
                    path: 'M -8,-8 L 8,-8 L 8,8 L -8,8 Z',
                    scale: 1,
                    fillColor: 'transparent',
                    fillOpacity: 0,
                    strokeColor: '#000000',
                    strokeWeight: 2
                }}
            }});

            // Add nodes to map
            data.nodes.forEach(node => {{
                const marker = new google.maps.Marker({{
                    position: {{ lat: node.lat, lng: node.lng }},
                    map: map,
                    title: `${{node.id}} (${{node.x}}, ${{node.y}})`,
                    icon: {{
                        path: google.maps.SymbolPath.CIRCLE,
                        scale: 8,
                        fillColor: '#4CAF50',
                        fillOpacity: 0.8,
                        strokeColor: '#2E7D32',
                        strokeWeight: 2
                    }}
                }});
                nodeMarkers[node.id] = marker;
            }});

            // Add obstacles to map
            data.obstacles.forEach(obstacle => {{
                const circle = new google.maps.Circle({{
                    strokeColor: '#F44336',
                    strokeOpacity: 0.8,
                    strokeWeight: 2,
                    fillColor: '#F44336',
                    fillOpacity: 0.2,
                    map: map,
                    center: {{ lat: obstacle.lat, lng: obstacle.lng }},
                    radius: obstacle.radius
                }});
                obstacleCircles[obstacle.id] = circle;
            }});

            // Initialize agent markers and paths
            data.agents.forEach(agent => {{
                const color = agent.type === 'UAV' ? '#2196F3' : '#FF9800';
                const marker = new google.maps.Marker({{
                    position: data.reference_point,
                    map: map,
                    title: agent.id,
                    icon: {{
                        path: agent.type === 'UAV' ? google.maps.SymbolPath.FORWARD_CLOSED_ARROW : google.maps.SymbolPath.CIRCLE,
                        scale: agent.type === 'UAV' ? 4 : 6,
                        fillColor: color,
                        fillOpacity: 1,
                        strokeColor: '#fff',
                        strokeWeight: 2
                    }}
                }});
                agentMarkers[agent.id] = marker;
                
                // Separate UAV and UGV markers for filtering
                if (agent.type === 'UAV') {{
                    uavMarkers[agent.id] = marker;
                }} else {{
                    ugvMarkers[agent.id] = marker;
                }}

                // Create path for this agent by collecting all movement coordinates
                const pathCoordinates = [];
                
                // Start from depot
                pathCoordinates.push(data.reference_point);
                
                agent.actions.forEach(action => {{
                    if (action.type === 'move_to_location' && action.dest_lat && action.dest_lng) {{
                        // Add destination to path
                        pathCoordinates.push({{ lat: action.dest_lat, lng: action.dest_lng }});
                    }} else if (action.lat && action.lng && action.type !== 'start' && action.type !== 'end') {{
                        // Add position for service actions, takeoffs, landings, etc.
                        pathCoordinates.push({{ lat: action.lat, lng: action.lng }});
                    }}
                }});

                // Create the path polyline if we have multiple points
                if (pathCoordinates.length > 1) {{
                    const path = new google.maps.Polyline({{
                        path: pathCoordinates,
                        geodesic: true,
                        strokeColor: color,
                        strokeOpacity: 0.7,
                        strokeWeight: agent.type === 'UAV' ? 2 : 3,
                        map: map
                    }});
                    pathLines[agent.id] = path;
                    
                    // Separate UAV and UGV paths for filtering
                    if (agent.type === 'UAV') {{
                        uavPaths[agent.id] = path;
                    }} else {{
                        ugvPaths[agent.id] = path;
                    }}
                    
                    console.log(`Created path for ${{agent.id}} with ${{pathCoordinates.length}} points`);
                }} else {{
                    console.log(`No path created for ${{agent.id}} - insufficient coordinates`);
                }}
            }});

            // Set up controls
            setupControls();
            updateVisualization();
        }}

        function setupControls() {{
            document.getElementById('playBtn').onclick = () => {{
                isPlaying = true;
                document.getElementById('playBtn').disabled = true;
                document.getElementById('pauseBtn').disabled = false;
                animate();
            }};

            document.getElementById('pauseBtn').onclick = () => {{
                isPlaying = false;
                document.getElementById('playBtn').disabled = false;
                document.getElementById('pauseBtn').disabled = true;
            }};

            document.getElementById('resetBtn').onclick = () => {{
                isPlaying = false;
                currentTime = 0;
                document.getElementById('timeline').value = 0;
                document.getElementById('playBtn').disabled = false;
                document.getElementById('pauseBtn').disabled = true;
                updateVisualization();
            }};

            document.getElementById('timeline').oninput = (e) => {{
                currentTime = parseFloat(e.target.value);
                updateVisualization();
            }};

            document.getElementById('speedSelect').onchange = (e) => {{
                animationSpeed = parseFloat(e.target.value);
            }};

            // Set up view filter controls
            document.getElementById('showDepot').onchange = (e) => {{
                if (depotMarker) depotMarker.setVisible(e.target.checked);
            }};
            
            document.getElementById('showPOIs').onchange = (e) => {{
                Object.values(nodeMarkers).forEach(marker => {{
                    marker.setVisible(e.target.checked);
                }});
            }};
            
            document.getElementById('showObstacles').onchange = (e) => {{
                Object.values(obstacleCircles).forEach(circle => {{
                    circle.setVisible(e.target.checked);
                }});
            }};
            
            document.getElementById('showUAVs').onchange = (e) => {{
                Object.values(uavMarkers).forEach(marker => {{
                    marker.setVisible(e.target.checked);
                }});
            }};
            
            document.getElementById('showUGVs').onchange = (e) => {{
                Object.values(ugvMarkers).forEach(marker => {{
                    marker.setVisible(e.target.checked);
                }});
            }};
            
            document.getElementById('showUAVPaths').onchange = (e) => {{
                Object.values(uavPaths).forEach(path => {{
                    if (path) path.setVisible(e.target.checked);
                }});
            }};
            
            document.getElementById('showUGVPaths').onchange = (e) => {{
                Object.values(ugvPaths).forEach(path => {{
                    if (path) path.setVisible(e.target.checked);
                }});
            }};
        }}

        function animate() {{
            if (!isPlaying) return;

            currentTime += 0.1 * animationSpeed;
            if (currentTime >= totalTime) {{
                currentTime = totalTime;
                isPlaying = false;
                document.getElementById('playBtn').disabled = false;
                document.getElementById('pauseBtn').disabled = true;
            }}

            document.getElementById('timeline').value = currentTime;
            updateVisualization();

            if (isPlaying) {{
                setTimeout(animate, 100);
            }}
        }}

        function updateVisualization() {{
            document.getElementById('currentTime').textContent = currentTime.toFixed(1);

            // Update agent positions and show current actions
            const currentActions = [];
            const agentStatuses = {{}};

            data.agents.forEach(agent => {{
                // Find current action for this agent
                let currentAction = null;
                for (const action of agent.actions) {{
                    if (currentTime >= action.start_time && currentTime <= action.end_time) {{
                        currentAction = action;
                        break;
                    }}
                }}

                if (currentAction) {{
                    // Update agent position based on action
                    let position = data.reference_point;
                    
                    if (currentAction.type === 'move_to_location' && currentAction.origin_lat && currentAction.dest_lat) {{
                        // Interpolate position during movement
                        const progress = (currentTime - currentAction.start_time) / (currentAction.end_time - currentAction.start_time);
                        const lat = currentAction.origin_lat + (currentAction.dest_lat - currentAction.origin_lat) * progress;
                        const lng = currentAction.origin_lng + (currentAction.dest_lng - currentAction.origin_lng) * progress;
                        position = {{ lat, lng }};
                    }} else if (currentAction.lat && currentAction.lng) {{
                        position = {{ lat: currentAction.lat, lng: currentAction.lng }};
                    }}

                    agentMarkers[agent.id].setPosition(position);
                    
                    currentActions.push(`${{agent.id}}: ${{currentAction.type}}`);
                    agentStatuses[agent.id] = currentAction.type;
                }} else {{
                    // Agent is idle or finished
                    agentStatuses[agent.id] = 'idle';
                    agentMarkers[agent.id].setPosition(data.reference_point);
                }}
            }});

            // Update info panels
            document.getElementById('currentActions').innerHTML = currentActions.length > 0 ? 
                currentActions.join('<br>') : 'No active actions';

            const statusHtml = Object.entries(agentStatuses)
                .map(([id, status]) => `${{id}}: ${{status}}`)
                .join('<br>');
            document.getElementById('agentStatus').innerHTML = statusHtml;
        }}

        function setPreset(preset) {{
            // Reset all checkboxes
            const checkboxes = ['showDepot', 'showPOIs', 'showObstacles', 'showUAVs', 'showUGVs', 'showUAVPaths', 'showUGVPaths'];
            
            switch(preset) {{
                case 'all':
                    checkboxes.forEach(id => {{
                        document.getElementById(id).checked = true;
                        document.getElementById(id).dispatchEvent(new Event('change'));
                    }});
                    break;
                    
                case 'uav-only':
                    checkboxes.forEach(id => document.getElementById(id).checked = false);
                    ['showDepot', 'showPOIs', 'showObstacles', 'showUAVs', 'showUAVPaths'].forEach(id => {{
                        document.getElementById(id).checked = true;
                        document.getElementById(id).dispatchEvent(new Event('change'));
                    }});
                    break;
                    
                case 'ugv-only':
                    checkboxes.forEach(id => document.getElementById(id).checked = false);
                    ['showDepot', 'showPOIs', 'showObstacles', 'showUGVs', 'showUGVPaths'].forEach(id => {{
                        document.getElementById(id).checked = true;
                        document.getElementById(id).dispatchEvent(new Event('change'));
                    }});
                    break;
                    
                case 'paths-only':
                    checkboxes.forEach(id => document.getElementById(id).checked = false);
                    ['showDepot', 'showUAVPaths', 'showUGVPaths'].forEach(id => {{
                        document.getElementById(id).checked = true;
                        document.getElementById(id).dispatchEvent(new Event('change'));
                    }});
                    break;
                    
                case 'static':
                    checkboxes.forEach(id => document.getElementById(id).checked = false);
                    ['showDepot', 'showPOIs', 'showObstacles'].forEach(id => {{
                        document.getElementById(id).checked = true;
                        document.getElementById(id).dispatchEvent(new Event('change'));
                    }});
                    break;
            }}
        }}

        window.initMap = initMap;
    </script>
    
    <script async defer src="https://maps.googleapis.com/maps/api/js?key=AIzaSyBdHvpnztZfdKPJSz8ctXWCDnaIFxMEWkg&callback=initMap"></script>
</body>
</html>'''
        
        # Create visualizations directory if it doesn't exist
        viz_dir = os.path.dirname(output_file)
        if viz_dir and not os.path.exists(viz_dir):
            os.makedirs(viz_dir)
            
        # Write HTML file
        with open(output_file, 'w') as f:
            f.write(html_content)
            
        print(f"✅ Visualization saved to: {output_file}")
        return output_file

def main():
    parser = argparse.ArgumentParser(description='Visualize patrol plan execution on Google Maps')
    parser.add_argument('scenario_file', help='Path to scenario YAML file')
    parser.add_argument('plan_file', help='Path to output_plan.yaml file')
    parser.add_argument('--output', help='Output HTML file path (default: visualizations/{scenario_name}_visualization.html)')
    parser.add_argument('--reference-point', help='Reference point coordinates as "lat,lng" (overrides scenario file)', type=str)
    parser.add_argument('--open-browser', action='store_true', help='Open browser automatically after generating')
    
    args = parser.parse_args()
    
    # Set default output path based on scenario file name
    if not args.output:
        scenario_basename = os.path.splitext(os.path.basename(args.scenario_file))[0]
        args.output = os.path.join('visualizations', f'{scenario_basename}_visualization.html')
    
    # Parse reference point if provided
    reference_point = None
    if args.reference_point:
        try:
            lat_str, lng_str = args.reference_point.split(',')
            reference_point = {'lat': float(lat_str.strip()), 'lng': float(lng_str.strip())}
            print(f"Using command-line reference point: {reference_point['lat']}, {reference_point['lng']}")
        except (ValueError, IndexError):
            print(f"❌ Invalid reference point format: {args.reference_point}. Expected format: 'lat,lng'")
            return 1
    
    # Create visualizer and load data
    visualizer = PlanVisualizer(args.plan_file, args.scenario_file, reference_point)
    try:
        visualizer.load_data()
    except Exception as e:
        print(f"❌ Error loading data: {e}")
        return 1
        
    # Generate HTML file
    try:
        output_file = visualizer.generate_html(args.output)
        
        if args.open_browser:
            print("Opening browser...")
            webbrowser.open(f'file://{os.path.abspath(output_file)}')
            
    except Exception as e:
        print(f"❌ Error generating visualization: {e}")
        return 1
        
    print(f"📱 Open the file in your browser: file://{os.path.abspath(output_file)}")
    return 0

if __name__ == '__main__':
    exit(main())