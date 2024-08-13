#include "PatrollingInput.h"

/*
 * Basic constructor. This expects an input file path. The input file should be a yaml file.
 *
 * Sets:
 * i in I, set of PoI, |I| = N
 * j in R^a, set of drones, |R^a| = Ma
 * j in R^g, set of UGVs, |R^g| = Mg
 */
PatrollingInput::PatrollingInput(std::string input_path) {
	if(SANITY_PRINT)
		printf("Reading input YAML file\n");

	// Read status
	bool read_success = true;

	try {
		// Load the YAML file
		YAML::Node config = YAML::LoadFile(input_path);

        // Extract and print the basic information
        std::string id = config["ID"].as<std::string>();
        double time = config["time"].as<double>();
        std::string description = config["description"].as<std::string>();

        std::cout << "ID: " << id << std::endl;
        std::cout << "Time: " << time << std::endl;
        std::cout << "Description: " << description << std::endl;

        // Parse and print agents
        const YAML::Node& agents = config["agents"];
        parseAgents(agents);

        // Parse and print scenario
        const YAML::Node& scenario = config["scenario"];
        parseScenario(scenario);

	} catch (const std::exception &e) {
		fprintf(stderr, "[ERROR]:PatrollingInput() %s\n", e.what());
		read_success = false;
	}

	// Verify that we successfully read the input file
	if(!read_success) {
		// Input file not formatted correctly, hard fail!
		fprintf(stderr, "[MASPInput::MASPInput] : Input file format off\n");
		exit(1);
	}
	else if(SANITY_PRINT) {
		printf("Successfully read input!\n");
		printf("N = %d, Ma = %d, Mg = %d\n\n", GetN(), GetMg(), GetMa());
	}
}

// Parse each agent
void PatrollingInput::parseAgents(const YAML::Node& agentsNode) {
	for (const auto& agentNode : agentsNode) {
        Agent agent;
        agent.ID = agentNode["ID"].as<std::string>();
        agent.type = agentNode["type"].as<std::string>();
        agent.subtype = agentNode["subtype"].as<std::string>();
        agent.location.x = agentNode["location"]["x"].as<double>();
        agent.location.y = agentNode["location"]["y"].as<double>();
        agent.battery_state.max_battery_energy = agentNode["battery_state"]["max_battery_energy"].as<double>();
        agent.battery_state.current_battery_energy = agentNode["battery_state"]["current_battery_energy"].as<double>();


		if(DEBUG_PATROLINPUT) {
			std::cout << "Agent ID: " << agent.ID << std::endl;
			std::cout << "  Type: " << agent.type << std::endl;
			std::cout << "  Subtype: " << agent.subtype << std::endl;
			std::cout << "  Location: (" << agent.location.x << ", " << agent.location.y << ")" << std::endl;
			std::cout << "  Max Battery Energy: " << agent.battery_state.max_battery_energy << std::endl;
			std::cout << "  Current Battery Energy: " << agent.battery_state.current_battery_energy << std::endl;
		}

		// What type of agent is this?
		if (agentNode["type"].as<std::string>() == "UAV") {
			// Drone type agent
            agent.stratum = agentNode["stratum"].as<std::string>();
            agent.charging_pad_ID = agentNode["charging_pad_ID"].as<std::string>();

			if(DEBUG_PATROLINPUT) {
				std::cout << "  Stratum: " << agent.stratum << std::endl;
				std::cout << "  Charging Pad ID: " << agent.charging_pad_ID << std::endl;
			}

	        mRa.push_back(agent);
		} else if (agentNode["type"].as<std::string>() == "UGV") {
			// UGV type agent
			const YAML::Node& charging_pads = agentNode["charging_pads"];
			for (const auto& padNode : charging_pads) {
                ChargingPad pad;
                pad.ID = padNode["ID"].as<std::string>();
                pad.mode = padNode["mode"].as<std::string>();
                pad.UAV_ID = padNode["UAV_ID"].as<std::string>();
                pad.is_charging = padNode["is_charging"].as<bool>();
                agent.charging_pads.push_back(pad);

				if(DEBUG_PATROLINPUT) {
					std::cout << "  Charging Pad ID: " << pad.ID << std::endl;
					std::cout << "    Mode: " << pad.mode << std::endl;
					std::cout << "    UAV ID: " << pad.UAV_ID << std::endl;
					std::cout << "    Is Charging: " << (pad.is_charging ? "true" : "false") << std::endl;
				}
			}

	        mRg.push_back(agent);
		}
	}
}

// Parse the scenario
void PatrollingInput::parseScenario(const YAML::Node& scenario) {
    std::string description = scenario["description"].as<std::string>();
    std::string type = scenario["type"].as<std::string>();
    std::string subtype = scenario["subtype"].as<std::string>();

    if(DEBUG_PATROLINPUT) {
        std::cout << "Scenario:" << std::endl;
        std::cout << "  Description: " << description << std::endl;
        std::cout << "  Type: " << type << std::endl;
        std::cout << "  Subtype: " << subtype << std::endl;
    }

    const YAML::Node& nodesNode = scenario["nodes"];

    // Get each PoI (node)
    for (const auto& nodeNode : nodesNode) {
		Node node;
		node.ID = nodeNode["ID"].as<std::string>();
		node.type = nodeNode["type"].as<std::string>();
		node.location.x = nodeNode["location"]["x"].as<double>();
		node.location.y = nodeNode["location"]["y"].as<double>();
		node.time_last_service = nodeNode["time_last_service"].as<double>();

		if(node.type == "air_only") {
			nodes.push_back(node);
		}

        if(DEBUG_PATROLINPUT) {
            std::cout << "  Node ID: " << node.ID << std::endl;
            std::cout << "    Type: " << node.type << std::endl;
            std::cout << "    Location: (" << node.location.x << ", " << node.location.y << ")" << std::endl;
            std::cout << "    Time Last Service: " << node.time_last_service << std::endl;
        }
    }
}


PatrollingInput::~PatrollingInput() {
}


// Get the location of the depot for UGV j
void PatrollingInput::GetDepot(int j, double* x, double* y) {
	// Bounds checking on j
	if(j >= 0 && j < boost::numeric_cast<int>(mRg.size())) {
		*x = mRg.at(j).location.x;
		*y = mRg.at(j).location.y;
	}
	else {
		// They asked for some non-existing vehicle... just return 0;
		*x = 0;
		*y = 0;
	}
}

// Get the max range of drone j (on a full charge)
double PatrollingInput::GetDroneMaxDist(int j) {
	// Determine energy efficiency at optimal speed (Watts -- Jules per second)
	double efficiency_v_opt = 396.743 - 1.695*UAV_V_OPT;
	// Determine max operation time (bat-capacity / efficiency) (based on full battery)
	double max_t = GetDroneBatCap(j)/efficiency_v_opt;
	// Max-dist = v_opt * max-t
	return UAV_V_OPT * max_t;
}

// Get the max range of drone j (on a full charge)
double PatrollingInput::GetUGVMaxDist(int j) {
	// Determine energy efficiency at optimal speed (Watts -- Jules per second)
	double efficiency_v_opt = 464.8*UGV_V_OPT + 356.3;
	// Determine max operation time (bat-capacity / efficiency) (based on full battery)
	double max_t = GetUGVBatCap(j)/efficiency_v_opt;
	// Max-dist = v_opt * max-t
	return UGV_V_OPT * max_t;
}

// Determines a theoretical lower-bound on a possible solution
double PatrollingInput::LowerBound() {
	return 0;
}

