#include "PatrollingInput.h"
#include "UAV.h"
#include "UGV.h"
#include "UGV.cpp"
#include "UAV.cpp"

/*
 * Basic constructor. This expects an input file path. The input file should be a yaml file.
 *
 * Sets:
 * i in I, set of PoI, |I| = N
 * j in R^a, set of drones, |R^a| = Ma
 * j in R^g, set of UGVs, |R^g| = Mg
 */
PatrollingInput::PatrollingInput(std::string scenario_input, std::string vehicle_input) {
	if(SANITY_PRINT)
		printf("Reading input YAML file\n");
	// std::cout << vehicle_input << " vehicle input" << std::endl;

	// Read status
	bool read_success = true;

	try {
		// Load the YAML file
		YAML::Node config = YAML::LoadFile(scenario_input);
		YAML::Node vehicleInfo = YAML::LoadFile(vehicle_input);

        // Extract and print the basic information
        std::string id = config["ID"].as<std::string>();
        double time = config["time"].as<double>();
        std::string description = config["description"].as<std::string>();

        std::cout << "ID: " << id << std::endl;
        std::cout << "Time: " << time << std::endl;
        std::cout << "Description: " << description << std::endl;

        // Parse and print agents. Create UAV and UGV objects and place into mRa and mRg
        const YAML::Node& agents = config["agents"];
        parseAgents(agents);

        // Parse and print scenario
        const YAML::Node& scenario = config["scenario"];
        parseScenario(scenario);

	} catch (const std::exception &e) {
		fprintf(stderr, "[ERROR]:PatrollingInput() %s\n", e.what());
		read_success = false;
	}

	try{
		std::cout << "Reading vehicle YAML file" << std::endl;
		YAML::Node vehicleInfo = YAML::LoadFile(vehicle_input);
		
		//Parsing vehicle file and adding all information to each UAV object
		const YAML::Node& UAVs = vehicleInfo["UAV"];
		parseUAVs(UAVs);

		//Parsing vehicle file and adding all information to each UGV object
		const YAML::Node& UGVs = vehicleInfo["UGV"];
		parseUGVs(UGVs);
		//Debug print UAV/UGV vector sizes
		// std::cout << "UAVs in mRa : " << mRa.size() << std::endl;
		// for (auto& uav : mRa) {
		// 	uav.printInfo();
		// }
		// std::cout << " UGV types: " << UGVs.size() << std::endl;
		// for (auto& ugv : mRg) {
		// 	ugv.printInfo();
		// }
	}
	catch(const std::exception &e){
		fprintf(stderr, "[ERROR]:VehicleInput() %s\n", e.what());
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


//Version that puts all of the information in each UAV object
void PatrollingInput::parseUAVs(const YAML::Node& uavList){
	std::cout << "Parsing UAVs" << std::endl;
	for(auto& drone : mRa){
		std::string droneSubtype = drone.subtype;
		for(const auto& droneStatObject : uavList){
			if(droneStatObject["subtype"].as<std::string>() == droneSubtype){
				drone.timeNeededToLaunch = droneStatObject["UAV_LAUNCH_TIME"].as<double>();
				drone.timeNeededToLand = droneStatObject["UAV_LAND_TIME"].as<double>();
				drone.energyToLand = droneStatObject["LAND_ENERGY"].as<double>();
				drone.energyToTakeOff = droneStatObject["LAUNCH_ENERGY"].as<double>();
				drone.slowChargePoint = droneStatObject["SLOW_CHARGE_POINT"].as<double>();
				drone.maxSpeed = droneStatObject["UAV_V_MAX"].as<double>();
				drone.maxSpeedAfield = droneStatObject["UAV_V_MAX_AFIELD"].as<double>();
				drone.speed_cubed_coefficient = droneStatObject["SPEED_CUBED_COEFFICIENT"].as<double>();
				drone.speed_squared_coefficient = droneStatObject["SPEED_SQUARED_COEFFICIENT"].as<double>();
				drone.speed_linear_coefficient = droneStatObject["SPEED_LINEAR_COEFFICIENT"].as<double>();
				drone.speed_const = droneStatObject["SPEED_CONST"].as<double>();
				drone.charge_startup_t = droneStatObject["CHARGE_STARTUP_T"].as<double>();
				break;
			}
		}
		if(DEBUG_PATROLINPUT){
			std::cout << "UAV vehicle information: " << std::endl;
			std::cout << "  Subtype: " << drone.subtype << std::endl;
			std::cout << "  Time Needed to Launch: " << drone.timeNeededToLaunch << std::endl;
			std::cout << "  Time Needed to Land: " << drone.timeNeededToLand << std::endl;
			std::cout << "  Energy to Land: " << drone.energyToLand << std::endl;
			std::cout << "  Energy to Take Off: " << drone.energyToTakeOff << std::endl;
			std::cout << "  Slow Charge Point: " << drone.slowChargePoint << std::endl;
			std::cout << "  Max Speed: " << drone.maxSpeed << std::endl;
			std::cout << "  Max Speed Afield: " << drone.maxSpeedAfield << std::endl;
		}
	}
}

void PatrollingInput::parseUGVs(const YAML::Node& UGVList){
	std::cout << "Parsing UGVs" << std::endl;
	for(auto& ugv : mRg){
		std::string ugvSubtype = ugv.subtype;
		for(const auto& ugvStatObject : UGVList){
			if(ugvStatObject["subtype"].as<std::string>() == ugvSubtype){
				ugv.subtype = ugvStatObject["subtype"].as<std::string>();
				ugv.maxDriveSpeed = ugvStatObject["UGV_MAX_SPEED"].as<double>();
				ugv.maxDriveAndChargeSpeed = ugvStatObject["UGV_MAX_SPEED_CHARGING"].as<double>();
				ugv.batterySwapTime = ugvStatObject["UGV_BAT_SWAP_TIME"].as<double>();
				ugv.joulesPerSecondWhileWaiting = ugvStatObject["UGV_JOULES_PER_SECONDS_WAIT"].as<double>();
				ugv.chargeEfficiency = ugvStatObject["CHARGE_EFFICIENCY"].as<double>();
				ugv.SPlineSegDist = ugvStatObject["UGV_SPLINE_SEG_DIST"].as<double>();
				ugv.dronesPerVehicle = ugvStatObject["DRONE_PER_UGV"].as<int>();
				ugv.ugv_v_crg = ugvStatObject["UGV_V_CRG"].as<double>();
				ugv.speed_cubed_coefficient = ugvStatObject["SPEED_CUBED_COEFFICIENT"].as<double>();
				ugv.speed_cubed_coefficient = ugvStatObject["SPEED_SQUARED_COEFFICIENT"].as<double>();
				ugv.speed_cubed_coefficient = ugvStatObject["SPEED_LINEAR_COEFFICIENT"].as<double>();
				ugv.speed_cubed_coefficient = ugvStatObject["SPEED_CONST"].as<double>();


				break;
			}
		}
		if(DEBUG_PATROLINPUT){
			std::cout << "UGV Vehicle Stat Information: " << std::endl;
			std::cout << "  Max Drive Speed: " << ugv.maxDriveSpeed << std::endl;
			std::cout << "  Max Drive and Charge Speed: " << ugv.maxDriveAndChargeSpeed << std::endl;
			std::cout << "  Battery Swap Time: " << ugv.batterySwapTime << std::endl;
			std::cout << "  Joules Per Second While Waiting: " << ugv.joulesPerSecondWhileWaiting << std::endl;
			std::cout << "  Charge Efficiency: " << ugv.chargeEfficiency << std::endl;
			std::cout << "  Spline Segment Distance: " << ugv.SPlineSegDist << std::endl;
			std::cout << "  Drones Per Vehicle: " << ugv.dronesPerVehicle << std::endl;
		}
	}
}

// Parse each agent (both UAV and UGV)
void PatrollingInput::parseAgents(const YAML::Node& agentNodes) {
	for (const auto& agentNode : agentNodes) {
        Agent agent;
		agent.type = agentNode["type"].as<std::string>();
		if(agent.type  == "UAV"){
			UAV uav;
			uav.ID = agentNode["ID"].as<std::string>();
			uav.type = agentNode["type"].as<std::string>();
			uav.subtype = agentNode["subtype"].as<std::string>();
			uav.location.x = agentNode["location"]["x"].as<double>();
			uav.location.y = agentNode["location"]["y"].as<double>();
			uav.battery_state.max_battery_energy = agentNode["battery_state"]["max_battery_energy"].as<double>();
			uav.battery_state.current_battery_energy = agentNode["battery_state"]["current_battery_energy"].as<double>();
			uav.stratum = agentNode["stratum"].as<std::string>();	
			uav.charging_pad_ID = agentNode["charging_pad_ID"].as<std::string>();
			mRa.push_back(uav);
			if(DEBUG_PATROLINPUT) {
				std::cout << "UAV  BASIC INFORMATION DEBUG" << std::endl;
				std::cout << "Agent ID: " << uav.ID << std::endl;
				std::cout << "  Type: " << uav.type << std::endl;
				std::cout << "  Subtype: " << uav.subtype << std::endl;
				std::cout << "  Location: (" << uav.location.x << ", " << uav.location.y << ")" << std::endl;
				std::cout << "  Max Battery Energy: " << uav.battery_state.max_battery_energy << std::endl;
				std::cout << "  Current Battery Energy: " << uav.battery_state.current_battery_energy << std::endl;
			}
		}	

		else if(agent.type == "UGV"){
			UGV ugv;
			ugv.ID = agentNode["ID"].as<std::string>();
			ugv.type = agentNode["type"].as<std::string>();
			ugv.subtype = agentNode["subtype"].as<std::string>();
			ugv.location.x = agentNode["location"]["x"].as<double>();
			ugv.location.y = agentNode["location"]["y"].as<double>();
			ugv.battery_state.max_battery_energy = agentNode["battery_state"]["max_battery_energy"].as<double>();
			ugv.battery_state.current_battery_energy = agentNode["battery_state"]["current_battery_energy"].as<double>();
			const YAML::Node& charging_pads = agentNode["charging_pads"];
			for (const auto& padNode : charging_pads) {
				ChargingPad pad;
				pad.ID = padNode["ID"].as<std::string>();
				pad.mode = padNode["mode"].as<std::string>();
				pad.UAV_ID = padNode["UAV_ID"].as<std::string>();
				pad.is_charging = padNode["is_charging"].as<bool>();
				ugv.charging_pads.push_back(pad);
				if(DEBUG_PATROLINPUT) {
					std::cout << "  Charging Pad ID: " << pad.ID << std::endl;
					std::cout << "    Mode: " << pad.mode << std::endl;
					std::cout << "    UAV ID: " << pad.UAV_ID << std::endl;
					std::cout << "    Is Charging: " << (pad.is_charging ? "true" : "false") << std::endl;
				}
			}
			if(DEBUG_PATROLINPUT) {
				std::cout << "UGV DEBUG" << std::endl;
				std::cout << "Agent ID: " << ugv.ID << std::endl;
				std::cout << "  Type: " << ugv.type << std::endl;
				std::cout << "  Subtype: " << ugv.subtype << std::endl;
				std::cout << "  Location: (" << ugv.location.x << ", " << ugv.location.y << ")" << std::endl;
				std::cout << "  Max Battery Energy: " << ugv.battery_state.max_battery_energy << std::endl;
				std::cout << "  Current Battery Energy: " << ugv.battery_state.current_battery_energy << std::endl;
			}
			mRg.push_back(ugv);
		}

		else{
			std::cerr << "Unknown agent type: " << agentNode["type"].as<std::string>() << std::endl;
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
		else if(node.type == "depot_a") {
			depot_x = nodeNode["location"]["x"].as<double>();
			depot_y = nodeNode["location"]["y"].as<double>();
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

// Get the initial location of drone j
void PatrollingInput::GetDroneInitLocal(int j, double* x, double* y) {
	// Bounds checking on j
	if(j >= 0 && j < boost::numeric_cast<int>(mRa.size())) {
		*x = mRa.at(j).location.x;
		*y = mRa.at(j).location.y;
	}
	else {
		// They asked for some non-existing vehicle... just return 0;
		*x = 0;
		*y = 0;
	}
}

// Get the initial location of UGV j
void PatrollingInput::GetUGVInitLocal(int j, double* x, double* y) {
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
	// double efficiency_v_opt = 396.743 - 1.695*UAV_V_OPT;
	UAV uav = mRa.at(j);
	double efficiency_v_opt = 396.743 - 1.695*uav.maxSpeed;

	// Determine max operation time (bat-capacity / efficiency) (based on full battery)
	double max_t = GetDroneBatCap(j)/efficiency_v_opt;
	// Max-dist = v_opt * max-t
	return uav.maxSpeed * max_t;
}

// Get the max range of drone j (on a full charge)
double PatrollingInput::GetUGVMaxDist(int j) {
	// Determine energy efficiency at optimal speed (Watts -- Jules per second)
	UGV ugv = mRg.at(j);
	// double efficiency_v_opt = 464.8*UGV_V_OPT + 356.3;
	double efficiency_v_opt = 464.8*ugv.maxDriveSpeed+ 356.3;
	// Determine max operation time (bat-capacity / efficiency) (based on full battery)
	double max_t = GetUGVBatCap(j)/efficiency_v_opt;
	// Max-dist = v_opt * max-t
	return ugv.maxDriveSpeed * max_t;
}

// Determines the time required to charge drone j for J jules
double PatrollingInput::calcChargeTime(int drone_j, double J) {
	// double time = CHARGE_STARTUP_T;
	double time = mRa.at(drone_j).charge_startup_t;

	// Which type of drone is this?
	if(mRa.at(drone_j).subtype == "standard") {
		// Standard drone, use default settings
		double batt_charge = GetDroneBatCap(drone_j) - J;

		// Will we be fast-charging?
		UAV drone = mRa.at(drone_j);
		if(batt_charge < drone.slowChargePoint) {
			// How many jules can we fast charge?
			// double fast_charge_joules = SLOW_CHARGE_POINT - batt_charge;
			double fast_charge_joules = drone.slowChargePoint - batt_charge;

			// Find roots of polynomial
			Roots roots;
			roots.FindRoots(FAST_CHARGE_A, FAST_CHARGE_B, (-1*fast_charge_joules));
			if(!roots.imaginary) {
				double fast_charge_time = std::max(roots.root1, roots.root2);
				time += fast_charge_time + (T_MAX - T_STAR);
			}
			else {
				// Not expected to be here...
				fprintf(stderr, "[ERROR] : Solver::calcChargeTime() : Roots of charge time are imaginary (subtype standard)!\n");
				exit(0);
			}
		}
		else {
			// We are in the slow charging range. Find time to charge to where we are..
			double charge_to_current = T_STAR - (log(1+(ALPHA/P_STAR)*(E_STAR - J)))/ALPHA;
			time += T_MAX - charge_to_current;

		}
	}
	else if(mRa.at(drone_j).subtype == "a_field") {
		// This is the dummed-down drone used on A field - assume that we remain in the linear range
		// Find roots of polynomial
		Roots roots;
		roots.FindRoots(FAST_CHARGE_A, FAST_CHARGE_B, (-1*J));
		if(!roots.imaginary) {
			time += std::max(roots.root1, roots.root2);
		}
		else {
			// Not expected to be here...
			fprintf(stderr, "[ERROR] : Solver::calcChargeTime() : Roots of charge time are imaginary (subtype a_field)!\n");
			exit(0);
		}
	}
	else {
		// Not expected to be here...
		fprintf(stderr, "[ERROR] : Solver::calcChargeTime() : Drone subtype not recognized!\n");
		exit(0);
	}

	return time;
}

// Get the maximum time required to fully charge drone j
double PatrollingInput::GetTMax(int drone_j) {
	return calcChargeTime(drone_j, GetDroneBatCap(drone_j));
}

// Get the maximum speed of drone j
double PatrollingInput::GetDroneVMax(int drone_j) {
	// Which type of drone is this?
	if(mRa.at(drone_j).subtype == "standard") {
		return mRa.at(drone_j).maxSpeed;
		// return UAV_V_MAX;
	}
	else if(mRa.at(drone_j).subtype == "a_field") {
		return mRa.at(drone_j).maxSpeedAfield;
		// return UAV_V_MAX_AFIELD;
	}
	else {
		// Not expected to be here...
		fprintf(stderr, "[ERROR] : Solver::calcChargeTime() : Drone subtype not recognized!\n");
		exit(0);
	}
}

// Determines a theoretical lower-bound on a possible solution
double PatrollingInput::LowerBound() {
	return 0;
}

// agent class, uav/ugv inherit  from thi. All agent fields are public. Put charging pad, location and bat state 
// in agent class

