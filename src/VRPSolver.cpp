#include "VRPSolver.h"


VRPSolver::VRPSolver() {

}

VRPSolver::~VRPSolver() {

}

bool VRPSolver::SolveVRP(std::vector<VRPPoint>& nodes, int num_vehicles, std::vector<std::vector<int>>& tours) {
	bool ret_val = false;

	// Which VRP solver are we using..?
	switch(VRP_ALGORITHM) {
	case 0:
		ret_val = SolveRichVRP(nodes, num_vehicles, tours);
		break;
	case 1:
	default:
		ret_val = SolveFastVRP(nodes, num_vehicles, tours);
	}

	return ret_val;
}

bool VRPSolver::SolveRichVRP(std::vector<VRPPoint>& nodes, int num_vehicles, std::vector<std::vector<int>>& tours) {
	//below is the rich solver
	std::string inputFilename = "vrp_input.json";
	std::string outputFilename = "vrp_output.json";

	// Transform the nodes into VRP Jobs (with lat/long coordinates)
	std::vector<VRPJob> jobs;
	for(long unsigned int i = 0; i < nodes.size() - 1; i++) {
		VRPJob job;
		job.id = itos(nodes.at(i).ID);
		job.location.lat = getLat(nodes.at(i).y);
		job.location.lng = getLng(nodes.at(i).x);
		job.demand = 1;
		job.duration = 1;
		jobs.push_back(job);
	}

	// Generate vehicles
	std::vector<VRPVehicle> vehicles;
	int capacity = ceil((nodes.size() - 1.0)/double(num_vehicles));
	double depot_lat = getLat(nodes.back().y);
	double depot_lng = getLng(nodes.back().x);
	for(int i = 0; i < num_vehicles; i++) {
		VRPVehicle truck;
		truck.id = itos(i);
		truck.start_location = {depot_lat, depot_lng};
		truck.end_location = {depot_lat, depot_lng};
		truck.start_time = "2024-06-28T09:00:00Z";
		truck.end_time = "2024-06-29T09:00:00Z";
		truck.fixed_cost = 0.0;
		truck.distance_cost = 0;
		truck.time_cost = 1.0;
		truck.capacity = capacity;
		vehicles.push_back(truck);
	}

	if(DEBUG_VRP) {
		printf("Running VRP Solver\nJobs:\n");
		for(VRPJob n : jobs) {
			printf(" %s : %f, %f\n", n.id.c_str(), n.location.lat, n.location.lng);
		}
		printf("Vehicles:\n");
		for(VRPVehicle truck : vehicles) {
			printf(" %s : %f, %f -- C = %d\n", truck.id.c_str(), truck.start_location.lat, truck.start_location.lng, truck.capacity);
		}
	}

	generateInputJson(inputFilename, jobs, vehicles);
	runVrpCli(inputFilename, outputFilename);
	readOutputJson(outputFilename, tours);

	return true;}

bool VRPSolver::SolveFastVRP(std::vector<VRPPoint>& nodes, int num_vehicles, std::vector<std::vector<int>>& tours) {
	std::vector<std::vector<kPoint>> cluster_k;

	/// Form clusters for each vehicle
	// Put each node into a kPoint
	std::vector<kPoint> nodePoints;
	for(int i = 0; i < (int)nodes.size() - 1; i++) {
		// Node* n_i = nodes.at(i);
		VRPPoint n_i(i, nodes.at(i).x, nodes.at(i).y);
		kPoint pnt_i(i, 0, n_i.x, n_i.y, 0);
		nodePoints.push_back(pnt_i);
	}

	// Run clustering algorithm
	ClusteringAlgorithm clusteringAlg;
	clusteringAlg.Solve(num_vehicles, &nodePoints, &cluster_k);

	/// Solve TSP on each cluster
	LKH_TSP_Solver tspSolver;
	for(int k = 0; k < num_vehicles; k++) {
		// Create a vector with all of the stops (and the BS at the end)
		std::vector<Stop> vStops;
		// Fill vector with each point in the cluster
		for(kPoint point : cluster_k.at(k)) {
			Stop stp(point.point_ID, point.X, point.Y, point.Z);
			vStops.push_back(stp);
		}
		// Add base station as last stop
		{
			Stop stp(-1, nodes.back().x, nodes.back().y, 0);
			vStops.push_back(stp);
		}

		// Create empty path vector (solver will fill this with the solution)
		std::vector<int> vPath;

		// Run TSP solver
		tspSolver.Solve_TSP(vStops, vPath);

		// Tour vector (these are the node's actual i values)
		std::vector<int> vTour_i;

		// We need to put node -1 first.. Find this node
		{
			int iteration = 0, nodes_found = 0;
			bool found_bs = false;
			while(true) {
				int i = iteration%boost::numeric_cast<int>(vStops.size());

				if(found_bs) {
					// We found the BS. Add this node to the tour
					vTour_i.push_back(vStops.at(vPath.at(i)).ID);
					nodes_found++;
				}
				else {
					// Still searching..
					if(vStops.at(vPath.at(i)).ID == -1) {
						// Found the base station!
						found_bs = true;
						nodes_found++;
					}
				}

				// Have we found all of the stops?
				if(nodes_found >= boost::numeric_cast<int>(vPath.size())) {
					break;
				}

				iteration++;
			}
		}

		// Store the final solution
		tours.push_back(vTour_i);
	}
	return true;
}



void VRPSolver::generateInputJson(const std::string& filename, const std::vector<VRPJob>& jobs, const std::vector<VRPVehicle>& vehicles) {
	json inputJson;
	inputJson["plan"]["jobs"] = json::array();
	for (const auto& job : jobs) {
		json jobJson;
		jobJson["id"] = job.id;
		jobJson["pickups"] = {
			{
				{"places", { { {"location", { {"lat", job.location.lat}, {"lng", job.location.lng} } }, {"duration", job.duration} } }},
				{"demand", { job.demand }}
			}
		};
		inputJson["plan"]["jobs"].push_back(jobJson);
	}

	inputJson["fleet"]["vehicles"] = json::array();
	for (const auto& vehicle : vehicles) {
		json vehicleJson;
		vehicleJson["typeId"] = vehicle.id;
		vehicleJson["vehicleIds"] = { vehicle.id };
		vehicleJson["profile"]["matrix"] = "normal_car";
		vehicleJson["costs"] = {
			{"fixed", vehicle.fixed_cost},
			{"distance", vehicle.distance_cost},
			{"time", vehicle.time_cost}
		};
		vehicleJson["shifts"] = {
			{
				{"start", { {"earliest", vehicle.start_time}, {"location", { {"lat", vehicle.start_location.lat}, {"lng", vehicle.start_location.lng} } }} },
				{"end", { {"latest", vehicle.end_time}, {"location", { {"lat", vehicle.end_location.lat}, {"lng", vehicle.end_location.lng} } }} }
			}
		};
		vehicleJson["capacity"] = { vehicle.capacity };
		inputJson["fleet"]["vehicles"].push_back(vehicleJson);
	}

	inputJson["fleet"]["profiles"] = { { {"name", "normal_car"} } };

	std::ofstream file(filename);
	if (!file) {
		std::cerr << "Could not open the file for writing: " << filename << std::endl;
		return;
	}
	file << inputJson.dump(4);
	file.close();
}

void VRPSolver::runVrpCli(const std::string& inputFilename, const std::string& outputFilename) {
	std::string command = "vrp-cli solve pragmatic " + inputFilename + " -o " + outputFilename;

	int result = std::system(command.c_str());
	if (result != 0) {
		std::cerr << "Error running vrp-cli command." << std::endl;
	}
}

void VRPSolver::readOutputJson(const std::string& filename, std::vector<std::vector<int>>& tours) {
	std::ifstream file(filename);
	if (!file) {
		std::cerr << "Could not open the file for reading: " << filename << std::endl;
		return;
	}

	json outputJson;
	file >> outputJson;

	if (outputJson.contains("tours")) {
		for (const auto& tour : outputJson["tours"]) {
			std::vector<int> jobOrder;
			for (const auto& stop : tour["stops"]) {
				for (const auto& activity : stop["activities"]) {
					std::string jobId = activity["jobId"];
					if (jobId != "departure" && jobId != "arrival") {
						jobOrder.push_back(std::stoi(jobId));
					}
				}
			}
			tours.push_back(jobOrder);
		}
	} else {
		std::cerr << "No tours found in the output JSON." << std::endl;
	}

	if(DEBUG_VRP) {
		printf("Job order: \n");
		for(auto tour : tours) {
			for(int v : tour) {
				printf(" %d", v);
			}
			printf("\n");
		}
		printf("\n");
	}
}


double VRPSolver::getLat(double dy) {
	return base_lat + (dy / 6371000) * (180 / PI);
}

double VRPSolver::getLng(double dx) {
	return base_long + (dx / 6371000) * (180 / PI) / cos(base_lat * PI/180);
}
