/*
 * MASInput.h
 *
 * Created by:	Jonathan Diller
 * On: 			Jun 25, 2024
 *
 * Description: Patrolling problem input class
 *
 */

#pragma once

#include <vector>
#include <map>
#include <iostream>
#include <boost/numeric/conversion/cast.hpp>
// External library for YAML files
#include <yaml-cpp/yaml.h>

#include "Input.h"
#include "Roots.h"
#include "UGV.h"
#include "UAV.h"
#include "Agent.h"

#define DEBUG_PATROLINPUT	DEBUG || 0


struct Node {
	std::string ID;
	std::string type;
	Location location;
	double time_last_service;
};


class Obstacle : public Node {
public:
	double radius;

	Obstacle() {radius = 0.0;}

	Obstacle(const std::string& id, const std::string& type, const Location& loc, double r)
		: radius(r) {
		this->ID = id;
		this->type = type;
		this->location = loc;
		this->time_last_service = 0.0;
	}

	bool containsPoint(double px, double py) const {
		double dx = px - location.x;
		double dy = py - location.y;
		return (dx * dx + dy * dy) <= (radius * radius);
	}

	void printInfo() const {
    std::cout << "Obstacle ID: " << ID << "\n";
    std::cout << "  Type: " << type << "\n";
    std::cout << "  Location: (" << location.x << ", " << location.y << ")\n";
    std::cout << "  Radius: " << radius << "\n";
	}

	int get_id() const {
		if (ID.size() <= 2 || ID.substr(0, 2) != "o_") {
			throw std::invalid_argument("Obstacle ID format invalid: " + ID);
		}

		std::string numericPart = ID.substr(2);  // Skip "o_"
		try {
			return std::stoi(numericPart);
		} catch (const std::exception& e) {
			throw std::invalid_argument("Failed to convert obstacle ID '" + ID + "' to integer.");
		}
	}

	bool static checkForObstacle(double x1, double y1, double x2, double y2, Obstacle obstacle) {
	    /*
	    * Determines whether the line segment between two points intersects a circular obstacle.
	    *
	    * The function computes:
	    *  - The vector from (x1, y1) to (x2, y2) (the segment being evaluated)
	    *  - The projection of the obstacle's center onto that segment
	    *  - The closest point on the line to the obstacle
	    *
	    * It returns true if:
	    *  - Either endpoint is inside the obstacle, OR
	    *  - The closest point lies within the segment and is within the obstacle's radius
	    *
	    * Special handling is included for degenerate cases where the segment length is effectively zero.
	    */
	    double lineVecX = x2 - x1;
	    double lineVecY = y2 - y1;
	    double dx = obstacle.location.x - x1;
	    double dy = obstacle.location.y - y1;
	    double lineLength = std::sqrt(lineVecX * lineVecX + lineVecY * lineVecY);

	    if (lineLength < std::numeric_limits<double>::epsilon()) {
	        // If line length is essentially zero, check direct distance to obstacle
	        double directDistance = std::sqrt(dx * dx + dy * dy);
	        return directDistance <= obstacle.radius;
	    }

	    double unitLineVecX = lineVecX / lineLength;
	    double unitLineVecY = lineVecY / lineLength;

	    double projectionLength = dx * unitLineVecX + dy * unitLineVecY;

	    double closestX = x1 + projectionLength * unitLineVecX;
	    double closestY = y1 + projectionLength * unitLineVecY;

	    double distanceToCenter = std::sqrt(
	        (closestX - obstacle.location.x) * (closestX - obstacle.location.x) +
	        (closestY - obstacle.location.y) * (closestY - obstacle.location.y)
	    );

	    bool isClosestPointOnSegment =
	        projectionLength >= 0 &&
	        projectionLength <= lineLength;

	    bool isEndpoint1InCircle = std::sqrt(
	        (x1 - obstacle.location.x) * (x1 - obstacle.location.x) +
	        (y1 - obstacle.location.y) * (y1 - obstacle.location.y)
	    ) <= obstacle.radius;

	    bool isEndpoint2InCircle = std::sqrt(
	        (x2 - obstacle.location.x) * (x2 - obstacle.location.x) +
	        (y2 - obstacle.location.y) * (y2 - obstacle.location.y)
	    ) <= obstacle.radius;

	    return isEndpoint1InCircle || isEndpoint2InCircle ||
	           ((distanceToCenter <= obstacle.radius) && isClosestPointOnSegment);
	}

};


class PatrollingInput : public Input {
public:
	/*
	 * Sets:
	 * i in I, set of PoI, |I| = N
	 * j in R^a, set of drones, |R^a| = Ma
	 * j in R^g, set of UGVs, |R^g| = Mg
	 */

	PatrollingInput(std::string scenario_input, std::string vehicle_input);
	virtual ~PatrollingInput();

	

	/// Getters
	// Number of agents
	int GetN() {return nodes.size();}
	// Number of drones
	int GetMa() {return mRa.size();}
	// Number of UGVs
	int GetMg() {return mRg.size();}
	// Get list of nodes
	const std::vector<Node>& GetNodes() {return nodes;}
	// Get list of obstacles 
	const std::vector<Obstacle>& GetObstacles() const { return obstacles; }
	// Get the location of the depot (we assume a single depot)
	void GetDepot(int j, double* x, double* y);
	// Get the initial location of drone j
	void GetDroneInitLocal(int j, double* x, double* y);
	// Get the initial location of UGV j
	void GetUGVInitLocal(int j, double* x, double* y);
	// Get the designated name for node i
	std::string GetNodeID(int i) {return nodes.at(i).ID;}
	// Get the designated name for drone j
	std::string GetDroneID(int j) {return mRa.at(j).ID;}
	// Get the designated name for UGV j
	std::string GetUGVID(int j) {return mRg.at(j).ID;}
	// Get the max battery capacity of drone j
	double GetDroneBatCap(int j) {return mRa.at(j).battery_state.max_battery_energy;}
	// Get the max battery capacity of drone j
	double GetUGVBatCap(int j) {return mRg.at(j).battery_state.max_battery_energy;}
	// Get the max range of drone j (on a full charge)
	double GetDroneMaxDist(int j);
	// Get the max range of drone j (on a full charge)
	double GetUGVMaxDist(int j);
	// Determines the time required to charge drone j for J jules
	double calcChargeTime(int drone_j, double J);
	// Get the maximum time required to fully charge drone j
	double GetTMax(int drone_j);
	// Get the maximum speed of drone j
	double GetDroneVMax(int drone_j);
	// Assign UAVs to UGVs
	void AssignDronesToUGV(std::vector<std::vector<int>>& drones_to_UGV);
//    // * Helper function that sees if a obstacle is between two points
//    bool checkForObstacle(double x1, double y1, double x2, double y2, Obstacle obstacle);

	

	UAV getUAV(int ID){
		return mRa.at(ID);
	}
	UGV getUGV(int ID){
		return mRg.at(ID);
	}


	// Determines a theoretical upper bound on a possible solution
	double LowerBound();

private:
	std::vector<UAV> mRa;
	std::vector<UGV> mRg;
	std::vector<Node> nodes;
	std::vector<Obstacle> obstacles;
	double depot_x;
	double depot_y;
	void parseAgents(const YAML::Node& agents);
	void parseScenario(const YAML::Node& scenario);
	void parseUAVs(const YAML::Node& UAVs);
	void parseUGVs(const YAML::Node& UGVs);
	void parseObstacles(const YAML::Node& Obstacles); 
};
