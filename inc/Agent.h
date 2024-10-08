#include <string>
#include <vector>
#ifndef AGENT_H_
#define AGENT_H_

#define E_MIN 0 //minimum energy in a battery
struct Location {
	double x;
	double y;
};

struct BatteryState {
	double max_battery_energy;
	double current_battery_energy;
};

struct ChargingPad {
	std::string ID;
	std::string mode;
	std::string UAV_ID;
	bool is_charging;
};
struct Agent {
	std::string ID;
	std::string type;
	std::string subtype;
	Location location;
	BatteryState battery_state;
};

#endif // LIST_H_