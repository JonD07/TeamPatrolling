#include <string>
#include <vector>
#include <iostream>
#include "Agent.h"
#include "math.h"

#pragma once


class UGV : public Agent{
    //Battery state stored in Agent object
public:
    UGV();
    std::vector<ChargingPad> charging_pads;

    double maxDriveSpeed;
    double maxDriveAndChargeSpeed;
    double batterySwapTime;
    double joulesPerSecondWhileWaiting;
    double chargeEfficiency;
    double SPlineSegDist;
    double ugv_v_crg;
    int dronesPerVehicle;
    double speed_cubed_coefficient;
    double speed_squared_coefficient;
    double speed_linear_coefficient;
    double speed_const;
    
    void printInfo(){
        std::cout << "UGV " << this->ID << std::endl;
        std::cout << "Type: " << this->type << std::endl;
        std::cout << "Subtype: " << this->subtype << std::endl;
        std::cout << "Location: (" << this->location.x << ", " << this->location.y << ")" << std::endl;
        std::cout << "Max Battery Energy: " << this->battery_state.max_battery_energy << std::endl;
        std::cout << "Current Battery Energy: " << this->battery_state.current_battery_energy << std::endl;
        std::cout << "Max Drive Speed: " << this->maxDriveSpeed << std::endl;
        std::cout << "Max Drive and Charge Speed: " << this->maxDriveAndChargeSpeed << std::endl;
        std::cout << "Battery Swap Time: " << this->batterySwapTime << std::endl;
        std::cout << "Joules Per Second While Waiting: " << this->joulesPerSecondWhileWaiting << std::endl;
        std::cout << "Charge Efficiency: " << this->chargeEfficiency << std::endl;
        std::cout << "Spline Segment Distance: " << this->SPlineSegDist << std::endl;
        std::cout << "Drones Per Vehicle: " << this->dronesPerVehicle << std::endl;
        std::cout << "UGV Charge Speed: " << this->ugv_v_crg << std::endl;
        std::cout << "Speed Cubed Coefficient: " << this->speed_cubed_coefficient << std::endl;
        std::cout << "Speed Squared Coefficient: " << this->speed_squared_coefficient << std::endl;
        std::cout << "Speed Linear Coefficient: " << this->speed_linear_coefficient << std::endl;
        std::cout << "Speed Constant: " << this->speed_const << std::endl;
        
        for(auto& pad : charging_pads) {
            std::cout << "Charging Pad ID: " << pad.ID << std::endl;
            std::cout << "  Mode: " << pad.mode << std::endl;
            std::cout << "  UAV ID: " << pad.UAV_ID << std::endl;
            std::cout << "  Is Charging: " << (pad.is_charging ? "true" : "false") << std::endl;
        }
    }

    double getJoulesPerSecondDriving(double velocity){
        return ((speed_cubed_coefficient * pow(velocity, 3)) + (speed_squared_coefficient * pow(velocity, 2)) + 
        (speed_linear_coefficient *  velocity) + speed_const);
    }
      
};