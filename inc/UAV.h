#include <string>
#include <iostream>
#include "Agent.h"
#include <cmath>
#include "defines.h"

#pragma once

class UAV : public Agent{
    //Battery state is stored in agent
public:
    UAV();
    //UAV specific fields
    std::string stratum;
    std::string charging_pad_ID;

    double timeNeededToLaunch;
    double timeNeededToLand;
    double energyToLand;
    double energyToTakeOff;
    double slowChargePoint; //also known in the documentation as E*
    double maxSpeed;
    double maxSpeedAfield;
    double speed_cubed_coefficient;
    double speed_squared_coefficient;
    double speed_linear_coefficient;
    double speed_const;
    double charge_startup_t;
    
    // Defining battery attributes statically. These will need to be changed in the future given a new battery.
    #define FAST_CHARGE_A 0.06384091
    #define FAST_CHARGE_B	403.886 
    #define T_MAX	991.4 
    #define T_STAR	555.6 //gvien from the battery work done in the paper
    #define P_STAR	476.407 // power given from derivative of equation 3 and using time = 555.6
    #define E_STAR	244537 //gvien from the battery polynomial equation #3 where time = 555.6
    
    void printInfo(){
        // Print UAV information
        std::cout << "UAV " << this->ID << std::endl;
        std::cout << "Type: " << this->type << std::endl;
        std::cout << "Subtype: " << this->subtype << std::endl;
        std::cout << "Location: (" << this->location.x << ", " << this->location.y << ")" << std::endl;
        std::cout << "Max Battery Energy: " << this->battery_state.max_battery_energy << std::endl;
        std::cout << "Current Battery Energy: " << this->battery_state.current_battery_energy << std::endl;
        std::cout << "Stratum: " << this->stratum << std::endl;
        std::cout << "Charging Pad ID: " << this->charging_pad_ID << std::endl;
        std::cout << "Time Needed to Launch: " << this->timeNeededToLaunch << std::endl;
        std::cout << "Time Needed to Land: " << this->timeNeededToLand << std::endl;
        std::cout << "Energy to Land: " << this->energyToLand << std::endl;
        std::cout << "Energy to Take Off: " << this->energyToTakeOff << std::endl;
        std::cout << "Slow Charge Point: " << this->slowChargePoint << std::endl;
        std::cout << "Max Speed: " << this->maxSpeed << std::endl;
        std::cout << "Max Speed Afield: " << this->maxSpeedAfield << std::endl;
    }

    double getJoulesPerSecondFlying(double velocity){
        return ((speed_cubed_coefficient * pow(velocity, 3)) + (speed_squared_coefficient * pow(velocity, 2)) + 
        (speed_linear_coefficient *  velocity) + speed_const);
    }

};