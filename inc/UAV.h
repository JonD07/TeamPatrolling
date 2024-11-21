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
    double fast_charge_a;
    double fast_charge_b;
    double t_max;
    double t_star;
    double p_star;
    double e_star;

    
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
        std::cout << "Speed Cubed Coefficient: " << this->speed_cubed_coefficient << std::endl;
        std::cout << "Speed Squared Coefficient: " << this->speed_squared_coefficient << std::endl;
        std::cout << "Speed Linear Coefficient: " << this->speed_linear_coefficient << std::endl;
        std::cout << "Speed Constant: " << this->speed_const << std::endl;
        std::cout << "Charge Startup Time: " << this->charge_startup_t << std::endl;
        std::cout << "Fast Charge A: " << this->fast_charge_a << std::endl;
        std::cout << "Fast Charge B: " << this->fast_charge_b << std::endl;
        std::cout << "T Max: " << this->t_max << std::endl;
        std::cout << "T Star: " << this->t_star << std::endl;
        std::cout << "P Star: " << this->p_star << std::endl;
        std::cout << "E Star: " << this->e_star << std::endl;
    }

    double getJoulesPerSecondFlying(double velocity){
        return ((speed_cubed_coefficient * pow(velocity, 3)) + (speed_squared_coefficient * pow(velocity, 2)) + 
        (speed_linear_coefficient *  velocity) + speed_const);
    }

    double returnSlowEnergyGain(double time){
        return ((this->fast_charge_a * pow(time, 2)) + (this->fast_charge_b * time));
    }

    double returnFastEnergyGain(double time){
        return this->e_star + (this->p_star / ALPHA) * (1 - exp(-ALPHA * (time - this->t_star)));
    }

    double returnTimeToChargeToEStar(double energy){
        return -this->fast_charge_b + sqrt(pow(this->fast_charge_b, 2) - 4 * this->fast_charge_a * energy) / (2 * this->fast_charge_a);
    }

    double returnTimeToChargeToEMax(double energy){
        return this->t_max - (log(1+(ALPHA / this->p_star)*(this->e_star - energy)) / ALPHA);
    }

};