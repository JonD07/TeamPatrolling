/*
 * OnlineIP.h
 *
 * Created by:	Jonathan Diller
 * On: 			May 25, 2023
 *
 * Description: Basic program utilities.
 */


#pragma once

#include <math.h>
#include <string.h>
#include <sstream>

#include "defines.h"

// Returns true if c is roughly 0
bool isZero(double c);

// Returns true a and b are roughly equal
bool floatEquality(double a, double b);

// Returns the distance from point (x_1, y_1) to point (x_2, y_2)
double distAtoB(double x_1, double y_1, double x_2, double y_2);

// Takes an integer and returns a string
std::string itos(int i);
