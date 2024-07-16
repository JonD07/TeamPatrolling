/*
 * Roots.h
 *
 * Created by:	Jonathan Diller
 * On: 			Jul 14, 2022
 *
 * Description: Used to find and store the roots of a polynomial.
 */

#pragma once

#include <stdlib.h>
#include <stdio.h>
#include <math.h>

#include "defines.h"
#include "Utilities.h"

#define DEBUG_ROOTS		0 && DEBUG


class Roots {
public:
	Roots();
	Roots(const Roots&);

	// Find the roots of a polynomial in the form ax^2 + bx + c = 0
	void FindRoots(double a, double b, double c);

	double root1;
	double root2;
	bool imaginary;
protected:
private:
};
