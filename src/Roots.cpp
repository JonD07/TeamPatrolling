#include "Roots.h"


Roots::Roots() {
	root1 = 0;
	root2 = 0;
	imaginary = false;
}

Roots::Roots(const Roots& rt) {
	root1 = rt.root1;
	root2 = rt.root2;
	imaginary = rt.imaginary;
}


//***********************************************************
// Public Member Functions
//***********************************************************

// Find the roots of a polynomial in the form ax^2 + bx + c = 0, store roots in rt scruct.
// Assumes that we were given an actual quadratic equation (a != 0).
void Roots::FindRoots(double a, double b, double c) {
	double discriminant, realPart, imaginaryPart, x1, x2;
	if(isZero(a)) {
		printf("Requested to find root of non-quadratic\n");
		exit(1);
	}
	else {
		discriminant = b*b - 4*a*c;
		if (discriminant > 0) {
			x1 = (-b + sqrt(discriminant)) / (2*a);
			x2 = (-b - sqrt(discriminant)) / (2*a);
			if(DEBUG_ROOTS) {
				printf("  Roots are real and different.\n");
				printf("  Root 1 = %f\n", x1);
				printf("  Root 2 = %f\n", x2);
			}

			root1 = x1;
			root2 = x2;
			imaginary = false;
		}
		else if(discriminant == 0) {
			if(DEBUG_ROOTS)
				printf("  Roots are real and same.\n");
			x1 = (-b + sqrt(discriminant)) / (2*a);
			if(DEBUG_ROOTS)
				printf("  Root 1 = Root 2 = %f\n", x1);

			root1 = root2 = x1;
			imaginary = false;
		}
		else {
			realPart = -b/(2*a);
			imaginaryPart = sqrt(-discriminant)/(2*a);
			if(DEBUG_ROOTS) {
				printf("  Roots are complex and different.\n");
				printf("  Root 1 = %f + %fi\n", realPart, imaginaryPart);
				printf("  Root 1 = %f - %fi\n", realPart, imaginaryPart);
			}

			root1 = realPart;
			root2 = imaginaryPart;
			imaginary = true;
		}
	}
}


//***********************************************************
// Protected Member Functions
//***********************************************************


//***********************************************************
// Private Member Functions
//***********************************************************


