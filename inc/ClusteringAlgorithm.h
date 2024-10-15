/*
 * ClusteringAlgorithm.h
 *
 * Created by:	Jonathan Diller
 * On: 			Sep 24, 2024
 *
 * Description: Runs Lloyd's k-means clustering algorithm. The algorithm takes an optional
 * argument that limits the number of iterations to force-stop if it fails to settle on a
 * solution.
 */

#pragma once

#include <math.h>
#include <vector>
#include <iostream>
#include <string>
#include <limits>
#include <list>
#include <boost/numeric/conversion/cast.hpp>

#include "Utilities.h"

#define DEBUG_HL_KMEANS		DEBUG || 0

struct kPoint {
	int point_ID;
	int centroid_ID;
	double X,Y,Z;

	kPoint(int pnt_id, int cntr_id, double x, double y, double z) {
		point_ID = pnt_id;
		centroid_ID = cntr_id;
		X = x;
		Y = y;
		Z = z;
	}
	kPoint(const kPoint& other) {
		point_ID = other.point_ID;
		centroid_ID = other.centroid_ID;
		X = other.X;
		Y = other.Y;
		Z = other.Z;
	}
	kPoint& operator=(const kPoint& other) {
		point_ID = other.point_ID;
		centroid_ID = other.centroid_ID;
		X = other.X;
		Y = other.Y;
		Z = other.Z;

	    return *this;
	}
};


class ClusteringAlgorithm {
public:
	ClusteringAlgorithm();

	void Solve(int k, std::vector<kPoint>* all_points, std::vector<std::vector<kPoint>>* clustered_points, int max_iterations = 1000);

protected:
private:
};
