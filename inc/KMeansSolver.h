/*
 * KMeansSolver.h
 *
 * Created by:	Jonathan Diller
 * On: 			Jun 26, 2024
 *
 * Description:
 */


#pragma once

#include <iostream>
#include <vector>
#include <cstdlib>
#include <ctime>
#include <cmath>
#include <limits>

#include "defines.h"

#define DEBUG_KMEANS	DEBUG || 0

#define DEFAULT_ITERATIONS	1000


struct ClusteringPoint {
    double x, y;
    int ID;
    int cluster;

    ClusteringPoint(int id, double x, double y, int clstr) {
    	ID = id;
    	this->x = x;
    	this->y = y;
    	cluster = clstr;
    }

    ClusteringPoint(const ClusteringPoint& other) {
    	ID = other.ID;
    	x = other.x;
    	y = other.y;
    	cluster = other.cluster;
    }
};


class KMeansSolver {
public:
	KMeansSolver();
	virtual ~KMeansSolver();

	// Run k-means clustering algorithm
	bool SolveKMeans(std::vector<ClusteringPoint>& points, int k, int max_iterations = DEFAULT_ITERATIONS);
protected:
private:
	void initializeCentroids(int k, std::vector<ClusteringPoint>& points, std::vector<ClusteringPoint>& centroids);
	bool assignClusters(int k, std::vector<ClusteringPoint>& points, std::vector<ClusteringPoint>& centroids);
	bool updateCentroids(int k, std::vector<ClusteringPoint>& points, std::vector<ClusteringPoint>& centroids);
	double euclideanDistance(const ClusteringPoint& a, const ClusteringPoint& b);
};
