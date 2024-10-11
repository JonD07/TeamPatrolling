#include "KMeansSolver.h"


KMeansSolver::KMeansSolver() {
//	std::srand(std::time(0));
}

KMeansSolver::~KMeansSolver() {

}

bool KMeansSolver::SolveKMeans(std::vector<ClusteringPoint>& points, int k, int max_iterations) {
	if(DEBUG_KMEANS)
		printf("Running K-Means cluster, k = %d, num-points = %ld\n", k, points.size());
	// Create centroids
	std::vector<ClusteringPoint> centroids;
	bool run_again = true;
	// Initialize the centroids
	initializeCentroids(k, points, centroids);
	// For MAX_ITERATIONS, or until we no longer make changes...
	int i = 0;
	for(; i < max_iterations && run_again; ++i) {
		// Assign nodes to clusters
		run_again = assignClusters(k, points, centroids);
		// Update where each centroid sits
		run_again |= updateCentroids(k, points, centroids);
	}
	if(DEBUG_KMEANS)
		printf("Finished clustering after %d iterations!\n", i);

	// Return true if we timed-out
	return i == max_iterations;
}


void KMeansSolver::initializeCentroids(int k, std::vector<ClusteringPoint>& points, std::vector<ClusteringPoint>& centroids) {
	if(DEBUG_KMEANS)
		printf("Initial Clusters:\n");
	// For each cluster
	for (int i = 0; i < k; ++i) {
		// Pick a random point to make an initial cluster centroid
		int index = std::rand() % points.size();
		ClusteringPoint centroid(points[index]);
		centroid.cluster = i;
		centroids.push_back(centroid);
		if(DEBUG_KMEANS)
			printf(" %d (%f, %f)\n", i, centroids[i].x, centroids[i].y);
	}
}

bool KMeansSolver::assignClusters(int k, std::vector<ClusteringPoint>& points, std::vector<ClusteringPoint>& centroids) {
	bool run_again = false;

	// For each point
	for(auto& point : points) {
		// Determine the closest cluster
		double min_distance = std::numeric_limits<double>::max();
		int closest_centroid = -1;
		for(int i = 0; i < k; ++i) {
			double distance = euclideanDistance(point, centroids[i]);
			if(distance < min_distance) {
				min_distance = distance;
				closest_centroid = i;
			}
		}

		// Did we change clusters?
		if(point.cluster != closest_centroid) {
			// Update cluster
			point.cluster = closest_centroid;
			// Run algorithm again
			run_again = true;
		}
	}

	return run_again;
}

bool KMeansSolver::updateCentroids(int k, std::vector<ClusteringPoint>& points, std::vector<ClusteringPoint>& centroids) {
	if(DEBUG_KMEANS)
		printf("Updating centroids\n");
	bool bad_centroid = false;
	std::vector<int> counts(k, 0);
	std::vector<double> avgX(k, 0);
	std::vector<double> avgY(k, 0);

	// Add up the number of points assigned to each centroid
	for(const auto& point : points) {
		counts[point.cluster]++;
		avgX[point.cluster] += point.x;
		avgY[point.cluster] += point.y;
	}

	// For each centroid..
	for(int i = 0; i < k; ++i) {
		if(counts[i] == 0) {
			if(DEBUG_KMEANS)
				printf(" EMPTY CLUSTER! Moving centroid\n");
			bad_centroid = true;
			int index = std::rand() % points.size();
			ClusteringPoint centroid(points[index]);
			centroids[i].x = points[index].x;
			centroids[i].y = points[index].y;

			if(DEBUG_KMEANS)
				printf("  %d: (%f, %f) with %d nodes\n", i, centroids[i].x, centroids[i].y, counts[i]);
		}
		else {
			// Find the new average x/y
			centroids[i].x = avgX[i]/counts[i];
			centroids[i].y = avgY[i]/counts[i];
			if(DEBUG_KMEANS)
				printf(" %d: (%f, %f) with %d nodes\n", i, centroids[i].x, centroids[i].y, counts[i]);
		}
	}

	return bad_centroid;
}

double KMeansSolver::euclideanDistance(const ClusteringPoint& a, const ClusteringPoint& b) {
	return std::sqrt(std::pow(a.x - b.x, 2) + std::pow(a.y - b.y, 2));
}
