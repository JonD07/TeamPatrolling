#include "ClusteringAlgorithm.h"


ClusteringAlgorithm::ClusteringAlgorithm() {}


void ClusteringAlgorithm::Solve(int k, std::vector<kPoint>* all_points, std::vector<std::vector<kPoint>>* clustered_points, int max_iterations) {
	if(DEBUG_HL_KMEANS) {
		printf("Running k-means clustering for k = %d\n", k);
	}

	// Create initial centroids
	std::vector<kPoint> centroids;

	// Create a list of points to randomly pull from
	std::vector<int> pointsAvailable;
	for(int i = 0; i < boost::numeric_cast<int>(all_points->size()); i++) {
		pointsAvailable.push_back(i);
	}

	// Create k centroids using randomly picked kPoints
	for(int i = 0; i < k; i++) {
		// Pick random point
		int rnd_index = rand()%pointsAvailable.size();
		int node_id = pointsAvailable.at(rnd_index);
		// Create centroid at this point
		kPoint cPoint(-1, i, all_points->at(node_id).X, all_points->at(node_id).Y, all_points->at(node_id).Z);
		// Store centroid
		centroids.push_back(cPoint);
		if(DEBUG_HL_KMEANS) {
			printf(" New cluster at (%f, %f, %f) (at node %d)\n", cPoint.X, cPoint.Y, cPoint.Z, node_id);
		}
		// Remove that point from the list
		pointsAvailable.erase(pointsAvailable.begin()+rnd_index);
	}

	// Track if centroids have moved
	bool cent_moved = true;

	// Run clustering iterations
	for(int i = 0; i < max_iterations && cent_moved; ++i) {
		if(DEBUG_HL_KMEANS) {
			printf(" New iteration %d\n", i);
		}
		cent_moved = false;
		// Check every point against every centroid
		for(std::vector<kPoint>::iterator pnt_it = all_points->begin(); pnt_it != all_points->end(); ++pnt_it) {
			// For each centroid, compute distance from the centroid to the point
			double min_dist = std::numeric_limits<double>::max();
			int best_ctr = pnt_it->centroid_ID;
			for(std::vector<kPoint>::iterator c = centroids.begin(); c != centroids.end(); ++c) {
				// Find distance for the centroid to the node point
				// double dist_to_c = distAtoB(pnt_it->X, pnt_it->Y, pnt_it->Z, c->X, c->Y, c->Z);
				double dist_to_c = distAtoB(pnt_it->X, pnt_it->Y, c->X, c->Y);

				// Is this better than the current minimum?
				if(dist_to_c < min_dist) {
					// Update incumbent best centroid
					min_dist = dist_to_c;
					best_ctr = c->centroid_ID;
				}
			}
			// Do we update the points centroid?
			if(best_ctr != pnt_it->centroid_ID) {
				if(DEBUG_HL_KMEANS) {
					printf("  Point %d moves %d -> %d\n", pnt_it->point_ID, pnt_it->centroid_ID, best_ctr);
				}

				// Yes, update which centroid this node is in
				pnt_it->centroid_ID = best_ctr;
				cent_moved = true;
			}
		}

		// Did any points move centroids?
		if(cent_moved) {
			if(DEBUG_HL_KMEANS) {
				printf("  Updating centroids\n");
			}

			// Create new centroid x,y,z averages
			std::vector<double> sum_x;
			std::vector<double> sum_y;
			std::vector<double> sum_z;
			std::vector<int> point_count;

			// Set initial values
			for(int i = 0; i < k; i++) {
				sum_x.push_back(0.0);
				sum_y.push_back(0.0);
				sum_z.push_back(0.0);
				point_count.push_back(0);
			}

			// Sum up the locations of each point
			for(std::vector<kPoint>::iterator pnt_it = all_points->begin(); pnt_it != all_points->end(); pnt_it++) {
				int cntrd_i = pnt_it->centroid_ID;
				sum_x[cntrd_i] += pnt_it->X;
				sum_y[cntrd_i] += pnt_it->Y;
				sum_z[cntrd_i] += pnt_it->Z;
				point_count[cntrd_i]++;
			}

			// Update each centroid location
			for(int i = 0; i < k; i++) {
				if(point_count[i] == 0) {
					if(DEBUG_HL_KMEANS) {
						printf("  ** Centroid %d has no points!\n", i);
					}

					// Pick a new point
					int rnd_index = rand()%pointsAvailable.size();
					centroids.at(i).X = all_points->at(rnd_index).X;
					centroids.at(i).Y = all_points->at(rnd_index).Y;
					centroids.at(i).Z = all_points->at(rnd_index).Z;
				}
				else {
					centroids.at(i).X = sum_x[i]/point_count[i];
					centroids.at(i).Y = sum_y[i]/point_count[i];
					centroids.at(i).Z = sum_z[i]/point_count[i];
				}

				if(DEBUG_HL_KMEANS) {
					printf("   Centroid %d has %d points, at (%f, %f, %f)\n", i, point_count[i], centroids.at(i).X, centroids.at(i).Y, centroids.at(i).Z);
				}
			}
		}
		else {
			// Done!
			if(DEBUG_HL_KMEANS) {
				printf(" Converged!\n Final solution:\n");
			}
		}
	}

	// Clear any old solution
	clustered_points->clear();
	for(int i = 0; i < k; i++) {
		std::vector<kPoint> clstr_i;
		clustered_points->push_back(clstr_i);
	}

	// Save found solution
	for(std::vector<kPoint>::iterator pnt_it = all_points->begin(); pnt_it != all_points->end(); pnt_it++) {
		clustered_points->at(pnt_it->centroid_ID).push_back(*pnt_it);
		if(DEBUG_HL_KMEANS) {
			printf("  %d : %d\n", pnt_it->point_ID, pnt_it->centroid_ID);
		}
	}
}
