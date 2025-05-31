/*
 * ActionSwapper.h
 * Created by:	Jonathan Diller
 * On: 			May 30, 2025
 *
 * Description: Class used to swap actions that are overlapping.
 */

#pragma once

#include <tuple>
#include <queue>
#include <cmath> 
#include <boost/numeric/conversion/cast.hpp>

#include "Utilities.h"
#include "Solution.h"
#include "LaunchOptimizer.h"

#define DEBUG_SWAP	DEBUG || 0


class ActionSwapper {
public:
	ActionSwapper();
	~ActionSwapper();

	// Used for lazy action swapping
	void LazySwap(PatrollingInput* input, Solution* sol_final, int ugv_num, std::vector<std::vector<int>>& drones_to_UGV);
protected:
private:
	LaunchOptimizer optimizer;
	void LazyLLS(int ugv_num, std::vector<std::vector<int>>& drones_to_UGV, PatrollingInput* input, Solution* sol_current);
	void LLSRelaxAll(int ugv_num, std::vector<std::vector<int>>& drones_to_UGV, PatrollingInput* input, Solution* sol_current);
	bool areActionsOverlapping(const UGVAction& action1, const UGVAction& action2);
	bool isMoveADummy(const UGVAction& action, const UGVAction& prev_action);
	bool isOverlappingLaunchOrReceive(const UGVAction& action1, const UGVAction& action2);
};
