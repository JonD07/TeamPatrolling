/*
 * Input.h
 *
 * Created by:	Jonathan Diller
 * On: 			Jun 25, 2024
 *
 * Description: General Input class used to help process
 *  text file problem inputs.
 *
 */

#pragma once

#include <sstream>
#include <fstream>

#include "defines.h"

class Input {
public:
	Input();
	virtual ~Input();

protected:
	// Get next line from input an input file, ignores lines that start with '#'
	bool getNextLine(std::ifstream* file, std::string* line);
};
