#include "Input.h"

Input::Input() {
}

Input::~Input() {
}


// Gets the next valid line from file, stores it in line. Will ignore
// lines that start with '#' symbol.
bool Input::getNextLine(std::ifstream* file, std::string* line) {
	bool run_again = true;
	bool read_success = false;
	// Grab next line un-commented line
	while(run_again) {
		if(std::getline(*file, *line)) {
			if(0) {
				printf("Next line: \"%s\"\n", (*line).c_str());
				printf("First char: \'%d\'\n", (*line)[0]);
			}
			// Successfully read next line, verify it doesn't start with '#'
			if((*line)[0] != '#') {
				// Found next valid input line
				run_again = false;
				read_success = true;
			}
			else {
				// Line starts with '#', get next line
				run_again = true;
				if(0)
					puts("Ignoring line");
			}
		}
		else {
			// Failed to read next line
			run_again = false;
			read_success = false;
		}
	}

	if(0)
		printf("Return Line: \"%s\"\n", (*line).c_str());

	return read_success;
}
