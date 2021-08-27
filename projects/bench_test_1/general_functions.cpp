/****************************************************************************
 Module
    module_name.c
 Description
    description of module
 Notes
    additional notes

 History
 When             Who    What/Why
 --------------   ---    --------
 DD MMMM YYYY     XXX    changes
*****************************************************************************/

/*----------------------------- Include Files ------------------------------*/
#include "general_functions.hpp"
#include <iostream>
/*--------------------------- External Variables ---------------------------*/
/*----------------------------- Module Defines -----------------------------*/
/*------------------------------ Module Types ------------------------------*/
/*---------------------------- Module Variables ----------------------------*/

/*--------------------- Module Function Prototypes -------------------------*/
/*------------------------------ Module Code -------------------------------*/
// General Functions
char msgUser(const char* msg) {
	std::cout << msg;
	return getchar();
}
void vectorPrint(std::vector<double> const& a, std::string comment) {
	//Prints a std::vector with a given in
	std::cout << comment << "(";

	for (int i = 0; i < a.size(); i++)
		std::cout << a.at(i) << ',' << " ";
	std::cout << "\b\b)\n";
}

std::vector<double> parseString(std::string input, char delimiter) {
	int last_slice = 0;
	std::vector<double> tmp_vec;
	for (size_t i = 0; i <= input.length(); i++) {
		if (i == input.length()) {
			try {
				double val = std::stod(input.substr(last_slice, std::string::npos));
				tmp_vec.push_back(val);
			}
			catch (...) {
				continue;
			}
		}
		else if (input.at(i) == delimiter || input.at(i) == ' ') {
			try {
				double val = std::stod(input.substr(last_slice, i - last_slice));
				tmp_vec.push_back(val);
				last_slice = i + 1;
			}
			catch (...) {
				continue;
			}
		}
	}

	return tmp_vec;
}
/*----------------------------- Test Harness -------------------------------*/

/*------------------------------- Footnotes --------------------------------*/
/*------------------------------ End of file -------------------------------*/
