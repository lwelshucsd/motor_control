/****************************************************************************
 Module
	general_functions.cpp
 Description
	This is a set of miscellaneous and general use functions for the clearpath
	motor machine contorl software.

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
char msg_user_f(const char* msg) {

	/// Summary: 
	/// Params: 
	/// Returns: 
	/// Notes: 

	std::cout << msg;
	return getchar();
}
void print_vector_f(std::vector<double> const& a, std::string comment) {

	/// Summary: 
	/// Params: 
	/// Returns: 
	/// Notes:	

	//Prints a std::vector with a given in
	std::cout << comment << "(";

	for (int i = 0; i < a.size(); i++)
		std::cout << a.at(i) << ',' << " ";
	std::cout << "\b\b)\n";
}

std::vector<double> parse_string_f(std::string input, char delimiter) {

	/// Summary: 
	/// Params: 
	/// Returns: 
	/// Notes: 
	input.erase(std::remove_if(input.begin(), input.end(), isspace), input.end());
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

std::vector<double> user_input_vector_f(std::string prompt, int expected_size) {
	std::string input_str;
	std::vector <double> input_vec;
	while (true) {
		std::cout << prompt;
		std::cin >> input_str;

		input_str.erase(std::remove_if(input_str.begin(), input_str.end(), isspace), input_str.end());

		input_vec = parse_string_f(input_str, ',');	// Parse the user input string into 
		if (input_vec.size() == expected_size) {
			return input_vec;
		}
		else {
			std::cout << "\nSize of input vector does not match the number of axes in the system: " << expected_size;
			std::cout << "\nPlease try again.";
		}
	}
}
/*----------------------------- Test Harness -------------------------------*/

/*------------------------------- Footnotes --------------------------------*/
/*------------------------------ End of file -------------------------------*/
