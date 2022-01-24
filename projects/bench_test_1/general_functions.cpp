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

	/// Summary: Requests user input of a single character, given a msg prompt
	/// Params: 
	///		msg - message to prompt user input in command line
	/// Returns: 
	///		Returns the next character input
	/// Notes: 

	std::cout << msg;
	return getchar();
}
void print_vector_f(std::vector<double> const& a, std::string comment) {

	/// Summary: Funciton to print vector variable to commandline with elements separated by commas
	/// Params: 
	///		a - input vector to print
	///		comment- string to print before the the vector
	/// Returns: void
	/// Notes:	

	//Prints a std::vector with a given in
	std::cout << comment << "(";

	for (int i = 0; i < a.size(); i++)	// Print each element and add a comma and space
		std::cout << a.at(i) << ',' << " ";
	std::cout << "\b\b)\n";	//Delete the final comma and space, close the parentheses, and go to newline
}

std::vector<double> parse_string_f(std::string input, char delimiter) {

	/// Summary: Parses string representation of a vector with values delimited by some character into a vector variable
	/// Params: 
	///		input - input string of all values, using a constant delimiter. Input can have spaces, but should not be surrounded by brackets.
	///		delimiter - delimiter character to separate each value. must be the same character in the string. Cannot be a whitespace. 
	/// Returns: 
	///		tmp_vec - parsed input vector
	/// Notes: 
	/// 

	//Remove spaces from string
	input.erase(std::remove_if(input.begin(), input.end(), isspace), input.end());


	int last_slice = 0;				// last_slice is the index after the last delimiter found
	std::vector<double> tmp_vec;	//create temporary return vector

	//Iterate through characters in the string to separate values
	for (size_t i = 0; i <= input.length(); i++) {
		//if the string is completed, take the last value
		if (i == input.length()) {
			try {
				double val = std::stod(input.substr(last_slice, std::string::npos));
				tmp_vec.push_back(val);
			}
			catch (...) {
				continue;
			}
		}
		// if the character is the delimiter or is a space, take the value between the last delimiter and the new one
		// update the delimiter posiiton
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
		// else (i.e. character is not a delimiter and i =/= input.length()) continue to the next character
	}

	return tmp_vec;
}

std::vector<double> user_input_vector_f(std::string prompt, int expected_size) {

	/// Summary: Similar to msg_user_f(), allows the user to input a 1d, numeric vector when given a prompt
	/// Params: 
	///		prompt - message to prompt user input in command line
	///		expected_size - the expected size of the vector input, to prevent inputs too long or too short.
	/// Returns: 
	///		input_vec - parsed numeric vector
	/// Notes: 
	/// 

	std::string input_str;
	std::vector <double> input_vec;

	while (true) {
		// Prompt user and wait for response
		std::cout << prompt;
		std::cin >> input_str;

		// Remove whitespaces from the string
		input_str.erase(std::remove_if(input_str.begin(), input_str.end(), isspace), input_str.end());

		// Parse the user input string into a numeric vector
		input_vec = parse_string_f(input_str, ',');

		//Check that the vector is the right size
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
