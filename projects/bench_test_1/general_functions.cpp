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
#include <fstream>
#include <filesystem>
#include <ctime>
#include <Windows.h>
/*--------------------------- External Variables ---------------------------*/
/*----------------------------- Module Defines -----------------------------*/
/*------------------------------ Module Types ------------------------------*/
/*---------------------------- Module Variables ----------------------------*/
static const char* logfile_dir = "logs";
static const char* logfile_prefix = "gantrylog_";
static const char* logfile_suffix = ".txt";
#define LOGFILE_FNAME_BUF_LEN	64
static char logfile_fname_buf[LOGFILE_FNAME_BUF_LEN];
#define LOG_MSG_BUF_LEN		64
static char log_msg_buf[LOG_MSG_BUF_LEN];

static std::fstream* logfile_ptr;

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

void save_array_f(std::vector<std::vector<double>> input_array) {
	std::ofstream ofs("test.txt", std::ofstream::out);
	for (auto& row : input_array) {
		for (auto col : row)
			ofs << col << ',';
		ofs << '\n';
	}
}




// 2026 09 18 LW: Added logging functionality


bool open_log_file_f() {

	bool ret = false;

	// Get the current time
	std::time_t epoch = std::time(nullptr);

	// Create a formatted time string
	const uint8_t ts_buf_len = 16;
	char ts_buf[ts_buf_len];
	std::strftime(ts_buf, ts_buf_len, "%Y%m%d-%H%M%S", std::gmtime(& epoch));

	// Create the logfile name (prefix + time)
	std::snprintf(logfile_fname_buf, LOGFILE_FNAME_BUF_LEN, "%s/%s%s%s", logfile_dir, logfile_prefix, ts_buf, logfile_suffix);

	// Set up log folder
	std::filesystem::create_directories(logfile_dir);

	// Attempt to open the logfile
	logfile_ptr = new std::fstream(logfile_fname_buf, std::ios::out);

	// If file opened successfully, assign pointer & return true
	if (logfile_ptr->is_open()) {
		ret = true;
	}

	return ret;
}


void log_str_f(const char* str) {

	logfile_ptr->write(str, strlen(str));
	logfile_ptr->flush();
}


void log_move_linear_f(int axis, double pos, double input_vec, bool absolute, double vel)
{
	// Calculate the distance for this move
	double distance = input_vec - (absolute ? pos : 0);

	// Note the time
	std::time_t epoch = std::time(nullptr);

	// Generate the log message for the linear move
	snprintf(log_msg_buf, LOG_MSG_BUF_LEN, "%lu,move,%d,%.2f,%.2f\n", (unsigned long)epoch, axis, distance, vel);

	// Write the log message string to the logfile
	log_str_f(log_msg_buf);
}


void log_home_axis_f(int axis)
{
	// Note the time
	std::time_t epoch = std::time(nullptr);

	// Generate the log message for the homing command
	snprintf(log_msg_buf, LOG_MSG_BUF_LEN, "%lu,home,%d\n", (unsigned long)epoch, axis);

	// Write the log message string to the logfile
	log_str_f(log_msg_buf);
}


void close_log_file_f(std::fstream file) {

	logfile_ptr->close();
}


void execute_command_script_f(machine my_machine)
{
	std::string script_fname;

	// Print the CWD to help the user navigate to the script file properly
	std::cout << "Current path: " << std::filesystem::current_path().string() << "\n";

	// Get the script file path from the user
	printf("Please input the name of the script file (relative to current path): ");
	std::cin >> script_fname;
	std::cout << script_fname << '\n';

	// Try to open the script file
	std::fstream script(script_fname, std::ios::in);
	if (script.is_open()) {

		// If successful, handle each line in the script file
		std::string line;
		while (std::getline(script, line))
		{
			// Print out line (for debugging)
			std::cout << line << '\n';

			// Check if this might be a valid command line
			if ((line.length() > 2) && (line[1] == ' ')) {

				// Isolate the command character and the argument
				char cmd = line[0];
				std::string arg = line.substr(2, std::string::npos);
				std::vector argv = parse_string_f(arg, ',');

				std::string msg_str = "";
				std::time_t epoch = std::time(nullptr);

				switch (cmd)
				{
					// 1: Change position
					case '1':
						my_machine.current_position = my_machine.move_linear_f(argv, true);
						break;

					// 2: Linear jog
					case '2':
						my_machine.current_position = my_machine.move_linear_f(argv, false);
						break;

					// 3: Set max velocity
					case '3':
						if (argv[0] <= my_machine.config.machine_velocity_max) {
							my_machine.config.machine_velocity_limit = argv[0];
						}
						break;

					// 4: Home axis
					case '4':
						my_machine.home_axis_f(argv[0]);
						break;

					// d: Delay
					//		Delay for given number of milliseconds
					case 'd':
						Sleep((DWORD)argv[0]);
						break;

					// l: Log message
					//		Add a custom message to the logfile
					case 'l':
						msg_str.append(std::to_string(epoch));
						msg_str.append(",msg,");
						msg_str.append(arg);
						msg_str.append("\n");

						log_str_f(msg_str.c_str());

						break;

					// w: Wait for user confirmation
					case 'w':
						std::cin.clear();
						std::cin.ignore(100, '\n');
						msg_user_f(arg.c_str());
						break;

					// Catch non-command lines here
					default:
						//std::cout << line;
						break;
				}


			}



		}
	}



}


/*----------------------------- Test Harness -------------------------------*/

/*------------------------------- Footnotes --------------------------------*/
/*------------------------------ End of file -------------------------------*/
