/****************************************************************************
 Module
    general_functions.hpp
 Description
    This is a set of miscellaneous and general use functions for the clearpath
    motor machine contorl software.

*****************************************************************************/
#ifndef GENERAL_FUNCTIONS_HPP_
#define GENERAL_FUNCTIONS_HPP_
/*----------------------------- Include Files ------------------------------*/

#include "pubSysCls.h"	
#include <string>
#include <vector> 


/*-------------------------------- Defines ---------------------------------*/

/*--------------------------------- Types ----------------------------------*/

/*------------------------------- Variables --------------------------------*/

/*---------------------- Public Function Prototypes ------------------------*/
// IO Functions
char msg_user_f(const char* msg);
void print_vector_f(std::vector<double> const& a, std::string comment);
std::vector<double> user_input_vector_f(std::string prompt, int expected_size);
void save_array_f(std::vector<std::vector<double>> input_array);
// 2026 09 18 LW: Added logging functionality
bool open_log_file_f();
void log_str_f(const char* str);
void log_move_linear_f(int axis, double pos, double input_vec, bool absolute, double vel);
void log_home_axis_f(int axis);
void close_log_file_f(std::fstream file);
void execute_command_script_f();
/*------------------------------ End of file -------------------------------*/
#endif /* MOTOR_FUNCTIONS_HPP_ */
