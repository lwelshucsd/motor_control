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

/*------------------------------ End of file -------------------------------*/
#endif /* MOTOR_FUNCTIONS_HPP_ */
