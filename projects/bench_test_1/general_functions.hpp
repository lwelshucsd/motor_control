/****************************************************************************
 Module
    module_name.h
 Description
    description of module
 Notes
    additional notes

 History
 When             Who    What/Why
 --------------   ---    --------
 DD MMMM YYYY     XXX    changes
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
char msgUser(const char* msg);
void vectorPrint(std::vector<double> const& a, std::string comment);
std::vector<double> parseString(std::string input, char delimiter);

/*------------------------------ End of file -------------------------------*/
#endif /* MOTOR_FUNCTIONS_HPP_ */
