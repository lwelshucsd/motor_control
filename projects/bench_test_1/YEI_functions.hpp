#pragma once
/****************************************************************************
 Module
	motor_funcitons.hpp
 Description
	This is a set of funcitons utilizing the YEI 3-space sensor API intended 
	for use with the test tank functions.

*****************************************************************************/
#ifndef YEI_FUNCTIONS_HPP_
#define YEI_FUNCTIONS_HPP_
/*----------------------------- Include Files ------------------------------*/
#include <vector>
#include "vector_operators.hpp"
#include "ThreeSpace_API_C_3.0.6/threespace_api_export.h"

/*-------------------------------- Defines ---------------------------------*/

/*--------------------------------- Types ----------------------------------*/

class accelerometer {
private:
	TSS_ComPort YEI_port;
	tss_device_id YEI_device_id;
public:
	struct accel_config
	{
		
	} YEI_config;
	void initialize_f();
	std::vector<double> measure_all_f();
	std::vector<double> measure_accel_f();
	std::vector<double> measure_gyro_f();

};

/*------------------------------- Variables --------------------------------*/

/*---------------------- Public Function Prototypes ------------------------*/
//bool moveIsDone(class IPort& SC4_port);

/*------------------------------ End of file -------------------------------*/
#endif /* YEI_FUNCTIONS_HPP_ */
