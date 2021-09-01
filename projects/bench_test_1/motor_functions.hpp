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
#ifndef MOTOR_FUNCTIONS_HPP_
#define MOTOR_FUNCTIONS_HPP_
/*----------------------------- Include Files ------------------------------*/
#include "pubSysCls.h"	
#include <vector>
#include "vector_operators.hpp"

/*-------------------------------- Defines ---------------------------------*/

/*--------------------------------- Types ----------------------------------*/

class machine {
private:
	sFnd::SysManager* myMgr;
	void loadConfig(char delimiter);
	int openPorts();
	int enableNodes();
	int disableNodes();
	int setConfig();
	void closePorts();
public:
	struct mechConfig
	{
		std::vector<double> is_follower_node;
		std::vector<double> node_sign;
		std::vector<double> lead;
		std::vector<double> is_rotary_axis;
		std::vector<double> lead_per_cnt;
		std::vector<double> parent_axis;
		double velocity_limit;
		double max_velocity_limit;
		double num_axes = is_rotary_axis.size();
	} config;
	struct machineSettings {
		bool r_mode = false;
	} settings;
	std::vector<double> current_pos;
	void setSpeed();
	int homePosn();
	std::vector<double> measurePosn();
	std::vector<double> linearMove(std::vector<double> input_vec, bool target_is_absolut);
	int startUp();
	void shutDown();
};

/*------------------------------- Variables --------------------------------*/

/*---------------------- Public Function Prototypes ------------------------*/
//bool moveIsDone(class IPort& myPort);

/*------------------------------ End of file -------------------------------*/
#endif /* MOTOR_FUNCTIONS_HPP_ */
