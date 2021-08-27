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
struct mConfig
{
	std::vector<double> is_follower_node;
	std::vector<double> node_sign;
	std::vector<double> lead;
	std::vector<double> is_rotary_axis;
	std::vector<double> lead_per_cnt;
	std::vector<double> node_2_axis;
	double velocity_limit;
	double num_axes = is_rotary_axis.size();
};

class machine {
private:
	sFnd::SysManager* myMgr;
	void loadConfig();
	int openPorts();
	int enableNodes();
	void disableNodes();
	int setConfig();
	void closePorts();
	//IPort& myPort;
public:
	std::vector<double> current_pos;
	mConfig config;
	void setSpeed();
	std::vector<double> homePosn();
	std::vector<double> measurePosn();
	std::vector<double> linearMove(bool r_mode, bool target_is_absolut);
	bool startUp();
	void shutDown();
};

/*------------------------------- Variables --------------------------------*/

/*---------------------- Public Function Prototypes ------------------------*/
//bool moveIsDone(class IPort& myPort);

/*------------------------------ End of file -------------------------------*/
#endif /* MOTOR_FUNCTIONS_HPP_ */
