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
#include <stdio.h>	
#include <string>
#include <math.h>
#include <iostream>
#include <Windows.h>
#include <valarray>
#include <vector>
#include "pubSysCls.h"
#include "vector_operators.hpp"
#include "general_functions.hpp"


using namespace sFnd;
using std::valarray;
using std::vector;
using std::cin;
using std::cout;
using std::string;
/*-------------------------------- Defines ---------------------------------*/

/*--------------------------------- Types ----------------------------------*/
struct mConfig
{
	vector<double> is_follower_node;
	vector<double> node_sign;
	vector<double> lead;
	vector<double> is_rotary_axis;
	vector<double> lead_per_cnt;
	vector<double> node_2_axis;
	double velocity_limit;
	double num_axes = is_rotary_axis.size();
};

class machine {
private:
	SysManager* myMgr;
	void loadConfig();
	int openPorts();
	int enableNodes();
	void disableNodes();
	int setConfig();
	void closePorts();
	//IPort& myPort;
public:
	vector<double> current_pos;
	mConfig config;
	void setSpeed();
	vector<double> homePosn();
	vector<double> measurePosn();
	vector<double> linearMove(bool r_mode, bool target_is_absolut);
	bool startUp();
	void shutDown();
};

/*------------------------------- Variables --------------------------------*/

/*---------------------- Public Function Prototypes ------------------------*/
//bool moveIsDone(class IPort& myPort);

/*------------------------------ End of file -------------------------------*/
#endif /* MOTOR_FUNCTIONS_HPP_ */
