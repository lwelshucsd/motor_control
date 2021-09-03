/****************************************************************************
 Module
	motor_funcitons.hpp
 Description
	This is a set of funcitons utilizing the clearpath sFoundation motor
	control library. This library is intended to abstract the node-based
	functions in sFoundation to work on a machine level with an arbitrary
	number of axes.

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
	sFnd::SysManager* my_mgr;
	void load_config_f(char delimiter);
	int open_ports_f();
	int enable_nodes_f();
	int disable_nodes_f();
	int set_config_f();
	void close_ports_f();
public:
	struct mech_config
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
		double accel_limit = 100000 * 6400 / 60;
	} config;
	struct machine_settings {
		bool r_mode = false;
	} settings;
	std::vector<double> current_position;
	//void setSpeed();
	int home_position_f();
	std::vector<double> measure_position_f();
	std::vector<double> move_linear_f(std::vector<double> input_vec, bool target_is_absolute);
	int start_up_f();
	void shut_down_f();
};

/*------------------------------- Variables --------------------------------*/

/*---------------------- Public Function Prototypes ------------------------*/
//bool moveIsDone(class IPort& my_port);

/*------------------------------ End of file -------------------------------*/
#endif /* MOTOR_FUNCTIONS_HPP_ */
