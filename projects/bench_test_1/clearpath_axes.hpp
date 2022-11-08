/****************************************************************************
 Module
	motor_funcitons.hpp
 Description
	This is a set of functions utilizing the Clearpath sFoundation motor
	control library. This library is intended to abstract the node-based
	functions in sFoundation to work on a machine level with an arbitrary
	number of axes and an arbitrary number of nodes per axis.
	It also incorporates Accelerometer Feedback from a YEI 3-space sensor.

*****************************************************************************/
#ifndef MOTOR_FUNCTIONS_HPP_
#define MOTOR_FUNCTIONS_HPP_
/*----------------------------- Include Files ------------------------------*/
#include "pubSysCls.h"	
#include <vector>
#include "vector_operators.hpp"
//#include "YEI_functions.hpp"

/*-------------------------------- Defines ---------------------------------*/

/*--------------------------------- Types ----------------------------------*/

class machine {
private:
	sFnd::SysManager* SC4_mgr;
	void load_config_f(char delimiter);
	int open_ports_f();
	int enable_nodes_f();
	int disable_nodes_f();
	int set_config_f();
	void close_ports_f();
public:
	struct mech_config
	{
		std::vector<double> node_is_follower;
		std::vector<double> node_sign;
		std::vector<double> node_lead;
		std::vector<double> node_is_rotary_axis;
		std::vector<double> node_lead_per_cnt;
		std::vector<double> node_cnts_per_rev;
		std::vector<double> node_parent_axis;
		double machine_velocity_limit;
		double machine_velocity_max;
		double machine_num_axes;
		double machine_accel_limit;
		double homing_speed;
	} config;
	struct machine_settings {
		bool remote_mode = false;
	} settings;
	std::vector<double> current_position;
	std::vector<double> measure_position_f();
	std::vector<double> move_linear_f(std::vector<double> input_vec, bool target_is_absolute);
	int home_axis_f(int axis_id);
	int start_up_f();
	void shut_down_f();
};

/*------------------------------- Variables --------------------------------*/

/*---------------------- Public Function Prototypes ------------------------*/
//bool moveIsDone(class IPort& SC4_port);

/*------------------------------ End of file -------------------------------*/
#endif /* MOTOR_FUNCTIONS_HPP_ */
