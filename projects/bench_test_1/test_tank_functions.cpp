/****************************************************************************
 Module
	motor_funcitons.cpp
 Description
	This is a set of funcitons utilizing the clearpath sFoundation motor
	control library. This library is intended to abstract the node-based
	functions in sFoundation to work on a machine level with an arbitrary
	number of axes.

*****************************************************************************/

/*----------------------------- Include Files ------------------------------*/
#include "test_tank_functions.hpp"
#include "general_functions.hpp"
#include <Windows.h>
#include <iostream>
#include <fstream>
#include <filesystem>
#include <chrono>


#define LONG_DELAY				500
#define SHORT_DELAY				100
#define TIME_TILL_TIMEOUT		2500000 //The timeout used for homing(ms)
//#define ACC_LIM_CNTS_PER_SEC2	640000000
#define MAX_VEL_LIM				2000

using namespace sFnd;
namespace fs = std::filesystem;
/*--------------------------- External Variables ---------------------------*/
/*----------------------------- Module Defines -----------------------------*/
/*------------------------------ Module Types ------------------------------*/
/*---------------------------- Module Variables ----------------------------*/
/*--------------------- Module Function Prototypes -------------------------*/
/*------------------------------ Module Code -------------------------------*/
bool move_is_done_f(class IPort& SC4_port) {

	/// Summary: Checks to see if all nodes have triggered the MoveIsDone flag, meaning that whatever movement is planned has been completed by all nodes.
	/// Params: SC4_port: IPort object of port connecting to nodes
	/// Returns: bool true if all nodes have completed their movement. false if ANY node is still moving.
	/// Notes:

	// Check all nodes on port.
	for (size_t iNode = 0; iNode < SC4_port.NodeCount(); iNode++) {
		if (!SC4_port.Nodes(iNode).Motion.MoveIsDone()) { return false; }	// If any node is still moving, return false.
	}
	return true;	// If all nodes are done, return true.
}

std::vector<double> push_back_string_f(std::vector<double> input_vector, std::string input_str, char delimiter) {

	/// Summary: Takes string of vector and fills into the back of an input vector using std::vector.push_back()
	/// Params:	input_vector: The vector to fill the data into; must be declared prior to use 
	///			input_str: The string version of the data to include. This can be surrounded by the following brackets: [], (), {}, but
	///						must not contain anything else outside the brackets. It can also have spaces.
	///			delimiter: The character separating the data points in the input string vector. Must not be a space, but can be any reasonable
	///						character, such as ; , |
	/// Returns: An expanded copy of input_vector including the new data at the end of the vector. If input_vector is exmpty originally, output is
	///				the input_string data in std::vector form
	/// Notes: 

	// Remove extra characters
	input_str.erase(std::remove_if(input_str.begin(), input_str.end(), isspace), input_str.end());
	input_str.erase(std::remove(input_str.begin(), input_str.end(), '{'), input_str.end());
	input_str.erase(std::remove(input_str.begin(), input_str.end(), '}'), input_str.end());
	input_str.erase(std::remove(input_str.begin(), input_str.end(), '['), input_str.end());
	input_str.erase(std::remove(input_str.begin(), input_str.end(), ']'), input_str.end());
	input_str.erase(std::remove(input_str.begin(), input_str.end(), '('), input_str.end());
	input_str.erase(std::remove(input_str.begin(), input_str.end(), ')'), input_str.end());

	int delimiter_pos = 0;
	double input_val;
	// std::vector.find() returns -1 if not found. After last data point, all delimiters have been removed from string, so the loop ends
	while (delimiter_pos != -1) {
		delimiter_pos = input_str.find(delimiter);
		input_val = std::stod(input_str.substr(0, delimiter_pos));
		input_str.erase(0, delimiter_pos + 1);		// delete the parsed data from the string
		input_vector.push_back(input_val);			// insert data to vector
	}
	return input_vector;

}

// Machine Specific functions
void machine::load_config_f(char delimiter) {
	YEI_port.port_name = new char[64];
	TSS_ERROR error = TSS_NO_ERROR;

	printf("====Creating a Three Space Device from Search====\n");
	tss_findSensorPorts(TSS_FIND_ALL_KNOWN ^ TSS_DONGLE);

	error = tss_getNextSensorPort(YEI_port.port_name, &YEI_port.device_type, &YEI_port.connection_Type);
	if (error == TSS_NO_ERROR)
	{
		error = tss_createSensor(YEI_port.port_name, &YEI_device_id);

		if (error)
		{
			printf("Failed to create TSS Sensor on %s!\n", YEI_port.port_name);
			tss_deinitAPI();
			printf("Finished press Enter to continue");
			getchar();
		}
		else
		{
			printf("====Starting Streaming====\n");
			error = tss_sensor_startStreamingWired(YEI_device_id, TSS_STREAM_CORRECTED_SENSOR_DATA, 500, TSS_STREAM_DURATION_INFINITE, 0);
			error = tss_sensor_enableTimestampsWired(YEI_device_id);

			Sleep(10);
			TSS_Stream_Packet packet;
			std::vector<std::vector<double>> threeSpaceData;
			using namespace std::chrono;
			long long start = duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count();

			for (int i = 0; i < 10; i++)
			{
				long long ms = duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count();
				ms = ms - start;
				threeSpaceData.push_back({ double(ms) });
				Sleep(50);
				printf("Press Enter to get next packet.\n");
				error = tss_sensor_getLastStreamingPacket(YEI_device_id, &packet);
				printf("Gyro	: (%.03f,%.03f,%.03f)\n", packet.correctedSensorData[0], packet.correctedSensorData[1], packet.correctedSensorData[2]);
				printf("Accel	: (%.03f,%.03f,%.03f)\n", packet.correctedSensorData[3], packet.correctedSensorData[4], packet.correctedSensorData[5]);
				printf("Magnet	: (%.03f,%.03f,%.03f)\n", packet.correctedSensorData[6], packet.correctedSensorData[7], packet.correctedSensorData[8]);
				for (int j = 0; j < 9; j++) {
					//threeSpaceData[j][i] = packet.correctedSensorData[j];
					threeSpaceData[i].push_back(packet.correctedSensorData[j]);
				}
			}

			save_array_f(threeSpaceData);
			tss_sensor_stopStreamingWired(YEI_device_id);

			tss_removeSensor(YEI_device_id);
		}
	}
	else
	{
		printf("Failed to get the port!\n");
		tss_deinitAPI();
		printf("Finished press Enter to continue");
		getchar();
	}

	tss_deinitAPI();

	printf("Finished press Enter to continue");
	getchar();
	/// Summary: Loads the mechanical configuration data of the machine into the machine.config data structure
	/// Params: delimiter: The character separating the nnput config data from the data names
	/// Returns: Void
	/// Notes: 
	///		(08/30/21) - Building way to load mechanical config from .cfg file. -TH
	///		(11/12/21) - Able to load configs with set variables frm .txt file, but not thoroughly tested. -TH

	// Include the name of all expected config file variables here
	std::vector<std::string> var_names = {
		"node_is_follower",
		"node_sign",
		"node_lead",
		"node_cnts_per_rev",
		"node_parent_axis",
		"machine_accel_limit",
		"machine_velocity_limit",
		"machine_velocity_max",
		"node_is_rotary_axis",
		"homing_speed"
	};

	// Prep file for data parsing
	int delimiter_pos;
	std::string line;
	std::string key;
	std::vector<std::string> values(var_names.size());
	std::ifstream myReadFile("mech_config.txt");
	//std::cout << "\n";
	int n_found_variables = 0;
	int key_index;

	// Scan through file line by line for variables
	while (std::getline(myReadFile, line)) {
		line.erase(std::remove_if(line.begin(), line.end(), isspace), line.end()); 			// Remove spaces from line
		delimiter_pos = line.find(delimiter);												// Find delimiter
		key = line.substr(0, line.find("["));												// Find variable name, occurs before [variable units]
		if (std::find(var_names.begin(), var_names.end(), key) != var_names.end()) {		// Try to find variable name in expected variables
			key_index = find(var_names.begin(), var_names.end(), key) - var_names.begin();	// Get index of variable to store in string values vector
			values[key_index] = line.substr(delimiter_pos + 1);								// Store input string
			n_found_variables += 1;
		}
		else if (line.substr(0, 2) == "//") {	// double slash indicates comments
			continue;
		}
		else {									// If variable in config is not expected, it is skipped, but noted in command-line
			std::cout << "Ignoring unrecognized config element: " << key << "\n";
		}
	}
	myReadFile.close();

	// Run check that all expected variables are found
	if (n_found_variables > var_names.size()) {
		std::cout << "Number of variables in config file exceeds expected.\n";
		std::cout << "Found Variables: " << n_found_variables << " ||| Expected: " << var_names.size();
		std::cout << "Please check for repeat variables.";
		throw "Too many recognized variables in config file.";
	}
	else if (n_found_variables < var_names.size()) {
		std::cout << "Number of variables in config file is less than expected.\n";
		std::cout << "Found Variables: " << n_found_variables << " ||| Expected: " << var_names.size();
		std::cout << "Please check for missing variables.";
		throw "Missing variables in config file.";
	}
	else {	// If expected number of variables are found
		// Assign each expected config variable to their input value
		// New config variables must be added here
		config.node_is_follower = push_back_string_f(config.node_is_follower, values[0], ',');
		config.node_sign = push_back_string_f(config.node_sign, values[1], ',');
		config.node_lead = push_back_string_f(config.node_lead, values[2], ',');
		config.node_cnts_per_rev = push_back_string_f(config.node_cnts_per_rev, values[3], ',');
		config.node_parent_axis = push_back_string_f(config.node_parent_axis, values[4], ',');
		config.machine_accel_limit = stod(values[5]);
		config.machine_velocity_limit = stod(values[6]);
		config.machine_velocity_max = stod(values[7]);
		config.node_is_rotary_axis = push_back_string_f(config.node_is_rotary_axis, values[8], ',');
		config.homing_speed = stod(values[9]);
	}

	// Calculate derived config variables that are not needed in .txt file
	config.machine_num_axes = config.node_is_follower.size() - vector_sum(config.node_is_follower);
	config.node_lead_per_cnt = config.node_lead / config.node_cnts_per_rev;

}

int machine::set_config_f() {

	/// Summary: Loads required parameters/operation modes into each node
	/// Params: 
	/// Returns: int representing success (1) or failure (-1)
	/// Notes:	(09/01/2021) - May be worthwile to have these parameters be config variables -TH
	///			(11/12/2021) - Clarification: it may be worthwile to have the AccUnit and VelUnit be config variables
	///							because in rotary systems, using RPM/s and RPM may be simpler. Counts/s^2 and Counts/s
	///							make more sense in translational systems such as the gantry. It would be more robust 
	///							to have these in the config .txt file. -TH

	try {
		IPort& SC4_port = SC4_mgr->Ports(0);
		// Iterate through each node in the machine
		for (size_t iNode = 0; iNode < SC4_port.NodeCount(); iNode++) {
			SC4_port.Nodes(iNode).AccUnit(INode::COUNTS_PER_SEC2);			// set acceleration limit tracking unit
			SC4_port.Nodes(iNode).VelUnit(INode::COUNTS_PER_SEC);				// set velocity limit unit
			SC4_port.Nodes(iNode).Motion.AccLimit = config.machine_accel_limit;		// set acceleration limit
			SC4_port.Nodes(iNode).Motion.VelLimit = config.machine_velocity_limit;	// set default velocity limit
			SC4_port.Nodes(iNode).Motion.PosnMeasured.AutoRefresh(true);
		}
		return 1;
	}
	catch (mnErr& theErr)
	{
		printf("Failed to change node settings(s)\n");
		//This statement will print the address of the error, the error code (defined by the mnErr class), 
		//as well as the corresponding error message.
		printf("Caught error: addr=%d, err=0x%08x\nmsg=%s\n", theErr.TheAddr, theErr.ErrorCode, theErr.ErrorMsg);

		msg_user_f("Press any key to continue."); //pause so the user can see the error message; waits for user to press a key
		return -1;
	}
}

std::vector<double> machine::measure_position_f() {

	/// Summary: Measures position of all axes by measuring all non-follower nodes and converting to real space.
	/// Params: None
	/// Returns: Double vector of real-space position measured on leader nodes.
	/// Notes: 

	IPort& SC4_port = SC4_mgr->Ports(0);	// Create Port object
	int machine_num_axes = config.machine_num_axes;

	std::vector<double> position(config.node_is_follower.size());	// Initialize position vector

	int dim_ind = 0;
	// Measure each node's position and fill them into a vector
	for (size_t iNode = 0; iNode < SC4_port.NodeCount(); iNode++) {
		position[iNode] = (SC4_port.Nodes(iNode).Motion.PosnMeasured) * config.node_sign[iNode];
	}

	position = position * config.node_lead_per_cnt;	//convert count-space to real-space
	for (size_t iNode = 0; iNode < position.size(); iNode++) {
		if (config.node_is_follower[iNode]) {
			position.erase(position.begin() + iNode);
		}
	}
	return position;
}

std::vector<double> machine::move_linear_f(std::vector<double> input_vec, bool target_is_absolute) {

	/// Summary: Abstracts the triangular node.motion.movePosnStart() to a multi-axis system, using trigger groups to enforce 
	///			a simultaneous movement start. Waits until move is completed on all nodes to return the measured position after
	///			said move. Velocity on each node is calculated and set such that the mvoement is linear and all nodes complete together.
	/// Params:	input_vec: a double vector of the desired position/job distance for the machine
	///			target_is_absolute: a bool representing if the target of the move is an absolute positional change or a relative position jog.
	/// Returns: Function returns a vector of the measured position of the machine after the movement is completed (or after it times out).
	/// Notes:	

	IPort& SC4_port = SC4_mgr->Ports(0);	// Create a shortcut for the port

	// Initialize position vectors to be used in velocity calculations
	std::vector<double> end_pos;
	std::vector<double> vel_vec;
	std::vector<double> current_pos = machine::measure_position_f();

	// Create shortcuts to important config members
	int machine_num_axes = config.machine_num_axes;
	double machine_velocity_limit = config.machine_velocity_limit;
	std::vector<double> node_is_follower = config.node_is_follower;
	std::vector<double> lead_per_cnt = config.node_lead_per_cnt;
	std::vector<double> node_sign = config.node_sign;

	// Compute velocity vector for movement 
	vel_vec = input_vec - (current_pos | target_is_absolute);	// Compute movement direction vector
	vel_vec = normalize(vel_vec);								// Normalize movement direction vector
	vel_vec = vel_vec | machine_velocity_limit;							// Multiply by overall velocity limit

	int node_axis;
	double node_input_cnts;
	double node_machine_velocity_limit;

	// Set up trigger group  & velocity for all nodes
	for (size_t iNode = 0; iNode < SC4_port.NodeCount(); iNode++) {
		node_axis = config.node_parent_axis[iNode];

		// Convert velocity to counts/s and apply limit
		node_machine_velocity_limit = vel_vec[node_axis] / lead_per_cnt[iNode];
		SC4_port.Nodes(iNode).Motion.VelLimit = abs(node_machine_velocity_limit);

		//Convert distance to counts and set up trigger
		node_input_cnts = input_vec[node_axis] / lead_per_cnt[iNode] * node_sign[iNode];
		SC4_port.Nodes(iNode).Motion.Adv.TriggerGroup(1);	// add all to same trigger group
		SC4_port.Nodes(iNode).Motion.Adv.MovePosnStart(node_input_cnts, target_is_absolute, true);
	}
	SC4_port.Nodes(0).Motion.Adv.TriggerMovesInMyGroup();	// Trigger group


	double timeout = SC4_mgr->TimeStampMsec() + TIME_TILL_TIMEOUT; //define a timeout in case the node is unable to enable or takes too long
	while (!move_is_done_f(SC4_port)) {
		//printf("%00008.2f   ", SC4_mgr->TimeStampMsec() - timeout + TIME_TILL_TIMEOUT);
		//print_vector_f(measurePosn(), "");
		if (SC4_mgr->TimeStampMsec() > timeout) {
			printf("Error: timed out waiting for move to complete\n");
			msg_user_f("press any key to continue."); //pause so the user can see the error message; waits for user to press a key
			SC4_port.NodeStop();	// Stops the nodes at their current position
			return measure_position_f();
		}
	}

	Sleep(SHORT_DELAY);
	end_pos = measure_position_f();	// Measure ending position of machine to return
	
	return end_pos;
}

int machine::enable_nodes_f() {

	/// Summary: Enables all nodes on a port and prints detected node data
	/// Params: None
	/// Returns: Int of -2 to imply fialure, 1 to imply success
	/// Notes: 

	IPort& SC4_port = SC4_mgr->Ports(0);			// Create a shortcut for the port
	printf("\n===== Detected Node Data =====\n");
	printf("        Type || FW Version || Serial # || Model\n");
	try {
		for (size_t iNode = 0; iNode < SC4_port.NodeCount(); iNode++) {
			// Create a shortcut reference for a node
			INode& the_node = SC4_port.Nodes(iNode);

			the_node.EnableReq(false);				//Ensure Node is disabled before loading config file

			SC4_mgr->Delay(SHORT_DELAY);

			// If config is not stored on the motor we need to enable this
			//the_node.Setup.ConfigLoad("Config File path");

			// Print detected node data
			printf(
				"Node[%d]:  %d  || %s || %d || %s\n",
				int(iNode), the_node.Info.NodeType(),
				the_node.Info.FirmwareVersion.Value(),
				the_node.Info.SerialNumber.Value(),
				the_node.Info.Model.Value()
			);

			//The following statements will attempt to enable the node.  First,
			// any shutdowns or NodeStops are cleared, finally the node is enabled
			the_node.Status.AlertsClear();					//Clear Alerts on node 
			the_node.Motion.NodeStopClear();	//Clear Nodestops on Node  				
			the_node.EnableReq(true);					//Enable node 
			//At this point the node is enabled
			//printf("Node[%d] enabled.\n", int(iNode));
			double timeout = SC4_mgr->TimeStampMsec() + TIME_TILL_TIMEOUT;	//define a timeout in case the node is unable to enable
			while (!the_node.Motion.IsReady()) {								//This will loop checking on the Real time values of the node's Ready status
				if (SC4_mgr->TimeStampMsec() > timeout) {
					printf("Error: Timed out waiting for Node %d to enable\n", iNode);
					msg_user_f("Press any key to continue."); //pause so the user can see the error message; waits for user to press a key
					return -2;
				}
			}
		}
		// Prints the User-defined node IDs for the user to check
		printf("\n===== User Node IDs =====\n");
		for (size_t iNode = 0; iNode < SC4_port.NodeCount(); iNode++) {
			printf("Node[%d]: %s\n", int(iNode), SC4_port.Nodes(iNode).Info.UserID.Value());
		}
		return 1;
	}
	catch (mnErr& theErr)
	{
		printf("Failed to enable nodes(s)\n");
		//This statement will print the address of the error, the error code (defined by the mnErr class), 
		//as well as the corresponding error message.
		printf("Caught error: addr=%d, err=0x%08x\nmsg=%s\n", theErr.TheAddr, theErr.ErrorCode, theErr.ErrorMsg);

		msg_user_f("Press any key to continue."); //pause so the user can see the error message; waits for user to press a key
		return -1;
	}
}

int machine::disable_nodes_f() {

	/// Summary: Disables all nodes on the port
	/// Params: 
	/// Returns: 
	/// Notes: 

	try {
		IPort& SC4_port = SC4_mgr->Ports(0);
		printf("\nDisabling nodes\n");
		for (size_t iNode = 0; iNode < SC4_port.NodeCount(); iNode++) {
			// Create a shortcut reference for a node
			SC4_port.Nodes(iNode).EnableReq(false);
		}
		return 1;
	}
	catch (mnErr& theErr)
	{
		printf("Failed to disable nodes. Please manually disable all nodes in Clearview.\n");
		//This statement will print the address of the error, the error code (defined by the mnErr class), 
		//as well as the corresponding error message.
		printf("Caught error: addr=%d, err=0x%08x\nmsg=%s\n", theErr.TheAddr, theErr.ErrorCode, theErr.ErrorMsg);

		msg_user_f("Press any key to continue."); //pause so the user can see the error message; waits for user to press a key
		return -1;  //This terminates the main program
	}
};


int machine::home_axis_f(int axis_id) {

	/// Summary: Starts homing routine defined for each motor. Motors have to be set up to home in the correct directions using the
	///				clearview software.
	/// Params: 
	/// Returns: 
	/// Notes: 

	//	//////////////////////////////////////////////////////////////////////////////////////////////////
	//	//////////////////////////////////////////////////////////////////////////////////////////////////
	//	//	NEED TO CHANGE HOMING OPERATION TO HOME LEADER-FOLLOWER SETS TOGETHER
	//	//	OR TO HOME ALL NODES AT THE SAME TIME, BASICALLY AS A MOVE TO ORIGIN
	//	//	THIS IS NOT CURRENTLY USED
	//	//////////////////////////////////////////////////////////////////////////////////////////////////
	//	//////////////////////////////////////////////////////////////////////////////////////////////////
	printf("\n===== Homing Axis =====\n");

	try {
		IPort& SC4_port = SC4_mgr->Ports(0);									// Create a shortcut for the port
		const int machine_num_axes = config.machine_num_axes;						
		std::vector<double> node_is_follower = config.node_is_follower;
		std::vector<double> node_axis = config.node_parent_axis;
		double divisor = 10;


		printf("Homing Axis %d\n", axis_id);
		int num_axis_nodes = std::count(node_axis.begin(), node_axis.end(), axis_id);
		if (num_axis_nodes == 1) {	//MIGHT BE POSSIBLE TO NOT USE THIS AT ALL
			int iNode = std::distance(node_axis.begin(), std::find(node_axis.begin(), node_axis.end(), axis_id));// index of node
			INode& the_node = SC4_port.Nodes(iNode);
			
			// THIS USES BUILT IN CLEARVIEW HOMING - CONSIDER CHANGING TO CUSTOM HOMING OPERATION
			if (the_node.Motion.Homing.HomingValid())
			{
				printf("Node [%d]: Homing now...\n", iNode);
				the_node.Motion.Homing.Initiate();
			}
			else {
				printf("Node[%d] has not had homing setup through ClearView.  The node will not be homed.\n", iNode);
				return(1);
			}
			double timeout = SC4_mgr->TimeStampMsec() + TIME_TILL_TIMEOUT;	//define a timeout in case the node is unable to enable
			while (!SC4_port.Nodes(iNode).Motion.Homing.WasHomed()) {
				if (SC4_mgr->TimeStampMsec() > timeout) {
					printf("Node[%d] did not complete homing:  \n\t -Ensure Homing settings have been defined through ClearView. \n\t -Check for alerts/Shutdowns \n\t -Ensure timeout is longer than the longest possible homing move.\n", iNode);
					msg_user_f("Press any key to continue."); //pause so the user can see the error message; waits for user to press a key
					return -2;
				}
			}
			printf("Completed homing\n");
			current_position = measure_position_f();
			return 1;
		}
		else {
			//something for n>1 node axes
			size_t lastnode = -1;
			size_t leader = -1;
			bool leader_home_found = 0;
			std::vector<int> axis_nodes;
			std::vector<bool> was_homed;
			double posn = 0;
			for (size_t iNode = 0; iNode < SC4_port.NodeCount(); iNode++) {
				if (config.node_parent_axis[iNode] == axis_id) {

				double node_machine_velocity_limit = config.homing_speed / config.node_lead_per_cnt[iNode] * config.node_sign[iNode];
				SC4_port.Nodes(iNode).Motion.VelLimit = abs(node_machine_velocity_limit);

				// Set up trigger
				SC4_port.Nodes(iNode).Motion.Adv.TriggerGroup(1);	// add all to same trigger group
				SC4_port.Nodes(iNode).Motion.Adv.MoveVelStart(-node_machine_velocity_limit, true);
				SC4_port.Nodes(iNode).Motion.Homing.SignalInvalid();
				axis_nodes.push_back(iNode);
				was_homed.push_back(false);
				lastnode = iNode;
				}
			}
			SC4_port.Nodes(lastnode).Motion.Adv.TriggerMovesInMyGroup();	// Trigger group

			while (std::any_of(was_homed.begin(), was_homed.end(), [](bool i) { return !i; })) {
				for (int i = 0; i < axis_nodes.size(); i++) {
					size_t iNode = axis_nodes[i];
					if (SC4_port.Nodes(iNode).Motion.Homing.WasHomed() || was_homed[i]) {
						continue;
					}
					else if (!leader_home_found && SC4_port.Nodes(iNode).Status.RT.Value().cpm.InA) {
						SC4_port.Nodes(iNode).Motion.NodeStop(STOP_TYPE_ABRUPT);
						leader = iNode;
						leader_home_found = true;
						SC4_port.Nodes(iNode).Motion.Homing.SignalComplete();
						was_homed[i] = true;
						posn = SC4_port.Nodes(iNode).Motion.PosnMeasured;
						SC4_port.Nodes(iNode).Motion.AddToPosition(-posn);
						continue;
					}
					else if (leader_home_found && !SC4_port.Nodes(iNode).Status.RT.Value().cpm.InA) {
						double node_machine_velocity_limit = (config.homing_speed/divisor) / config.node_lead_per_cnt[iNode];
						SC4_port.Nodes(iNode).Motion.VelLimit = abs(node_machine_velocity_limit);
						SC4_port.Nodes(iNode).Motion.Adv.MoveVelStart(-node_machine_velocity_limit, false);
						continue;
					}
					else if (leader_home_found && SC4_port.Nodes(iNode).Status.RT.Value().cpm.InA) {
						SC4_port.Nodes(iNode).Motion.NodeStop(STOP_TYPE_ABRUPT);
						SC4_port.Nodes(iNode).Motion.Homing.SignalComplete();
						was_homed[i] = true;
						posn = SC4_port.Nodes(iNode).Motion.PosnMeasured;
						SC4_port.Nodes(iNode).Motion.AddToPosition(-posn);

						continue;
					}
					else {
						continue;
					}
				}
			}
			
			std::vector<double> offset_distance(config.machine_num_axes, 0.0);
			offset_distance[axis_id] = 25.4;
			double prev_limit = config.machine_velocity_limit;
			config.machine_velocity_limit = 25.4;
			move_linear_f(offset_distance, false);
			config.machine_velocity_limit = prev_limit;
			for (size_t iNode = 0; iNode < SC4_port.NodeCount(); iNode++) {
				if (config.node_parent_axis[iNode] == axis_id) {
					posn = SC4_port.Nodes(iNode).Motion.PosnMeasured;
					SC4_port.Nodes(iNode).Motion.AddToPosition(-posn);
				}
			}
		}

		printf("Completed homing\n");
		current_position = measure_position_f();
		return 1;
	}
	catch (mnErr& theErr)
	{
		printf("Failed to open port(s)\n");
		//This statement will print the address of the error, the error code (defined by the mnErr class), 
		//as well as the corresponding error message.
		printf("Caught error: addr=%d, err=0x%08x\nmsg=%s\n", theErr.TheAddr, theErr.ErrorCode, theErr.ErrorMsg);

		msg_user_f("Press any key to continue."); //pause so the user can see the error message; waits for user to press a key
		return -1;
	}
}

int machine::open_ports_f() {

	/// Summary: Searches for viable SC Hub Ports and opens the first one
	/// Params: 
	/// Returns: 
	/// Notes: 

	size_t port_count = 0;
	std::vector<std::string> comHubPorts;

	//Create the SysManager object. This object will coordinate actions among various ports
	// and within nodes. In this example we use this object to setup and open our port.
	SC4_mgr = SysManager::Instance();							//Create System Manager SC4_mgr

   //This will try to open the port. If there is an error/exception during the port opening,
   //the code will jump to the catch loop where detailed information regarding the error will be displayed;
   //otherwise the catch loop is skipped over
	try
	{

		SysManager::FindComHubPorts(comHubPorts);
		printf("Found %d SC Hubs\n", comHubPorts.size());

		for (port_count = 0; port_count < comHubPorts.size() && port_count < NET_CONTROLLER_MAX; port_count++) {

			SC4_mgr->ComHubPort(port_count, comHubPorts[port_count].c_str()); 	//define the first SC Hub port (port 0) to be associated 
											// with COM portnum (as seen in device manager)
		}

		if (port_count <= 0) {
		}
		else {
			SC4_mgr->PortsOpen(port_count);				//Open the port
			IPort& SC4_port = SC4_mgr->Ports(0);
			printf(" Port[%d]: state=%d, nodes=%d\n",
				SC4_port.NetNumber(), SC4_port.OpenState(), SC4_port.NodeCount());

		}
		return port_count;

	}
	catch (mnErr& theErr)
	{
		printf("Failed to open port(s)\n");
		//This statement will print the address of the error, the error code (defined by the mnErr class), 
		//as well as the corresponding error message.
		printf("Caught error: addr=%d, err=0x%08x\nmsg=%s\n", theErr.TheAddr, theErr.ErrorCode, theErr.ErrorMsg);

		msg_user_f("Press any key to continue."); //pause so the user can see the error message; waits for user to press a key
		return -1;
	}
}

void machine::close_ports_f() {

	/// Summary: closes SC Hub port
	/// Params: 
	/// Returns: 
	/// Notes: 
	
	printf("Closing Ports\n");
	try {
		SC4_mgr->PortsClose();
	}
	catch (mnErr& theErr)
	{
		printf("Failed to close port(s)\n");
		//This statement will print the address of the error, the error code (defined by the mnErr class), 
		//as well as the corresponding error message.
		printf("Caught error: addr=%d, err=0x%08x\nmsg=%s\n", theErr.TheAddr, theErr.ErrorCode, theErr.ErrorMsg);

		msg_user_f("Press any key to continue."); //pause so the user can see the error message; waits for user to press a key
	}
}

int machine::start_up_f() {

	/// Summary: Combines general machine start-up functions into one function.
	///			Loads mechanical configuration document
	///			Opens SC Hub Ports
	///			Enables the nodes
	///			Will eventually home the axes
	///			sets config
	/// Params: 
	/// Returns: 
	/// Notes: 

	load_config_f(':');


	size_t port_count = open_ports_f();

	if (port_count <= 0) {
		// If there is no hub, quit the program.
		printf("Unable to locate SC hub port. Quitting Program\n");
		msg_user_f("Press any key to continue.");//waits for user to press a key
		return 0;  //This terminates the main program
	}
	else {
		try {
			// This section initializes the motors and tries to home them.
			// Once the code gets past this point, it can be assumed that the Port has been opened without issue
			// Now we can get a reference to our port object which we will use to access the node objects
			int res = enable_nodes_f();
			if (res != 1) { return res; }

			//res = home_position_f();
			//if (res != 1) { return res; }

			res = set_config_f(/*mode - counts vs revs*/);
			if (res != 1) { return res; }

			current_position = measure_position_f();

			return 1;
		}
		catch (mnErr& theErr)
		{
			printf("Failed to enable and home Nodes.\n");
			printf("Caught error: addr=%d, err=0x%08x\nmsg=%s\n", theErr.TheAddr, theErr.ErrorCode, theErr.ErrorMsg);
			msg_user_f("Press any key to continue."); //pause so the user can see the error message; waits for user to press a key
			return -2;  //This terminates the main program
		}
	}

}

void machine::shut_down_f() {

	/// Summary: Disables nodes and closes SC Hub port
	/// Params: 
	/// Returns: 
	/// Notes: 

	try {
		disable_nodes_f();
	}
	catch (mnErr& theErr)
	{
		printf("Failed to disable nodes.\n");
		printf("Caught error: addr=%d, err=0x%08x\nmsg=%s\n", theErr.TheAddr, theErr.ErrorCode, theErr.ErrorMsg);
		msg_user_f("Press any key to continue."); //pause so the user can see the error message; waits for user to press a key
		std::exit(1);  //This terminates the main program
	}
	try {
		close_ports_f();
	}
	catch (mnErr& theErr)
	{
		printf("Failed to close ports.\n");
		printf("Caught error: addr=%d, err=0x%08x\nmsg=%s\n", theErr.TheAddr, theErr.ErrorCode, theErr.ErrorMsg);
		msg_user_f("Press any key to continue."); //pause so the user can see the error message; waits for user to press a key
		std::exit(1);  //This terminates the main program
	}
}
/*----------------------------- Test Harness -------------------------------*/

/*------------------------------- Footnotes --------------------------------*/
/*------------------------------ End of file -------------------------------*/