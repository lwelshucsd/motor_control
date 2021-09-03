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
#include "motor_functions.hpp"
#include "general_functions.hpp"
#include <Windows.h>
#include <iostream>
#include <fstream>
#include <filesystem>

#define LONG_DELAY				500
#define SHORT_DELAY				100
#define TIME_TILL_TIMEOUT		100000	//The timeout used for homing(ms)
#define ACC_LIM_CNTS_PER_SEC2	320000000
#define MAX_VEL_LIM				2000

using namespace sFnd;
namespace fs = std::filesystem;
/*--------------------------- External Variables ---------------------------*/
/*----------------------------- Module Defines -----------------------------*/
/*------------------------------ Module Types ------------------------------*/
/*---------------------------- Module Variables ----------------------------*/
/*--------------------- Module Function Prototypes -------------------------*/
/*------------------------------ Module Code -------------------------------*/
bool move_is_done_f(class IPort& my_port) {

	/// Summary: Checks to see if all nodes have triggered the MoveIsDone flag, meaning that whatever movement is planned has been completed by all nodes.
	/// Params: my_port
	/// Returns: bool true if all nodes have completed their movement. false if any node is still moving.
	/// Notes:

	// Check all nodes on port.
	for (size_t iNode = 0; iNode < my_port.NodeCount(); iNode++) {
		if (!my_port.Nodes(iNode).Motion.MoveIsDone()) { return false; }	// If any node is still moving, return false.
	}
	return true;	// If all nodes are done, return true.
}


// Machine Specific functions
void machine::load_config_f(char delimiter) {

	/// Summary: Loads the mechanical configuration data of the machine into the machine.config data structure
	/// Params: None
	/// Returns: Void
	/// Notes: 
	///		(08/30/21) - Building way to load mechanical config from .cfg file. -TH

	std::vector<std::string> var_names = {
		"num_axes",
		"velocity_limit_default",
		"velocity_limit_max",
		"cnts_per_rev",
		"is_follower_node",
		"is_rotary_axis",
		"parent_axis",
		"lead"
	};
	std::string line;
	int delimiter_pos;
	std::string key, value;
	std::ifstream myReadFile("mech_config.cfg");
	std::cout << "\n";
	while (std::getline(myReadFile, line)) {
		line.erase(std::remove_if(line.begin(), line.end(), isspace), line.end());
		delimiter_pos = line.find(delimiter);
		key = line.substr(0, delimiter_pos);
		value = line.substr(delimiter_pos + 1);
		if (std::find(var_names.begin(), var_names.end(), key) != var_names.end()) {
			std::cout << key << "\n";
		}
		else if (line.substr(0, 2) == "//") {
			continue;
		}
		else {
			std::cout << "Ignoring unrecognized config element: " << key << "\n";
		}

	}

	myReadFile.close();

	config.is_follower_node = { 0, 0 };	// denotes whether the node is a leader or follower node
	config.node_sign = { 1, -1 };	// denotes the direction that we consider positive relative to the nodbe's positive
	config.lead = { 108, 108 };	// spatial change (mm) per roation of the node shaft
	config.is_rotary_axis = { 0, 0 };
	config.parent_axis = { 0, 1 };
	config.velocity_limit = 250; //mm/s
	config.num_axes = config.is_follower_node.size() - vector_sum(config.is_follower_node);
	config.lead_per_cnt = config.lead | (1 / double(6400));
	config.max_velocity_limit = double(3000)/60*36;
	//config.accel_limit
}

int machine::set_config_f() {

	/// Summary: Loads required parameters/operation modes into each node
	/// Params: 
	/// Returns: int representing success (1) or failure (-1)
	/// Notes: (09/01/2021) - May be worthwile to have these parameters be config variables -TH

	try {
		IPort& my_port = my_mgr->Ports(0);
		// Iterate through each node in the machine
		for (size_t iNode = 0; iNode < my_port.NodeCount(); iNode++) {
			my_port.Nodes(iNode).AccUnit(INode::COUNTS_PER_SEC2);			// set acceleration limit tracking unit
			my_port.Nodes(iNode).VelUnit(INode::COUNTS_PER_SEC);				// set velocity limit unit
			my_port.Nodes(iNode).Motion.AccLimit = config.accel_limit;		// set acceleration limit
			my_port.Nodes(iNode).Motion.VelLimit = config.velocity_limit;	// set default velocity limit
			my_port.Nodes(iNode).Motion.PosnMeasured.AutoRefresh(true);
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

	IPort& my_port = my_mgr->Ports(0);	// Create Port object
	int num_axes = config.num_axes;

	std::vector<double> position(config.is_follower_node.size());	// Initialize position vector

	int dim_ind = 0;
	// Measure each node's position and fill them into a vector
	for (size_t iNode = 0; iNode < my_port.NodeCount(); iNode++) {
		position[iNode] = (my_port.Nodes(iNode).Motion.PosnMeasured)*config.node_sign[iNode];
	}

	position = position * config.lead_per_cnt;	//convert count-space to real-space
	for (size_t iNode = 0; iNode < position.size(); iNode++) {
		if (config.is_follower_node[iNode]) {
			position.erase(position.begin() + iNode);
		}
	}
	return position;
}

//change name to "linear"
std::vector<double> machine::move_linear_f(std::vector<double> input_vec, bool target_is_absolute) {

	/// Summary: Abstracts the triangular node.motion.movePosnStart() to a multi-axis system, using trigger groups to enforce 
	///			a simultaneous movement start. Waits until move is completed on all nodes to return the measured position after
	///			said move. Velocity on each node is calculated and set such that the mvoement is linear and all nodes complete together.
	/// Params:	input_vec: a double vector of the desired position/job distance for the machine
	///			target_is_absolute: a bool representing if the target of the move is an absolute positional change or a relative position jog.
	/// Returns: Function returns a vector of the measured position of the machine after the movement is completed (or after it times out).
	/// Notes:	(08/31/21)	- Need to move the user input section outside of the function to allow for use in preset toolpaths. -TH
	///						- Need to adjust velocity calculations to account for lead & follower nodes

	IPort& my_port = my_mgr->Ports(0);	// Create a shortcut for the port
	bool r_mode = settings.r_mode;

	// Initialize position vectors to be used in velocity calculations
	std::vector<double> end_pos;
	std::vector<double> vel_vec;
	std::vector<double> current_pos = machine::measure_position_f();

	// Create shortcuts to important config members
	int num_axes = config.num_axes;
	double velocity_limit = config.velocity_limit;
	std::vector<double> is_follower_node = config.is_follower_node;
	std::vector<double> lead_per_cnt = config.lead_per_cnt;
	std::vector<double> node_sign = config.node_sign;

	// Compute velocity vector for movement 
	vel_vec = input_vec - (current_pos | target_is_absolute);	// Compute movement direction vector
	vel_vec = normalize(vel_vec);								// Normalize movement direction vector
	vel_vec = vel_vec | velocity_limit;							// Multiply by overall velocity limit

	int node_axis;
	double node_input_cnts;
	double node_velocity_limit;


	if (r_mode) {
		printf("Simulating movement.\n");
		Sleep(SHORT_DELAY);							// Update to incorporate speed and calculate simulated time
		print_vector_f(current_pos, " ");
		std::cout << "\n";
		print_vector_f(input_vec, " ");
		end_pos = (current_pos | (!target_is_absolute)) + input_vec;
		return end_pos;
	}
	else {
		// Set up trigger group  & velocity for all nodes
		for (size_t iNode = 0; iNode < my_port.NodeCount(); iNode++) {
			node_axis = config.parent_axis[iNode];

			// Convert velocity to counts/s and apply limit
			node_velocity_limit = vel_vec[node_axis] / lead_per_cnt[iNode];
			my_port.Nodes(iNode).Motion.VelLimit = abs(node_velocity_limit);

			//Convert distance to counts and set up trigger
			node_input_cnts = input_vec[node_axis] / lead_per_cnt[iNode] * node_sign[iNode];
			my_port.Nodes(iNode).Motion.Adv.TriggerGroup(1);	// add all to same trigger group
			my_port.Nodes(iNode).Motion.Adv.MovePosnStart(node_input_cnts, target_is_absolute, true);
		}
		my_port.Nodes(0).Motion.Adv.TriggerMovesInMyGroup();	// Trigger group


		double timeout = my_mgr->TimeStampMsec() + TIME_TILL_TIMEOUT; //define a timeout in case the node is unable to enable or takes too long
		while (!move_is_done_f(my_port)) {
			//printf("%00008.2f   ", my_mgr->TimeStampMsec() - timeout + TIME_TILL_TIMEOUT);
			//print_vector_f(measurePosn(), "");
			if (my_mgr->TimeStampMsec() > timeout) {
				printf("Error: timed out waiting for move to complete\n");
				msg_user_f("press any key to continue."); //pause so the user can see the error message; waits for user to press a key
				my_port.NodeStop();	// Stops the nodes at their current position
				return measure_position_f();
			}
		}

		Sleep(SHORT_DELAY);
		end_pos = measure_position_f();	// Measure ending position of machine to return
	}
	return end_pos;
}

int machine::enable_nodes_f() {

	/// Summary: Enables all nodes on a port and prints detected node data
	/// Params: None
	/// Returns: Int of -2 to imply fialure, 1 to imply success
	/// Notes: 

	IPort& my_port = my_mgr->Ports(0);			// Create a shortcut for the port
	printf("\n===== Detected Node Data =====\n");
	printf("        Type || FW Version || Serial # || Model\n");
	try {
		for (size_t iNode = 0; iNode < my_port.NodeCount(); iNode++) {
			// Create a shortcut reference for a node
			INode& the_node = my_port.Nodes(iNode);

			the_node.EnableReq(false);				//Ensure Node is disabled before loading config file

			my_mgr->Delay(SHORT_DELAY);

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
			double timeout = my_mgr->TimeStampMsec() + TIME_TILL_TIMEOUT;	//define a timeout in case the node is unable to enable
			while (!the_node.Motion.IsReady()) {								//This will loop checking on the Real time values of the node's Ready status
				if (my_mgr->TimeStampMsec() > timeout) {
					printf("Error: Timed out waiting for Node %d to enable\n", iNode);
					msg_user_f("Press any key to continue."); //pause so the user can see the error message; waits for user to press a key
					return -2;
				}
			}
		}
		// Prints the User-defined node IDs for the user to check
		printf("\n===== User Node IDs =====\n");
		for (size_t iNode = 0; iNode < my_port.NodeCount(); iNode++) {
			printf("Node[%d]: %s\n", int(iNode), my_port.Nodes(iNode).Info.UserID.Value());
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

	/// Summary: 
	/// Params: 
	/// Returns: 
	/// Notes: 

	try {
		IPort& my_port = my_mgr->Ports(0);
		printf("\nDisabling nodes\n");
		for (size_t iNode = 0; iNode < my_port.NodeCount(); iNode++) {
			// Create a shortcut reference for a node
			my_port.Nodes(iNode).EnableReq(false);
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

int machine::home_position_f() {

	/// Summary: 
	/// Params: 
	/// Returns: 
	/// Notes: 

	//	//////////////////////////////////////////////////////////////////////////////////////////////////
	//	//////////////////////////////////////////////////////////////////////////////////////////////////
	//	//	NEED TO CHANGE HOMING OPERATION TO HOME LEADER-FOLLOWER SETS TOGETHER
	//	//	OR TO HOME ALL NODES AT THE SAME TIME, BASICALLY AS A MOVE TO ORIGIN
	//	//////////////////////////////////////////////////////////////////////////////////////////////////
	//	//////////////////////////////////////////////////////////////////////////////////////////////////
	printf("\n===== Homing All Axes =====\n");
	try {
		IPort& my_port = my_mgr->Ports(0);		// Create a shortcut for the port
		int num_axes = config.num_axes;
		std::vector<double> is_follower_node = config.is_follower_node;
		for (size_t iAxis = 0; iAxis < num_axes; iAxis++) {
			printf("Homing Axis %d\n", iAxis);
			for (size_t iNode = 0; iNode < my_port.NodeCount(); iNode++) {
				if (config.parent_axis[iNode] == iAxis) {
					INode& the_node = my_port.Nodes(iNode);
					if (the_node.Motion.Homing.HomingValid())
					{
						if (the_node.Motion.Homing.WasHomed())
						{
							printf("Node[%d] has already been homed, current position is: \t%8.0f \n", iNode, the_node.Motion.PosnMeasured.Value());
							printf("Rehoming Node[%d]... \n", iNode);
						}
						else
						{
							printf("Node [%d] has not been homed.  Homing Node now...\n", iNode);
						}
						//Now we will home the Node

						the_node.Motion.Homing.Initiate();

						double timeout = my_mgr->TimeStampMsec() + TIME_TILL_TIMEOUT;	//define a timeout in case the node is unable to enable
																				// Basic mode - Poll until disabled
						while (!the_node.Motion.Homing.WasHomed()) {
							if (my_mgr->TimeStampMsec() > timeout) {
								printf("Node[%d] did not complete homing:  \n\t -Ensure Homing settings have been defined through ClearView. \n\t -Check for alerts/Shutdowns \n\t -Ensure timeout is longer than the longest possible homing move.\n", iNode);
								msg_user_f("Press any key to continue."); //pause so the user can see the error message; waits for user to press a key
								return -2;
							}
						}
						printf("Node[%d] completed homing\n", iNode);
					}
					else {
						printf("Node[%d] has not had homing setup through ClearView.  The node will not be homed.\n", iNode);
					}
				}
			}
		}
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

	/// Summary: 
	/// Params: 
	/// Returns: 
	/// Notes: 

	size_t port_count = 0;
	std::vector<std::string> comHubPorts;

	//Create the SysManager object. This object will coordinate actions among various ports
	// and within nodes. In this example we use this object to setup and open our port.
	my_mgr = SysManager::Instance();							//Create System Manager my_mgr

   //This will try to open the port. If there is an error/exception during the port opening,
   //the code will jump to the catch loop where detailed information regarding the error will be displayed;
   //otherwise the catch loop is skipped over
	try
	{

		SysManager::FindComHubPorts(comHubPorts);
		printf("Found %d SC Hubs\n", comHubPorts.size());

		for (port_count = 0; port_count < comHubPorts.size() && port_count < NET_CONTROLLER_MAX; port_count++) {

			my_mgr->ComHubPort(port_count, comHubPorts[port_count].c_str()); 	//define the first SC Hub port (port 0) to be associated 
											// with COM portnum (as seen in device manager)
		}
		bool r_mode = false; // assume that there will be a hub with motors

		if (port_count <= 0) {
		}
		else {
			my_mgr->PortsOpen(port_count);				//Open the port
			IPort& my_port = my_mgr->Ports(0);

			printf(" Port[%d]: state=%d, nodes=%d\n",
				my_port.NetNumber(), my_port.OpenState(), my_port.NodeCount());

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
	/// Summary: 
	/// Params: 
	/// Returns: 
	/// Notes: 
	printf("Closing Ports\n");
	try {
		my_mgr->PortsClose();
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

	/// Summary: 
	/// Params: 
	/// Returns: 
	/// Notes: 

	load_config_f('=');


	size_t port_count = open_ports_f();

	if (port_count <= 0) {
		//Initialize some remote mode thing

		// If there is no hub, either quit the program or enable remote work mode.
		printf("Unable to locate SC hub port, enable remote mode?\n");
		char r_modeChar;
		std::cout << "y/n?:";
		std::cin >> r_modeChar; //y/n input
		std::cin.clear();
		std::cin.ignore(100, '\n');
		if (r_modeChar == 'y' || r_modeChar == 'Y') {
			settings.r_mode = true;
			printf("Remote Work Mode enabled.\n");
			printf("Commands will be simulated, but no attempt to pass the command over USB will be made.\n");
			port_count = 1;
			return 1;
		}
		else if (r_modeChar == 'n' || r_modeChar == 'N') {
			msg_user_f("Quitting program.\nPress any key to continue.");//waits for user to press a key
			return 0;  //This terminates the main program
		}
		else {
			msg_user_f("Inavlid response. Quitting program.\nPress any key to continue."); //pause so the user can see the error message; waits for user to press a key
			return 0;  //This terminates the main program
		}
	}
	else {
		try {
			//This section initializes the motors and tries to home them.
			//Once the code gets past this point, it can be assumed that the Port has been opened without issue
			//Now we can get a reference to our port object which we will use to access the node objects
			int res = enable_nodes_f();
			if (res != 1) { return res; }

			res = home_position_f();
			if (res != 1) { return res; }

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

	/// Summary: 
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
