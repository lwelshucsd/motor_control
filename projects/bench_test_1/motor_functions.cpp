/****************************************************************************
 Module
	module_name.c
 Description
	description of module
 Notes
	additional notes

 History
 When             Who    What/Why
 --------------   ---    --------
 DD MMMM YYYY     XXX    changes
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
#define VEL_LIM_CNTS_PER_SEC	64000
#define MAX_VEL_LIM				2000

using namespace sFnd;
namespace fs = std::filesystem;
/*--------------------------- External Variables ---------------------------*/
/*----------------------------- Module Defines -----------------------------*/
/*------------------------------ Module Types ------------------------------*/
/*---------------------------- Module Variables ----------------------------*/
/*--------------------- Module Function Prototypes -------------------------*/
/*------------------------------ Module Code -------------------------------*/
bool moveIsDone(class IPort& myPort) {

	/// Summary: Checks to see if all nodes have triggered the MoveIsDone flag, meaning that whatever movement is planned has been completed by all nodes.
	/// Params: myPort
	/// Returns: bool true if all nodes have completed their movement. false if any node is still moving.
	/// Notes:

	// Check all nodes on port.
	for (size_t iNode = 0; iNode < myPort.NodeCount(); iNode++) {
		if (!myPort.Nodes(iNode).Motion.MoveIsDone()) { return false; }	// If any node is still moving, return false.
	}
	return true;	// If all nodes are done, return true.
}


// Machine Specific functions
void machine::loadConfig(char delimiter) {

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

	config.is_follower_node = { 0, 1 };	// denotes whether the node is a leader or follower node
	config.node_sign = { 1, -1 };	// denotes the direction that we consider positive relative to the node's positive
	config.lead = { 108, 36 };	// spatial change (mm) per roation of the node shaft
	config.is_rotary_axis = { 0, 0 };
	config.parent_axis = { 0, 0 };
	config.velocity_limit = 250; //mm/s
	config.num_axes = config.is_follower_node.size() - vsum(config.is_follower_node);
	config.lead_per_cnt = config.lead | (1 / double(6400));
	config.max_velocity_limit = 3000/60*36;
}

int machine::setConfig() {

	/// Summary: 
	/// Params: 
	/// Returns: 
	/// Notes: 

	// have global struct here that is changed and updated to match file
	try {
		IPort& myPort = myMgr->Ports(0);
		for (size_t iNode = 0; iNode < myPort.NodeCount(); iNode++) {
			myPort.Nodes(iNode).AccUnit(INode::COUNTS_PER_SEC2);
			myPort.Nodes(iNode).VelUnit(INode::COUNTS_PER_SEC);
			myPort.Nodes(iNode).Motion.AccLimit = ACC_LIM_CNTS_PER_SEC2;//change to config var
			myPort.Nodes(iNode).Motion.VelLimit = VEL_LIM_CNTS_PER_SEC;//change to config var
			myPort.Nodes(iNode).Motion.PosnMeasured.AutoRefresh(true);
		}
		return 1;
	}
	catch (mnErr& theErr)
	{
		printf("Failed to change node settings(s)\n");
		//This statement will print the address of the error, the error code (defined by the mnErr class), 
		//as well as the corresponding error message.
		printf("Caught error: addr=%d, err=0x%08x\nmsg=%s\n", theErr.TheAddr, theErr.ErrorCode, theErr.ErrorMsg);

		msgUser("Press any key to continue."); //pause so the user can see the error message; waits for user to press a key
		return -1;
	}
}

std::vector<double> machine::measurePosn() {

	/// Summary: Measures position of all axes by measuring all non-follower nodes and converting to real space.
	/// Params: None
	/// Returns: Double vector of real-space position measured on leader nodes.
	/// Notes: 

	IPort& myPort = myMgr->Ports(0);	// Create Port object
	int num_axes = config.num_axes;

	std::vector<double> position(config.is_follower_node.size());	// Initialize position vector

	int dim_ind = 0;
	// Measure each node's position and fill them into a vector
	for (size_t iNode = 0; iNode < myPort.NodeCount(); iNode++) {
		position[iNode] = (myPort.Nodes(iNode).Motion.PosnMeasured);
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
std::vector<double> machine::linearMove(std::vector<double> input_vec, bool target_is_absolute) {

	/// Summary: Abstracts the triangular node.motion.movePosnStart() to a multi-axis system, using trigger groups to enforce 
	///			a simultaneous movement start. Waits until move is completed on all nodes to return the measured position after
	///			said move. Velocity on each node is calculated and set such that the mvoement is linear and all nodes complete together.
	/// Params:	input_vec: a double vector of the desired position/job distance for the machine
	///			target_is_absolute: a bool representing if the target of the move is an absolute positional change or a relative position jog.
	/// Returns: Function returns a vector of the measured position of the machine after the movement is completed (or after it times out).
	/// Notes:	(08/31/21)	- Need to move the user input section outside of the function to allow for use in preset toolpaths. -TH
	///						- Need to adjust velocity calculations to account for lead & follower nodes

	IPort& myPort = myMgr->Ports(0);	// Create a shortcut for the port
	bool r_mode = settings.r_mode;

	// Initialize position vectors to be used in velocity calculations
	std::vector<double> end_pos;
	std::vector<double> vel_vec;
	std::vector<double> current_pos = machine::measurePosn();

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
		vectorPrint(current_pos, " ");
		std::cout << "\n";
		vectorPrint(input_vec, " ");
		end_pos = (current_pos | (!target_is_absolute)) + input_vec;
		return end_pos;
	}
	else {
		// Set up trigger group  & velocity for all nodes
		for (size_t iNode = 0; iNode < myPort.NodeCount(); iNode++) {
			node_axis = config.parent_axis[iNode];

			// Convert velocity to counts/s and apply limit
			node_velocity_limit = vel_vec[node_axis] / lead_per_cnt[iNode];
			myPort.Nodes(iNode).Motion.VelLimit = abs(node_velocity_limit);

			//Convert distance to counts and set up trigger
			node_input_cnts = input_vec[node_axis] / lead_per_cnt[iNode] * node_sign[iNode];
			myPort.Nodes(iNode).Motion.Adv.TriggerGroup(1);	// add all to same trigger group
			myPort.Nodes(iNode).Motion.Adv.MovePosnStart(node_input_cnts, target_is_absolute, true);
		}
		myPort.Nodes(0).Motion.Adv.TriggerMovesInMyGroup();	// Trigger group


		double timeout = myMgr->TimeStampMsec() + TIME_TILL_TIMEOUT; //define a timeout in case the node is unable to enable or takes too long
		while (!moveIsDone(myPort)) {
			//printf("%00008.2f   ", myMgr->TimeStampMsec() - timeout + TIME_TILL_TIMEOUT);
			//vectorPrint(measurePosn(), "");
			if (myMgr->TimeStampMsec() > timeout) {
				printf("Error: timed out waiting for move to complete\n");
				msgUser("press any key to continue."); //pause so the user can see the error message; waits for user to press a key
				myPort.NodeStop();	// Stops the nodes at their current position
				return measurePosn();
			}
		}

		Sleep(SHORT_DELAY);
		end_pos = measurePosn();	// Measure ending position of machine to return
	}
	return end_pos;
}

int machine::enableNodes() {

	/// Summary: Enables all nodes on a port and prints detected node data
	/// Params: None
	/// Returns: Int of -2 to imply fialure, 1 to imply success
	/// Notes: 

	IPort& myPort = myMgr->Ports(0);			// Create a shortcut for the port
	printf("\n===== Detected Node Data =====\n");
	printf("        Type || FW Version || Serial # || Model\n");
	try {
		for (size_t iNode = 0; iNode < myPort.NodeCount(); iNode++) {
			// Create a shortcut reference for a node
			INode& theNode = myPort.Nodes(iNode);

			theNode.EnableReq(false);				//Ensure Node is disabled before loading config file

			myMgr->Delay(SHORT_DELAY);

			// If config is not stored on the motor we need to enable this
			//theNode.Setup.ConfigLoad("Config File path");

			// Print detected node data
			printf(
				"Node[%d]:  %d  || %s || %d || %s\n",
				int(iNode), theNode.Info.NodeType(),
				theNode.Info.FirmwareVersion.Value(),
				theNode.Info.SerialNumber.Value(),
				theNode.Info.Model.Value()
			);

			//The following statements will attempt to enable the node.  First,
			// any shutdowns or NodeStops are cleared, finally the node is enabled
			theNode.Status.AlertsClear();					//Clear Alerts on node 
			theNode.Motion.NodeStopClear();	//Clear Nodestops on Node  				
			theNode.EnableReq(true);					//Enable node 
			//At this point the node is enabled
			//printf("Node[%d] enabled.\n", int(iNode));
			double timeout = myMgr->TimeStampMsec() + TIME_TILL_TIMEOUT;	//define a timeout in case the node is unable to enable
			while (!theNode.Motion.IsReady()) {								//This will loop checking on the Real time values of the node's Ready status
				if (myMgr->TimeStampMsec() > timeout) {
					printf("Error: Timed out waiting for Node %d to enable\n", iNode);
					msgUser("Press any key to continue."); //pause so the user can see the error message; waits for user to press a key
					return -2;
				}
			}
		}
		// Prints the User-defined node IDs for the user to check
		printf("\n===== User Node IDs =====\n");
		for (size_t iNode = 0; iNode < myPort.NodeCount(); iNode++) {
			printf("Node[%d]: %s\n", int(iNode), myPort.Nodes(iNode).Info.UserID.Value());
		}
		return 1;
	}
	catch (mnErr& theErr)
	{
		printf("Failed to enable nodes(s)\n");
		//This statement will print the address of the error, the error code (defined by the mnErr class), 
		//as well as the corresponding error message.
		printf("Caught error: addr=%d, err=0x%08x\nmsg=%s\n", theErr.TheAddr, theErr.ErrorCode, theErr.ErrorMsg);

		msgUser("Press any key to continue."); //pause so the user can see the error message; waits for user to press a key
		return -1;
	}
}

int machine::disableNodes() {

	/// Summary: 
	/// Params: 
	/// Returns: 
	/// Notes: 

	try {
		IPort& myPort = myMgr->Ports(0);
		printf("\nDisabling nodes\n");
		for (size_t iNode = 0; iNode < myPort.NodeCount(); iNode++) {
			// Create a shortcut reference for a node
			myPort.Nodes(iNode).EnableReq(false);
		}
		return 1;
	}
	catch (mnErr& theErr)
	{
		printf("Failed to disable nodes. Please manually disable all nodes in Clearview.\n");
		//This statement will print the address of the error, the error code (defined by the mnErr class), 
		//as well as the corresponding error message.
		printf("Caught error: addr=%d, err=0x%08x\nmsg=%s\n", theErr.TheAddr, theErr.ErrorCode, theErr.ErrorMsg);

		msgUser("Press any key to continue."); //pause so the user can see the error message; waits for user to press a key
		return -1;  //This terminates the main program
	}
};

int machine::homePosn() {

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
		IPort& myPort = myMgr->Ports(0);		// Create a shortcut for the port
		int num_axes = config.num_axes;
		std::vector<double> is_follower_node = config.is_follower_node;
		for (size_t iAxis = 0; iAxis < num_axes; iAxis++) {
			printf("Homing Axis %d\n", iAxis);
			for (size_t iNode = 0; iNode < myPort.NodeCount(); iNode++) {
				if (config.parent_axis[iNode] == iAxis) {
					INode& theNode = myPort.Nodes(iNode);
					if (theNode.Motion.Homing.HomingValid())
					{
						if (theNode.Motion.Homing.WasHomed())
						{
							printf("Node[%d] has already been homed, current position is: \t%8.0f \n", iNode, theNode.Motion.PosnMeasured.Value());
							printf("Rehoming Node[%d]... \n", iNode);
						}
						else
						{
							printf("Node [%d] has not been homed.  Homing Node now...\n", iNode);
						}
						//Now we will home the Node

						theNode.Motion.Homing.Initiate();

						double timeout = myMgr->TimeStampMsec() + TIME_TILL_TIMEOUT;	//define a timeout in case the node is unable to enable
																				// Basic mode - Poll until disabled
						while (!theNode.Motion.Homing.WasHomed()) {
							if (myMgr->TimeStampMsec() > timeout) {
								printf("Node[%d] did not complete homing:  \n\t -Ensure Homing settings have been defined through ClearView. \n\t -Check for alerts/Shutdowns \n\t -Ensure timeout is longer than the longest possible homing move.\n", iNode);
								msgUser("Press any key to continue."); //pause so the user can see the error message; waits for user to press a key
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
		current_pos = measurePosn();
		return 1;
	}
	catch (mnErr& theErr)
	{
		printf("Failed to open port(s)\n");
		//This statement will print the address of the error, the error code (defined by the mnErr class), 
		//as well as the corresponding error message.
		printf("Caught error: addr=%d, err=0x%08x\nmsg=%s\n", theErr.TheAddr, theErr.ErrorCode, theErr.ErrorMsg);

		msgUser("Press any key to continue."); //pause so the user can see the error message; waits for user to press a key
		return -1;
	}
}

int machine::openPorts() {

	/// Summary: 
	/// Params: 
	/// Returns: 
	/// Notes: 

	size_t port_count = 0;
	std::vector<std::string> comHubPorts;

	//Create the SysManager object. This object will coordinate actions among various ports
	// and within nodes. In this example we use this object to setup and open our port.
	myMgr = SysManager::Instance();							//Create System Manager myMgr

   //This will try to open the port. If there is an error/exception during the port opening,
   //the code will jump to the catch loop where detailed information regarding the error will be displayed;
   //otherwise the catch loop is skipped over
	try
	{

		SysManager::FindComHubPorts(comHubPorts);
		printf("Found %d SC Hubs\n", comHubPorts.size());

		for (port_count = 0; port_count < comHubPorts.size() && port_count < NET_CONTROLLER_MAX; port_count++) {

			myMgr->ComHubPort(port_count, comHubPorts[port_count].c_str()); 	//define the first SC Hub port (port 0) to be associated 
											// with COM portnum (as seen in device manager)
		}
		bool r_mode = false; // assume that there will be a hub with motors

		if (port_count <= 0) {
		}
		else {
			myMgr->PortsOpen(port_count);				//Open the port
			IPort& myPort = myMgr->Ports(0);

			printf(" Port[%d]: state=%d, nodes=%d\n",
				myPort.NetNumber(), myPort.OpenState(), myPort.NodeCount());

		}
		return port_count;

	}
	catch (mnErr& theErr)
	{
		printf("Failed to open port(s)\n");
		//This statement will print the address of the error, the error code (defined by the mnErr class), 
		//as well as the corresponding error message.
		printf("Caught error: addr=%d, err=0x%08x\nmsg=%s\n", theErr.TheAddr, theErr.ErrorCode, theErr.ErrorMsg);

		msgUser("Press any key to continue."); //pause so the user can see the error message; waits for user to press a key
		return -1;
	}
}

void machine::closePorts() {
	/// Summary: 
	/// Params: 
	/// Returns: 
	/// Notes: 
	printf("Closing Ports\n");
	try {
		myMgr->PortsClose();
	} 
	catch (mnErr& theErr)
	{
		printf("Failed to close port(s)\n");
		//This statement will print the address of the error, the error code (defined by the mnErr class), 
		//as well as the corresponding error message.
		printf("Caught error: addr=%d, err=0x%08x\nmsg=%s\n", theErr.TheAddr, theErr.ErrorCode, theErr.ErrorMsg);

		msgUser("Press any key to continue."); //pause so the user can see the error message; waits for user to press a key
	}
}

int machine::startUp() {

	/// Summary: 
	/// Params: 
	/// Returns: 
	/// Notes: 

	loadConfig('=');


	size_t port_count = openPorts();

	if (port_count <= 0) {
		//Initialize some remote mode thing

		// If there is no hub, either quit the program or enable remote work mode.
		printf("Unable to locate SC hub port, enable remote mode?\n");
		char r_modeChar;
		std::cout << "y/n?:";
		std::cin >> r_modeChar; //y/n input
		if (r_modeChar == 'y' || r_modeChar == 'Y') {
			settings.r_mode = true;
			printf("Remote Work Mode Enabled\n");
			printf("Commands will be simulated, but no attempt to pass the command over USB will be made.\n");
			port_count = 1;
			return 1;
		}
		else if (r_modeChar == 'n' || r_modeChar == 'N') {
			msgUser("Quitting program. Press any key to continue."); //pause so the user can see the error message; waits for user to press a key
			msgUser("");
			return 0;  //This terminates the main program
		}
		else {
			msgUser("Inavlid response. Quitting program. Press any key to continue."); //pause so the user can see the error message; waits for user to press a key
			msgUser("");
			return 0;  //This terminates the main program
		}
	}
	else {
		try {
			//This section initializes the motors and tries to home them.
			//Once the code gets past this point, it can be assumed that the Port has been opened without issue
			//Now we can get a reference to our port object which we will use to access the node objects
			int res = enableNodes();
			if (res == -2) { return res; }

			res = homePosn();

			res = setConfig(/*mode - counts vs revs*/);

			current_pos = measurePosn();

			return false;
		}
		catch (mnErr& theErr)
		{
			printf("Failed to enable and home Nodes.\n");
			printf("Caught error: addr=%d, err=0x%08x\nmsg=%s\n", theErr.TheAddr, theErr.ErrorCode, theErr.ErrorMsg);
			msgUser("Press any key to continue."); //pause so the user can see the error message; waits for user to press a key
			return -2;  //This terminates the main program
		}
	}

}

void machine::shutDown() {

	/// Summary: 
	/// Params: 
	/// Returns: 
	/// Notes: 

	try {
		disableNodes();
	}
	catch (mnErr& theErr)
	{
		printf("Failed to disable Nodes.\n");
		printf("Caught error: addr=%d, err=0x%08x\nmsg=%s\n", theErr.TheAddr, theErr.ErrorCode, theErr.ErrorMsg);
		msgUser("Press any key to continue."); //pause so the user can see the error message; waits for user to press a key
		std::exit(1);  //This terminates the main program
	}
	try {
		closePorts();
	}
	catch (mnErr& theErr)
	{
		printf("Failed to close ports.\n");
		printf("Caught error: addr=%d, err=0x%08x\nmsg=%s\n", theErr.TheAddr, theErr.ErrorCode, theErr.ErrorMsg);
		msgUser("Press any key to continue."); //pause so the user can see the error message; waits for user to press a key
		std::exit(1);  //This terminates the main program
	}
}
/*----------------------------- Test Harness -------------------------------*/

/*------------------------------- Footnotes --------------------------------*/
/*------------------------------ End of file -------------------------------*/
