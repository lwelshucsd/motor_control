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

#define LONG_DELAY				500
#define SHORT_DELAY				100
#define TIME_TILL_TIMEOUT		100000	//The timeout used for homing(ms)
#define ACC_LIM_CNTS_PER_SEC2	320000000
#define VEL_LIM_CNTS_PER_SEC	64000
#define MAX_VEL_LIM				2000

using namespace sFnd;

/*--------------------------- External Variables ---------------------------*/
/*----------------------------- Module Defines -----------------------------*/
/*------------------------------ Module Types ------------------------------*/
/*---------------------------- Module Variables ----------------------------*/
/*--------------------- Module Function Prototypes -------------------------*/
/*------------------------------ Module Code -------------------------------*/
bool moveIsDone(class IPort& myPort) {
	// Checks if all movement is completed on all axes.
	// If any axis is not done moving, return false.
	for (size_t iNode = 0; iNode < myPort.NodeCount(); iNode++) {
		if (!myPort.Nodes(iNode).Motion.MoveIsDone()) { return false; }
	}
	return true;
}

void machine::loadConfig() {
	config.is_follower_node = { 0, 0 };	// denotes whether the node is a leader or follower node
	config.node_sign = { -1, 1 };	// denotes the direction that we consider positive relative to the node's positive
	config.lead = { 60, 20 };	// spatial change (mm) per roation of the node shaft
	config.is_rotary_axis = { 0, 0 };
	config.node_2_axis = { 0, 0 };
	config.velocity_limit = 250; //mm/s
	config.num_axes = config.is_follower_node.size() - vsum(config.is_follower_node);
	config.lead_per_cnt = config.lead | (1 / double(6400));
}

std::vector<double> machine::measurePosn() {
	// Measrues position on all nodes.
	// Will eventually discard follower nodes, to only display leader axis dimensions.
	// Returns a double vector of position.
	IPort& myPort = myMgr->Ports(0);
	int num_axes = config.num_axes;
	std::vector<double> is_follower_node = config.is_follower_node;
	std::vector<double> lead_per_cnt = config.lead_per_cnt;

	std::vector<double> position(num_axes);

	int dim_ind = 0;
	for (size_t iNode = 0; iNode < myPort.NodeCount(); iNode++) {
		if (!is_follower_node[iNode]) {
			position[dim_ind] = (myPort.Nodes(iNode).Motion.PosnMeasured);
		}
		dim_ind += 1;
	}
	position = position * lead_per_cnt;
	return position;
}

//change name to "linear"
std::vector<double> machine::linearMove(bool r_mode, bool target_is_absolute) {
	// Expands the motion.moveposnstart command into an arbitrary number of nodes/axes and waits for all to finish.
	IPort& myPort = myMgr->Ports(0);
	std::vector<double> end_pos;
	std::vector<double> dir_vec;
	std::vector<double> input_vec;
	std::vector<double> current_pos = machine::measurePosn();

	int num_axes = config.num_axes;
	double velocity_limit = config.velocity_limit;
	std::vector<double> is_follower_node = config.is_follower_node;
	std::vector<double> lead_per_cnt = config.lead_per_cnt;
	std::vector<double> node_sign = config.node_sign;

	std::string move_dist_str;
	while (true) {
		std::cout << "Type a distance/position in mm, using commas to separate axes:";
		std::cin >> move_dist_str;
		input_vec = parseString(move_dist_str, ',');
		if (input_vec.size() == num_axes) {
			break;
		}
		else {
			std::cout << "\nSize of input vector does not match the number of axes in the system: " << num_axes;
			std::cout << "\nPlease try again.";
		}
	}

	// Compute direciton of movement
	if (target_is_absolute) {
		dir_vec = input_vec - current_pos;
	}
	else {
		dir_vec = input_vec;
	}
	//reset each node's velocity such that velocity vector matches direction vector
	// NEED TO ACCOUNT FOR follower NODES, MAYBE WITH SIMPLE A=>B MAP
	double dir_vec_norm = 1 / norm(dir_vec);
	dir_vec = dir_vec | dir_vec_norm;	// Normalize Movement
	dir_vec = dir_vec | velocity_limit;						// Multiply by overall velocity limit
	dir_vec = dir_vec / lead_per_cnt;						// convert to counts/sec
	for (size_t iNode = 0; iNode < myPort.NodeCount(); iNode++) {
		myPort.Nodes(iNode).Motion.VelLimit = abs(dir_vec[iNode]);
	}
	//convert input vector to counts
	std::vector<double> input_vec_cnts = input_vec / lead_per_cnt;
	input_vec_cnts = input_vec_cnts * node_sign; // reverse required nodes
	if (r_mode) {
		printf("Simulating movement.\n");
		Sleep(SHORT_DELAY);							// Update to incorporate speed and calculate simulated time
		vectorPrint(current_pos, " ");
		std::cout << "\n";
		vectorPrint(input_vec, " ");
		end_pos = (current_pos | (!target_is_absolute)) + input_vec;
	}
	else {
		for (size_t iNode = 0; iNode < myPort.NodeCount(); iNode++) {
			myPort.Nodes(iNode).Motion.Adv.TriggerGroup(1);
			myPort.Nodes(iNode).Motion.Adv.MovePosnStart(input_vec_cnts[iNode], target_is_absolute, true);
		}
		myPort.Nodes(0).Motion.Adv.TriggerMovesInMyGroup();

		double timeout = myMgr->TimeStampMsec() + TIME_TILL_TIMEOUT; //define a timeout in case the node is unable to enable
		while (!moveIsDone(myPort)) {
			if (myMgr->TimeStampMsec() > timeout) {
				printf("Error: timed out waiting for move to complete\n");
				msgUser("press any key to continue."); //pause so the user can see the error message; waits for user to press a key
				return measurePosn();
			}
		}
		myMgr->Delay(SHORT_DELAY);
		end_pos = measurePosn();
	}
	return end_pos;
}

int machine::enableNodes() {
	IPort& myPort = myMgr->Ports(0);
	printf("\n===== Detected Node Data =====\n");
	printf("        Type || FW Version || Serial # || Model\n");
	for (size_t iNode = 0; iNode < myPort.NodeCount(); iNode++) {
		// Create a shortcut reference for a node
		INode& theNode = myPort.Nodes(iNode);

		theNode.EnableReq(false);				//Ensure Node is disabled before loading config file

		myMgr->Delay(200);

		//theNode.Setup.ConfigLoad("Config File path");
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
																	//This will loop checking on the Real time values of the node's Ready status
		while (!theNode.Motion.IsReady()) {
			if (myMgr->TimeStampMsec() > timeout) {
				printf("Error: Timed out waiting for Node %d to enable\n", iNode);
				msgUser("Press any key to continue."); //pause so the user can see the error message; waits for user to press a key
				return -2;
			}
		}
	}
	printf("\n===== User Node IDs =====\n");
	for (size_t iNode = 0; iNode < myPort.NodeCount(); iNode++) {
		printf("Node[%d]: %s\n", int(iNode), myPort.Nodes(iNode).Info.UserID.Value());
	}
	return 1;
}

std::vector<double> machine::homePosn() {
	//	//////////////////////////////////////////////////////////////////////////////////////////////////
	//	//////////////////////////////////////////////////////////////////////////////////////////////////
	//	//	NEED TO CHANGE HOMING OPERATION TO HOME LEADER-FOLLOWER SETS TOGETHER
	//	//	OR TO HOME ALL NODES AT THE SAME TIME, BASICALLY AS A MOVE TO ORIGIN
	//	//////////////////////////////////////////////////////////////////////////////////////////////////
	//	//////////////////////////////////////////////////////////////////////////////////////////////////
	printf("\n===== Homing All Axes =====\n");
	try {
		IPort& myPort = myMgr->Ports(0);
		int num_axes = config.num_axes;
		std::vector<double> is_follower_node = config.is_follower_node;
		std::vector<double> node_2_axis = config.node_2_axis;
		for (size_t iAxis = 0; iAxis < num_axes; iAxis++) {
			for (size_t iNode = 0; iNode < myPort.NodeCount(); iNode++) {
				if (node_2_axis[iNode] == iAxis) {
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
								printf("Node did not complete homing:  \n\t -Ensure Homing settings have been defined through ClearView. \n\t -Check for alerts/Shutdowns \n\t -Ensure timeout is longer than the longest possible homing move.\n");
								msgUser("Press any key to continue."); //pause so the user can see the error message; waits for user to press a key
								std::exit(1);
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
		return measurePosn();
	}
	catch (mnErr& theErr)
	{
		printf("Failed to open port(s)\n");
		//This statement will print the address of the error, the error code (defined by the mnErr class), 
		//as well as the corresponding error message.
		printf("Caught error: addr=%d, err=0x%08x\nmsg=%s\n", theErr.TheAddr, theErr.ErrorCode, theErr.ErrorMsg);

		msgUser("Press any key to continue."); //pause so the user can see the error message; waits for user to press a key
		std::exit(1);
	}
}

int machine::setConfig() {
	// have global struct here that is changed and updated to match file
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

void machine::disableNodes() {
	try {
		IPort& myPort = myMgr->Ports(0);
		printf("\nDisabling nodes\n");
		for (size_t iNode = 0; iNode < myPort.NodeCount(); iNode++) {
			// Create a shortcut reference for a node
			myPort.Nodes(iNode).EnableReq(false);
		}
	}
	catch (mnErr& theErr)
	{
		printf("Failed to disable Nodes n\n");
		//This statement will print the address of the error, the error code (defined by the mnErr class), 
		//as well as the corresponding error message.
		printf("Caught error: addr=%d, err=0x%08x\nmsg=%s\n", theErr.TheAddr, theErr.ErrorCode, theErr.ErrorMsg);

		msgUser("Press any key to continue."); //pause so the user can see the error message; waits for user to press a key
		std::exit(1);  //This terminates the main program
	}
};

int machine::openPorts() {
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

		//if (!r_mode) {
		//	myMgr->PortsOpen(port_count);				//Open the port
		//	IPort& mPort = myMgr->Ports(0);

		//	printf(" Port[%d]: state=%d, nodes=%d\n",
		//		mPort.NetNumber(), mPort.OpenState(), mPort.NodeCount());

		//	myPort = mPort;
		//}
		//else {
		//	//Create an empty port object to use in funcitons
		//	IPort& mPort = myMgr->Ports(0);
		//	myPort = mPort;

		//}
		//
	}
	catch (mnErr& theErr)
	{
		printf("Failed to open port(s)\n");
		//This statement will print the address of the error, the error code (defined by the mnErr class), 
		//as well as the corresponding error message.
		printf("Caught error: addr=%d, err=0x%08x\nmsg=%s\n", theErr.TheAddr, theErr.ErrorCode, theErr.ErrorMsg);

		msgUser("Press any key to continue."); //pause so the user can see the error message; waits for user to press a key
		std::exit(1);
	}
}

void machine::closePorts() {
	printf("Closing Ports\n");
	myMgr->PortsClose();
}

bool machine::startUp() {
	size_t port_count = openPorts();
	loadConfig();

	if (port_count <= 0) {
		//Initialize some remote mode thing
		/*
		// If there is no hub, either quit the program or enable remote work mode.
		printf("Unable to locate SC hub port, enable remote mode?\n");
		char r_modeChar;
		std::cout << "y/n?:";
		std::cin >> r_modeChar; //y/n input
		if (r_modeChar == 'y' || r_modeChar == 'Y') {
			r_mode = 1;
			printf("Remote Work Mode Enabled\n");
			printf("Commands will be simulated, but no attempt to pass the command over USB will be made.\n");
			port_count = 1;
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
		}*/
		return true;
	}
	else {
		try {
			//This section initializes the motors and tries to home them.
			//Once the code gets past this point, it can be assumed that the Port has been opened without issue
			//Now we can get a reference to our port object which we will use to access the node objects
			int res = enableNodes();
			if (res == -2) { return res; }

			current_pos = homePosn();

			res = setConfig(/*mode - counts vs revs*/);

			current_pos = measurePosn();
			return false;
		}
		catch (mnErr& theErr)
		{
			printf("Failed to enable and home Nodes.\n");
			printf("Caught error: addr=%d, err=0x%08x\nmsg=%s\n", theErr.TheAddr, theErr.ErrorCode, theErr.ErrorMsg);
			msgUser("Press any key to continue."); //pause so the user can see the error message; waits for user to press a key
			return 0;  //This terminates the main program
		}
	}

}

void machine::shutDown() {
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
