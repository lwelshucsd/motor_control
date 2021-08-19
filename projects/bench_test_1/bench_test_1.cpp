//Required include files
#include <stdio.h>	
#include <string>
#include <math.h>
#include <iostream>
#include "pubSysCls.h"	
#include <Windows.h>
#include <valarray>

using namespace sFnd;
using std::vector;
using std::cin;
using std::cout;
using std::string;
using std::valarray;


// Send message and wait for newline
char msgUser(const char *msg) {
	cout << msg;
	return getchar();
}

//*********************************************************************************
//This program will assume all configuration files are preloaded on the motor.
//*********************************************************************************

#define ACC_LIM_RPM_PER_SEC	50000
#define VEL_LIM_RPM			180
#define MAX_VEL_LIM			1000
#define LONG_DELAY			500
#define SHORT_DELAY			100
#define TIME_TILL_TIMEOUT	100000	//The timeout used for homing(ms)
#define CNTS_PER_REV		6400

// Global Variables for later use
valarray<double> current_pos;

// Find a way to replace this section so that it is not hardcoded
// Configuration specific mapping variables
valarray<bool>		is_follower_node	= { 0, 0 };	// denotes whether the node is a leader or follower node
valarray<double>	node_sign		= { -1, 1 };	// denotes the direction that we consider positive relative to the node's positive
valarray<double>	lead			= { 60, 20 };	// spatial change (mm) per roation of the node shaft
//valarray<bool>		is_follower_node	= { 0, 0, 1, 0 };
//valarray<double>	node_sign		= { -1, 1, 1, 1 };
//valarray<double>	lead			= { 60, 60, 60, 8 };
valarray<double>	lead_per_cnt	= lead/CNTS_PER_REV;	
int					num_axes		= is_follower_node.size()-is_follower_node.sum();
double				velocity_limit	= 250; //mm/s


//// General Use Functions
void vectorPrint(vector<double> const&a, string comment) {
	//Prints a vector with a given in
	cout << comment <<"(";

	for (int i = 0; i < a.size(); i++)
		cout << a.at(i) << ',' << " ";
	cout << "\b\b)\n";
}

void valarrayPrint(valarray<double> const&a, string comment) {
	//Prints a vector with a given in
	cout << comment << "(";

	for (int i = 0; i < a.size(); i++)
		cout << a[i] << ',' << " ";
	cout << "\b\b)\n";
}

valarray<double> parseString(string input, char delimiter) {
	int last_slice = 0;
	vector<double> tmp_vec;
	for (size_t i = 0; i <= input.length(); i++) {
		if (i == input.length()) {
			try {
				double val = std::stod(input.substr(last_slice, std::string::npos));
				tmp_vec.push_back(val);
			}
			catch (...) {
				continue;
			}
		}
		else if (input.at(i) == delimiter || input.at(i) == ' ') {
			try {
				double val = std::stod(input.substr(last_slice, i - last_slice));
				tmp_vec.push_back(val);
				last_slice = i + 1;
			}
			catch (...) {
				continue;
			}
		}
	}

	valarray<double> output;
	output.resize(tmp_vec.size(), 0);
	for (size_t i = 0; i < tmp_vec.size(); i++) {
		output[i] = tmp_vec[i];
	}
	return output;
}


//// Clearpath Specific Funcitons
bool moveIsDone(class IPort&myPort) {
	// Checks if all movement is completed on all axes.
	// If any axis is not done moving, return false.
	for (size_t iNode = 0; iNode < myPort.NodeCount(); iNode++) {
		//cout << "\nnode " << iNode << ":     " << myPort.Nodes(iNode).Motion.MoveIsDone();
		if (!myPort.Nodes(iNode).Motion.MoveIsDone()) { return false; }
	}
	return true;
}

valarray<double> measurePosn(class IPort& myPort) {
	// Measrues position on all nodes.
	// Will eventually include a counts=>distance mapping for each individual node.
	// Will eventually discard follower nodes, to only display axis dimensions.
	// Returns a double vector of position.
	// 

	//int num_dims = myPort.NodeCount() - sum(isfollowerNode(map))
	//int num_dims = 3;
	valarray<double> position (num_axes);
	int dim_ind = 0;
	for (size_t iNode = 0; iNode < myPort.NodeCount(); iNode++) {
		if (!is_follower_node[iNode]) {
			position[dim_ind] = (myPort.Nodes(iNode).Motion.PosnMeasured);
		}
		dim_ind += 1;
	}
	position *= lead_per_cnt;// *= node_sign;
	return position;
}


valarray<double> changePosn(class IPort&myPort, SysManager* myMgr, bool r_mode, bool target_is_absolute) {
	// Expands the motion.moveposnstart command into an arbitrary number of nodes/axes and waits for all to finish.
	valarray<double> end_pos;
	valarray<double> dir_vec;
	valarray<double> input_vec;

	string move_dist_str;
	while (true) {
		cout << "Type a distance/position in mm, using commas to separate axes:";
		cin >> move_dist_str;
		input_vec = parseString(move_dist_str, ',');
		if (input_vec.size() == num_axes) {
			break;
		}
		else {
			cout << "\nSize of input vector does not match the number of axes in the system: " << num_axes;
			cout << "\nPlease try again.";
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
	dir_vec /= sqrt((dir_vec * dir_vec).sum());		// Normalize Movement
	dir_vec *= velocity_limit;						// Multiply by overall velocity limit
	dir_vec /= lead_per_cnt;						// convert to counts/sec
	for (size_t iNode = 0; iNode < myPort.NodeCount(); iNode++) {
		myPort.Nodes(iNode).Motion.VelLimit = abs(dir_vec[iNode]);
	}


	//convert input vector to counts
	valarray<double> input_vec_cnts = input_vec / lead_per_cnt;
	input_vec_cnts *= node_sign; // reverse required nodes

	if (r_mode) {
		printf("Simulating movement.\n");
		Sleep(SHORT_DELAY);							// Update to incorporate speed and calculate simulated time
		valarrayPrint(current_pos, " ");
		cout << "\n";
		valarrayPrint(input_vec, " ");
		end_pos = current_pos*(!target_is_absolute) + input_vec;
	}
	else {
		for (size_t iNode = 0; iNode < myPort.NodeCount(); iNode++) {
			myPort.Nodes(iNode).Motion.MovePosnStart(input_vec_cnts[iNode], target_is_absolute);
		}

		double timeout = myMgr->TimeStampMsec() + TIME_TILL_TIMEOUT; //define a timeout in case the node is unable to enable
		while (!moveIsDone(myPort)) {
			if (myMgr->TimeStampMsec() > timeout) {
				printf("Error: timed out waiting for move to complete\n");
				msgUser("press any key to continue."); //pause so the user can see the error message; waits for user to press a key
				return measurePosn(myPort);
			}
		}
		myMgr->Delay(SHORT_DELAY);
		end_pos = measurePosn(myPort);
	}
	return end_pos;
}

//Insert initialization function that returns myport value, and possibly also r_mode
//IPort&myPort = initializePort(class lorem, ipsum)

int commandLineControl(class IPort& myPort, SysManager* myMgr, bool r_mode) {
	// Initialize mapping variables


	//move to initialization function
	if (!r_mode) {
		//This section initializes the motors and tries to home them.
		//Once the code gets past this point, it can be assumed that the Port has been opened without issue
		//Now we can get a reference to our port object which we will use to access the node objects

		for (size_t iNode = 0; iNode < myPort.NodeCount(); iNode++) {
			// Create a shortcut reference for a node
			INode& theNode = myPort.Nodes(iNode);

			theNode.EnableReq(false);				//Ensure Node is disabled before loading config file

			myMgr->Delay(200);

			//theNode.Setup.ConfigLoad("Config File path");

			printf("	 Node[%d]: type=%d\n", int(iNode), theNode.Info.NodeType());
			printf("            userID: %s\n", theNode.Info.UserID.Value());
			printf("        FW version: %s\n", theNode.Info.FirmwareVersion.Value());
			printf("          Serial #: %d\n", theNode.Info.SerialNumber.Value());
			printf("             Model: %s\n", theNode.Info.Model.Value());

			//The following statements will attempt to enable the node.  First,
			// any shutdowns or NodeStops are cleared, finally the node is enabled
			theNode.Status.AlertsClear();					//Clear Alerts on node 
			theNode.Motion.NodeStopClear();	//Clear Nodestops on Node  				
			theNode.EnableReq(true);					//Enable node 
			//At this point the node is enabled
			printf("Node \t%zi enabled\n", iNode);
			double timeout = myMgr->TimeStampMsec() + TIME_TILL_TIMEOUT;	//define a timeout in case the node is unable to enable
																		//This will loop checking on the Real time values of the node's Ready status
			while (!theNode.Motion.IsReady()) {
				if (myMgr->TimeStampMsec() > timeout) {
					printf("Error: Timed out waiting for Node %d to enable\n", iNode);
					msgUser("Press any key to continue."); //pause so the user can see the error message; waits for user to press a key
					return -2;
				}
			}
			//At this point the Node is enabled, and we will now check to see if the Node has been homed
			//Check the Node to see if it has already been homed, 
			//////////////////////////////////////////////////////////////////////////////////////////////////
			//////////////////////////////////////////////////////////////////////////////////////////////////
			//	NEED TO CHANGE HOMING OPERATION TO HOME LEADER-FOLLOWER SETS TOGETHER
			//	OR TO HOME ALL NODES AT THE SAME TIME, BASICALLY AS A MOVE TO ORIGIN
			//////////////////////////////////////////////////////////////////////////////////////////////////
			//////////////////////////////////////////////////////////////////////////////////////////////////
			if (theNode.Motion.Homing.HomingValid())
			{
				if (theNode.Motion.Homing.WasHomed())
				{
					printf("Node %d has already been homed, current position is: \t%8.0f \n", iNode, theNode.Motion.PosnMeasured.Value());
					printf("Rehoming Node... \n");
				}
				else
				{
					printf("Node [%d] has not been homed.  Homing Node now...\n", iNode);
				}
				//Now we will home the Node
				
				theNode.Motion.Homing.Initiate();

				timeout = myMgr->TimeStampMsec() + TIME_TILL_TIMEOUT;	//define a timeout in case the node is unable to enable
																		// Basic mode - Poll until disabled
				while (!theNode.Motion.Homing.WasHomed()) {
					if (myMgr->TimeStampMsec() > timeout) {
						printf("Node did not complete homing:  \n\t -Ensure Homing settings have been defined through ClearView. \n\t -Check for alerts/Shutdowns \n\t -Ensure timeout is longer than the longest possible homing move.\n");
						msgUser("Press any key to continue."); //pause so the user can see the error message; waits for user to press a key
						return -2;
					}
				}
				printf("Node completed homing\n");
			}
			else {
				printf("Node[%d] has not had homing setup through ClearView.  The node will not be homed.\n", iNode);
			}

		}
		
		for (size_t iNode = 0; iNode < myPort.NodeCount(); iNode++) {
			//cout << iNode;
			myPort.Nodes(iNode).AccUnit(INode::RPM_PER_SEC);
			myPort.Nodes(iNode).VelUnit(INode::COUNTS_PER_SEC);
			myPort.Nodes(iNode).Motion.AccLimit = ACC_LIM_RPM_PER_SEC;
			myPort.Nodes(iNode).Motion.VelLimit = VEL_LIM_RPM;
			myPort.Nodes(iNode).Motion.PosnMeasured.AutoRefresh(true);
		}

		printf("Moving Nodes...Current Positions: \n");
	}	
	
	bool quit = false;
	int command;
	
	if (r_mode) { 
		cout << "Please input the number of axes to simulate: ";
		cin >> num_axes;
		current_pos.resize(num_axes,0); 
		lead_per_cnt.resize(num_axes, 1);
	}
	else {
		cout << "Please confirm the following configuration.\n";
		//print out the config
		cout << "If configuration is incorrect, please quit the program and fix mappings in code";
		current_pos = measurePosn(myPort);
		valarrayPrint(current_pos, "\n");
	}

	//double velocity_limit = VEL_LIM_RPM;
	while (!quit) {

		cin.clear();
		command = 0;
		cout << "\n\nCurrent velocity limit (mm/s): " << velocity_limit;
		cout << "\nPlease input an operation number.\n";
		cout << "1: Change Position\n2: Jog Position\n3: Change Velocity Limit\n4: Quit\n";
		cin >> command;

		if (cin.fail()) {
			cout << "That is not a valid command.\n";
			cin.clear();
			cin.ignore(100, '\n');
			continue;
		}
		switch (command)
		{
		case 1:
			current_pos = changePosn(myPort, myMgr, r_mode, true);
			valarrayPrint(current_pos *= node_sign, "The current position is : ");
			break;
		case 2:
			current_pos = changePosn(myPort, myMgr, r_mode, false);
			valarrayPrint(current_pos *= node_sign, "The current position is : ");
			break;
		case 3:
			cout << "Please enter a new velocity in mm/s:";
			cin >> velocity_limit;
			if (velocity_limit > MAX_VEL_LIM) {
				cout << "Input velocity limit is greater than defined maximum limit. Setting velocity to maximum.\n";
				velocity_limit = MAX_VEL_LIM;
				break;
			}
			// remove this later for new velocity in mm/s
			//for (size_t iNode = 0; iNode < myPort.NodeCount(); iNode++) {
			//	myPort.Nodes(iNode).Motion.VelLimit = velocity_limit;
			//}
			break;
		case 4:
			cout << "Closing program.";
			quit = true;
			break;
		default:
			cout << "That is not a valid command.\n";
		}
		Sleep(2*SHORT_DELAY);
	}
	return 0;
}


// Main Loop Funciton
// parameterize initialization

int main(int argc, char* argv[])
{
	msgUser("Motion Example starting. Press Enter to continue.");

	size_t port_count = 0;
	vector<string> comHubPorts;


	//Create the SysManager object. This object will coordinate actions among various ports
	// and within nodes. In this example we use this object to setup and open our port.
	SysManager* myMgr = SysManager::Instance();							//Create System Manager myMgr

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
			
			// If there is no hub, either quit the program or enable remote work mode.
			printf("Unable to locate SC hub port, enable remote mode?\n");
			char r_modeChar;
			cout << "y/n?:";
			cin >> r_modeChar; //y/n input 
			if (r_modeChar == 'y' || r_modeChar == 'Y') {
				r_mode = 1;
				printf("Remote Work Mode Enabled\n");
				printf("Commands will be simulated, but no attempt to pass the command over USB will be made.\n");
				port_count = 1;
			}
			else if (r_modeChar == 'n' || r_modeChar == 'N') {
				char a = msgUser("Quitting program. Press any key to continue."); //pause so the user can see the error message; waits for user to press a key
				msgUser("");
				return 0;  //This terminates the main program
			}
			else {
				char a = msgUser("Inavlid response. Quitting program. Press any key to continue."); //pause so the user can see the error message; waits for user to press a key
				msgUser("");
				return 0;  //This terminates the main program
			}
		}
		if (!r_mode) {
			myMgr->PortsOpen(port_count);				//Open the port
			IPort& myPort = myMgr->Ports(0);

			printf(" Port[%d]: state=%d, nodes=%d\n",
				myPort.NetNumber(), myPort.OpenState(), myPort.NodeCount());

			//Initiate the acutal control loop
			commandLineControl(myPort, myMgr, r_mode);

			//////////////////////////////////////////////////////////////////////////////////////////////
			//After the program is quit, disable nodes, and close ports
			//////////////////////////////////////////////////////////////////////////////////////////////
			printf("Disabling nodes, and closing port\n");
			for (size_t iNode = 0; iNode < myPort.NodeCount(); iNode++) {
				// Create a shortcut reference for a node
				myPort.Nodes(iNode).EnableReq(false);
			}
		}
		else {
		//Create an empty port object to use in funcitons
		IPort& myPort = myMgr->Ports(0);
		commandLineControl(myPort, myMgr, r_mode);
		}


	}
	catch (mnErr& theErr)
	{
		printf("Failed to disable Nodes n\n");
		//This statement will print the address of the error, the error code (defined by the mnErr class), 
		//as well as the corresponding error message.
		printf("Caught error: addr=%d, err=0x%08x\nmsg=%s\n", theErr.TheAddr, theErr.ErrorCode, theErr.ErrorMsg);

		msgUser("Press any key to continue."); //pause so the user can see the error message; waits for user to press a key
		return 0;  //This terminates the main program
	}
	
	msgUser("");	//The following line would not actually pause without this.
	msgUser("Press any key to continue."); //pause so the user can see the error message; waits for user to press a key

	// Close down the ports
	myMgr->PortsClose();

	return 0;			//End program
}
