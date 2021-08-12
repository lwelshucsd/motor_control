//Required include files
#include <stdio.h>	
#include <string>
#include <iostream>
#include "pubSysCls.h"	
#include <Windows.h>

using namespace sFnd;

// Send message and wait for newline
char msgUser(const char *msg) {
	std::cout << msg;
	return getchar();
}


//*********************************************************************************
//This program will load configuration files onto each node connected to the port, then executes
//sequential repeated moves on each axis.
//*********************************************************************************

#define ACC_LIM_RPM_PER_SEC	50000
#define VEL_LIM_RPM			180
#define MAX_VEL_LIM			300
#define DELAY				500
//#define MOVE_DISTANCE_CNTS	6400
//#define NUM_MOVES			1

#define TIME_TILL_TIMEOUT	100000	//The timeout used for homing(ms)



std::vector<double> setPosn(class INode&Node1, class INode&Node2, SysManager* myMgr, bool rMode) {
	double move_dist_cnts;
	bool moveIsAbsolute = true;
	std::cout << "Type a distance in counts:";
	std::cin >> move_dist_cnts;
	if (rMode) {
		printf("Simulating movement.\n");
		return std::vector<double> {move_dist_cnts, move_dist_cnts};
	}
	else {
		Node1.Motion.MovePosnStart(move_dist_cnts, moveIsAbsolute);			//execute encoder count move 
		Node2.Motion.MovePosnStart(-move_dist_cnts, moveIsAbsolute);			//execute encoder count move 

		double timeout = myMgr->TimeStampMsec() + TIME_TILL_TIMEOUT; //mymgr->timestampmsec() + thenode.motion.moveposndurationmsec(move_distance_cnts) + 100;			//define a timeout in case the node is unable to enable


		while (!Node1.Motion.MoveIsDone()){// || !Node2.Motion.MoveIsDone()) {
			if (myMgr->TimeStampMsec() > timeout) {
				printf("error: timed out waiting for move to complete\n");
				msgUser("press any key to continue."); //pause so the user can see the error message; waits for user to press a key
				/*return*/;
			}
		}
		myMgr->Delay(DELAY);
		//std::vector<int> 
		return std::vector<double> {(double)Node1.Motion.PosnMeasured};//, (double)Node2.Motion.PosnMeasured};
		//printf("Node 0: %.1f, Node 1: %.1f\n", (double)Node1.Motion.PosnMeasured, (double)Node2.Motion.PosnMeasured);
	}
}

std::vector<double> jogPosn(class INode& Node1, class INode& Node2, SysManager* myMgr, bool rMode, std::vector<double> posn) {
	double move_dist_cnts;
	std::cout << "Type a jog distance in counts:";
	std::cin >> move_dist_cnts;

	if (rMode) {
		printf("Simulating movement.\n");
		return std::vector<double> {posn[0]+move_dist_cnts, posn[1]+move_dist_cnts};
	}
	else {
		Node1.Motion.MovePosnStart(move_dist_cnts, false);			//execute encoder count move 
		//Node2.Motion.MovePosnStart(-move_dist_cnts, false);			//execute encoder count move 

		double timeout = myMgr->TimeStampMsec() + TIME_TILL_TIMEOUT; //mymgr->timestampmsec() + thenode.motion.moveposndurationmsec(move_distance_cnts) + 100;			//define a timeout in case the node is unable to enable


		while (!Node1.Motion.MoveIsDone()){// || !Node2.Motion.MoveIsDone()) {
			if (myMgr->TimeStampMsec() > timeout) {
				printf("error: timed out waiting for move to complete\n");
				msgUser("press any key to continue."); //pause so the user can see the error message; waits for user to press a key
				/*return*/;
			}
		}
		myMgr->Delay(DELAY);
		//std::vector<int> 
		return std::vector<double> {(double)Node1.Motion.PosnMeasured};//, (double)Node2.Motion.PosnMeasured};
		//printf("Node 0: %.1f, Node 1: %.1f\n", (double)Node1.Motion.PosnMeasured, (double)Node2.Motion.PosnMeasured);
	}
}

void posnPrint(std::vector <double> const& a) {
	std::cout << "The current position is : (";

	for (int i = 0; i < a.size(); i++)
		std::cout << a.at(i) << ',' << " ";
	std::cout << "\b\b)\n";
}

int commandLineControl(class IPort& myPort, SysManager* myMgr, bool rMode) {
	// Initialize variables for use in 
	INode& Node1 = myPort.Nodes(0);
	INode& Node2 = myPort.Nodes(1);

	if (!rMode) {
		//This section initializes the motors and tries to home them.
		//Once the code gets past this point, it can be assumed that the Port has been opened without issue
		//Now we can get a reference to our port object which we will use to access the node objects

		for (size_t iNode = 0; iNode < myPort.NodeCount(); iNode++) {
			// Create a shortcut reference for a node
			INode& theNode = myPort.Nodes(iNode);

			theNode.EnableReq(false);				//Ensure Node is disabled before loading config file

			myMgr->Delay(200);


			//theNode.Setup.ConfigLoad("Config File path");


			printf("   Node[%d]: type=%d\n", int(iNode), theNode.Info.NodeType());
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

		///////////////////////////////////////////////////////////////////////////////////////
		//At this point we will create the node shortcuts and set their parameters.
		//////////////////////////////////////////////////////////////////////////////////////
		
		//INode& Node1 = myPort.Nodes(0);
		//INode& Node2 = myPort.Nodes(1);
		for (size_t iNode = 0; iNode < myPort.NodeCount(); iNode++) {
			//std::cout << iNode;
			myPort.Nodes(iNode).AccUnit(INode::RPM_PER_SEC);
			myPort.Nodes(iNode).VelUnit(INode::RPM);
			myPort.Nodes(iNode).Motion.AccLimit = ACC_LIM_RPM_PER_SEC;
			myPort.Nodes(iNode).Motion.VelLimit = VEL_LIM_RPM;
		}
		//Node1.AccUnit(INode::RPM_PER_SEC);				//Set the units for Acceleration to RPM/SEC
		//Node2.AccUnit(INode::RPM_PER_SEC);				//Set the units for Acceleration to RPM/SEC

		//Node1.VelUnit(INode::RPM);						//Set the units for Velocity to RPM
		//Node2.VelUnit(INode::RPM);						//Set the units for Velocity to RPM

		//Node1.Motion.PosnMeasured.AutoRefresh(true);
		//Node2.Motion.PosnMeasured.AutoRefresh(true);

		//Node1.Motion.AccLimit = ACC_LIM_RPM_PER_SEC;
		//Node2.Motion.AccLimit = ACC_LIM_RPM_PER_SEC;

		//Node1.Motion.VelLimit = VEL_LIM_RPM;
		//Node2.Motion.VelLimit = VEL_LIM_RPM;
		printf("Moving Nodes...Current Positions: \n");
	}	
	
	bool quit = false;
	int command;
	std::vector<double> posn;
	if (rMode) { posn = { 0,0 }; }
	double velocity_limit = VEL_LIM_RPM;
	while (!quit) {

		std::cin.clear();
		command = 0;
		printf("Current velocity limit (RPM): %d\n", velocity_limit);
		std::cout << "Please input an operation number.\n";
		std::cout << "1: Set Position\n2: Jog Position\n3: Change Velocity Limit\n4: Quit\n";
		std::cin >> command;

		if (std::cin.fail()) {
			std::cout << "That is not a valid command.\n";
			std::cin.clear();
			std::cin.ignore(100, '\n');
			continue;
		}
		switch (command)
		{
		case 1:
			posn = setPosn(Node1, Node2, myMgr, rMode);
			posnPrint(posn);
			break;
		case 2:
			posn = jogPosn(Node1, Node2, myMgr, rMode, posn);
			posnPrint(posn);
			break;
		case 3:
			std::cout << "Please enter a new velocity in RPM:";
			std::cin >> velocity_limit;
			for (size_t iNode = 0; iNode < myPort.NodeCount(); iNode++) {
				myPort.Nodes(iNode).Motion.VelLimit = VEL_LIM_RPM;
			}
			break;
		case 4:
			std::cout << "Closing program.";
			quit = true;
			break;
		default:
			std::cout << "That is not a valid command.\n";
		}
	}
	return 0;
}


int main(int argc, char* argv[])
{
	msgUser("Motion Example starting. Press Enter to continue.");

size_t portCount = 0;
	std::vector<std::string> comHubPorts;


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

		for (portCount = 0; portCount < comHubPorts.size() && portCount < NET_CONTROLLER_MAX; portCount++) {
			
			myMgr->ComHubPort(portCount, comHubPorts[portCount].c_str()); 	//define the first SC Hub port (port 0) to be associated 
											// with COM portnum (as seen in device manager)
		}
		bool rMode = false; // assume that there will be a hub with motors
		if (portCount <= 0) {
			
			// If there is no hub, either quit the program or enable remote work mode.
			printf("Unable to locate SC hub port, enable remote mode?\n");
			char rModeChar;
			std::cout << "y/n?:";
			std::cin >> rModeChar; //y/n input 
			if (rModeChar == 'y' || rModeChar == 'Y') {
				rMode = 1;
				printf("Remote Work Mode Enabled\n");
				printf("Commands will be simulated, but no attempt to pass the command over USB will be made.\n");
				portCount = 1;
			}
			else if (rModeChar == 'n' || rModeChar == 'N') {
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
		if (!rMode) {
			myMgr->PortsOpen(portCount);				//Open the port
			IPort& myPort = myMgr->Ports(0);

			printf(" Port[%d]: state=%d, nodes=%d\n",
				myPort.NetNumber(), myPort.OpenState(), myPort.NodeCount());

			//Initiate the acutal control loop
			commandLineControl(myPort, myMgr, rMode);

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
		commandLineControl(myPort, myMgr, rMode);
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
