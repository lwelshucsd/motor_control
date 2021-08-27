//Required include files
#include <stdio.h>	
#include <string>
#include <math.h>
#include <iostream>
#include "pubSysCls.h"	
#include <Windows.h>
#include <vector>
#include "general_functions.hpp"
#include "vector_operators.hpp"
#include "motor_functions.hpp"

using namespace sFnd;
using std::vector;
using std::cin;
using std::cout;
using std::string;


//*********************************************************************************
//This program will assume configuration files are preloaded on the motor.
//*********************************************************************************

#define MAX_VEL_LIM				2000		//mm/s
#define SHORT_DELAY				100


int commandLineControl(machine myMachine, bool r_mode) {

	bool quit = false;
	int command;

	while (!quit) {
		cin.clear();
		command = 0;

		vectorPrint(myMachine.measurePosn() * myMachine.config.node_sign, "\nThe current position is : ");
		cout << "\nCurrent velocity limit (mm/s): " << myMachine.config.velocity_limit;
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
			myMachine.current_pos = myMachine.linearMove(r_mode, true);
			break;
		case 2:
			myMachine.current_pos = myMachine.linearMove(r_mode, false);
			break;
		case 3:
			cout << "Please enter a new velocity in mm/s:";
			cin >> myMachine.config.velocity_limit;
			if (myMachine.config.velocity_limit > MAX_VEL_LIM) {
				cout << "Input velocity limit is greater than defined maximum limit. Setting velocity to maximum.\n";
				myMachine.config.velocity_limit = MAX_VEL_LIM;
				break;
			}
			break;
		case 4:
			cout << "Closing program.";
			quit = true;
			break;
		default:
			cout << "That is not a valid command.\n";
		}
		Sleep(2 * SHORT_DELAY);
	}
	return 0;
}


// Main Loop Funciton
// parameterize initialization

int main(int argc, char* argv[])
{
	msgUser("Motion Example starting. Press Enter to continue.");

	machine myMachine;

	bool r_mode = myMachine.startUp();
	try
	{
		// run CLI command loop
		commandLineControl(myMachine, r_mode);
	}
	catch (mnErr& theErr)
	{
		printf("Error in commandLineControl.\n");
		//This statement will print the address of the error, the error code (defined by the mnErr class), 
		//as well as the corresponding error message.
		printf("Caught error: addr=%d, err=0x%08x\nmsg=%s\n", theErr.TheAddr, theErr.ErrorCode, theErr.ErrorMsg);

		msgUser("Press any key to continue."); //pause so the user can see the error message; waits for user to press a key
		return 0;  //This terminates the main program
	}
	
	//Disable the nodes and close the port
	myMachine.shutDown();

	return 0;			//End program
}
