//Required include files
#include <iostream>
#include <Windows.h>
#include "general_functions.hpp"
#include "motor_functions.hpp"

using namespace sFnd;
using std::cin;
using std::cout;


//*********************************************************************************
//This program will assume configuration files are preloaded on the motor.
//*********************************************************************************

#define SHORT_DELAY				100


int commandLineControl(machine my_machine) {

	/// Summary: 
	/// Params: 
	/// Returns: 
	/// Notes: 


	bool quit = false;
	int command;
	int op_num = 0;
	std::vector<double> input_vec;
	cout << "\n\n========== Begin Operation ==========";
	while (!quit) {
		op_num += 1;
		cin.clear();
		command = 0;

		printf("\n===== Operation #%i =====\n",op_num);
		if (!my_machine.settings.r_mode) {
			vectorPrint(my_machine.current_pos, "The current position is : ");
		} else {
			my_machine.current_pos = my_machine.measurePosn();
			vectorPrint(my_machine.current_pos * my_machine.config.node_sign, "The current position is : ");
		}
		printf("Current velocity limit: %0.2f mm/s\n", my_machine.config.velocity_limit);
		cout << "Please input an operation number.\n";
		cout << "0: Quit\n";
		cout << "1: Change Position\n";
		cout << "2: Linear Jog\n";
		cout << "3: Change Velocity Limit\n";
		//cout << "4: Repeat last operation\n"
		//cout << "4: Preset toolpaths\n";
		//cout << "5: Machine Info\n";
		//cout << "N: OPERATION\n";

		
		cin >> command;	// Await command input

		if (cin.fail()) {	//If the cin fails (i.e. a char is input when it expected int), ignore the rest of the input and clear the input stream.
			cout << "That is not a valid command.\n";
			cin.clear();
			cin.ignore(100, '\n');
			continue;		// Go back to the command menu
		}
		switch (command)
		{
		case 0:
			quit = true;
			break;
		case 1:
			input_vec = userVectorInput("Please input the target position, separated by commas: ", my_machine.config.num_axes);
			my_machine.current_pos = my_machine.linearMove(input_vec, true);
			break;
		case 2:
			input_vec = userVectorInput("Please input a jog distance, separated by commas: ", my_machine.config.num_axes);
			my_machine.current_pos = my_machine.linearMove(input_vec, false);
			break;
		case 3:
			cout << "Please enter a new velocity in mm/s:";
			cin >> my_machine.config.velocity_limit;
			if (my_machine.config.velocity_limit > my_machine.config.max_velocity_limit) {
				cout << "Input velocity limit is greater than defined maximum limit. Setting velocity to maximum.\n";
				my_machine.config.velocity_limit = my_machine.config.max_velocity_limit;
				break;
			}
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

	machine my_machine;

	my_machine.startUp();
	try
	{
		// run CLI command loop
		commandLineControl(my_machine);
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
	my_machine.shutDown();
	cout << "Closing program.\n";
	msgUser("Press any key to continue.");
	msgUser("");
	return 0;			//End program
}
