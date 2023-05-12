#pragma once
//Required include files
#include <Windows.h>
#include <iostream>
#include "general_functions.hpp"
#include "clearpath_axes.hpp"
using namespace sFnd;
using std::cin;
using std::cout;

//*********************************************************************************
//This program will assume configuration files are preloaded on the motor.
//*********************************************************************************

#define SHORT_DELAY				100


int CML_control_loop_f(machine my_machine) {

	/// Summary: Main Commandline control loop of motor controller. On each loop it:
	///							1. Prints the operation number
	///							2. Prints the current position of the machine
	///							3. Prompts the next input command
	///							4. Waits for a response
	///							5. Executes requested operation
	/// Params: 
	/// Returns: 
	/// Notes: 


	bool quit = false;
	int command;
	int last_command = 0;
	int op_num = 0;
	int selected_axis;
	std::vector<double> input_vec;
	std::vector<double> last_input_vec;
	cout << "\n\n========== Begin Operation ==========";
	// Keep looping until quit is requested
	while (!quit) {
		op_num += 1;
		cin.clear();
		command = 0;
		printf("\n===== Operation #%i =====\n", op_num);
		print_vector_f(my_machine.current_position, "The current position is : ");

		//Print operation menu
		printf("Current velocity limit: %0.2f mm/s\n", my_machine.config.machine_velocity_limit);
		cout << "Please input an operation number.\n";
		cout << "0: Quit\n";
		cout << "1: Change Position\n";
		cout << "2: Linear Jog\n";
		cout << "3: Change Velocity Limit\n";
		cout << "4: Home Axis\n";
		cout << "5: No Command Currently Added (Will be 'Print Machine Info)'\n";
		//cout << "x: Repeat last operation\n"
		//cout << "x: Preset toolpaths\n";
		//cout << "x: Machine Info\n";
		//cout << "N: OPERATION NAME\n";


		cin >> command;	// Await command input

		if (cin.fail()) {	//If the cin fails (i.e. a char is input when it expected int), ignore the rest of the input and clear the input stream.
			cout << "That is not a valid command.\n";
			cin.clear();
			cin.ignore(100, '\n');
			continue;		// Go back to the command menu
		}

		// Switch statement handles all commands 
		switch (command)
		{
		case 0:	//Quit
			quit = true;
			break;
		case 1:	//Set Position
			input_vec = user_input_vector_f("Please input the target position, separated by commas: ", my_machine.config.machine_num_axes);
			my_machine.current_position = my_machine.move_linear_f(input_vec, true);
			break;
		case 2:	//Job Position
			input_vec = user_input_vector_f("Please input a jog distance, separated by commas: ", my_machine.config.machine_num_axes);
			my_machine.current_position = my_machine.move_linear_f(input_vec, false);
			break;
		case 3:	//Change Velocity Limit
			cout << "Please enter a new velocity in mm/s:";
			cin >> my_machine.config.machine_velocity_limit;
			if (my_machine.config.machine_velocity_limit > my_machine.config.machine_velocity_max) {
				cout << "Input velocity limit is greater than defined maximum limit. Setting velocity to maximum.\n";
				my_machine.config.machine_velocity_limit = my_machine.config.machine_velocity_max;
				break;
			}
			break;
		case 4: //Repeat Last Command
			// SHOULD BE MORE ABSTRACTED
			cout << "Please select an axis.\n";
			cout << "0: X\n";
			cout << "1: Y\n";
			cout << "2: Z\n";
			cin >> selected_axis;	// Await command input
			switch (selected_axis) 
			{
			case 0:
				my_machine.home_axis_f(0);
				break;
			case 1:
				my_machine.home_axis_f(1);
				break;
			case 2:
				my_machine.home_axis_f(2);
				break;
			default:
				cout << "Invalid Axis\n";
			}

			break;
		case 5: //Display Machine Info

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
	msg_user_f("Test Tank starting. Press Enter to continue.");


	machine my_machine;

	int res = my_machine.start_up_f();


	if (res == 1) {
		try
		{
			// run CLI command loop
			CML_control_loop_f(my_machine);
		}
		catch (mnErr& theErr)
		{
			printf("Error in CML_control_loop_f.\n");
			//This statement will print the address of the error, the error code (defined by the mnErr class), 
			//as well as the corresponding error message.
			printf("Caught error: addr=%d, err=0x%08x\nmsg=%s\n", theErr.TheAddr, theErr.ErrorCode, theErr.ErrorMsg);

			msg_user_f("Press any key to continue."); //pause so the user can see the error message; waits for user to press a key
			return 0;  //This terminates the main program
		}

		//Disable the nodes and close the port
		my_machine.shut_down_f();
		msg_user_f("Closing program.");
		msg_user_f("\nPress any key to continue.");
		return 0;			//End program
	}
}
