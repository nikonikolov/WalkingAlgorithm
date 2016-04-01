/*
#include "src/Leg.h"
#include "src/Tripod.h"
#include "src/Robot.h"
#include "src/PC.h"
#include "src/wkquad.h"
*/

#include "src/DNXServo.h"
#include "src/AX12A.h"
#include "src/XL320.h"
#include "src/include.h"

/* Global vars:
	PC pc - specifies connected pc for debugging purposes
	Serial deviceHipsKnees - connected to serial pins 9 and 10
	Serial deviceArmsWings - connected to serial pins 13 and 14

*/
int main(){

	pc.set_debug();
	
	pc.print_debug("MAIN started\n");

	// Remember to set that to 1000000 - both for servos and for mbed
	int baud = 115200;

	deviceHipsKnees.baud(baud);
	deviceArmsWings.baud(baud);

	XL320 ArmsWings(&deviceArmsWings, baud);
	AX12A HipsKnees(&deviceHipsKnees, baud);

	pc.print_debug("HipsKnees initialized\n");

	int val = HipsKnees.GetValue(14, AX_RETURN_LEVEL);
	pc.print_debug("Return level is " + itos(val) + "\n");

	val = HipsKnees.GetValue(14, 4);
	pc.print_debug("baud rate read from id 25 is " + itos(val) + "\n");

	HipsKnees.SetGoalPosition(XL_ID_Broadcast, 512);

	pc.print_debug("Goal Position set\n");


/*	pc.print_debug("ArmsWings initialized\n");

	int val = ArmsWings.GetValue(25, 3);
	pc.print_debug("ID read from id 25 is " + itos(val) + "\n");

	val = ArmsWings.GetValue(XL_ID_Broadcast, 3);
	pc.print_debug("ID read from id 25 is " + itos(val) + "\n");

	val = ArmsWings.GetValue(25, 4);
	pc.print_debug("baud rate read from id 25 is " + itos(val) + "\n");

	ArmsWings.SetGoalPosition(XL_ID_Broadcast, 0);

	pc.print_debug("Goal Position set\n");
*/
	return 0;
}



