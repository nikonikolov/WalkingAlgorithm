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
int main(int argc, char* argv[]){

	// Set baud rates of PC and serial ports


	bool debug=false;
	if(argc>1){
		if(!strcmp(argv[1],"-debug")) pc.set_debug();
	}




	return 0;
}



