#include <cstdlib>

#include "src/wkq.h"

#include "src/ServoJoint.h"
#include "src/Leg.h"
#include "src/Tripod.h"
#include "src/Robot.h"

#include "src/include.h"


int main(){

	
	cout<<"MAIN started"<<endl;

	int baud = 115200;
	double init_height = 10.0;
	
	Robot* WkQuad;
	// Instantiate Robot
	try{
		WkQuad = new Robot(NULL, NULL, init_height, wkq::RS_standing);
	}
	catch(const string& msg){
		pc.print_debug(msg);
		exit(EXIT_FAILURE);
	}

	cout<<"Robot Initialized\n";

	WkQuad->Stand();
	pc.print_debug("Robot Standing\n");
	WkQuad->StandQuad();
	pc.print_debug("Robot Standing as Quad\n");
	WkQuad->FlattenLegs();
	pc.print_debug("Legs Flattened\n");
	WkQuad->Default();
	pc.print_debug("Legs in Default position\n");
	WkQuad->Center();
	pc.print_debug("Legs Centered\n");

	// Further tests - lifting body after center to stand
	// Centering for arbitrary height

	return 0;
}



