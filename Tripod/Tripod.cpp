#include "Tripod.h"

Tripod::Tripod (Leg& front_in, Leg middle_in, Leg& rear_in){

	Legs[FRONTLEG] = &front_in;
	Legs[MIDDLELEG] = &middle_in;	
	Legs[REARLEG] = &rear_in;	
}


void Tripod::BodyForward (int distance){
	
	for(int i=0; i<LEG_COUNT; i++){
		Legs[i]->IKForward(distance);
	}
	
	WriteAngles();
}


void Tripod::BodyRotate(double angle){
	for(int i=0; i<LEG_COUNT; i++){
		Legs[i]->IKRotate(angle);
	}

	WriteAngles();
}


void Tripod::WriteAngles(){
	for(int i=0; i<LEG_COUNT; i++){
		Legs[i]->WriteAngles(distance);
	}
}
