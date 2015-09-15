#include "Tripod.h"

Tripod::Tripod (Leg& front_in, Leg& back_in, Leg middle_in):
	FrontLeg(front_in), BackLeg(back_in), MiddleLeg(middle_in) {}

void Tripod::BodyForward (int distance){
	
	for(int i=0; i<LEG_COUNT; i++){
		Legs[i]->IKForward(distance);
	}
	
	WriteAngles();
}

void Tripod::WriteAngles(){
	for(int i=0; i<LEG_COUNT; i++){
		Legs[i]->WriteAngles(distance);
	}
}
