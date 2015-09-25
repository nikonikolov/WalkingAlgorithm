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

void Tripod::Raise(const double& height){
	for(int i=0; i<LEG_COUNT; i++){
		Legs[i]->Lift(height);
	}

	WriteAngles();
}

void Tripod::Down(const double& height){
	for(int i=0; i<LEG_COUNT; i++){
		Legs[i]->Down(height);
	}

	WriteAngles();
}


/*
// Think about creating a function wrapper for all Body movements, that takes as input pointer to the desired function. Sth like
void Tripod::BodyMove(double parameter, void (*IKLegMovement)(double) ){
	for(int i=0; i<LEG_COUNT; i++){
		Legs[i]->IKLegMovement(parameter);
	}

	WriteAngles();
}

void Tripod::BodyRotate(double angle){
	BodyMove(angle, &Leg::IKRotate);
}



void Tripod::LegAction(Leg* LegIn, void (Leg::*LegMethod)(double), const double& parameter){
	LegIn->LegMethod(parameter);
}

// issues with this implementation are differences between pointers to member functions and normal functions 
//and objects to call them on or declaring the member function as static
*/