#include "Master.h"

#ifndef SIMULATION
Master::Master(PinName tx, PinName rx, int baud_in) :
	port(new mbed::Serial(tx, rx)), baud(baud_in), bit_period_(1000000.0/baud_in){}
#endif
/*
bool Master::inputWalkForward(){
    static int out=0;
	int steps;
    if(call==1){
        steps = steps1;
    }
    if(call=2){
        steps = steps2;

    }
    if(call=3){
        steps = steps3;
    }
	
    if(out<steps){
        out++;
        return true;
    } 
    call++;
    out=0;
    return false;
}
*/
bool Master::inputWalkForward(){
    static int out=0;
    int steps=20;
    if(out<steps){
        out++;
        return true;
    } 
    return false;
}

