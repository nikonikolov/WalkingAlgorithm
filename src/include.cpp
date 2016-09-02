#include "include.h"

PCSerial pc(false);

string to_hex(uint8_t dec){
	stringstream ss;
	ss<<std::hex<<(int)((uint32_t)dec);
	return ss.str();
}


string itos(int num){
	stringstream ss;
	ss<<num;
	return ss.str();
}

string dtos(double num){
	stringstream ss;
	ss<<num;
	return ss.str();	
}


bool almost_equals(double a, double b){
	return fabs(a - b) <= 0.01;
}

#ifdef SIMULATION

double wait(double arg){
	return 0;
}

#endif
