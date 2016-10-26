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


bool compare_doubles(double a, double b, double error/* = 0.01*/){
	return fabs(a - b) <= error;
}

#ifdef SIMULATION

double wait(double arg){
	return 0;
}

#endif
