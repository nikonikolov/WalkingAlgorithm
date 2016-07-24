#ifndef INCLUDE_H
#define INCLUDE_H


/*
 * General header including libraries and defining functions
 */


#include <iostream>
#include <string>
#include <sstream>
#include <cmath>

#include "PCSerial.h"

#ifndef SIMULATION

#include "mbed.h"

#else
typedef int DNXServo;

using std::cout;
using std::endl;
using std::fabs;

double wait(double arg);

#endif	//SIMULATION

using std::string;
using std::stringstream;

extern PCSerial pc;

string to_hex(const uint8_t& dec);
string itos(const int& num);
string dtos(const double& num);

bool almost_equals(const double& a, const double& b);

#endif
