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
typedef int DnxSerialBase;

using std::cout;
using std::endl;
using std::fabs;

double wait(double arg);

#endif	//SIMULATION

using std::string;
using std::stringstream;

extern PCSerial pc;

string to_hex(uint8_t dec);
string itos(int num);
string dtos(double num);

bool almost_equals(double a, double b);

#endif
