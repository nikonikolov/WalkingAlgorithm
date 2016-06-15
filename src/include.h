#ifndef INCLUDE_H
#define INCLUDE_H

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

#endif	//SIMULATION

using std::string;
using std::stringstream;

extern PCSerial pc;

string to_hex(const uint8_t& dec);
string itos(const int& num);
string dtos(const double& num);

bool almost_equals(const double& a, const double& b);

#endif
