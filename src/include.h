#ifndef INCLUDE_H
#define INCLUDE_H

#include <iostream>
#include <string>
#include <sstream>
//#include "../mbed/api/mbed.h"
#include "mbed.h"
#include "PCSerial.h"

using std::string;
using std::stringstream;
//using std::to_string;

extern PCSerial pc;
extern mbed::Serial deviceHipsKnees;
extern mbed::Serial deviceArmsWings;

string to_hex(const int& dec);
string itos(const int& num);

#endif
