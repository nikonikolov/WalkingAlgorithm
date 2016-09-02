/* 

Dynamixel Communication Abstract class 
===========================================================================================

FUNCTIONALITY:
	1. Defines the basic functionality that a protocol implementation must support
	1. Connects to a Serial Port
	2. Provides Hardware Abstraction Layer - subclasses do not need to implement 
		hardware writing or reading associated with the serial port

-------------------------------------------------------------------------------------------

*/

#ifndef DNXSERVO_H
#define DNXSERVO_H

#include "include.h"
#include <math.h>
#include <cstdint>


class DNXServo{

public:

	DNXServo(PinName tx, PinName rx, int baudIn, const int ReturnLvlIn =1);

	virtual ~DNXServo();

    int SetID(int ID, int newID);
	int GetValue(int ID, int address);

    virtual int SetBaud	(int ID, int rate) =0;
    virtual int SetReturnLevel(int ID, int lvl) =0;
    virtual int SetLED(int ID, int colour) =0; 
	virtual int SetGoalPosition(int ID, int angle) =0;
	virtual int SetGoalPosition(int ID, double angle) =0;
	virtual int SetGoalVelocity(int ID, int velocity) =0;
	virtual int SetGoalTorque(int ID, int torque) =0;
	virtual int SetPunch(int ID, int punch) =0;

protected:
	
	template<class Type>
	inline uint8_t lobyte(const Type& num);
	template<class Type>
	inline uint8_t hibyte(const Type& num);
	template<class T1, class T2>
	inline uint16_t makeword(const T1& num1, const T2& num2);

	void flush();
	void write(uint8_t* buf, int n);
	int read(uint8_t* buf, int nMax=255);

	int angleScale(double angle);
	int AddressLength(int address, const uint8_t * TWO_BYTE_ADDRESSES);
	void packetPrint(int bytes, uint8_t* buf);
	
	virtual int statusError(uint8_t* buf, int n) =0;
	virtual int send(int ID, int bytes, uint8_t* parameters, uint8_t ins) =0;

	virtual int dataPack(uint8_t ins, uint8_t ** parameters, int address, int value =0) =0;
	virtual int dataPush(int ID, int address, int value) =0;
	virtual int dataPull(int ID, int address) =0;

	// REPLY BUFFER - SIZE 256 Overflow should never occur no matter the number of servos - you only communicate with one ID
	// and others don't respond. ID_Broadcast does not reply as well 
    uint8_t reply_buf[256];		

    mbed::Serial* port;
    int baud;
    double bitPeriod;
    int ReturnLvl = 1;

};

// Control table: Only matching addresses are included
#define DNXSERVO_ID 						3
#define DNXSERVO_BAUD 						4

const uint8_t ID_Broadcast = 0xFE; // 254(0xFE) ID writes to all servos on the line

template<class Type>
inline uint8_t DNXServo::lobyte(const Type& num){
	return (uint8_t)num;
}

template<class Type>
inline uint8_t DNXServo::hibyte(const Type& num){
	return (uint8_t) (((uint16_t)num)>>8);
}

template<class T1, class T2>
inline uint16_t DNXServo::makeword(const T1& num1, const T2& num2){
	return ( ((uint16_t)num1 & 0x00ff) | ( ((uint16_t)(num2) & 0x00ff) << 8 ) );
}



#endif	
