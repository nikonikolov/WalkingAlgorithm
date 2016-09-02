#include "DNXServo.h"

/* ******************************** PUBLIC METHODS ************************************** */

DNXServo::DNXServo(PinName tx, PinName rx, int baudIn, const int ReturnLvlIn /*=1*/):
	port(new mbed::Serial(tx, rx)), baud(baudIn), bitPeriod(1000000.0/baudIn), ReturnLvl(ReturnLvlIn){
	
	// Set the baud rate of the port
	port->baud(baud);
}

DNXServo::~DNXServo(){
	delete port;
}

// SetID
int DNXServo::SetID(int ID, int newID){
    return dataPush(ID, DNXSERVO_ID, newID);
};

// Read Value from Control Table
int DNXServo::GetValue(int ID, int address){
	return dataPull(ID, address);
}




/* ******************************** END OF PUBLIC METHODS ************************************** */

/* ******************************** PROTECTED METHODS ************************************** */


// Clear input buffer
void DNXServo::flush() {	
	while (port->readable()) {
		port->getc();
	}
}


// Write buffer to servo 
void DNXServo::write(uint8_t* buf, int n) {
	for (int i=0; i < n; i++) {
		port->putc(buf[i]);
	}

	int i=0;

//	pc.print_debug("about to clear buf\n");
	while(i<n){
		if (port->readable()){	
			int inf = port->getc();		//empty buffer because tx has written to rx	(only in case of tx and rx connected)																
			i++;						//rate of the loop does not equal rate of communication
		}
	}

//	pc.print_debug("buf cleared\n");

/*
	int i=0;
	int timeout = 0; 	// Timeout
	//pc.print_debug("about to clear buf\n");

	while ((timeout < 2*n) && (i<n)) {
		if (port->readable()) {
			port->getc();			//empty buffer because tx has written to rx	(only in case of tx and rx connected)																
			i++;					//rate of the loop does not equal rate of communication
			timeout = 0;
		}
		else{
			wait_us(bitPeriod);																	
			timeout++;		
		}															
	}
	//pc.print_debug("buf cleared\n");
*/
}


// Read reply returns payload length, 0 if error.
int DNXServo::read(uint8_t* buf, int nMax /* =255 */) {			//check readBytesUntil()
	int n = 0; 		 	// Bytes read
	int timeout = 0; 	// Timeout

	while ((timeout < 16) && (n < nMax)) {
		if (port->readable()) {
			buf[n] = port->getc();
			n++;
			timeout = 0;
		}
		else{
			wait_ms(bitPeriod);																	
			timeout++;		
		}															
	}

	return n;
}


// CW - positive input, CCW - negative input
int DNXServo::angleScale(double angle){
	
	// 2.61799387799 rad = 150 degrees
	int result = 512 - ((angle/2.61799387799)*512);
	// 0 is end CW and 1024 is end CCW

	if (result>1024){
		pc.print_debug("CCW out of range\n");
		return 1024;	
	} 

	else if (result<0){
		pc.print_debug("CW out of range\n");
		return 0;	
	}  
	
	else return result;
}


// Length of address
int DNXServo::AddressLength(int address, const uint8_t * TWO_BYTE_ADDRESSES) {
	bool found=false;
	
	for(int i=0; i<11 && !found; i++){
		if (address == TWO_BYTE_ADDRESSES[i]) found=true;
	}
	
	if(found) return 2;
	else return 1;
}


// packetPrint
void DNXServo::packetPrint(int bytes, uint8_t* buf) {
	if(!pc.get_debug()) return;
	pc.print_debug("PACKET {");
	for (int i=0; i < bytes; i++) {
		pc.print_debug(to_hex(buf[i])+" ");
	}
	pc.print_debug(" } \n");
}



/* ******************************** END OF PROTECTED METHODS ************************************** */
