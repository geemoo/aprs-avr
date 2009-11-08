#include <avr/io.h>
#include "aprs.h"


int main(void) {

	// initialize our aprs engine
	aprs_init();

	// set the callsign we'll be using
	aprs_set_callsign("VE3DNI");

	// set the SSID we'll be using
	aprs_set_ssid(11);
	
	// now send out a message to everyone
	aprs_broadcast_msg("Hello World!");

	return 0;
}
