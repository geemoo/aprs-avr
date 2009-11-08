#include <stdint.h>
#include <avr/io.h>
#include "aprs.h"


int main(void) {

	// initialize our aprs engine
	aprs_init();

	// set the callsign we'll be using
	aprs_set_callsign("VE3DNI");

	// set the SSID we'll be using
	aprs_set_ssid(11);
	
	// send out our version packet
	aprs_send_version();

	// now send out a message to everyone
	aprs_send_msg("WIDE2", 2, "Hello World!");

	return 0;
}
