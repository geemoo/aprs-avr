#include <stdint.h>
#include "ax25.h"
#include "aprs.h"


static struct aprs_info ai;

/********************************************************************
* initialize our aprs engine */
uint8_t aprs_init()
{
	struct ax25_info ax25info;

	ax25info.protocol_id = 0xf0;

	// initialize our ax25 (can we call this a socket? endpoint?)
	ax25_init(&ax25info);

	// so far this doesn't really need to do anything
	return 1;
}


/********************************************************************
* set the callsign we'll be using */
uint8_t aprs_set_callsign(char *callsign)
{
	uint8_t i;
	char *ptr;

	// initialize our loop variables
	i = CALLSIGN_LIMIT - 1;
	ptr = callsign;

	// loop until we get to 8 characters
	while(i && *ptr) {
		// copy the characters
		*ai.callsign = *ptr;

		// increment our counter and pointer
		i--;
		ptr++;
	}
	
	// return success
	return 1;
}


/********************************************************************
* set the SSID we'll be using */
uint8_t aprs_set_ssid(uint8_t ssid)
{
	// copy the ssid
	ai.ssid = ssid;

	// return a successful outcome
	return 1;
}


/********************************************************************
* send out the version message */
uint8_t aprs_send_version(void)
{
	// send out the ax25 header
	ax25_send_header(ai.callsign, 0, "APDNI", 0);

	// send out the ax25 footer
	ax25_send_footer();

	return 1;
}


/********************************************************************
* now send out a message to everyone */
uint8_t aprs_send_msg(char *to, uint8_t tossid, char *msg)
{
	// send out the ax25 header
	ax25_send_header(ai.callsign, ai.ssid, to, tossid);

	// send out the message character
	ax25_send_char(':');

	// send the string they want to send
	ax25_send_string(msg);

	// now send the ax25 footer
	ax25_send_footer();

	return 1;
}
