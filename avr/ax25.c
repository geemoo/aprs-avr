#include <stdint.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include "ax25.h"


/********************************************************************
* some global variables this library will use */

// somewhere to store ax25 header info we don't want to hard code
static struct ax25_info ai;

/********************************************************************
* Interrupt routine for TIMER0 */
ISR(TIMER0_OVF_vect)
{

}

/********************************************************************
* Interrupt routine for ANA_COMP */
ISR(ANA_COMP_vect)
{

}

/********************************************************************
** ax25 frame format
1 		Flag (0x7e)
7 		Destination Address
7 		Source Address
0-56	Digipeater Addresses
1		Control Field (set to 0x03 for aprs)
1 		Protocol ID (set to 0xf0 for aprs)
1-256	Information Fields
2		FCS
1		Flag


1200Hz and 2200Hz tones are used to represent a 1 and 0 digital bits
http://en.wikipedia.org/wiki/Bell_202_modem


*/

/********************************************************************
* initialize some of our ax25 information */
uint8_t ax25_init(struct ax25_info *info)
{
	// copy all the fields to our internal info struct
	ai.protocol_id = info->protocol_id;

	return 1;
}


/********************************************************************
* send out the AX25 header */
uint8_t ax25_send_header(char *from, uint8_t fromssid, char *to, uint8_t tossid)
{
	// send the flag character 
	ax25_send_char(0x7e);
	
	// send the destination address

	// send the source address

	// send the control field
	ax25_send_char(0x03);

	// send the protocol ID
	ax25_send_char(ai.protocol_id);



	// send the flag character 
	ax25_send_char(0x7e);

	return 1;
}


uint8_t ax25_send_char(uint8_t ch)
{

	return 1;
}


/********************************************************************
* send out a string of characters */
uint8_t ax25_send_string(char *msg)
{

	// loop through the message until you get an "". send each one
	while (*msg) {
		ax25_send_char(*msg);
		msg++;
	}

	// return success now that we are done
	return 1;
}


uint8_t ax25_send_footer(void)
{

	return 1;
}
