#ifndef __APRS_H__
#define __APRS_H__ 1

/*********************************************************************
* a bunch of defines to make our code a bit more readable */
#define CALLSIGN_LIMIT 8

/*********************************************************************
* aprs information  */
struct aprs_info
{
	char callsign[CALLSIGN_LIMIT+1];
	uint8_t callsign_size;
	uint8_t ssid;
};


/********************************************************************
* define our functions */
uint8_t aprs_init(void);
uint8_t aprs_set_callsign(char *);
uint8_t aprs_set_ssid(uint8_t);
uint8_t aprs_send_version(void);
uint8_t aprs_send_msg(char *, uint8_t, char *);

#endif
