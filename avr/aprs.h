#ifndef __APRS_H__
#define __APRS_H__ 1


// define our functions
uint8_t aprs_init(void);
uint8_t aprs_set_callsign(char *);
uint8_t aprs_set_ssid(uint8_t);
uint8_t aprs_broadcast_msg(char *);

#endif
