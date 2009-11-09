#ifndef __AX25_H__
#define __AX25_H__ 1


/********************************************************************
* define some structures that we'll use */
struct ax25_info {
	uint8_t protocol_id;
};

/********************************************************************
* function declarations */
uint8_t ax25_init(struct ax25_info *);
uint8_t ax25_send_header(char *, uint8_t, char *, uint8_t);
uint8_t ax25_send_char(uint8_t);
uint8_t ax25_send_string(char *);
uint8_t ax25_send_footer(void);

#endif
