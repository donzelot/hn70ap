#ifndef HN70AP_EEPROM_H
#define HN70AP_EEPROM_H

#include <stdint.h>
#include <stdbool.h>
#include <netinet/in.h>

#define TYPE_BOOL 1 /* 8 bools mapped in 1 byte */
#define TYPE_IP   2 /* 4 bytes */

/* Name of the EEPROM device */

#define Hn70AP_EECONFIG_DEVICE "/dev/eeprom"

/* Routines usable by all applications */

/* Initialize the EEPROM configuration management */
int hn70ap_eeconfig_init(void);
int hn70ap_eeconfig_describe(int index, char *namebuf, int namebuflen, uint32_t *type);
int hn70ap_eeconfig_getbool(char *name, bool *value);
int hn70ap_eeconfig_getip(char *name, struct in_addr *value);

#endif
