#ifndef HN70AP_EEPROM_H
#define HN70AP_EEPROM_H

#include <stdint.h>
#include <stdbool.h>
#include <netinet/in.h>

#define EECONFIG_TYPE_BOOL 1 /* 8 bools mapped in 1 byte */
#define EECONFIG_TYPE_IP   2 /* 4 bytes */

/* Name of the EEPROM device */

#define HN70AP_EECONFIG_DEVICE "/dev/eeprom"

/* Routines usable by all applications */

/* Initialize the EEPROM configuration management */
int hn70ap_eeconfig_init(bool *default_set);
int hn70ap_eeconfig_describe(int index, char *namebuf, int namebuflen, uint32_t *type);
int hn70ap_eeconfig_getbool(char *name, bool *value);
int hn70ap_eeconfig_getip(char *name, struct in_addr *value);

int hn70ap_eeprom_read (uint32_t addr, uint8_t *buf, uint32_t len);
int hn70ap_eeprom_write(uint32_t addr, uint8_t *buf, uint32_t len);

#endif
