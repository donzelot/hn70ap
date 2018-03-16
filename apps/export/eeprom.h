#ifndef HN70AP_EEPROM_H
#define HN70AP_EEPROM_H

#include <stdint.h>
#include <stdbool.h>
#include <netinet/in.h>

#define EECONFIG_TYPE_BOOL 1 /* 8 bools mapped in 1 byte */
#define EECONFIG_TYPE_IP   2 /* 4 bytes */
#define EECONFIG_TYPE_CALL 3 /* 8 bytes */
#define EECONFIG_TYPE_BYTE 4 /* 1 byte  */

/* Name of the EEPROM device */

#define HN70AP_EECONFIG_DEVICE "/dev/eeprom"

/* Routines usable by all applications */

/* Initialize the EEPROM configuration management */
int hn70ap_eeconfig_init(bool *default_set);
int hn70ap_eeconfig_find(char *name);
int hn70ap_eeconfig_describe(int index, char *namebuf, int namebuflen, uint32_t *type);

int hn70ap_eeconfig_getbool(char *name, bool *value);
int hn70ap_eeconfig_getip  (char *name, in_addr_t *value);
int hn70ap_eeconfig_getbyte(char *name, uint8_t *value);
int hn70ap_eeconfig_getcall(char *name, char *value);

int hn70ap_eeconfig_setbool(char *name, bool value);
int hn70ap_eeconfig_setip  (char *name, in_addr_t value);
int hn70ap_eeconfig_setbyte(char *name, uint8_t value);
int hn70ap_eeconfig_setcall(char *name, char *value);

int hn70ap_eeprom_read (uint32_t addr, uint8_t *buf, uint32_t len);
int hn70ap_eeprom_write(uint32_t addr, uint8_t *buf, uint32_t len);

#endif
