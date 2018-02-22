#ifndef HN70AP_EEPROM_H
#define HN70AP_EEPROM_H

/* EEPROM configuration
 * Address length Desciption
 * 0       1      Config validation marker. If FF, EEPROM is virgin, default config is loaded
 * 0       1      Network options bits 0..7
 *                Bit 0x01: DHCP Client Enable - board will request an IPv4 address via DHCP
 * 1-127          Reserved for future use
 */

#define HN70AP_EECONFIG_VALID               0x00
#define HN70AP_EECONFIG_NETOPTIONS1         0x01
#define HN70AP_EECONFIG_NETOPTIONS1_DHCPCLI (1 << 0)

/* Name of the EEPROM device */
#define Hn70AP_EECONFIG_DEVICE "/dev/eeprom"

/* Routines usable by all applications */

/* Initialize the EEPROM configuration management */
int hn70ap_eeconfig_init(void);

#endif
