#ifndef HN70AP_EEPROM_H
#define HN70AP_EEPROM_H

/* EEPROM configuration
 * Address length Default Name Type Desciption
 * 0       1                        Config validation marker. If FF, EEPROM is virgin, and default config is loaded
 * 1       1                        Network options bits 0..7
 *                1       dhcp bool Bit 0x01: DHCP Client Enable - board will request an IPv4 address via DHCP
 * 2-3                              Reserved for future use
 * 4-7                    ip   ip   Static IP address to use if DHCP is not enabled
 * 8-11                   mask ip   Static IP mask to use if DHCP is not enabled
 * 12-15                  gw   ip   IP address of default gateway
 * 16-19                  dns  ip   IP address of DNS server (will be used if set even if DHCP is enabled)
 * 20-127          
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
