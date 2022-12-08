/*
 * Licensed under the GNU General Public License version 2 with exceptions. See
 * LICENSE file in the project root for full license information
 */

 /** \file
 * \brief
 * ESC hardware specifoc EEPROM emulation functions.
 */

#ifndef __esc_hw__
#define __esc_hw__

#include <stdint.h>

#define SYNC0_PIN 17

#define SM2_sma          0x1100
#define SM3_sma          0x1D00

#define MAX_MAPPINGS_SM2 96
#define MAX_MAPPINGS_SM3 96

#define MAX_RXPDO_SIZE   116
#define MAX_TXPDO_SIZE   116



#define ESCREG_ADDRESS              0x0010
#define ESCREG_DLSTATUS             0x0110
#define ESCREG_ALCONTROL            0x0120
#define ESCREG_ALSTATUS             0x0130
#define ESCREG_ALERROR              0x0134
#define ESCREG_ALEVENTMASK          0x0204
#define ESCREG_ALEVENT              0x0220
#define ESCREG_ALEVENT_SM_MASK      0x0310
#define ESCREG_ALEVENT_SMCHANGE     0x0010
#define ESCREG_ALEVENT_CONTROL      0x0001
#define ESCREG_ALEVENT_DC_LATCH     0x0002
#define ESCREG_ALEVENT_DC_SYNC0     0x0004
#define ESCREG_ALEVENT_DC_SYNC1     0x0008
#define ESCREG_ALEVENT_EEP          0x0020
#define ESCREG_ALEVENT_WD           0x0040
#define ESCREG_ALEVENT_SM0          0x0100
#define ESCREG_ALEVENT_SM1          0x0200
#define ESCREG_ALEVENT_SM2          0x0400
#define ESCREG_ALEVENT_SM3          0x0800
#define ESCREG_WDSTATUS             0x0440
#define ESCREG_EECONTSTAT           0x0502
#define ESCREG_EEDATA               0x0508
#define ESCREG_SM0                  0x0800
#define ESCREG_SM0STATUS            (ESCREG_SM0 + 5)
#define ESCREG_SM0ACTIVATE          (ESCREG_SM0 + 6)
#define ESCREG_SM0PDI               (ESCREG_SM0 + 7)
#define ESCREG_SM1                  (ESCREG_SM0 + 0x08)
#define ESCREG_SM2                  (ESCREG_SM0 + 0x10)
#define ESCREG_SM3                  (ESCREG_SM0 + 0x18)
#define ESCREG_LOCALTIME            0x0910
#define ESCREG_LOCALTIME_OFFSET     0x0920
#define ESCREG_SYNC_ACT             0x0981
#define ESCREG_SYNC_ACT_ACTIVATED   0x01
#define ESCREG_SYNC_SYNC0_EN        0x02
#define ESCREG_SYNC_SYNC1_EN        0x04
#define ESCREG_SYNC_AUTO_ACTIVATED  0x08
#define ESCREG_SYNC0_CYCLE_TIME     0x09A0
#define ESCREG_SYNC1_CYCLE_TIME     0x09A4
#define ESCREG_SMENABLE_BIT         0x01
#define ESCREG_AL_STATEMASK         0x001f
#define ESCREG_AL_ALLBUTINITMASK    0x0e
#define ESCREG_AL_ERRACKMASK        0x0f



void ESC_write (uint16_t address, void *buf, uint16_t len);
void ESC_read (uint16_t address, void *buf, uint16_t len);
void ESC_init (void);
#endif
