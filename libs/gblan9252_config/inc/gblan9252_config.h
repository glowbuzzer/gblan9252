/**
 ******************************************************************************
 * @file           :  gblan9252_config.h
 * @brief          :  source for GBLAN9252 config hash defines
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright 2022 Glowbuzzer.
 * All rights reserved.</center></h2>
 *
 ******************************************************************************
 */


#ifndef GBLAN9252_CONFIG_H
#define GBLAN9252_CONFIG_H

#include "gblan9252_config_autogen.h"


/** This defines the size of the stack in bytes allocated to the main ecrxtx thread */
#define STACK64K                                        (64 * 1024)


/*** *** SIZES & LENGTHS CONFIGURATION *** ***/

/* Defines for length of strings, buffers etc. */

/** Max length of string for gbc process name */
#define GBC_PROCESS_NAME_MAX_LENGTH                     100



#endif //GBLAN9252_CONFIG_H
