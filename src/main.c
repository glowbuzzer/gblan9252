/**
 ******************************************************************************
 * @file           :  main.c
 * @brief          :
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2022 Glowbuzzer.
 * All rights reserved.</center></h2>
 *
 ******************************************************************************
 */


#include <stdio.h>
#include <stdlib.h>
#include "log.h"
#include "user_message.h"
#include "gberror.h"
#include "log.h"
#include "gblan9252_config.h"
#include "string.h"
#include "signal.h"
#include "std_utils.h"
#include <unistd.h>
#include "linux_shm.h"
#include "pid.h"
#include "dpm.h"
#include "bcm2835.h"
#include "esc_hw.h"
#include "std_defs_and_macros.h"

/** storage for gbc process name */
char proc_name[GBC_PROCESS_NAME_MAX_LENGTH] = {0};
struct shm_msg *shmp;
int gbc_pid = 0;

//key boolean indicating if GBC has a shared mem connection to GBLAN9252
bool gbc_connected = false;
bool esm_in_op = false;
uint8_t inA[SIZE_OF_GBC_PDO];
uint8_t inB[SIZE_OF_GBC_PDO];
uint8_t outA[SIZE_OF_GBC_PDO];
uint8_t outB[SIZE_OF_GBC_PDO];
int kill_rc = 0;

#define LAN9252_IRQ 1


#define MAX_TIME_WAIT_SYNC0 800000


uint8_t txpdo[MAX_TXPDO_SIZE];

uint8_t rxpdo[MAX_RXPDO_SIZE];


gberror_t ESC_wait_for_sync0(uint8_t pin) {
    uint32_t count = 0;
    bcm2835_gpio_set_eds(pin);//clear the previous event. makes sure that only next event is detected
    while ((!bcm2835_gpio_eds(pin)) && count < MAX_TIME_WAIT_SYNC0) {
        count++;
        delayMicroseconds(1);
    }//waits until next event/tick

    if (count == MAX_TIME_WAIT_SYNC0) {
        LL_ERROR(GBLAN9252_GEN_LOG_EN, "GBLAN9252: Timeout waiting sync0");
        return E_TIMEOOUT;
    }

    return E_SUCCESS;

}

//copy_to_shm(rxpdo, shmp->sm_buf_in);


void copy_to_shm(uint8_t *bufin, uint8_t *bufout) {

    //bufin=rxpdo
    //bufout = shm

    //start 0 - copy 76 bytes
    //start 76 - copy 40 bytes

    //on lan9252 we strip out joint vel and hoint torque (total of 40 bytes)

    memcpy(bufout, bufin, 76);
    memcpy(bufout + 76 + 40 + 40, bufin + 76, 40);

}

//copy_from_shm(shmp->sm_buf_out, txpdo);

void copy_from_shm(uint8_t *bufin, uint8_t *bufout) {

    //bufin=shm
    //bufout=txpo
    //start 0 - copy 76 bytes
    //start 76 - copy 40 bytes

    //on lan9252 we strip out joint vel and hoint torque (total of 40 bytes)

    memcpy(bufout, bufin, 76);
    memcpy(bufout + 76, bufin + 76 + 40 + 40, 40);

}


int main(void) {
    gberror_t rc = E_GENERAL_FAILURE;

    outA[0] = 34;
    outA[1] = 35;
    outA[2] = 36;
    outA[3] = 37;

    //solves missing output in debugger log output (a gdb thing)
    setbuf(stdout, 0);

    /* These set where the user message output will be sent. Uncomment the one you want.
     * This is just for user messages UM_* not logging LL_ which, if enabled, always goes to stdout
    */
    logger_set_stdout();
    //    logger_set_log_file("gbem.log", GBEM_UM_EN);
    //    logger_set_syslog("Glowbuzzer");



    UM_INFO(GBLAN9252_UM_EN, "GBLAN9252: **************************************************************************");
    UM_INFO(GBLAN9252_UM_EN, "GBLAN9252: ***                  Starting GB LAN9252 interface                     ***");
    UM_INFO(GBLAN9252_UM_EN, "GBLAN9252: **************************************************************************");


    ESC_init();


    char *username_buf;
    username_buf = (char *) malloc(11 * sizeof(char));
    memset(username_buf, 0, 11 * sizeof(char));
    if (getlogin_r(username_buf, 10) != 0) {
        strncpy(username_buf, "<unknown>", 10);
    }
    UM_INFO(GBLAN9252_UM_EN, "GBLAN9252: We are running as user [%s]", username_buf);


    if (SIGNAL_TO_SEND > 31) {
        UM_FATAL(
                "GBLAN9252: We have a signal number defined (with SIGNAL_TO_SEND) to a number greater than 31. This is outside the range of normal Linux signals. We must exit");
    }
    char *str = strdup(sys_siglist[SIGNAL_TO_SEND]);
    if (str) {
        upcase(str);
        UM_INFO(GBLAN9252_UM_EN,
                "GBLAN9252: We are using Linux signal [SIG %s] (we are sending this out to GBC to advance its cycle)",
                str);
        free(str);
    } else {
        UM_ERROR(GBLAN9252_UM_EN, "GNETX: Error matching the signal number [%u], to the standard Linux signals",
                 SIGNAL_TO_SEND);
    }

    gberror_t grc;

    strcpy(proc_name, GBC_PROCESS_NAME);
    grc = establish_shared_mem_and_signal_con(&shmp, proc_name, true, &gbc_pid, 1);
    if (grc != E_SUCCESS) {
        gbc_connected = false;
        UM_ERROR(GBLAN9252_UM_EN,
                 "GBLAN9252: Connection to shared memory & GBC process could not be established - we will continue without a connection to GBC");
    } else {
        gbc_connected = true;
        UM_INFO(GBLAN9252_UM_EN,
                "GBLAN9252: We have a connection to shared memory & GBC process >successfully< established ");
        UM_INFO(GBLAN9252_UM_EN,
                "GBLAN9252: Shared memory address [%p] (this is a virtual address so will not match across processes)",
                shmp);
        memset(shmp->sm_buf_in, 0, sizeof(uint8_t) * SHM_BUF_SIZE);
        memset(shmp->sm_buf_out, 0, sizeof(uint8_t) * SHM_BUF_SIZE);
    }


#pragma clang diagnostic push
#pragma ide diagnostic ignored "EndlessLoop"
    while (1) {

        uint16_t wd_status = 0;
        ESC_read(ESCREG_WDSTATUS, &wd_status, sizeof(wd_status));

        printf("wd [0x%x]\n");
        if ((wd_status & 0x01) == 0x01) {
            printf("watchdog is inactive or disabled\n");
        } else {
            printf("Watchdog is expired\n");
        }


        if (gbc_connected) {
            UM_INFO(GBLAN9252_UM_EN,
                    "GBLAN9252: GBC is successfully connected");
        } else {
            UM_INFO(GBLAN9252_UM_EN,
                    "GBLAN9252: GBC NOT connected");
        }


        uint32_t alstatus = 0;
        ESC_read(ESCREG_ALSTATUS, &alstatus, sizeof(alstatus));

#define ESM_INIT 0x01   // init
#define ESM_PREOP 0x02  // pre-operational
#define ESM_BOOT 0x03   // bootstrap
#define ESM_SAFEOP 0x04 // safe-operational
#define ESM_OP 0x08     // operational

        switch (alstatus & 0x0f) {
            case ESM_INIT:
                printf("init\n");
                esm_in_op = false;
                break;
            case ESM_PREOP:
                printf("preop\n");
                esm_in_op = false;
                break;
            case ESM_BOOT:
                printf("boot\n");
                esm_in_op = false;
                break;
            case ESM_SAFEOP:
                printf("safeop\n");
                esm_in_op = false;
                break;
            case ESM_OP:
                printf("op\n");
                esm_in_op = true;
                break;
        }


        if (gbc_connected && esm_in_op) {

            if (ESC_wait_for_sync0(SYNC0_PIN) == E_TIMEOOUT) {
                UM_ERROR(GBLAN9252_UM_EN, "GBLAN9252: Timeout waiting for sync0");
            } else {
                LL_TRACE(GBLAN9252_GEN_LOG_EN, "GBLAN9252: Sync0 received");

                ESC_read(SM2_sma, rxpdo, MAX_RXPDO_SIZE);

                copy_to_shm(rxpdo, shmp->sm_buf_in);

                copy_from_shm(shmp->sm_buf_out, txpdo);


                ESC_write(SM3_sma, txpdo, MAX_TXPDO_SIZE);

                printf("data0: %u, ", inA[0]);
                printf("data1: %u, ", inA[1]);
                printf("data2: %u, ", inA[2]);
                printf("data3: %u\n", inA[3]);


                /* send the signal to GBC to do the shared mem memcpy */
                kill_rc = kill((pid_t) gbc_pid, SIGNAL_TO_SEND);
            }
        }
    }
#pragma clang diagnostic pop


}

