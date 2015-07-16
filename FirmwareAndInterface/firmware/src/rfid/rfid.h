/*******************************************************************************
 * @file
 * @date            9 April 2015
 * @author          Graham Harvey
 * @brief           Definitions and prototypes for WISP RFID communication
 *                  using the WISP monitor.
 ******************************************************************************/

#ifndef RFID_H
#define RFID_H

#include <stdint.h>

void RFID_setup();
void RFID_start_event_stream();
void RFID_stop_event_stream();
void RFID_send_ready_rf_events_buf();

#endif
