#ifndef RFID_DECODER_H
#define RFID_DECODER_H

#include "rfid_protocol.h"

typedef void rfid_cmd_handler_t(rfid_cmd_code_t cmd_code);
typedef void rfid_rsp_handler_t(rfid_rsp_code_t rsp_code);

void rfid_decoder_init(rfid_cmd_handler_t *rfid_cmd_handler_cb,
                       rfid_rsp_handler_t *rfid_rsp_handler_cb);

void rfid_decoder_start();
void rfid_decoder_stop();

void rfid_decoder_tx_pin_isr();

#endif // RFID_DECODER_H
