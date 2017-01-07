#ifndef LIBEDBSERVER_EDB_H
#define LIBEDBSERVER_EDB_H

#include <libedb/target_comm.h>

/* @brief Initialize hardware and state for EDB functionality */
void edb_server_init();

/* @brief The app must call this periodically from the main loop */
void edb_service();

void enter_debug_mode(interrupt_type_t int_type, unsigned flags);
void exit_debug_mode();

#endif // LIBEDBSERVER_EDB_H
