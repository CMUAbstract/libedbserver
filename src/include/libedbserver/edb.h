#ifndef LIBEDBSERVER_EDB_H
#define LIBEDBSERVER_EDB_H

/* @brief Initialize hardware and state for EDB functionality */
void edb_server_init();

/* @brief The app must call this periodically from the main loop */
void edb_service();

#endif // LIBEDBSERVER_EDB_H
