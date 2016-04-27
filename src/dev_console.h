#ifndef DEV_CONSOLE_H
#define DEV_CONSOLE_H

#ifdef CONFIG_DEV_CONSOLE
#include <libio/log.h>
#else
#define LOG(...)
#define PRINTF(...)
#endif

#endif // DEV_CONSOLE_H
