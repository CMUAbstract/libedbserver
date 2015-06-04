/*******************************************************************************
 * @file
 * @date            28 April 2015
 * @author          Graham Harvey
 * @brief           Definitions and prototypes for the WISP monitor.
 ******************************************************************************/

#ifndef MONITOR_H
#define MONITOR_H

#include <msp430.h>

// type safe MIN, MAX from http://stackoverflow.com/questions/3437404/min-and-max-in-c
#define MIN(a, b) \
    ({ __typeof__ (a) _a = (a); \
       __typeof__ (b) _b = (b); \
       _a < _b ? _a : _b; })
#define MAX(a, b) \
    ({ __typeof__ (a) _a = (a); \
       __typeof__ (b) _b = (b); \
       _a > _b ? _a : _b; })

#ifndef NULL
#define NULL	0
#endif

#endif // MONITOR_H
