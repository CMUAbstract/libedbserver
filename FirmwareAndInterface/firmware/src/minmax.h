/*******************************************************************************
 * @file
 * @date            28 April 2015
 * @author          Graham Harvey
 * @brief           Useful macros for taking the min and the max of two values.
 ******************************************************************************/

#ifndef MINMAX_H
#define MINMAX_H

// type safe MIN, MAX from http://stackoverflow.com/questions/3437404/min-and-max-in-c
#define MIN(a, b) \
    ({ __typeof__ (a) _a = (a); \
       __typeof__ (b) _b = (b); \
       _a < _b ? _a : _b; })
#define MAX(a, b) \
    ({ __typeof__ (a) _a = (a); \
       __typeof__ (b) _b = (b); \
       _a > _b ? _a : _b; })

#endif // MINMAX_H
