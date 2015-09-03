/*******************************************************************************
 * @file
 * @date            28 April 2015
 * @author          Graham Harvey
 * @brief           Useful macros for taking the min and the max of two values.
 ******************************************************************************/

#ifndef MINMAX_H
#define MINMAX_H

static inline uint16_t MIN(uint16_t a, uint16_t b) {
    return a < b ? a : b;
}
static inline uint16_t MAX(uint16_t a, uint16_t b) {
    return a > b ? a : b;
}

#endif // MINMAX_H
