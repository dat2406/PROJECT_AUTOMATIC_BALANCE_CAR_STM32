#ifndef _UTILS_H_
#define _UTILS_H_
#include "main.h"
#define CYCLES_PER_US (SystemCoreClock / 1000000UL)
#define DEG2RAD       0.0174532925f
static inline uint32_t micros(void){
    return DWT->CYCCNT / CYCLES_PER_US;
}
#endif
