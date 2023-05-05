#ifndef __TIMING_H__
#define __TIMING_H__

#include <pigpio.h>
#include <chrono>

void my_interrupt_handler(int gpio, int level, uint32_t tick);
int iscInit();

#endif // __TIMING_H__