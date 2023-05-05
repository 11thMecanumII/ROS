#include "timing.h"

// Define a function that will be called at 50Hz
void my_interrupt_handler(int gpio, int level, uint32_t tick) {
    std::cout<<"tick: "<<tick<<"\n";
    std::cout<<"level: "<<level<<"\n";
    std::cout<<"gpio: "<<gpio<<"\n";
    std::cout<<"------------------\n";
    
}

int iscInit() {
    // Initialize pigpio
    gpioInitialise();
    
    // Configure the timer to generate interrupts at 50Hz
    // Assume the timer is called "timer0"
    int period_us = 20000; // Period in microseconds
    int frequency_hz = 50; // Frequency in Hz
    int prescaler = 192; // Prescaler value
    int timer_period = (int)(period_us * frequency_hz / 1000000.0 * prescaler);
    gpioSetTimerFunc(0, timer_period, my_interrupt_handler); // Set up the timer
    return 0;
}
