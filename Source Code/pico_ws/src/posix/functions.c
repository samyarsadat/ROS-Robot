#include "functions.h"
#include "pico/stdlib.h"
#include "FreeRTOS.h"
#include "task.h"



// POSIX microsecond delay function.
int usleep(uint64_t us)
{
    vTaskDelay(pdMS_TO_TICKS(us / 1000 + (us % 1000 != 0)));
    return 0;
}


// POSIX get current time function.
int clock_gettime(clockid_t unused, struct timespec *tp)
{
    uint64_t m = time_us_64();
    tp->tv_sec = m / 1000000;
    tp->tv_nsec = (m % 1000000) * 1000;
    return 0;
}