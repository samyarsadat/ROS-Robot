#include "functions.h"
#include "pico/stdlib.h"
#include "FreeRTOS.h"
#include "task.h"
#include <errno.h>



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


// Not a POSIX function, but a stub to temporarily silence
// a linker warning that occurs when using GCC 13.
int getentropy(void *buffer, size_t length)
{
    buffer = buffer; length = length;
    return -ENOSYS;
}