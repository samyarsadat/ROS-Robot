#include "functions.h"
#include "pico/stdlib.h"
#include "FreeRTOS.h"
#include "task.h"
#include "time.h"



// POSIX microsecond delay function.
void usleep(uint64_t us)
{
    // If the delay is less than 1ms, use busy_wait_us instead of vTaskDelay.
	if (us < 1000)
    {
		busy_wait_us(us);
		return;
	}

    TickType_t ticks = pdMS_TO_TICKS(us / 1000);

    if (ticks < 1)
    {
	    ticks = 1;
    }

    vTaskDelay(ticks);
}


// POSIX get current time function.
int clock_gettime(clockid_t unused, struct timespec *tp)
{
    uint64_t m = time_us_64();
    tp->tv_sec = m / 1000000;
    tp->tv_nsec = (m % 1000000) * 1000;
    return 0;
}