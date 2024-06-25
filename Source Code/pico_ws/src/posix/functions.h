#pragma once
#include "pico/stdlib.h"
#include "time.h"

void usleep(uint64_t us);
int clock_gettime(clockid_t unused, struct timespec *tp);