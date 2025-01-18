#pragma once
#include "pico/stdlib.h"
#include "time.h"

int usleep(uint64_t us);
int clock_gettime(clockid_t unused, struct timespec *tp);

// Not a POSIX function, but a stub to temporarily silence
// a linker warning that occurs when using GCC 13.
int getentropy(void *buffer, size_t length);