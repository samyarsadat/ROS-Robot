## Note
As noted in the `libfreertos` directory, I have decided not to use the POSIX extension for FreeRTOS
as MicroROS only needs two (`clock_gettime()` and `usleep()`) POSIX functions and it is much more
efficient to implement those two functions here.