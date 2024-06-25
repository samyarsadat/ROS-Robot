## Note
`FreeRTOS + POSIX` is no longer used as MicroROS only needs two POSIX functions, and it's much more efficient
just to implement those manually elsewhere. I'm still going to leave the config file for `FreeRTOS + POSIX`
in case it is needed down the road, even though I doubt it will.