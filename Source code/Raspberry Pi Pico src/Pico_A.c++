#include "lib/helpers.h"
#include "pico/stdlib.h"

int main() 
{
    init_pin(0, OUTPUT);
    
    while (true)
    {
        gpio_put(0, HIGH);
        sleep_ms(1000);
        gpio_put(0, LOW);
        sleep_ms(1000);
    }
}