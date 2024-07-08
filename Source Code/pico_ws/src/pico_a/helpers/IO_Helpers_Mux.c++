/*
    The ROS robot project - IO Helper Module - Multiplexer
    Copyright 2022-2024 Samyar Sadat Akhavi
    Written by Samyar Sadat Akhavi, 2022-2024.
 
    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.
  
    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.
 
    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <https: www.gnu.org/licenses/>.
*/



// ------- Libraries & Modules -------
#include "IO_Helpers_Mux.h"
#include "local_helpers_lib/Local_Helpers.h"
#include "FreeRTOS.h"
#include "semphr.h"



// ------- Global variables -------
uint8_t mux_a, mux_b, mux_c, mux_d, mux_io;
SemaphoreHandle_t mux_io_mutex = NULL;



// ------- Functions ------- 

// ---- Sets the mux pins ----
void set_mux_pins(uint8_t addr_a, uint8_t addr_b, uint8_t addr_c, uint8_t addr_d, uint8_t io)
{
    mux_a = addr_a;
    mux_b = addr_b;
    mux_c = addr_c;
    mux_d = addr_d;
    mux_io = io;

    init_pin(mux_a, OUTPUT);
    init_pin(mux_b, OUTPUT);
    init_pin(mux_c, OUTPUT);
    init_pin(mux_d, OUTPUT);
}


// ---- Set mux address pins according to mux IO pin number ----
void set_mux_addr(uint pin)
{
    if (xSemaphoreGetMutexHolder(mux_io_mutex) == xTaskGetCurrentTaskHandle())
    {
        bool add_0, add_1, add_2, add_3;

        add_0 = (pin / 1) % 2;
        add_1 = (pin / 2) % 2;
        add_2 = (pin / 4) % 2;
        add_3 = (pin / 8) % 2;

        gpio_put(mux_a, add_0);
        gpio_put(mux_b, add_1);
        gpio_put(mux_c, add_2);
        gpio_put(mux_d, add_3);

        sleep_us(10);
    }
}


// ---- Set mux IO mode ----
void set_mux_io_mode(PIN_CONFIG_MODE mode)
{
    if (xSemaphoreGetMutexHolder(mux_io_mutex) == xTaskGetCurrentTaskHandle())
    {
        gpio_deinit(mux_io);
        init_pin(mux_io, mode);

        if (mode == INPUT_ADC)
        {
            check_bool(adc_select_input_with_mutex(get_gpio_adc_channel(mux_io)), RT_HARD_CHECK);
        }
    }
}


// ---- Take the multiplexer mutex ----
bool take_mux_mutex()
{
    if (mux_io_mutex == NULL)
    {
        mux_io_mutex = xSemaphoreCreateMutex();
        if (mux_io_mutex == NULL) { return false; }
    }
    
    return (xSemaphoreTake(mux_io_mutex, portMAX_DELAY) == pdTRUE);
}


// ---- Release the multiplexer mutex ----
void release_mux_mutex()
{
    if (xSemaphoreGetMutexHolder(mux_io_mutex) == xTaskGetCurrentTaskHandle())
    {
        xSemaphoreGive(mux_io_mutex);
    }
}