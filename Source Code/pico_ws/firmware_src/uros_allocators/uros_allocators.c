/*
    The ROS robot project
    Custom MicroROS allocators for use with FreeRTOS.
    These override the default MicroROS allocators with the onesprovided by FreeRTOS.

    Copyright 2024 Samyar Sadat Akhavi
    Written by Samyar Sadat Akhavi, 2024.
 
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


#include "uros_allocators.h"
#include "pico/stdlib.h"
#include "FreeRTOS.h"
#include <string.h>
//#include <rcutils/allocator.h>



// Allocator functions
void *uros_rtos_allocate(size_t size, void *state)
{
    return (void *)pvPortMalloc(size);
}


void uros_rtos_deallocate(void *pointer, void *state)
{
    vPortFree(pointer);
}


void *uros_rtos_reallocate(void *pointer, size_t size, void *state)
{
    if (pointer == NULL)
    {
        return (void *)pvPortMalloc(size);
    } 
    
    else 
    {
        vPortFree(pointer);
        return (void *)pvPortMalloc(size);
    }
}


void *uros_rtos_zero_allocate(size_t number_of_elements, size_t size_of_element, void *state)
{
    void *res = (void *)pvPortMalloc(number_of_elements * size_of_element);
    return memset(res, 0, number_of_elements * size_of_element);   // memset() returns 'res' unmodified.
}