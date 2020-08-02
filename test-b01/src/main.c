/* GPIO Example
   This example code is in the Public Domain (or CC0 licensed, at your option.)
   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include <sys/time.h>

#define GPIO_OUTPUT_IO_0      22                               
#define GPIO_OUTPUT_PIN_SEL   ((1ULL<<GPIO_OUTPUT_IO_0))


void app_main()
{
    gpio_config_t io_conf;
    
    io_conf.pin_bit_mask = GPIO_OUTPUT_PIN_SEL;         //bit mask of the pins that you want to set,e.g.GPIO22
    io_conf.mode = GPIO_MODE_OUTPUT;                    //Configure in Output Mode
    io_conf.intr_type = GPIO_PIN_INTR_DISABLE;          //disable interrupt
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 0;
    gpio_config(&io_conf);                              //configure GPIO with the given settings

    bool status = 0;
    while(1) {
        gpio_set_level(GPIO_OUTPUT_IO_0, (uint32_t) status);
        status = !status;
        vTaskDelay(1000 / portTICK_RATE_MS);
    }
}

