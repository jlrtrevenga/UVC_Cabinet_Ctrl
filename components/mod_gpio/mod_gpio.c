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
#include "mod_gpio.h"

static const char* TAG = "MOD_GPIO";
static xQueueHandle gpio_evt_queue = NULL;      //queue to send input "events"
TaskHandle_t  TaskHandle_mod_gpio;              // GPIO task handle
TaskHandle_t* pxTaskHandle_mod_gpio;            // GPIO task handle


#define BUTTON_DEBOUNCE_MILLIS 300
static int state_LR = 0;                                // 0-LOCAL / 1-REMOTE
static TickType_t button_Remote_ActualTick = 0;
static TickType_t button_Remote_PreviousTick = 0;

static int state_CMP_SEL = 0;                           // 0-None / 1-Lower / 2-Higher / 3-both
static TickType_t button_Cmp_sel_ActualTick = 0;
static TickType_t button_Cmp_sel_PreviousTick = 0;

static int state_RADIATE = 0;                           // 0-OFF / 1-ON
static TickType_t button_Radiate_ActualTick = 0;
static TickType_t button_Radiate_PreviousTick = 0;

//static int door_locked = 0;                           // 0-OPEN / 1-CLOSED



void gpio_init(void);
void button_REMOTE(void);
void button_CMP_SEL(void);
void button_RADIATE(void);


/****************************************************************************** 
* gpio_isr_handler
*******************************************************************************
* @brief gpio_isr_handler: receives input GPIO signals and sends to queue to be processed.
*******************************************************************************/
static void IRAM_ATTR gpio_isr_handler(void* arg) {
    uint32_t gpio_num = (uint32_t) arg;
    xQueueSendFromISR(gpio_evt_queue, &gpio_num, NULL);
}


/****************************************************************************** 
* gpio_task
*******************************************************************************
* @brief gpio_task: process GPIO input events received via gpio queue
*******************************************************************************/
static void gpio_task(void* arg) {
    uint32_t io_num;
    for(;;) {
        if(xQueueReceive(gpio_evt_queue, &io_num, portMAX_DELAY)) {
            printf("GPIO[%d] intr, val: %d\n", io_num, gpio_get_level(io_num));

            switch (io_num) {
                case GI_CMD_REMOTE:
                    ESP_LOGI(TAG, "GPIO[%d] (LOCAL/REMOTE) received, level: %d\n", io_num, gpio_get_level(io_num));
                    button_REMOTE();
                    break;

                case GI_CMD_CMP_SEL:
                    ESP_LOGI(TAG, "GPIO[%d] (CPM_SEL) received, level: %d\n", io_num, gpio_get_level(io_num));
                    button_CMP_SEL();
                    break;

                case GI_CMD_IRRADIATE:
                    ESP_LOGI(TAG, "GPIO[%d] (START/STOP) received, level: %d\n", io_num, gpio_get_level(io_num));
                    button_RADIATE();
                    break;

                default:
                    ESP_LOGI(TAG, "GPIO[%d] (Unknown) received, level: %d\n", io_num, gpio_get_level(io_num));
                    button_RADIATE();
                    break;
            }
        }
    }
}


/****************************************************************************** 
* gpio_task_create
*******************************************************************************
* @brief gpio_task_create: Creates gpio_isr_handler and gpio_task (listener) to process GPIO input events
* @return -> 0: OK / 1: FAIL   
*******************************************************************************/
int gpio_task_create(void) {
    int error = 0;
    gpio_config_t io_conf;

    io_conf.mode = GPIO_MODE_OUTPUT;                    //set as output mode
    io_conf.intr_type = GPIO_PIN_INTR_DISABLE;          //disable interrupt
    io_conf.pin_bit_mask = GPIO_OUTPUT_PIN_SEL;         //bit mask of the pins that you want to set
    io_conf.pull_down_en = 0;                           //disable pull-down mode
    io_conf.pull_up_en = 0;                             //disable pull-up mode
    gpio_config(&io_conf);                              //configure GPIO with the given settings

    io_conf.mode = GPIO_MODE_INPUT;                     //set as input mode    
    io_conf.pull_down_en = 0;                           //disable pull-down mode    
    io_conf.pull_up_en = 0;                           //enable pull-up mode
    io_conf.intr_type = GPIO_PIN_INTR_NEGEDGE;          //interrupt of falling edge
    io_conf.pin_bit_mask = GPIO_INPUT_PIN_SEL;          //bit mask of the pins, use GPIO4/5 here
    gpio_config(&io_conf);


    gpio_evt_queue = xQueueCreate(10, sizeof(uint32_t));                                    //create a queue to handle gpio event from isr
    //xTaskCreate(gpio_task, "gpio_task", 2048, NULL, 10, NULL);                            //start gpio task

    pxTaskHandle_mod_gpio = &TaskHandle_mod_gpio;
    if ( xTaskCreatePinnedToCore(gpio_task, "gpio_task", 2048 * 2, NULL, 5,
                                 pxTaskHandle_mod_gpio, APP_CPU_NUM) != pdPASS ) {    
        error = 1;
        ESP_LOGE(TAG, "GPIO Task creation failed");
        } 
    else { 
        error = 0;
        ESP_LOGI(TAG, "GPIO Task created");        
        gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);                                    //install gpio isr service
        gpio_isr_handler_add(GI_CMD_REMOTE  , gpio_isr_handler, (void*) GI_CMD_REMOTE);       //hook isr handler for specific gpio pin
        gpio_isr_handler_add(GI_CMD_CMP_SEL , gpio_isr_handler, (void*) GI_CMD_CMP_SEL);       //hook isr handler for specific gpio pin
        gpio_isr_handler_add(GI_CMD_IRRADIATE, gpio_isr_handler, (void*) GI_CMD_IRRADIATE);       //hook isr handler for specific gpio pin
        }
    
    gpio_init();
    return (error);
}


/****************************************************************************** 
* gpio_task_destroy
*******************************************************************************
* @brief gpio_task_destroy: Destroys gpio task listener
*******************************************************************************/
void gpio_task_destroy(void) {
    vTaskDelete(TaskHandle_mod_gpio);
}


/****************************************************************************** 
* Button LOCAL/REMOTE
*******************************************************************************
* @brief Change LOCAL -> REMOTE and REMOTE -> LOCAL
*******************************************************************************/
void button_REMOTE(void) {
    button_Remote_ActualTick = xTaskGetTickCount();
    ESP_LOGI(TAG, "Button REMOTE PRESSED"); 

    if ((button_Remote_ActualTick - button_Remote_PreviousTick) < (BUTTON_DEBOUNCE_MILLIS/portTICK_PERIOD_MS) ) {
            // debouncing, Filter button signals during BUTTON_DEBOUNCE_MILLIS ms.
    }
    else {
        button_Remote_PreviousTick = button_Remote_ActualTick;      //enable new input
        if (state_LR == 0) {state_LR = 1;}                          // Change from LOCAL  -> REMOTE
        else { state_LR = 0;}                                       // Change from REMOTE -> LOCAL
    }

    ESP_LOGI(TAG, "state_LR = %d", state_LR); 

    if (state_LR == 0){
        gpio_set_level(GO_FBK_LOCAL, ON);
        gpio_set_level(GO_FBK_REMOTE, OFF);            
    }
    else {
        gpio_set_level(GO_FBK_LOCAL, OFF);
        gpio_set_level(GO_FBK_REMOTE, ON);            
    }
}



/****************************************************************************** 
* Button CMP_SEL
*******************************************************************************
* @brief Change COMPARTIMENT SELECTION
*******************************************************************************/
void button_CMP_SEL(void) {
    ESP_LOGI(TAG, "Button COMPARTMENT_SELECTION PRESSED");     

    if (state_LR == 0 ){     // This button can only be operated when in LOCAL mode
        button_Cmp_sel_ActualTick = xTaskGetTickCount();

        if ((button_Cmp_sel_ActualTick - button_Cmp_sel_PreviousTick) < (BUTTON_DEBOUNCE_MILLIS/portTICK_PERIOD_MS) ) {
                // debouncing, Filter button signals during BUTTON_DEBOUNCE_MILLIS ms.
        }
        else {
            button_Cmp_sel_PreviousTick = button_Cmp_sel_ActualTick;     //enable new input
            if (state_CMP_SEL == 3) {state_CMP_SEL = 0;}                 // ReStart count
            else { state_CMP_SEL++;}                                     // INcrement count

            // 0-None / 1-Lower / 2-Higher / 3-both
            switch (state_CMP_SEL) {
                case 0:
                    gpio_set_level(GO_FBK_CMP01_SEL, OFF);
                    gpio_set_level(GO_FBK_CMP02_SEL, OFF);     
                    break;     

                case 1:
                    gpio_set_level(GO_FBK_CMP01_SEL, ON);
                    gpio_set_level(GO_FBK_CMP02_SEL, OFF);
                    break;

                case 2:
                    gpio_set_level(GO_FBK_CMP01_SEL, OFF);
                    gpio_set_level(GO_FBK_CMP02_SEL, ON);
                    break;
                    
                case 3:
                    gpio_set_level(GO_FBK_CMP01_SEL, ON);
                    gpio_set_level(GO_FBK_CMP02_SEL, ON);
                    break;
                }
            }    
    }   
}


/****************************************************************************** 
* Button RADIATE
*******************************************************************************
* @brief Change Status: ON <-> OFF
*******************************************************************************/
void button_RADIATE(void) {
    ESP_LOGI(TAG, "Button RADIATE PRESSED"); 

    if (state_LR == 0 ){     // This button can only be operated when in LOCAL mode
        button_Radiate_ActualTick = xTaskGetTickCount();
        if ((button_Radiate_ActualTick - button_Radiate_PreviousTick) < (BUTTON_DEBOUNCE_MILLIS/portTICK_PERIOD_MS) ) {
                // debouncing, Filter button signals during BUTTON_DEBOUNCE_MILLIS ms.
        }
        else {
            button_Radiate_PreviousTick = button_Radiate_ActualTick;      //enable new input
            if (state_RADIATE == 0) {state_RADIATE = 1;}                          // Change from ON <-> OFF
            else { state_RADIATE = 0;}                                       
        }

        ESP_LOGI(TAG, "state_RADIATE = %d", state_RADIATE); 
        ESP_LOGI(TAG, "GO_FBK_CMP01_SEL = %d", GO_FBK_CMP01_SEL); 
        ESP_LOGI(TAG, "GO_FBK_CMP02_SEL = %d", GO_FBK_CMP02_SEL); 


        if (state_RADIATE == 1){
            // 0-None / 1-Lower / 2-Higher / 3-both
            switch (state_CMP_SEL) {
                case 0:
                    gpio_set_level(GO_CMD_CMP01_IRRAD_ON, FS_OFF);
                    gpio_set_level(GO_CMD_CMP02_IRRAD_ON, FS_OFF);            
                    gpio_set_level(GO_FBK_IRRAD_ON, OFF);
                    gpio_set_level(GO_FBK_IRRAD_OFF, ON);
                    break;     

                case 1:
                    gpio_set_level(GO_CMD_CMP01_IRRAD_ON, FS_ON);
                    gpio_set_level(GO_CMD_CMP02_IRRAD_ON, FS_OFF);            
                    gpio_set_level(GO_FBK_IRRAD_ON, ON);
                    gpio_set_level(GO_FBK_IRRAD_OFF, OFF);
                    break;

                case 2:
                    gpio_set_level(GO_CMD_CMP01_IRRAD_ON, FS_OFF);
                    gpio_set_level(GO_CMD_CMP02_IRRAD_ON, FS_ON);            
                    gpio_set_level(GO_FBK_IRRAD_ON, ON);
                    gpio_set_level(GO_FBK_IRRAD_OFF, OFF); 
                    break;
                    
                case 3:
                    gpio_set_level(GO_CMD_CMP01_IRRAD_ON, FS_ON);
                    gpio_set_level(GO_CMD_CMP02_IRRAD_ON, FS_ON);            
                    gpio_set_level(GO_FBK_IRRAD_ON, ON);
                    gpio_set_level(GO_FBK_IRRAD_OFF, OFF);
                    break;

                default:
                    gpio_set_level(GO_CMD_CMP01_IRRAD_ON, OFF);
                    gpio_set_level(GO_CMD_CMP02_IRRAD_ON, OFF);            
                    gpio_set_level(GO_FBK_IRRAD_ON, OFF);
                    gpio_set_level(GO_FBK_IRRAD_OFF, ON);
                    break;
                }
        }   
        else {
            gpio_set_level(GO_CMD_CMP01_IRRAD_ON, FS_OFF);
            gpio_set_level(GO_CMD_CMP02_IRRAD_ON, FS_OFF);            
            gpio_set_level(GO_FBK_IRRAD_ON, OFF);
            gpio_set_level(GO_FBK_IRRAD_OFF, ON);        
        }
    }
}


/****************************************************************************** 
* GPIO INIT
*******************************************************************************
* @brief INTIALIZES outputs
*******************************************************************************/
void gpio_init(void) {
    state_LR = 0;                                // 0-LOCAL / 1-REMOTE
    state_CMP_SEL = 0;                           // 0-None / 1-Lower / 2-Higher / 3-both
    state_RADIATE = 0;                           // 0-OFF / 1-ON

    gpio_set_level(GO_FBK_POWER_ON, ON);
    gpio_set_level(GO_FBK_LOCAL, ON);
    gpio_set_level(GO_FBK_REMOTE, OFF);
    gpio_set_level(GO_FBK_CMP01_SEL, OFF);
    gpio_set_level(GO_FBK_CMP02_SEL, OFF);
    gpio_set_level(GO_FBK_IRRAD_ON, OFF);
    gpio_set_level(GO_FBK_IRRAD_OFF, ON);
    gpio_set_level(GO_CMD_CMP01_IRRAD_ON, FS_OFF);
    gpio_set_level(GO_CMD_CMP02_IRRAD_ON, FS_OFF); 
}


// Relays Control: 
//    gpio_set_level(GPIO_OUTPUT_02, ON);
//    gpio_isr_handler_remove(GPIO_INPUT_01);                                               //remove isr handler for gpio number.
//    gpio_isr_handler_add(GPIO_INPUT_01, gpio_isr_handler, (void*) GPIO_INPUT_01);       //hook isr handler for specific gpio pin again 




