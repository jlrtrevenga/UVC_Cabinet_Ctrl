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
#include <esp_system.h>
#include <esp_log.h>
#include "esp_event_base.h"
#include <esp_event.h>
#include "driver/gpio.h"

#include "mod_gpio2.h"

// TODO - Esto pasa a librería de proceso
static xQueueHandle gpio_evt_queue = NULL;      //queue to send input "events"
TaskHandle_t  TaskHandle_mod_gpio;              // GPIO task handle
TaskHandle_t* pxTaskHandle_mod_gpio;            // GPIO task handle


#define BUTTON_DEBOUNCE_MILLIS 300          // Min time between consecutive rising edge events
#define BUTTON_ACCDELAY_MILLIS 2000         // Aceptable delay between button pressed and processed

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

#define RADIATION_TIME_CPM01  (60.0)       // Interval in seconds
#define RADIATION_TIME_CPM02  (60.0)       // Interval in seconds
#define START_DELAY_TIME      (10.0)       // Interval in seconds

void button_REMOTE(void);
void button_CMP_SEL(void);
void button_RADIATE(TickType_t TickTimePressed);
// TODO - Esto pasa a librería de proceso

static const char* TAG = "MOD_GPIO2";
ESP_EVENT_DEFINE_BASE(GPIO_EVENTS);               // Event source task related definitions

static esp_event_loop_handle_t  event_loop_handle; 	 // event loop handler where events will be registered by heater_ctrl_loop

// funciones internas
static void IRAM_ATTR gpio_isr_handler(void* arg);
void gpio_event_handler(void* handler_args, esp_event_base_t base, int32_t id, void* event_data);


/****************************************************************************** 
* gpio_isr_handler
*******************************************************************************
* @brief gpio_isr_handler: receives GPIO interruptions and generates events to be processed.
*******************************************************************************/
static void IRAM_ATTR gpio_isr_handler(void* arg) {
    uint32_t gpio_num = (uint32_t) arg;
    //xQueueSendFromISR(gpio_evt_queue, &gpio_num, NULL);

    switch (gpio_num) {
        case GI_CMD_REMOTE:
            // debounce: Filter button signals during BUTTON_DEBOUNCE_MILLIS ms.
            button_Remote_ActualTick = xTaskGetTickCount();            
            if ((button_Remote_ActualTick - button_Remote_PreviousTick) >= (BUTTON_DEBOUNCE_MILLIS/portTICK_PERIOD_MS) ) {
                    button_Remote_PreviousTick = button_Remote_ActualTick;
                    esp_event_isr_post_to(event_loop_handle, GPIO_EVENTS, BTN_REMOTE_PRESSED, NULL, 0, NULL);                    }
            break;

        case GI_CMD_CMP_SEL:
            // debounce: Filter button signals during BUTTON_DEBOUNCE_MILLIS ms.
            button_Cmp_sel_ActualTick = xTaskGetTickCount();            
            if ((button_Cmp_sel_ActualTick - button_Cmp_sel_PreviousTick) >= (BUTTON_DEBOUNCE_MILLIS/portTICK_PERIOD_MS) ) {
                    button_Cmp_sel_PreviousTick = button_Cmp_sel_ActualTick;
                    esp_event_isr_post_to(event_loop_handle, GPIO_EVENTS, BTN_CMP_SEL_PRESSED, NULL, 0, NULL);
                    }
            break;

        case GI_CMD_IRRADIATE:
            // debounce: Filter button signals during BUTTON_DEBOUNCE_MILLIS ms.
            button_Radiate_ActualTick = xTaskGetTickCount();
            if ((button_Radiate_ActualTick - button_Radiate_PreviousTick) >= (BUTTON_DEBOUNCE_MILLIS/portTICK_PERIOD_MS) ) {
                    button_Radiate_PreviousTick = button_Radiate_ActualTick;
                    esp_event_isr_post_to(event_loop_handle, GPIO_EVENTS, BTN_IRRADIATE_PRESSED, NULL, 0, NULL);
                    }
            break;
    }

}


/****************************************************************************** 
* GPIO INIT
*******************************************************************************
* @brief INTIALIZES outputs
* @param[in] esp_event_loop_handle_t: event_loop_handle where events are sent. 
*******************************************************************************/
void gpio2_init(esp_event_loop_handle_t  event_loop_handle_par) {
    
    //TODO - Pasa a la función de proceso
    state_LR = 0;                                // 0-LOCAL / 1-REMOTE
    state_CMP_SEL = 0;                           // 0-None / 1-Lower / 2-Higher / 3-both
    state_RADIATE = 0;                           // 0-OFF / 1-ON
    //TODO - Pasa a la función de proceso END

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

    //initialize signal values
    gpio_set_level(GO_FBK_POWER_ON, ON);
    gpio_set_level(GO_FBK_LOCAL, ON);
    gpio_set_level(GO_FBK_REMOTE, OFF);
    gpio_set_level(GO_FBK_CMP01_SEL, OFF);
    gpio_set_level(GO_FBK_CMP02_SEL, OFF);
    gpio_set_level(GO_FBK_IRRAD_ON, OFF);
    gpio_set_level(GO_FBK_IRRAD_OFF, ON);
    gpio_set_level(GO_CMD_CMP01_IRRAD_ON, FS_OFF);
    gpio_set_level(GO_CMD_CMP02_IRRAD_ON, FS_OFF); 

    // Initilize event_loop_handle before it is used
    event_loop_handle = event_loop_handle_par;

    //register isr service and handler for each signal
    gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);                                    //install gpio isr service
    gpio_isr_handler_add(GI_CMD_REMOTE,    gpio_isr_handler, (void*) GI_CMD_REMOTE);       //hook isr handler for specific gpio pin
    gpio_isr_handler_add(GI_CMD_CMP_SEL,   gpio_isr_handler, (void*) GI_CMD_CMP_SEL);       //hook isr handler for specific gpio pin
    gpio_isr_handler_add(GI_CMD_IRRADIATE, gpio_isr_handler, (void*) GI_CMD_IRRADIATE);       //hook isr handler for specific gpio pin

    // Register events
    ESP_ERROR_CHECK(esp_event_handler_register_with(event_loop_handle, GPIO_EVENTS, ESP_EVENT_ANY_ID, gpio_event_handler, NULL));
}


/****************************************************************************** 
* gpio_event_handler
*******************************************************************************
 * @brief processes gpio events.
 * *******************************************************************************/
void gpio_event_handler(void* handler_args, esp_event_base_t base, int32_t id, void* event_data) {
    ESP_LOGI(TAG, "EVENT_HANDLER: Event received: %s:%d", base, id);

    switch (id) {
        case BTN_REMOTE_PRESSED:
            button_REMOTE();                 
            break;

        case BTN_CMP_SEL_PRESSED:
            button_CMP_SEL();
            break;

        case BTN_IRRADIATE_PRESSED:
            button_RADIATE(button_Radiate_ActualTick);                  
            break;

        default:
            ESP_LOGI(TAG, "Event received, Unknown: %d\n", id);
            break;
    }
}



/****************************************************************************** 
* gpio_deinit
*******************************************************************************/
void gpio_deinit(void) {
    gpio_isr_handler_remove(GI_CMD_REMOTE); 
    gpio_isr_handler_remove(GI_CMD_CMP_SEL);
    gpio_isr_handler_remove(GI_CMD_IRRADIATE);          
}


/****************************************************************************** 
* Button LOCAL/REMOTE
*******************************************************************************
* @brief Change LOCAL -> REMOTE and REMOTE -> LOCAL
*******************************************************************************/
void button_REMOTE(void) {
    ESP_LOGI(TAG, "Button_REMOTE Processed"); 

    if (state_LR == 0) {state_LR = 1;}                          // Change from LOCAL  -> REMOTE
    else { state_LR = 0;}                                       // Change from REMOTE -> LOCAL

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
    ESP_LOGI(TAG, "Button COMPARTMENT_SELECTION Processed");     
    if (state_LR == 0 ){     // This button can only be operated when in LOCAL mode

        if (state_CMP_SEL == 3) {state_CMP_SEL = 0;}                 // ReStart count
        else { state_CMP_SEL++;}                                     // Increment count

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


/****************************************************************************** 
* Button RADIATE
*******************************************************************************
* @brief Change Status: ON <-> OFF
*******************************************************************************/
void button_RADIATE(TickType_t TickTimePressed) {
    ESP_LOGI(TAG, "Button RADIATE Processed"); 

    // Procesar solo si el retraso entre pulsación y proceso es inferior a BUTTON_ACCDELAY_MILLIS = 2 s.
    // Para que si se pulsa mientras está radiando, ignore estas pulsaciones.
    TickType_t ActualTick = xTaskGetTickCount();            
    if ((ActualTick - TickTimePressed) < (BUTTON_ACCDELAY_MILLIS/portTICK_PERIOD_MS) ) {

        if (state_LR == 0){                // if LOCAL mode active ...
            if (state_RADIATE == 0) {      // and no RADIATION PROCESS in process ...        
                state_RADIATE = 1;
                ESP_LOGI(TAG, "state_RADIATE = %d", state_RADIATE); 
                ESP_LOGI(TAG, "GO_FBK_CMP01_SEL = %d", GO_FBK_CMP01_SEL); 
                ESP_LOGI(TAG, "GO_FBK_CMP02_SEL = %d", GO_FBK_CMP02_SEL); 

                if (state_RADIATE == 1) {                // 0-None / 1-Lower / 2-Higher / 3-both
                    switch (state_CMP_SEL) {
                        case 0:
                            gpio_set_level(GO_CMD_CMP01_IRRAD_ON, FS_OFF);
                            gpio_set_level(GO_CMD_CMP02_IRRAD_ON, FS_OFF);            
                            gpio_set_level(GO_FBK_IRRAD_ON, OFF);
                            gpio_set_level(GO_FBK_IRRAD_OFF, ON);
                            break;     

                        case 1:
                            vTaskDelay(pdMS_TO_TICKS(START_DELAY_TIME * 1000));          // Start Delay Time
                            gpio_set_level(GO_CMD_CMP01_IRRAD_ON, FS_ON);
                            gpio_set_level(GO_CMD_CMP02_IRRAD_ON, FS_OFF);            
                            gpio_set_level(GO_FBK_IRRAD_ON, ON);
                            gpio_set_level(GO_FBK_IRRAD_OFF, OFF);

                            vTaskDelay(pdMS_TO_TICKS(RADIATION_TIME_CPM01 * 1000));          // Irradiation Time
                            gpio_set_level(GO_CMD_CMP01_IRRAD_ON, FS_OFF);
                            gpio_set_level(GO_CMD_CMP02_IRRAD_ON, FS_OFF);            
                            gpio_set_level(GO_FBK_IRRAD_ON, OFF);
                            gpio_set_level(GO_FBK_IRRAD_OFF, ON);                    
                            break;

                        case 2:
                            vTaskDelay(pdMS_TO_TICKS(START_DELAY_TIME * 1000));          // Start Delay Time
                            gpio_set_level(GO_CMD_CMP01_IRRAD_ON, FS_OFF);
                            gpio_set_level(GO_CMD_CMP02_IRRAD_ON, FS_ON);            
                            gpio_set_level(GO_FBK_IRRAD_ON, ON);
                            gpio_set_level(GO_FBK_IRRAD_OFF, OFF); 
                            
                            vTaskDelay(pdMS_TO_TICKS(RADIATION_TIME_CPM02 * 1000));          // Irradiation Time
                            gpio_set_level(GO_CMD_CMP01_IRRAD_ON, FS_OFF);
                            gpio_set_level(GO_CMD_CMP02_IRRAD_ON, FS_OFF);            
                            gpio_set_level(GO_FBK_IRRAD_ON, OFF);
                            gpio_set_level(GO_FBK_IRRAD_OFF, ON);                    
                            break;
                            
                        case 3:
                            vTaskDelay(pdMS_TO_TICKS(START_DELAY_TIME * 1000));          // Start Delay Time
                            gpio_set_level(GO_CMD_CMP01_IRRAD_ON, FS_ON);
                            gpio_set_level(GO_CMD_CMP02_IRRAD_ON, FS_ON);            
                            gpio_set_level(GO_FBK_IRRAD_ON, ON);
                            gpio_set_level(GO_FBK_IRRAD_OFF, OFF);

                            vTaskDelay(pdMS_TO_TICKS(RADIATION_TIME_CPM01 * 1000));          // Irradiation Time
                            gpio_set_level(GO_CMD_CMP01_IRRAD_ON, FS_OFF);
                            gpio_set_level(GO_CMD_CMP02_IRRAD_ON, FS_OFF);            
                            gpio_set_level(GO_FBK_IRRAD_ON, OFF);
                            gpio_set_level(GO_FBK_IRRAD_OFF, ON);                                        
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
            state_RADIATE = 0;      // TODO: Quitar cuando se programe bien, sólo debe ser llamada
        }
    }
}



// Relays Control: 
//    gpio_set_level(GPIO_OUTPUT_02, ON);
//    gpio_isr_handler_remove(GPIO_INPUT_01);                                               //remove isr handler for gpio number.
//    gpio_isr_handler_add(GPIO_INPUT_01, gpio_isr_handler, (void*) GPIO_INPUT_01);       //hook isr handler for specific gpio pin again 




