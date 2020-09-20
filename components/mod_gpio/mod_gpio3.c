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

#include "mod_gpio3.h"

// NOTAS: 
// Modificación para que solo tenga boton de IRRADIAR, para aprovechar las placas que salieron mal
// Esto es solo una aplicacion temporal.


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
#define DELAY_TIME_1s          (1.0)       // Interval in seconds


void button_RADIATE(TickType_t TickTimePressed);
void gpio_test();


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
void gpio3_init(esp_event_loop_handle_t  event_loop_handle_par) {
    
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
    gpio_set_level(GO_FBK_IRRAD_ON, OFF);
    gpio_set_level(GO_FBK_IRRAD_OFF, ON);
    gpio_set_level(GO_CMD_CMP01_IRRAD_ON, FS_OFF);
    
     state_CMP_SEL = 1;     // 0-None / 1-Lower / 2-Higher / 3-both

    // Initilize event_loop_handle before it is used
    event_loop_handle = event_loop_handle_par;

    //register isr service and handler for each signal
    gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);                                    //install gpio isr service
    //gpio_isr_handler_add(GI_CMD_REMOTE,    gpio_isr_handler, (void*) GI_CMD_REMOTE);       //hook isr handler for specific gpio pin
    //gpio_isr_handler_add(GI_CMD_CMP_SEL,   gpio_isr_handler, (void*) GI_CMD_CMP_SEL);       //hook isr handler for specific gpio pin
    gpio_isr_handler_add(GI_CMD_IRRADIATE, gpio_isr_handler, (void*) GI_CMD_IRRADIATE);       //hook isr handler for specific gpio pin

    //gpio_test();

    // Register events
    ESP_ERROR_CHECK(esp_event_handler_register_with(event_loop_handle, GPIO_EVENTS, ESP_EVENT_ANY_ID, gpio_event_handler, NULL));
}



/****************************************************************************** 
* GPIO INIT
*******************************************************************************
* @brief INTIALIZES outputs
*******************************************************************************/
void gpio_test(void) {
    state_LR = 0;                                // 0-LOCAL / 1-REMOTE
    //state_CMP_SEL = 0;                           // 0-None / 1-Lower / 2-Higher / 3-both
    state_RADIATE = 0;                           // 0-OFF / 1-ON

    gpio_set_level(GO_FBK_POWER_ON, OFF);
    gpio_set_level(GO_FBK_IRRAD_ON, OFF);
    gpio_set_level(GO_FBK_IRRAD_OFF, OFF);

    gpio_set_level(GO_FBK_POWER_ON, ON);
    vTaskDelay(pdMS_TO_TICKS(DELAY_TIME_1s * 1000));          // Start Delay Time
    gpio_set_level(GO_FBK_POWER_ON, OFF);

    gpio_set_level(GO_FBK_IRRAD_ON, ON);
    vTaskDelay(pdMS_TO_TICKS(DELAY_TIME_1s * 1000));          // Start Delay Time
    gpio_set_level(GO_FBK_IRRAD_ON, OFF);

    gpio_set_level(GO_FBK_IRRAD_OFF, ON);
    vTaskDelay(pdMS_TO_TICKS(DELAY_TIME_1s * 1000));          // Start Delay Time    
    gpio_set_level(GO_FBK_IRRAD_OFF, OFF);

    gpio_set_level(GO_FBK_POWER_ON, ON);
}




/****************************************************************************** 
* gpio_event_handler
*******************************************************************************
 * @brief processes gpio events.
 * *******************************************************************************/
void gpio_event_handler(void* handler_args, esp_event_base_t base, int32_t id, void* event_data) {
    ESP_LOGI(TAG, "EVENT_HANDLER: Event received: %s:%d", base, id);

    switch (id) {
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
    gpio_isr_handler_remove(GI_CMD_IRRADIATE);          
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

                if (state_RADIATE == 1) {                // 0-None / 1-Lower / 2-Higher / 3-both
                    switch (state_CMP_SEL) {
                        case 1:
                            vTaskDelay(pdMS_TO_TICKS(START_DELAY_TIME * 1000));          // Start Delay Time

                            gpio_set_level(GO_CMD_CMP01_IRRAD_ON, FS_ON);        
                            gpio_set_level(GO_FBK_IRRAD_ON, ON);
                            gpio_set_level(GO_FBK_IRRAD_OFF, OFF);
                            vTaskDelay(pdMS_TO_TICKS(RADIATION_TIME_CPM01 * 1000));          // Irradiation Time

                            gpio_set_level(GO_CMD_CMP01_IRRAD_ON, FS_OFF);
                            gpio_set_level(GO_FBK_IRRAD_ON, OFF);
                            gpio_set_level(GO_FBK_IRRAD_OFF, ON);                    
                            break;

                        default:
                            gpio_set_level(GO_CMD_CMP01_IRRAD_ON, FS_OFF);
                            gpio_set_level(GO_FBK_IRRAD_ON, OFF);
                            gpio_set_level(GO_FBK_IRRAD_OFF, ON);
                            break;
                        }
                }   
                else {
                    gpio_set_level(GO_CMD_CMP01_IRRAD_ON, FS_OFF);       
                    gpio_set_level(GO_FBK_IRRAD_ON, OFF);
                    gpio_set_level(GO_FBK_IRRAD_OFF, ON);        
                }
            }
            state_RADIATE = 0;      // TODO: Quitar cuando se programe bien, sólo debe ser llamada
        }
    }
}



