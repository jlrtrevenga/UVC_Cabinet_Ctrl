/* esp_event (event loop library) basic example
   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_system.h>
#include "esp_log.h"
#include "esp_event_base.h"
#include <esp_event.h>
#include "task_programmer01.h"
#include "mod_heater.h"
#include "sensor.h"

#define DELAY_1s             (pdMS_TO_TICKS( 1000))
#define DELAY_2s             (pdMS_TO_TICKS( 2000))
#define DELAY_5s             (pdMS_TO_TICKS( 5000))
#define EVENT_MAX_DELAY      (pdMS_TO_TICKS( 5000))

#define ACTIVE_PATTERN  2
#define LOOP_PERIOD     1000
#define DEBUG_MSG_LEVEL 1                           // 1:every minute / 2:every hour // 3: on event // 4: On error // >4: Never 

static const char* TAG = "HEATER_CTRL";             // Task identifier
ESP_EVENT_DEFINE_BASE(HEATER_EVENTS);               // Event source task related definitions

// Heater Control Loop Parameters, received via pvParameter when the loop task is created.
static heaterConfig_t heaterConfig;
static TaskHandle_t *pxHeaterTaskHandle;           

// internal functions
void heater_loop(void *pvParameter);
void heater_events_test_loop(void *pvParameter);
void heater_event_handler(void* handler_args, esp_event_base_t base, int32_t id, void* event_data);



/****************************************************************************** 
* heater_loop_start
*******************************************************************************
 * @brief Loads global variables, registers heater events, inits task programmer structures and 
 *        creates task that returns setpoint temperature according to program. 
 * @param[in] heaterConfig_t configuration values. 
*******************************************************************************/
int heater_loop_start(heaterConfig_t *config){

    // Init task global variables
    heaterConfig.task_name = config->task_name;
    heaterConfig.task_priority = config->task_priority;
    heaterConfig.task_stack_size = config->task_stack_size;
    heaterConfig.task_core_id = config->task_core_id;
    heaterConfig.event_loop_handle = config->event_loop_handle;
    heaterConfig.ulLoopPeriod = config->ulLoopPeriod;
    heaterConfig.active_pattern = config->active_pattern;
    heaterConfig.pxtemperature = config->pxtemperature;

    // Register events
    ESP_ERROR_CHECK(esp_event_handler_register_with(heaterConfig.event_loop_handle , HEATER_EVENTS, ESP_EVENT_ANY_ID, heater_event_handler, NULL));

    // Init task programmer structures and activate weekly pattern = 2      
    int error = 0;
    tp_init_structures();
    error = tp_activate_weekly_pattern(heaterConfig.active_pattern);
    //return(error);                                                      // TODO: send return value depending or correct execution
    
    if ( xTaskCreatePinnedToCore(&heater_loop, heaterConfig.task_name, heaterConfig.task_stack_size, 
                                (void*) &heaterConfig, heaterConfig.task_priority,
                                pxHeaterTaskHandle, heaterConfig.task_core_id) == pdPASS ) {         
        ESP_LOGI(TAG, "heater_ctrl task created");
        return(0);
        } 
    else { 
        ESP_LOGE(TAG, "heater_ctrl task creation failed");
        return(1);
    }                                                                 
}


/****************************************************************************** 
* heater_test_events
*******************************************************************************
 * @brief tests events: Creates task that periodically sends valid events
 * @return: 0:OK / 1: Task creation failed 
 *******************************************************************************/
int heater_test_events(void){
    if ( xTaskCreatePinnedToCore(&heater_events_test_loop, "heater_events_test_loop", 1024 * 2, 
                                NULL, 5, NULL, tskNO_AFFINITY) == pdPASS ) {
        ESP_LOGI(TAG, "heater_test created\r\n");
        return(0); 
    } else {
        ESP_LOGE(TAG, "heater_test creation failed");
        return(1);       
    }
}



/****************************************************************************** 
* heater_event_handler
*******************************************************************************
 * @brief reacts upon events. No event info included, to be improved.
 * *******************************************************************************/
void heater_event_handler(void* handler_args, esp_event_base_t base, int32_t id, void* event_data) 
{
    ESP_LOGI(TAG, "EVENT_HANDLER: Event received: %s:%d", base, id);

    switch (id) {

    case HEATER_EVENT_SETTING_UPDATE:                                   
   		//vTaskDelay(2000 / portTICK_PERIOD_MS);
        ESP_LOGI(TAG, "Event processed: HEATER_EVENT_SETTING_UPDATE");   
        break;

    case HEATER_EVENT_PROGRAM_UPDATE:
        //vTaskDelay(2000 / portTICK_PERIOD_MS);
        ESP_LOGI(TAG, "Event processed: HEATER_EVENT_PROGRAM_UPDATE");   
        break;


    case HEATER_EVENT_TEMP_UPDATE:                                      
        //vTaskDelay(2000 / portTICK_PERIOD_MS);
        ESP_LOGI(TAG, "Event processed: HEATER_EVENT_TEMP_UPDATE");   
        break;

    case HEATER_EVENT_TIME_UPDATE:
        //vTaskDelay(2000 / portTICK_PERIOD_MS);
        ESP_LOGI(TAG, "Event processed: HEATER_EVENT_TIME_UPDATE");   
        break;

    case HEATER_EVENT_HEATER_UPDATE:
        //vTaskDelay(2000 / portTICK_PERIOD_MS);
        ESP_LOGI(TAG, "Event processed: HEATER_EVENT_HEATER_UPDATE");   
        break;

    default:
        //vTaskDelay(2000 / portTICK_PERIOD_MS);
        ESP_LOGI(TAG, "Event processed: default");   
        break;    
    }

}


/****************************************************************************** 
* heater_loop
*******************************************************************************
 * @brief heaterConfig_t* pv -> Creates task loop that evaluates weekly temperature 
 *                              program and returns actual temperature_setpoint.
 *******************************************************************************/
void heater_loop(void *pvParameter) {
    heaterConfig_t* pxconfig_par = (heaterConfig_t*) pvParameter;
    heaterConfig_t config;
    config.ulLoopPeriod      = pxconfig_par->ulLoopPeriod;
    config.pxtemperature     = pxconfig_par->pxtemperature;
    config.event_loop_handle = pxconfig_par->event_loop_handle;

    // General variables definitions
    int error = 0;
    TickType_t tick; 
    time_t now;
    struct tm timeinfo;
    static bool override_active = false;
    static bool *poverride_active = &override_active;
    static int temperature = 0;
    static int *ptemperature = &temperature;
    static int override_temperature = 0;
    static int *poverride_temperature = &override_temperature; 

	for(;;) {
		//ESP_LOGI(TAG, "Periodic check, time based.");
		vTaskDelay(config.ulLoopPeriod / portTICK_PERIOD_MS);          // Definir cada minuto

        time(&now);
        localtime_r(&now, &timeinfo);

        // Select active program and setpoint temperature
        error = tp_get_target_value(now, poverride_active, poverride_temperature, ptemperature);

        tick = xTaskGetTickCount();  
        config.pxtemperature->tickTime = tick;  
        config.pxtemperature->value = (float) *ptemperature;

        // pxheater_ctrl_loop_params->pxtemperature->quality is calculated based on error value
        switch (error){
                    
            // No time transition => keep temperature reference
            case 0:
                config.pxtemperature->quality = GOOD_QUALITY;

                #if DEBUG_MSG_LEVEL == 1                    // minuto
                        ESP_LOGI(TAG, "TIME, Day: %d, %d:%d:%d -> Keep temperature %d ºC: ", 
                                timeinfo.tm_wday, timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec, temperature);    
                #endif

                #if DEBUG_MSG_LEVEL == 2                    // hora
                    if (timeinfo.tm_min == 0){
                        ESP_LOGI(TAG, "TIME, Day: %d, %d:%d:%d -> Keep temperature %d ºC: ", 
                                timeinfo.tm_wday, timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec, temperature);                    
                        } 
                #endif

                break;

            // Time transition => Set new temperarture reference and reset override temperature.
            case 1:
                config.pxtemperature->quality = GOOD_QUALITY;

                #if DEBUG_MSG_LEVEL <= 3                    // on event
                    ESP_LOGI(TAG, "TIME, Day: %d, %d:%d:%d -> New temperature Setpoint %d ºC: ", 
                            timeinfo.tm_wday, timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec, temperature);   
                #endif
                
                break;

            // Time transition  ERROR => Report error.
            default: 
                config.pxtemperature->quality = BAD_QUALITY;

                #if DEBUG_MSG_LEVEL <= 4                    // on event
                     ESP_LOGE(TAG, "TIME, Day: %d, Time: %d:%d:%d -> ERROR(%d) ----------------", 
                            timeinfo.tm_wday, timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec, error);                      
                #endif

                break;
        }
        ESP_ERROR_CHECK(esp_event_post_to(heaterConfig.event_loop_handle, HEATER_EVENTS, HEATER_EVENT_TEMP_SP_UPDATE, NULL, 0, DELAY_5s));
	}
}



//********************************************************************************************* 
//heater event test: Throws one event of each type to test it is received in an infinite loop
//*********************************************************************************************
void heater_events_test_loop(void *pvParameter)
{
//esp_err_t err;

	for(;;) {
		ESP_LOGI(TAG, ": Scheduled IN, start delay");

        ESP_ERROR_CHECK(esp_event_post_to(heaterConfig.event_loop_handle, HEATER_EVENTS, HEATER_EVENT_SETTING_UPDATE, NULL, 0, DELAY_5s));
		vTaskDelay(DELAY_5s);

        ESP_ERROR_CHECK(esp_event_post_to(heaterConfig.event_loop_handle, HEATER_EVENTS, HEATER_EVENT_PROGRAM_UPDATE, NULL, 0, DELAY_5s));
		vTaskDelay(DELAY_5s);

        ESP_ERROR_CHECK(esp_event_post_to(heaterConfig.event_loop_handle, HEATER_EVENTS, HEATER_EVENT_TEMP_UPDATE, NULL, 0, DELAY_5s));
		vTaskDelay(DELAY_5s);

        ESP_ERROR_CHECK(esp_event_post_to(heaterConfig.event_loop_handle, HEATER_EVENTS, HEATER_EVENT_TIME_UPDATE, NULL, 0, DELAY_5s));
		vTaskDelay(DELAY_5s);

        ESP_ERROR_CHECK(esp_event_post_to(heaterConfig.event_loop_handle, HEATER_EVENTS, HEATER_EVENT_HEATER_UPDATE, NULL, 0, DELAY_5s));
		vTaskDelay(DELAY_5s);
		}
}


/*
    ** EVENTS HADLING **********************************************************************
    1.- REGISTER THE HANDLER for each event, or group event ---------------------------------
    Ejemplo 1: Registro unico, todos los eventos se tratan por el mismo handler sin datos especificos asociados al evento
               ESP_ERROR_CHECK(esp_event_handler_register_with(event_loop_h, HEATER_EVENTS, ESP_EVENT_ANY_ID, heater_event_handler, NULL));
    Ejemplo 2: Cada evento tiene su propio handler y su propia estructura de datos
               ESP_ERROR_CHECK(esp_event_handler_register_with(event_loop_h, HEATER_EVENTS, HEATER_EVENT_01, event01_handler, event01_data));
               ESP_ERROR_CHECK(esp_event_handler_register_with(event_loop_h, HEATER_EVENTS, HEATER_EVENT_02, event02_handler, event02_data));

    2.- POST EVENT TO BE PROCESSED BY EVENT HANDLER ----------------------------------------- 
    // An external function shall invoke this function to post an event to be processed by events handler
    // Hay que controlar los errores y meterlo en un bucle en caso de que la cola esté llena y no pueda procesarse.
    //ESP_ERROR_CHECK(esp_event_post_to(loop_to_post_to, HEATER_EVENTS, HEATER_EVENT_00, NULL, 0, portMAX_DELAY));
    //ESP_ERROR_CHECK(esp_event_post_to(loop_to_post_to, HEATER_EVENTS, HEATER_EVENT_00, p_data_pointer, sizeof(p_data_pointer), portMAX_DELAY));    
    //ESP_ERROR_CHECK(esp_event_post_to(event_loop_h, HEATER_EVENTS, ESP_EVENT_xxxx, NULL, 0, portMAX_DELAY));
*/


