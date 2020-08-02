/* esp_event (event loop library) basic example
   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_event_base.h"
#include "mod_bmp280.h"

#define TOLERABLE_REPEATED_READ_FAILS 3
#define LOOP_PERIOD     1000

static const char *TAG = "BMP280_LOOP";             // Task identifier
ESP_EVENT_DEFINE_BASE(BMP280_EVENTS);                // Event source task related definitions

// Control Loop Parameters, received via pvParameter when the loop task is created.
static BMP280_loop_params_t  bmp280Config;
static TaskHandle_t *pxBMP280TaskHandle;           
bmp280_t bmp280_dev;                            // BMP280 device descriptor
static bool bme280p;                            // 1: BMP-No pressure sensor / 0: BME-Pressure sensor

// internal functions
void bmp280_loop(void *pvParameter);
void bmp280_events_test_loop(void *pvParameter);
void bmp280_event_handler(void* handler_args, esp_event_base_t base, int32_t id, void* event_data);



/********************************************************************************************************************** 
* bmp280_loop_start
***********************************************************************************************************************
 * @brief initializes bmp280_loop variables, bms280 measure variables, inits bmp280 device and registers heater events. 
 * @param[in] BMP280_loop_params_t configuration values. 
**********************************************************************************************************************/
int bmp280_loop_start(BMP280_loop_params_t *config){

    // Init task global variables
    bmp280Config.task_name = config->task_name;
    bmp280Config.task_priority = config->task_priority;
    bmp280Config.task_stack_size = config->task_stack_size;
    bmp280Config.task_core_id = config->task_core_id;
    bmp280Config.event_loop_handle = config->event_loop_handle;
    bmp280Config.ulLoopPeriod      = config->ulLoopPeriod;
    bmp280Config.sda_gpio          = config->sda_gpio;
    bmp280Config.scl_gpio          = config->scl_gpio;
    bmp280Config.I2C_port          = config->I2C_port;
    bmp280Config.pxBMP280_Measures = config->pxBMP280_Measures;

    //Inicializo las medidas de los sensores del BMS280
    bmp280Config.pxBMP280_Measures->temperature.value = 0;
    bmp280Config.pxBMP280_Measures->temperature.quality = NOT_INIT;
    bmp280Config.pxBMP280_Measures->temperature.tickTime = 0;
    bmp280Config.pxBMP280_Measures->temperature.type = TEMPERATURE;
    strcpy(bmp280Config.pxBMP280_Measures->temperature.unit, "ºC");
    strcpy(bmp280Config.pxBMP280_Measures->temperature.displayUnit, "ºC");

    bmp280Config.pxBMP280_Measures->pressure.value = 0;
    bmp280Config.pxBMP280_Measures->pressure.quality = NOT_INIT;
    bmp280Config.pxBMP280_Measures->pressure.tickTime = 0;
    bmp280Config.pxBMP280_Measures->pressure.type = PRESSURE;
    strcpy(bmp280Config.pxBMP280_Measures->pressure.unit, "Pa");
    strcpy(bmp280Config.pxBMP280_Measures->pressure.displayUnit, "Pa");    

    bmp280Config.pxBMP280_Measures->humidity.value = 0;
    bmp280Config.pxBMP280_Measures->humidity.quality = NOT_INIT;
    bmp280Config.pxBMP280_Measures->humidity.tickTime = 0;
    bmp280Config.pxBMP280_Measures->humidity.type = HUMIDITY;
    strcpy(bmp280Config.pxBMP280_Measures->humidity.unit, "pct");
    strcpy(bmp280Config.pxBMP280_Measures->humidity.displayUnit, "pct");

    // Register events
    ESP_ERROR_CHECK(esp_event_handler_register_with(bmp280Config.event_loop_handle, BMP280_EVENTS, ESP_EVENT_ANY_ID, bmp280_event_handler, NULL));

    // initialize BMP280 
    bmp280_params_t params;
    memset(&bmp280_dev, 0, sizeof(bmp280_t));

    ESP_ERROR_CHECK(bmp280_init_desc(&bmp280_dev, BMP280_I2C_ADDRESS_0, 0, bmp280Config.sda_gpio, bmp280Config.scl_gpio));     // check if "port" should be received as parameter
    bmp280_init_forced_default_params(&params);  
    ESP_ERROR_CHECK(bmp280_init(&bmp280_dev, &params));

    bme280p = bmp280_dev.id == BME280_CHIP_ID;
    if (bme280p) { bmp280Config.pxBMP280_Measures->humidity.quality = NOT_INIT; }
            else { bmp280Config.pxBMP280_Measures->humidity.quality = NOT_CONFIGURED; }
    ESP_LOGI(TAG, "BMP280: found %s\n", bme280p ? "BME280" : "BMP280");

    if ( xTaskCreatePinnedToCore(&bmp280_loop, bmp280Config.task_name, bmp280Config.task_stack_size, 
                                (void*) &bmp280Config, bmp280Config.task_priority,
                                pxBMP280TaskHandle, bmp280Config.task_core_id) == pdPASS ) {         
        ESP_LOGI(TAG, "BMP280_ctrl_loop task created");
        return(0);
        } 
    else { 
        ESP_LOGE(TAG, "BMP280_ctrl_loop task creation failed");
        return(1);
    }
                                                            
}


/****************************************************************************** 
* bmp280_test_events
*******************************************************************************
 * @brief tests bmp280 events
 * @return: 0:OK / 1: Task creation failed / 2: heater not initialized
 *******************************************************************************/
int bmp280_test_events(void){
    if ( xTaskCreatePinnedToCore(&bmp280_events_test_loop, "bmp280_events_test_loop", 1024 * 2, 
                                 NULL, 5, NULL, APP_CPU_NUM) == pdPASS ) {
        ESP_LOGE(TAG, "bmp280_test creation failed\r\n");
        return(0);        
    } else {
    	ESP_LOGI(TAG, "bmp280_test created\r\n");
        return(1);        
    }
}

/****************************************************************************** 
* bmp280_loop
*******************************************************************************
 * @brief actual loop that  updates BMP280 measures values.
 *******************************************************************************/
void bmp280_loop(void *pvParameter) {

    ESP_LOGI(TAG, "Process Start. Periodic Loop Check.");
    float pressure, temperature, humidity;
    TickType_t tick;

    while (1) {
        vTaskDelay(bmp280Config.ulLoopPeriod / portTICK_PERIOD_MS);
        ESP_LOGI(TAG, "Periodic BMP280 read operation. Period: %d -------", bmp280Config.ulLoopPeriod);
        ESP_ERROR_CHECK(bmp280_force_measurement(&bmp280_dev));
        tick = xTaskGetTickCount();        

        if (bmp280_read_float(&bmp280_dev, &temperature, &pressure, &humidity) == ESP_OK) {
            bmp280Config.pxBMP280_Measures->temperature.value = temperature;
            bmp280Config.pxBMP280_Measures->temperature.quality = GOOD_QUALITY;
            bmp280Config.pxBMP280_Measures->temperature.tickTime = tick;

            bmp280Config.pxBMP280_Measures->pressure.value = pressure;
            bmp280Config.pxBMP280_Measures->pressure.quality = GOOD_QUALITY;
            bmp280Config.pxBMP280_Measures->pressure.tickTime = tick;            


            if (bme280p) {
                bmp280Config.pxBMP280_Measures->humidity.value = humidity;
                bmp280Config.pxBMP280_Measures->humidity.quality = GOOD_QUALITY;
                bmp280Config.pxBMP280_Measures->humidity.tickTime = tick;  
                }

            if (bme280p) { ESP_LOGI(TAG, "Pressure: %.2f Pa, Temperature: %.2f C, Humidity: %.2f ", pressure, temperature, humidity); } 
            else         { ESP_LOGI(TAG, "Pressure: %.2f Pa, Temperature: %.2f C, Humidity: NOT AVAILABLE", pressure, temperature); }
            }

        //READ FAIL: change quality, keep value and last READ tick time     
        else {
            ESP_LOGE(TAG, "Temperature/pressure reading failed");
            // More than TOLERABLE_REPEATED_READ_FAILS  => BAD_QUALITY
            if (tick > (bmp280Config.pxBMP280_Measures->temperature.tickTime + TOLERABLE_REPEATED_READ_FAILS * bmp280Config.ulLoopPeriod)) {  
                bmp280Config.pxBMP280_Measures->temperature.quality = BAD_QUALITY;
                bmp280Config.pxBMP280_Measures->pressure.quality = BAD_QUALITY;
                if (bme280p) {bmp280Config.pxBMP280_Measures->humidity.quality = BAD_QUALITY;}

                }
            // Still < TOLERABLE_REPEATED_READ_FAILS  => TOL _QUALITY    
            else {  
                bmp280Config.pxBMP280_Measures->temperature.quality = TOL_QUALITY;
                bmp280Config.pxBMP280_Measures->pressure.quality = TOL_QUALITY;
                if (bme280p) {bmp280Config.pxBMP280_Measures->humidity.quality = TOL_QUALITY;}
                }            
            }
        }
}



/****************************************************************************** 
* bmp280_loop event handler
*******************************************************************************
 * @brief implements loop control based on events
 *******************************************************************************/
void bmp280_event_handler(void* handler_args, esp_event_base_t base, int32_t id, void* event_data) 
{
    ESP_LOGI(TAG, "EVENT_HANDLER: Event received: %s:%d", base, id);

    switch (id) {

    case BMP280_CL_CREATE_LOOP:
   		//vTaskDelay(2000 / portTICK_PERIOD_MS);
        ESP_LOGI(TAG, "Event processed: BMP280_CL_CREATE_LOOP");   
        break;

    case BMP280_CL_KILL_LOOP:
        //vTaskDelay(2000 / portTICK_PERIOD_MS);
        ESP_LOGI(TAG, "Event processed: BMP280_CL_KILL_LOOP");   
        break;

    case BMP280_CL_START: 
        //vTaskDelay(2000 / portTICK_PERIOD_MS);
        ESP_LOGI(TAG, "Event processed: BMP280_CL_START");   
        break;

    case BMP280_CL_STOP:
        //vTaskDelay(2000 / portTICK_PERIOD_MS);
        ESP_LOGI(TAG, "Event processed: BMP280_CL_STOP");   
        break;

    case BMP280_CL_RESET:
        //vTaskDelay(2000 / portTICK_PERIOD_MS);
        ESP_LOGI(TAG, "Event processed: BMP280_CL_RESET");   
        break;

    case BMP280_CL_CHANGE_FREQ:
        //vTaskDelay(2000 / portTICK_PERIOD_MS);
        ESP_LOGI(TAG, "Event processed: BMP280_CL_CHANGE_FREQ");   
        break;

    default:
        //vTaskDelay(2000 / portTICK_PERIOD_MS);
        ESP_LOGI(TAG, "Event processed: BMP280_CL_default");   
        break;    
    }

}

//***************************************************************************** 
//bmp280 event test: Throws one event of each type to test it is received in an infinite loop
//*****************************************************************************
void bmp280_events_test_loop(void *pvParameter)
{
//esp_err_t err;

	for(;;) {
		ESP_LOGI(TAG, ": Scheduled IN, start delay");

        ESP_ERROR_CHECK(esp_event_post_to(bmp280Config.event_loop_handle, BMP280_EVENTS, BMP280_CL_CREATE_LOOP, NULL, 0, EVENT_MAX_DELAY));
		vTaskDelay(DELAY_5s);

        ESP_ERROR_CHECK(esp_event_post_to(bmp280Config.event_loop_handle, BMP280_EVENTS, BMP280_CL_KILL_LOOP, NULL, 0, EVENT_MAX_DELAY));
		vTaskDelay(DELAY_5s);

        ESP_ERROR_CHECK(esp_event_post_to(bmp280Config.event_loop_handle, BMP280_EVENTS, BMP280_CL_START, NULL, 0, EVENT_MAX_DELAY));
		vTaskDelay(DELAY_5s);

        ESP_ERROR_CHECK(esp_event_post_to(bmp280Config.event_loop_handle, BMP280_EVENTS, BMP280_CL_STOP, NULL, 0, EVENT_MAX_DELAY));
		vTaskDelay(DELAY_5s);

        ESP_ERROR_CHECK(esp_event_post_to(bmp280Config.event_loop_handle, BMP280_EVENTS, BMP280_CL_RESET, NULL, 0, EVENT_MAX_DELAY));
		vTaskDelay(DELAY_5s);

        ESP_ERROR_CHECK(esp_event_post_to(bmp280Config.event_loop_handle, BMP280_EVENTS, BMP280_CL_CHANGE_FREQ, NULL, 0, EVENT_MAX_DELAY));
		vTaskDelay(DELAY_5s);

		}
}


/*
    // An external function shall invoke this function to post an event to be processed by events handler
    // Hay que controlar los errores y meterlo en un bucle en caso de que la cola esté llena y no pueda procesarse.
    //ESP_ERROR_CHECK(esp_event_post_to(loop_to_post_to, BMP280_EVENTS, BMP280_EVENT_00, NULL, 0, portMAX_DELAY));
    //ESP_ERROR_CHECK(esp_event_post_to(loop_to_post_to, BMP280_EVENTS, BMP280_EVENT_00, p_data_pointer, sizeof(p_data_pointer), portMAX_DELAY));    
    //ESP_ERROR_CHECK(esp_event_post_to(event_loop_h, BMP280_EVENTS, ESP_EVENT_xxxx, NULL, 0, portMAX_DELAY));
*/

