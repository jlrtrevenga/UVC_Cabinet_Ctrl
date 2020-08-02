/* UVC Cabinet Controller
   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
   
*/
#include <stdio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_system.h>
#include <esp_spi_flash.h>
#include <esp_log.h>
#include <esp_event_base.h>
#include <esp_event.h>
#include "esp_idf_lib_helpers.h"

#include "sensor.h"
#include "bmp280.h"
#include "wifi01.h"
#include "sntp2.h"
//#include "sensor.h"
//#include "driver/gpio.h"

#include "mod_bmp280.h"
#include "mod_heater.h"
#include "mod_mqtt.h"
#include "mqtt_client.h"
#include "mod_gpio.h"

#define DELAY_1s             (pdMS_TO_TICKS( 1000))
#define DELAY_2s             (pdMS_TO_TICKS( 2000))
#define DELAY_5s             (pdMS_TO_TICKS( 5000))
#define DELAY_60s             (pdMS_TO_TICKS( 60000))
#define DELAY_15m             (pdMS_TO_TICKS( 900000))
#define DELAY_1h             (pdMS_TO_TICKS( 3600000))


#define DEBUG_EVENTS 0              // 0:NO DEBUG , 1:DEBUG

//GPIO DEFINITION
//#define GPIO_INPUT_IO_0     12
//#define GPIO_INPUT_IO_1     14
//#define GPIO_INPUT_PIN_SEL  ((1ULL<<GPIO_INPUT_IO_0) | (1ULL<<GPIO_INPUT_IO_1))// I2C GPIO

//I2C GPIO
#define SDA_GPIO 21
#define SCL_GPIO 22

// heater weekly initial active pattern
#define ACTIVE_PATTERN  2
#define LOOP_PERIOD     10000
static const char* boardID = "home6534";

static const char* TAG = "UVC_CTRL";

// Events loop
esp_event_loop_handle_t event_loop_h;

// internal functions
int deadband_check(measure_t measure, measure_t setpoint, float deadband);


//***************************************************************************** 
//main task
//*****************************************************************************
void app_main()
{
    ESP_LOGI(TAG, "[APP] Startup..");
    ESP_LOGI(TAG, "[APP] Free memory: %d bytes", esp_get_free_heap_size());
    ESP_LOGI(TAG, "[APP] IDF version: %s", esp_get_idf_version());
    ESP_LOGI(TAG, "[APP] IDF version: %d.%d.%d", ESP_IDF_VERSION_MAJOR, ESP_IDF_VERSION_MINOR, ESP_IDF_VERSION_PATCH);


/*
typedef enum {
    ESP_LOG_NONE,       //!< No log output
    ESP_LOG_ERROR,      //!< Critical errors, software module can not recover on its own 
    ESP_LOG_WARN,       //!< Error conditions from which recovery measures have been taken
    ESP_LOG_INFO,       //!< Information messages which describe normal flow of events 
    ESP_LOG_DEBUG,      //!< Extra information which is not necessary for normal use (values, pointers, sizes, etc). 
    ESP_LOG_VERBOSE     //!< Bigger chunks of debugging information, or frequent messages which can potentially flood the output. 
} esp_log_level_t;
*/

    esp_log_level_set("BMP280_LOOP",        ESP_LOG_ERROR);
    esp_log_level_set("HEATER_CTRL",        ESP_LOG_ERROR);        // tiene compilacion condicional para errores
    esp_log_level_set("wifi",               ESP_LOG_INFO);
    esp_log_level_set("event",              ESP_LOG_INFO);
    esp_log_level_set("WIFI01",             ESP_LOG_INFO);
    esp_log_level_set("TASK_PROGRAMMER01",  ESP_LOG_ERROR);
    esp_log_level_set("MOD_MQTT",           ESP_LOG_INFO);
    esp_log_level_set("UVC_CTRL",           ESP_LOG_INFO);
    esp_log_level_set("MOD_GPIO",           ESP_LOG_INFO);    

    //esp_log_level_set("*", ESP_LOG_INFO);
    esp_log_level_set("MQTT_CLIENT", ESP_LOG_ERROR);
    esp_log_level_set("TRANSPORT_TCP", ESP_LOG_VERBOSE);
    esp_log_level_set("TRANSPORT_SSL", ESP_LOG_VERBOSE);
    esp_log_level_set("TRANSPORT", ESP_LOG_VERBOSE);
    esp_log_level_set("OUTBOX", ESP_LOG_VERBOSE);


	// time setting
    time_t now;
    struct tm timeinfo;
    time(&now);
    localtime_r(&now, &timeinfo);
    //char strftime_buf[64];

    // Configure timezone
    // TODO: Choose from screen/parameters
    // Set timezone to SPAIN Standard Time
    // TODO: Although it should be UTC+1, it seems we have to use UTC-1 (1 hour west from Grenwich, to make it work fine)
    setenv("TZ", "UTC-2,M3.5.0/2,M10.4.0/2", 1);
    tzset();

    // TODO: esto es prueba rapida, codificar bien mas adelante, sacar como var globales junto con todas las DOs
    static DO_t heater_command = {
        .value_actual = 0,
        .value_prev = 0
        };

    // 1.- COMMON SERVICES

    // COMMON EVENT LOOP
    esp_event_loop_args_t event_loop_args = {
        .queue_size = 5,
        .task_name = "event_loop_task",                 // task will be created (implicit)
        .task_priority = uxTaskPriorityGet(NULL),
        .task_stack_size = 2048,
        .task_core_id = tskNO_AFFINITY
    };
    ESP_ERROR_CHECK(esp_event_loop_create(&event_loop_args, &event_loop_h));
    ESP_LOGI(TAG, "event loop created");


    /*
    // COMMON I2C "services" (mutex access control)
    ESP_ERROR_CHECK(i2cdev_init());

	// Activate wifi and get time via NTP Server
    wifi_activate(true, true);

    // mqtt_Start. Requiere wifi activada y los servicios que aparecen abajo, que ya han sido activados para la wifi
    
    esp_err_t err = mqtt_app_start();
    if (err == ESP_FAIL) { ESP_LOGE(TAG, "mqtt_app_start() FAILED");}
    

    // 2.- SERVICES LOOPS
    
    // 2.1.- heater_ctrl task loop: Init "heater control loop parameters" and create task
    measure_t temperature_setpoint;                     // TODO: Sacarlo a IO_general
    heaterConfig_t heater_config = {
        .task_name = "heater_loop",                 // task will be created (implicit)
        .task_priority = uxTaskPriorityGet(NULL),
        .task_stack_size = 2048,
        .task_core_id = tskNO_AFFINITY,
        .event_loop_handle = event_loop_h,
        .ulLoopPeriod = LOOP_PERIOD,
        .active_pattern = ACTIVE_PATTERN,
        .pxtemperature = &temperature_setpoint       
    };
    ESP_ERROR_CHECK(heater_loop_start(&heater_config));

    // 2.2.- bmp280_ctrl task loop: Init "bmp280 control loop parameters" and create task
    BMP280_Measures_t BMP280_Measures;      // Values are updated in background by bmp280_control_loop
    BMP280_loop_params_t BMP280_loop_config = {
        .task_name = "bmp280_loop",                
        .task_priority = uxTaskPriorityGet(NULL),
        .task_stack_size = 2048,
        .task_core_id = tskNO_AFFINITY,        
        .event_loop_handle = event_loop_h,
        .ulLoopPeriod = LOOP_PERIOD,
        .sda_gpio = SDA_GPIO,
        .scl_gpio = SCL_GPIO,
        .pxBMP280_Measures = &BMP280_Measures      
    };
    ESP_ERROR_CHECK(bmp280_loop_start(&BMP280_loop_config));
    */

    // ENDLESS LOOP, REMOVE AND SUPRESS BY VALID CODE
    // COMPARES SETPOINT vs. MEASURED TEMPERATURE AND TRIGGERS COMMAND ON/OFF
    //TickType_t tick;
	
    ESP_LOGI(TAG, "ESP_ID: %s-------------------" ,boardID);    

    int result = gpio_task_create();
    ESP_LOGI(TAG, "gpio_task_create error: %d ", result); 


    for(;;) {
        TickType_t tick = xTaskGetTickCount();
		vTaskDelay(DELAY_60s);          // Definir cada minuto

        /*
        //Check last sync time
        time_t sync_time;
        struct tm sync_timeinfo;
        char strftime_sync_buf[64];
        sync_time = sntp_get_sync_time();
        localtime_r(&sync_time, &sync_timeinfo);
        strftime(strftime_sync_buf, sizeof(strftime_sync_buf), "%c", &sync_timeinfo);


        // Select temperature sensor that controls temperature and read room temperature
        // TODO, de momento me lo salto

        // Set mqtt msgs to send
        char room_tempVal[50], room_tempSp[50], heater_cmd[50], heater_stt[50];
        strcpy(room_tempVal, "/");
        strcat(room_tempVal, boardID);
        strncat(room_tempVal, "/room1/temp/value", 30);      

        strcpy(room_tempSp, "/");
        strcat(room_tempSp, boardID);
        strncat(room_tempSp, "/room1/temp/setpoint", 30);

        strcpy(heater_cmd, "/");
        strcat(heater_cmd, boardID);
        strncat(heater_cmd, "/room1/temp/heater_cmd", 30);

        strcpy(heater_stt, "/");
        strcat(heater_stt, boardID);
        strncat(heater_stt, "/room1/temp/heater_stt", 30);

        time_t now1;
        time(&now1);
        struct tm timeinfo;
        localtime_r(&now1, &timeinfo);

        int msg_id = 0;
        char valor[20]; 
        float deadband = 2;
        int result = deadband_check(BMP280_Measures.temperature, temperature_setpoint, deadband);
        switch (result){
            case 1:     // over db => SET HEATER OFF
                heater_command.value_prev = heater_command.value_actual;
                heater_command.value_actual = 0;
                if (heater_command.value_actual != heater_command.value_prev){
                    //gpio_set_level(GPIO_OUTPUT_01, OFF);                                        // GPIO_01 = BOILER 
                    ESP_LOGI(TAG, "%d / %d:%d SET HEATER OFF. Setpoint: %f, Temperature:%f ", 
                        timeinfo.tm_wday, timeinfo.tm_hour, timeinfo.tm_min,
                        temperature_setpoint.value, BMP280_Measures.temperature.value);

                    // MQTT Publish
            if (mqtt_connected() == ESP_OK){  
                        sprintf(valor, "%3.2f", BMP280_Measures.temperature.value);
                        msg_id = mqtt_client_publish(room_tempVal, valor, 0, 0, 0);
                        sprintf(valor, "%3.2f", temperature_setpoint.value);
                        msg_id = mqtt_client_publish(room_tempSp, valor, 0, 0, 0);
                        msg_id = mqtt_client_publish(heater_cmd , "OFF", 0, 0, 0);
                        if (heater_command.value_actual == 0) {strcpy(valor, "OFF");} else {strcpy(valor, "ON");};
                        msg_id = mqtt_client_publish(heater_stt, valor, 0, 0, 0);                        
                        }
                    }
                break;

            case 2:     // inside db => KEEP ACTUAL HEATER STATUS (ON OR OFF)
                heater_command.value_prev = heater_command.value_actual;
                //heater_command.value_actual = heater_command.value_actual;
                // TODO; Una vez que cambia, no cambiar en XXX segundos, a definir
                break;
            
            case 3:     // under db => SET HEATER ON
                heater_command.value_prev = heater_command.value_actual;
                heater_command.value_actual = 1;
                if (heater_command.value_actual != heater_command.value_prev){
                    //gpio_set_level(GPIO_OUTPUT_01, ON);                                        // GPIO_01 = BOILER                     
                    ESP_LOGI(TAG, "%d / %d:%d SET HEATER ON. Setpoint: %f, Temperature:%f ", 
                        timeinfo.tm_wday, timeinfo.tm_hour, timeinfo.tm_min,
                        temperature_setpoint.value, BMP280_Measures.temperature.value);

                    // MQTT Publish
            if (mqtt_connected() == ESP_OK){                  
                        sprintf(valor, "%3.2f", BMP280_Measures.temperature.value);
                        msg_id = mqtt_client_publish(room_tempVal, valor, 0, 0, 0);
                        sprintf(valor, "%3.2f", temperature_setpoint.value);
                        msg_id = mqtt_client_publish(room_tempSp, valor, 0, 0, 0);
                        msg_id = mqtt_client_publish(heater_cmd, "ON", 0, 0, 0);       
                        if (heater_command.value_actual == 0) {strcpy(valor, "OFF");} else {strcpy(valor, "ON");};
                        msg_id = mqtt_client_publish(heater_stt, valor, 0, 0, 0);
                        }
                    }
                break;

            // TODO: Casos en los que la calidad sea mala, decidir qué hacer
            // TODO: Convertir en caso general, dar mismo tratamiento a todas las salidas    
            }


        if ((timeinfo.tm_min%15) == 0){
            //ESP_LOGE(TAG, "tp_get_target_value: TIME ERROR - Localtime in Madrid (UTC-1,M3.5.0/2,M10.4.0/2) is: %s", strftime_buf);
            ESP_LOGI(TAG, "%d / %d:%d Trace........ Setpoint: %3.2f ºC (%d), Temperature:%3.2f %s (%d), last sync: %s", 
                    timeinfo.tm_wday, timeinfo.tm_hour, timeinfo.tm_min,
                    temperature_setpoint.value, temperature_setpoint.quality,
                    BMP280_Measures.temperature.value, BMP280_Measures.temperature.displayUnit, BMP280_Measures.temperature.quality, 
                    strftime_sync_buf);

            // MQTT Publish
            if (mqtt_connected() == ESP_OK){              
                sprintf(valor, "%3.2f", BMP280_Measures.temperature.value);
                msg_id = mqtt_client_publish(room_tempVal, valor, 0, 0, 0);
                sprintf(valor, "%3.2f", temperature_setpoint.value);
                msg_id = mqtt_client_publish(room_tempSp, valor, 0, 0, 0);
                if (heater_command.value_actual == 0) {strcpy(valor, "OFF");} else {strcpy(valor, "ON");};
                msg_id = mqtt_client_publish(heater_stt, valor, 0, 0, 0);
                }
            else {
                ESP_LOGI(TAG, "MQTT messaging not active");
            }    
            }
        */    
		}
}




// TODO- Eval also quality
/***************************************************************************** 
* deadband_check
******************************************************************************
 * @brief check if the value is under/inside/over deadband
 * @param[in] value
 * @param[in] deadband
 * @return: 1:over db / 2:Inside db / 3:under db
 */
int deadband_check(measure_t measure, measure_t setpoint, float deadband){   
    int output = 2;
    if (measure.value > (setpoint.value + deadband/2) )     {output = 1;}
    else if (measure.value < (setpoint.value - deadband/2)) {output = 3;}
    else {output = 2;}
    return (output);
}



