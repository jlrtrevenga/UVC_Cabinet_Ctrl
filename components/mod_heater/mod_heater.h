/* esp_event (event loop library) basic example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

#ifndef MOD_HEATER_H_
#define MOD_HEATER_H_

#include "esp_event.h"
#include "esp_event_loop.h"
#include "esp_timer.h"
#include "sensor.h"

#ifdef __cplusplus
extern "C" {
#endif

// Declarations for the event source
#define TASK_ITERATIONS_COUNT        10      // number of times the task iterates
#define TASK_PERIOD                2000      // period of the task loop in milliseconds

ESP_EVENT_DECLARE_BASE(HEATER_EVENTS);         // declaration of the task events family

// Heater status 
enum{
    NOT_OPERATIVE,
    OPERATIVE_WORKING,
    OPERATIVE_WAITING
} heater_status;

// Actions are based on objects and objects data. Events only inform to check data and act based on that.
// Events register may be made based only on BASE, without event_id. No object data is required as it will be read directly form objects.
enum {
    HEATER_EVENT_SETTING_UPDATE,            
    HEATER_EVENT_PROGRAM_UPDATE,            
    HEATER_EVENT_TEMP_UPDATE,               // Evaluate temperature and command heater relay. Plan for multi-thermostat option (only one controls heater)
    HEATER_EVENT_TEMP_SP_UPDATE,            // Temperature Setpoint update, notification to client program
    HEATER_EVENT_TIME_UPDATE,               
    HEATER_EVENT_HEATER_UPDATE              
} heater_events;

typedef struct {
    const char            * task_name;              // name of the event loop task; if NULL,
    UBaseType_t             task_priority;          // priority of the event loop task, ignored if task name is NULL 
    uint32_t                task_stack_size;        // stack size of the event loop task, ignored if task name is NULL
    BaseType_t              task_core_id;           // core to which the event loop task is pinned to,    
	uint32_t 	            ulLoopPeriod; 		    // loop period in ms.
	esp_event_loop_handle_t event_loop_handle; 		// event loop handler where events will be registered by heater_ctrl_loop
    measure_t             * pxtemperature;
    int                     active_pattern;         // weekly active pattern in use
	} heaterConfig_t;



/****************************************************************************** 
* heater_loop_start
*******************************************************************************
 * @brief Loads global variables, registers heater events, inits task programmer structures and 
 *        creates task that returns setpoint temperature according to program. 
 * @param[in] heaterConfig_t configuration values. 
*******************************************************************************/
int heater_loop_start(heaterConfig_t *config);


/****************************************************************************** 
* heater_test_events
*******************************************************************************
 * @brief tests events: Creates task that periodically sends valid events
 * @return: 0:OK / 1: Task creation failed 
 *******************************************************************************/
int heater_test_events(void);


#ifdef __cplusplus
}
#endif

#endif // #ifndef MOD_HEATER_H_



