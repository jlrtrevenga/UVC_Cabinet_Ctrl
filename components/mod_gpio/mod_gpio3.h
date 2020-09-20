/* 
   This example code is in the Public Domain (or CC0 licensed, at your option.)
   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

#ifndef MOD_GPIO_H_
#define MOD_GPIO_H_

#include <esp_log.h>
#include "esp_event.h"
//#include "esp_event_loop.h"

#ifdef __cplusplus
extern "C" {
#endif

// Declarations for the event source
#define TASK_ITERATIONS_COUNT        10      // number of times the task iterates
#define TASK_PERIOD                2000      // period of the task loop in milliseconds

ESP_EVENT_DECLARE_BASE(GPIO_EVENTS);         // declaration of the task events family

// Actions are based on objects and objects data. Events only inform to check data and act based on that.
// Events register may be made based only on BASE, without event_id. No object data is required as it will be read directly form objects.
enum {
   BTN_REMOTE_PRESSED,
   BTN_CMP_SEL_PRESSED,
   BTN_IRRADIATE_PRESSED
   } gpio_events;

typedef struct {
    const char            * task_name;              // name of the event loop task; if NULL,
    UBaseType_t             task_priority;          // priority of the event loop task, ignored if task name is NULL 
    uint32_t                task_stack_size;        // stack size of the event loop task, ignored if task name is NULL
    BaseType_t              task_core_id;           // core to which the event loop task is pinned to,    
	 uint32_t 	             ulLoopPeriod; 		    // loop period in ms.
	 esp_event_loop_handle_t  event_loop_handle; 	 // event loop handler where events will be registered by heater_ctrl_loop
	} gpioConfig_t;



//GPIO OUTPUTS
/************************************
#define GO_FBK_POWER_ON       18
#define GO_FBK_LOCAL          13
#define GO_FBK_REMOTE         19
#define GO_FBK_CMP01_SEL      26
#define GO_FBK_CMP02_SEL      27
#define GO_FBK_IRRAD_ON       33
#define GO_FBK_IRRAD_OFF      25
#define GO_CMD_CMP01_IRRAD_ON 17
#define GO_CMD_CMP02_IRRAD_ON 16

#define GPIO_OUTPUT_PIN_SEL   ((1ULL<<GO_FBK_POWER_ON)       | \
                               (1ULL<<GO_FBK_LOCAL)          | \
                               (1ULL<<GO_FBK_REMOTE)         | \
                               (1ULL<<GO_FBK_CMP01_SEL)      | \
                               (1ULL<<GO_FBK_CMP02_SEL)      | \
                               (1ULL<<GO_FBK_IRRAD_ON)       | \
                               (1ULL<<GO_FBK_IRRAD_OFF)      | \
                               (1ULL<<GO_CMD_CMP01_IRRAD_ON) | \
                               (1ULL<<GO_CMD_CMP02_IRRAD_ON)) 
*************************************/

/*
#define GO_FBK_POWER_ON       18
#define GO_FBK_LOCAL          33 
#define GO_FBK_REMOTE         19
#define GO_FBK_CMP01_SEL      25
#define GO_FBK_CMP02_SEL      14    //39
#define GO_FBK_IRRAD_ON       15    //35
#define GO_FBK_IRRAD_OFF       4    //34
#define GO_CMD_CMP01_IRRAD_ON 17
#define GO_CMD_CMP02_IRRAD_ON 16
*/

#define GO_FBK_POWER_ON       18
#define GO_FBK_IRRAD_ON       33 
#define GO_FBK_IRRAD_OFF      19
#define GO_CMD_CMP01_IRRAD_ON 17


#define GPIO_OUTPUT_PIN_SEL   ((1ULL<<GO_FBK_POWER_ON)       | \
                               (1ULL<<GO_FBK_IRRAD_ON)       | \
                               (1ULL<<GO_FBK_IRRAD_OFF)      | \
                               (1ULL<<GO_CMD_CMP01_IRRAD_ON))                                

//GPIO INPUTS
/************************************
#define GI_CMD_REMOTE          39
#define GI_CMD_CMP_SEL         34
#define GI_CMD_IRRADIATE       35

#define GPIO_INPUT_PIN_SEL    ((1ULL<<GI_CMD_REMOTE)       | \
                               (1ULL<<GI_CMD_CMP_SEL)      | \
                               (1ULL<<GI_CMD_IRRADIATE))
************************************/

/*
#define GI_CMD_REMOTE          27
#define GI_CMD_CMP_SEL         13
#define GI_CMD_IRRADIATE       26
*/

#define GI_CMD_IRRADIATE       27

#define GPIO_INPUT_PIN_SEL    (1ULL<<GI_CMD_IRRADIATE)

#define ESP_INTR_FLAG_DEFAULT 0
#define ON  1
#define OFF 0
#define FS_ON  0
#define FS_OFF 1


void gpio3_init(esp_event_loop_handle_t  event_loop_handle_par);
void gpio_deinit(void);


#ifdef __cplusplus
}
#endif

#endif // #ifndef MOD_GPIO_H_