/* 
   This example code is in the Public Domain (or CC0 licensed, at your option.)
   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

#ifndef MOD_GPIO_H_
#define MOD_GPIO_H_

#include <esp_log.h>

#ifdef __cplusplus
extern "C" {
#endif


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

#define GI_CMD_REMOTE          39
#define GI_CMD_CMP_SEL         34
#define GI_CMD_IRRADIATE       35
#define GPIO_INPUT_PIN_SEL    ((1ULL<<GI_CMD_REMOTE)       | \
                               (1ULL<<GI_CMD_CMP_SEL)      | \
                               (1ULL<<GI_CMD_IRRADIATE))

#define ESP_INTR_FLAG_DEFAULT 0
#define ON  1
#define OFF 0
#define FS_ON  0
#define FS_OFF 1




/****************************************************************************** 
* gpio_task_create
*******************************************************************************
 * @brief gpio_task_create: Creates gpio_task (listener) and gpio_isr_handler to process GPIO input events
*******************************************************************************/
int gpio_task_create(void);


/****************************************************************************** 
* gpio_task_destroy
*******************************************************************************
* @brief gpio_task_destroy: Destroys gpio task listener
*******************************************************************************/
void gpio_task_destroy(void);



#ifdef __cplusplus
}
#endif

#endif // #ifndef BMP280_CTRL_LOOP_H_