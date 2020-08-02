/* esp_event (event loop library) basic example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

#ifndef MOD_MQTT_H_
#define MOD_MQTT_H_

#include "mqtt_client.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
typedef struct {
esp_mqtt_client_handle_t client;
esp_err_t err;
} mqtt_client_par_t;
*/

/****************************************************************************** 
* mqtt_client_start
*******************************************************************************
 * @brief starts mqtt client. 
*******************************************************************************/
esp_err_t mqtt_app_start(void);

/****************************************************************************** 
* mqtt_client_start in active mqtt client
*******************************************************************************
 * @brief start mqtt client (typically force re-start after any incidence). 
*******************************************************************************/
int mqtt_client_start(void);

/****************************************************************************** 
* mqtt_client_stop in active mqtt client
*******************************************************************************
 * @brief stop mqtt client. 
*******************************************************************************/
int mqtt_client_stop(void);

/****************************************************************************** 
* mqtt_client_reconnect in active mqtt client
*******************************************************************************
 * @brief reconnects mqtt client. 
*******************************************************************************/
int mqtt_client_reconnect(void);


/****************************************************************************** 
* mqtt_client_publish in active mqtt client
*******************************************************************************
 * @brief publish in actual mqtt client. 
 * @param[in] *topic -> topic in which we write  
 * @param[in] *data -> published data 
 * @param[in] len   -> data lenght
 * @param[in] qos   -> quality of service (0,1,2)
 * @param[in] retain -> retain 
*******************************************************************************/
int mqtt_client_publish(const char *topic, const char *data, int len, int qos, int retain);


/****************************************************************************** 
* mqtt_client_subscribe in active mqtt client
*******************************************************************************
 * @brief subscribes on actual mqtt client. 
 * @param[in] *topic -> topic in which we write  
 * @param[in] qos   -> quality of service (0,1,2)
*******************************************************************************/
int mqtt_client_subscribe(const char *topic, int qos);


/****************************************************************************** 
* mqtt_client_unsubscribe in active mqtt client
*******************************************************************************
 * @brief unsubscribes on mqtt client. 
 * @param[in] *topic -> topic in which we write  
 * @param[in] qos   -> quality of service (0,1,2)
*******************************************************************************/
int mqtt_client_unubscribe(const char *topic, int qos);


/****************************************************************************** 
* mqtt_connected
*******************************************************************************
 * @brief returns mqtt connection status: ESP_OK / ESP_FAIL 
*******************************************************************************/
esp_err_t mqtt_connected();


#ifdef __cplusplus
}
#endif

#endif // #ifndef MOD_MQTT_H_

