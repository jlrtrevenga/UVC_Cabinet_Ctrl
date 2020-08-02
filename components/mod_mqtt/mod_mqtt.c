/* MQTT over SSL Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

#include <stdio.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include "esp_wifi.h"
#include "esp_system.h"
#include "esp_log.h"
//#include "nvs_flash.h"
#include "esp_event.h"
#include "esp_event_legacy.h"
#include "tcpip_adapter.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"
#include "lwip/sockets.h"
#include "lwip/dns.h"
#include "lwip/netdb.h"
#include "esp_tls.h"
#include "mod_mqtt.h"
#include "mqtt_client.h"


static const char *TAG = "MOD_MQTT";

static esp_mqtt_client_handle_t client;     //MQTT client handle
esp_mqtt_client_config_t mqtt_cfg;
static bool b_mqtt_connected = false;    

/*
#if CONFIG_BROKER_CERTIFICATE_OVERRIDDEN == 1
static const uint8_t mqtt_eclipse_org_pem_start[]  = "-----BEGIN CERTIFICATE-----\n" CONFIG_BROKER_CERTIFICATE_OVERRIDE "\n-----END CERTIFICATE-----";
#else
extern const uint8_t mqtt_eclipse_org_pem_start[]   asm("_binary_mqtt_eclipse_org_pem_start");
#endif
extern const uint8_t mqtt_eclipse_org_pem_end[]   asm("_binary_mqtt_eclipse_org_pem_end");
*/

//Internal functions
static void wifi_event_handler(void* arg, esp_event_base_t event_base,
                                int32_t event_id, void* event_data);

/****************************************************************************** 
* mqtt_event_handler_cb
*******************************************************************************
 * @brief mqtt event handler 
*******************************************************************************/
static esp_err_t mqtt_event_handler_cb(esp_mqtt_event_handle_t event)
{
    esp_mqtt_client_handle_t client = event->client;
    int msg_id;
    // your_context_t *context = event->context;
    switch (event->event_id) {
        case MQTT_EVENT_CONNECTED:
            b_mqtt_connected = true;
            ESP_LOGI(TAG, "MQTT_EVENT_CONNECTED");
            break;

        case MQTT_EVENT_DISCONNECTED:
            b_mqtt_connected = false;
            ESP_LOGI(TAG, "MQTT_EVENT_DISCONNECTED");
            break;

        case MQTT_EVENT_SUBSCRIBED:
            ESP_LOGI(TAG, "MQTT_EVENT_SUBSCRIBED, msg_id=%d", event->msg_id);
            break;

        case MQTT_EVENT_UNSUBSCRIBED:
            ESP_LOGI(TAG, "MQTT_EVENT_UNSUBSCRIBED, msg_id=%d", event->msg_id);
            break;

        case MQTT_EVENT_PUBLISHED:
            ESP_LOGI(TAG, "MQTT_EVENT_PUBLISHED, msg_id=%d", event->msg_id);
            break;

        case MQTT_EVENT_DATA:
            ESP_LOGV(TAG, "MQTT_EVENT_DATA");
            ESP_LOGV(TAG, "TOPIC=%.*s\r\n", event->topic_len, event->topic);
            ESP_LOGV(TAG, "TOPIC=%.*s\r\n", event->data_len, event->data);            
            //printf("TOPIC=%.*s\r\n", event->topic_len, event->topic);
            //printf("DATA=%.*s\r\n", event->data_len, event->data);
            break;


        case MQTT_EVENT_BEFORE_CONNECT:
            ESP_LOGV(TAG, "MQTT_EVENT_BEFORE_CONNECT received");        
            break;


        case MQTT_EVENT_ERROR:
            ESP_LOGI(TAG, "MQTT_EVENT_BEFORE_CONNECT");
            if (event->error_handle->error_type == MQTT_ERROR_TYPE_ESP_TLS) {
                ESP_LOGI(TAG, "Last error code reported from esp-tls: 0x%x", event->error_handle->esp_tls_last_esp_err);
                ESP_LOGI(TAG, "Last tls stack error number: 0x%x", event->error_handle->esp_tls_stack_err);
            } else if (event->error_handle->error_type == MQTT_ERROR_TYPE_CONNECTION_REFUSED) {
                ESP_LOGI(TAG, "Connection refused error: 0x%x", event->error_handle->connect_return_code);
            } else {
                ESP_LOGW(TAG, "Unknown error type: 0x%x", event->error_handle->error_type);
            }
            break;

        default:
            ESP_LOGI(TAG, "Other event id:%d", event->event_id);
            break;
    }
    return ESP_OK;
}



static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data) {
    ESP_LOGD(TAG, "Event dispatched from event loop base=%s, event_id=%d", base, event_id);
    mqtt_event_handler_cb(event_data);
}


/****************************************************************************** 
* mqtt_client_start
*******************************************************************************
 * @brief starts mqtt client. 
*******************************************************************************/
 esp_err_t mqtt_app_start(void){

/*
   const esp_mqtt_client_config_t mqtt_cfg = {
        .uri = CONFIG_BROKER_URI,
        .cert_pem = (const char *) mqtt_eclipse_org_pem_start,
    };
*/
  
/*    
   const esp_mqtt_client_config_t mqtt_cfg = {       
        .uri = CONFIG_BROKER_URI,
    };
*/
    // Inicializo mqtt_cfg como global y asigno aqui el valor porque de lo contrario da un core dump
    mqtt_cfg.uri = CONFIG_BROKER_URI;

    ESP_LOGI(TAG, "[APP] Free memory: %d bytes", esp_get_free_heap_size());
    ESP_LOGI(TAG, " %s ", mqtt_cfg.uri);

    client = esp_mqtt_client_init(&mqtt_cfg);       
    if (client == NULL) { 
        ESP_LOGE(TAG, "MQTT_client_init FAILED");
        return (ESP_FAIL);
        }
    else {
        // Register wifi events to disconnect and reconnect 
        esp_mqtt_client_register_event(client, ESP_EVENT_ANY_ID, mqtt_event_handler, client);
        ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID,
                                                &wifi_event_handler, NULL));
        ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, ESP_EVENT_ANY_ID,
                                                &wifi_event_handler, NULL));   

        //esp_mqtt_client_start(client);                                          // VER SI LO DEJO AQUI O AL LLEGAR EL EVENTO
        return (ESP_OK);
    }
}


/****************************************************************************** 
* mqtt_client_start in active mqtt client
*******************************************************************************
 * @brief start mqtt client (typically force re-start after any incidence). 
*******************************************************************************/
int mqtt_client_start(void){
    int msg_id = esp_mqtt_client_start(client);
    if (msg_id == ESP_OK) {ESP_LOGI(TAG, "MQTT Client Started");}
    else { ESP_LOGI(TAG, "MQTT Client Error: %d ", msg_id);}
    return(msg_id);    
}


/****************************************************************************** 
* mqtt_client_stop in active mqtt client
*******************************************************************************
 * @brief stop mqtt client. 
*******************************************************************************/
int mqtt_client_stop(void){
    int msg_id = esp_mqtt_client_stop(client);
    if (msg_id == ESP_OK) {ESP_LOGI(TAG, "MQTT Client Started");}
    else { ESP_LOGI(TAG, "MQTT Client Error: %d ", msg_id);}
    return(msg_id);    
}


/****************************************************************************** 
* mqtt_client_reconnect in active mqtt client
*******************************************************************************
 * @brief reconnects mqtt client. 
*******************************************************************************/
int mqtt_client_reconnect(void){
    int msg_id = esp_mqtt_client_reconnect(client);
    return(msg_id);    
}


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
int mqtt_client_publish(const char *topic, const char *data, int len, int qos, int retain){
    int msg_id = esp_mqtt_client_publish(client, topic, data, len, qos, retain);
    return(msg_id);
}


/****************************************************************************** 
* mqtt_client_subscribe in active mqtt client
*******************************************************************************
 * @brief subscribes on actual mqtt client. 
 * @param[in] *topic -> topic in which we write  
 * @param[in] qos   -> quality of service (0,1,2)
*******************************************************************************/
int mqtt_client_subscribe(const char *topic, int qos){
    int msg_id = esp_mqtt_client_subscribe(client, topic, qos);
    return(msg_id);    
}


/****************************************************************************** 
* mqtt_client_unsubscribe in active mqtt client
*******************************************************************************
 * @brief unsubscribes on mqtt client. 
 * @param[in] *topic -> topic in which we write  
 * @param[in] qos   -> quality of service (0,1,2)
*******************************************************************************/
int mqtt_client_unubscribe(const char *topic, int qos){
    int msg_id = esp_mqtt_client_unsubscribe(client, topic);
    return(msg_id);    
}


/****************************************************************************** 
* mqtt_connected
*******************************************************************************
 * @brief returns mqtt connection status: ESP_OK / ESP_FAIL 
*******************************************************************************/
esp_err_t mqtt_connected() {
    if (b_mqtt_connected) return (ESP_OK);
    else return(ESP_FAIL);
}


/****************************************************************************** 
* wifi event handler
*******************************************************************************
 * some asyncrhonous wifi actions are handled though events, here: 
 *  Old version, to be replaced by wifi_event_handler
 *  Check page when IDF4.1 is available
 * https://github.com/espressif/esp-idf/blob/7d75213/examples/wifi/getting_started/station/main/station_example_main.c
*******************************************************************************/
static void wifi_event_handler(void* arg, esp_event_base_t event_base,
                                int32_t event_id, void* event_data) {

    if (event_base == WIFI_EVENT){
        switch (event_id) {

            case WIFI_EVENT_STA_START:
                ESP_LOGI(TAG, "Event: WIFI ----- WIFI_EVENT_STA_START -> Received");        
                break;

            case WIFI_EVENT_STA_CONNECTED:
                ESP_LOGI(TAG, "Event: WIFI ----- WIFI_EVENT_STA_CONNECTED -> Received");
                break;

            case WIFI_EVENT_STA_DISCONNECTED:
                ESP_LOGI(TAG, "Event: WIFI ----- WIFI_EVENT_STA_DISCONNECTED -> Received");
                break;

            case WIFI_EVENT_STA_STOP:
                ESP_LOGI(TAG, "Event: WIFI ----- WIFI_EVENT_STA_STOP -> Received");        
                break;

            default:
                ESP_LOGI(TAG, "Default Event: WIFI -----  %d ", event_id);
                break;
            }
        }

    if (event_base == IP_EVENT){
        switch (event_id) {

            case IP_EVENT_STA_GOT_IP:
                ESP_LOGI(TAG, "Event: IP ----- IP_EVENT_STA_GOT_IP: ");
                mqtt_client_start(); 
                break;

            case IP_EVENT_STA_LOST_IP:
                ESP_LOGI(TAG, "Event: IP ----- IP_EVENT_STA_LOST_IP: ");                
                mqtt_client_stop();
                break;
            }
        }
}


/*

msg_id = esp_mqtt_client_subscribe(client, "/topic/qos1", 1);
ESP_LOGI(TAG, "sent subscribe successful, msg_id=%d", msg_id);

msg_id = esp_mqtt_client_unsubscribe(client, "/topic/qos1");
ESP_LOGI(TAG, "sent unsubscribe successful, msg_id=%d", msg_id);

msg_id = mqtt_client_publish("/topic/qos0", "data_blue_whale_01", 0, 0, 0);

*/ 




