
#include "wifi01.h"
#include "wifi_key.h"
#include "mod_mqtt.h"  //TODO MQTT

// Not required anymore, converted into parameters
//#define WIFI_AUTORECONNECT      true                          /* flag to configure wifi autoreconnection */
//#define SNTP_SYNC_ACTIVE        true                          /* flag to activate/deactivate sntp sync*/

//#define SNTP_UPDATE_DELAY      60000                          /* DELAY >= 15000 according to standard */ 
//#define SNTP_UPDATE_DELAY    3600000                          /* SNTP time update: 1h.    */
//#define SNTP_UPDATE_DELAY   43200000                          /* SNTP time update: 1 day */
#define SNTP_UPDATE_DELAY     21600000                          /* SNTP time update: 12h    */



//TODO: Learn how to check if the loop is already initialized and improve code
static bool wifi_event_loop_started = false;                // variable used to initialize the loop only the first time
static bool wifi_auto_reconnect = true;                 // reconnect if connection is lost (deactivate when stopping process)
static bool sntp_sync_active = true;                    // activate SNTP service update
static bool wifi_connected = false;                     // activated and deactivated by events


static void wifi_event_handler(void* arg, esp_event_base_t event_base,
                                int32_t event_id, void* event_data);

static const char *TAG = "WIFI01";


/****************************************************************************** 
* wifi activate
*******************************************************************************
 * @brief activates wifi in STA mode. 
 * @param[in] auto_reconnect = true -> Autoreconnect if connection is lost 
 * @param[in] sntp_sync = true -> connect to snmt servers and get time
*******************************************************************************/
void wifi_activate(bool auto_reconnect, bool sntp_sync)
{
    wifi_auto_reconnect = auto_reconnect;
    sntp_sync_active = sntp_sync;

    // Create these structures the first call. They are not deleted when wifi is stopped.
    if (!wifi_event_loop_started) {

        ESP_ERROR_CHECK(esp_event_loop_create_default());
        ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID,
                                               &wifi_event_handler, NULL));

        ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, ESP_EVENT_ANY_ID,
                                               &wifi_event_handler, NULL));
        //ESP_ERROR_CHECK( esp_event_loop_init(wifi_event_handler, NULL) );
        wifi_event_loop_started = true;

        ESP_ERROR_CHECK( nvs_flash_init() );        // Activate if WIFI_STORAGE_FLASH  is used (esp_wifi_set_storage)
    }

    // Check when IDF 4.1 is available
    // https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/network/esp_netif.html
    // and replace by esp_netif_init();

    tcpip_adapter_init(); 

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK( esp_wifi_init(&cfg) );
    ESP_ERROR_CHECK( esp_wifi_set_storage(WIFI_STORAGE_RAM) );

    // TODO: use parameters to send wifi usr and password
    static wifi_config_t wifi_config;
    strcpy(&wifi_config.sta.ssid,  &EXAMPLE_WIFI_SSID);
    strcpy(&wifi_config.sta.password, &EXAMPLE_WIFI_PASS);
    wifi_config.sta.bssid_set = 0;

    ESP_ERROR_CHECK( esp_wifi_set_mode(WIFI_MODE_STA) );
    ESP_ERROR_CHECK( esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config) );

    esp_err_t err = esp_wifi_start();
    ESP_LOGI(TAG, "esp_wifi_start err = %d ", err); 
    //esp_wifi_connect(); -> Called via event, when WIFI_EVENT_STA_START event is received.
}

/****************************************************************************** 
* wifi deactivate
*******************************************************************************
 * @brief deactivates wifi in STA mode. 
 * @brief SNTP service is also stopped.
*******************************************************************************/
void wifi_deactivate(void) {
    wifi_auto_reconnect = false;                //deactivate auto reconnection before stopping sntp
    sntp_stop();                                //validates internally if sntp service is started
    esp_err_t err = esp_wifi_stop();            //tcpip_adapter and DHCP server/client are automatically stopped.
    ESP_LOGI(TAG, "esp_wifi_stop err = %d ", err);   
}


/****************************************************************************** 
* wifi reconnect
*******************************************************************************
 * @brief reconnects wifi in case of disconnection 
 * @brief as when SYSTEM_EVENT_STA_DISCONNECTED is received.
 * @brief sntp must be disconnected after trying to reconnect wifi.
*******************************************************************************/
void wifi_reconnect(void){
    // bucle para reconnectar N veces
    // Corregir para lanzar un hilo y que sea independiente del proceso donde se gestiona el evento

    int multiplo = 60;
    //int retry_nbr_inc = 10;
    int retry_nbr_limit = 60;
    int retry_nbr = 0;

    do {
        ESP_LOGI(TAG, "wifi_reconnect, Retry nbr: %d ", ++retry_nbr);    
        esp_err_t error = esp_wifi_connect();
        if (error != 0) { ESP_LOGE(TAG, "wifi_reconnect, error: %d ", error);} 
        vTaskDelay(pdMS_TO_TICKS(multiplo * 1000));
        } 
    while (!wifi_connected || ((!wifi_connected) && (retry_nbr < retry_nbr_limit)));
}



/****************************************************************************** 
* sntp start
*******************************************************************************
 * @brief starts snmp service. 
*******************************************************************************/
void sntp_start(void)
{
    // TODO: Verify that wifi service is operative before calling it if the function is not static
    ESP_LOGI(TAG, "Initializing SNTP");
    // Before setting operation mode confirm sntp is not running,
    // as it crashes if operating mode is called when sntp is running.
    u8_t stat = sntp_enabled();
    if (stat == 0){
        sntp_setoperatingmode(SNTP_OPMODE_POLL);
        sntp_setservername(0, "pool.ntp.org");
        sntp_init();
        }
}

/****************************************************************************** 
* sntp stop
*******************************************************************************
 * Use sntp_stop() function, already exists.
*******************************************************************************/


/****************************************************************************** 
* wifi event handler
*******************************************************************************
 * some asyncrhonous wifi actions are handled though events, here: 
 *  Old version, to be replaced by wifi_event_handler2
 *  Check page when IDF4.1 is available
 * https://github.com/espressif/esp-idf/blob/7d75213/examples/wifi/getting_started/station/main/station_example_main.c
*******************************************************************************/
static void wifi_event_handler(void* arg, esp_event_base_t event_base,
                                int32_t event_id, void* event_data) {

    if (event_base == WIFI_EVENT){
        switch (event_id) {

            case WIFI_EVENT_STA_START:
                ESP_ERROR_CHECK( esp_wifi_connect() );
                ESP_LOGI(TAG, "Event: WIFI ----- WIFI_EVENT_STA_START -> Received");        
                break;

            case WIFI_EVENT_STA_CONNECTED:
                wifi_connected = true;
                ESP_LOGI(TAG, "Event: WIFI ----- WIFI_EVENT_STA_CONNECTED -> Received");
                break;

            case WIFI_EVENT_STA_DISCONNECTED:
                wifi_connected = false;
                ESP_LOGI(TAG, "Event: WIFI ----- WIFI_EVENT_STA_DISCONNECTED -> Received");
                // Workaround as ESP32 WiFi libs don't currently auto-reassociate.
                if (wifi_auto_reconnect) {
                    wifi_reconnect();
                }
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
                //ESP_LOGI(TAG, "Event: SYSTEM_EVENT_STA_GOT_IP: " IPSTR, IP2STR(&event->event_info.got_ip.ip_info.ip));
                ESP_LOGI(TAG, "Event: IP ----- IP_EVENT_STA_GOT_IP: ");
                if (sntp_sync_active){
                    sntp_start();
                    ESP_LOGI(TAG, "SNTP: Started");
                }
                break;

            case IP_EVENT_STA_LOST_IP:
                //ESP_LOGI(TAG, "Event: SYSTEM_EVENT_STA_GOT_IP: " IPSTR, IP2STR(&event->event_info.got_ip.ip_info.ip));
                ESP_LOGI(TAG, "Event: IP ----- IP_EVENT_STA_LOST_IP: ");                
                if (sntp_sync_active){
                    sntp_stop();
                    ESP_LOGI(TAG, "SNTP: Stopped");
                }
                break;

            default:
                ESP_LOGI(TAG, "Default Event: %d ", event_id);
                break;
            }
        }
}
