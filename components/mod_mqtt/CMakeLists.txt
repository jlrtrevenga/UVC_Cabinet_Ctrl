
#set(EXTRA_COMPONENT_DIRS $ENV{IDF_PATH}/components/mqtt/esp-mqtt)

idf_component_register( SRCS "mod_mqtt.c"
                        INCLUDE_DIRS "."
                        REQUIRES esp-tls)


#                        EMBED_TXTFILES "mqtt_eclipse_org.pem")

