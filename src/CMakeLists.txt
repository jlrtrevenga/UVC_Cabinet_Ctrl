#set(EXTRA_COMPONENT_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/../../components)
#set(EXTRA_COMPONENT_DIRS D:/Code/02-VS-ESP32/04-GitHub-repos/Uncle-Rus-esp-idf-lib/components)
#set(extra_component_dirs "./../components")

#set(EXTRA_COMPONENT_DIRS $ENV{IDF_PATH}/components/mqtt/esp-mqtt)

set(app_sources "main.c" )

set(comp_includes
	"."
	)

idf_component_register(
	SRCS ${app_sources}
	INCLUDE_DIRS ${comp_includes}	
    	REQUIRES bmp280 sensor mod_heater mod_mqtt
	)

