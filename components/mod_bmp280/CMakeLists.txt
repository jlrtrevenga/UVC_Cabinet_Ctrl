
set(app_sources "mod_bmp280.c")

set(comp_includes "." )

idf_component_register(
	SRCS ${app_sources}
	INCLUDE_DIRS ${comp_includes}	
    	REQUIRES i2cdev log esp_idf_lib_helpers bmp280 sensor
	)

