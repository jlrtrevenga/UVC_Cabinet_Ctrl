
set(app_sources "i2cdev.c")

set(comp_includes 
    "." 
    )

idf_component_register(
	SRCS ${app_sources}
	INCLUDE_DIRS ${comp_includes}
    	REQUIRES esp_idf_lib_helpers	
	)