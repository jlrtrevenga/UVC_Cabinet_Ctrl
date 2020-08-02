
set(app_sources "mod_heater.c")

set(comp_includes 
    "." 
    )

idf_component_register(
	SRCS ${app_sources}
	INCLUDE_DIRS ${comp_includes}
	REQUIRES task_programmer01 sensor
	)