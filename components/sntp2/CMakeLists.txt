set(app_sources 
	"sntp2.c"
	)

set(comp_includes
	"."
	)

idf_component_register(
		SRCS ${app_sources}
		INCLUDE_DIRS ${comp_includes}
#		REQUIRES nvs_flash spi_flash
		)