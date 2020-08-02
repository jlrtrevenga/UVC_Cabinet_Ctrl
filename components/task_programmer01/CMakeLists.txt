set(app_sources
	"task_programmer01.c"
	)

set(include_dirs
	"."
	)

idf_component_register(
	SRCS ${app_sources}
	INCLUDE_DIRS ${include_dirs}
	)
