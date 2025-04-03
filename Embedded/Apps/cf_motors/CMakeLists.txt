include_directories(${CMAKE_SOURCE_DIR}/src/lib/Common/)
include_directories(${CMAKE_SOURCE_DIR}/src/lib/Components/)

px4_add_module(
	MODULE modules__cf_motors
	COMPILE_FLAGS
        -D_MICROCONTROLLER
	MAIN cf_motors
	STACK_MAIN 4000
	SRCS
		mainCFMotors.cpp
	DEPENDS
		lib__Common
		lib__Components
	)
# vim: set noet ft=cmake fenc=utf-8 ff=unix :
