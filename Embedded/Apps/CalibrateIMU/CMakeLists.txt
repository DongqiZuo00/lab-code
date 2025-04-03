include_directories(${CMAKE_SOURCE_DIR}/src/lib/Common/)
include_directories(${CMAKE_SOURCE_DIR}/src/lib/Components/)

px4_add_module(
	MODULE modules__CalibrateIMU
	COMPILE_FLAGS
        -D_MICROCONTROLLER
	STACK_MAIN 3000
	MAIN calibrateIMU
	SRCS
		main.cpp
	DEPENDS
		lib__Common
		lib__Components
	)
# vim: set noet ft=cmake fenc=utf-8 ff=unix :
