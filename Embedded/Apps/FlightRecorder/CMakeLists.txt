include_directories(${CMAKE_SOURCE_DIR}/src/lib/Common/)
include_directories(${CMAKE_SOURCE_DIR}/src/lib/Components/)

px4_add_module(
	MODULE Apps__FlightRecorder
	COMPILE_FLAGS
        -D_MICROCONTROLLER
	MAIN flightrecorder
	SRCS
		mainFlightRecorder.cpp
	DEPENDS
		platforms__common
		lib__Common
		lib__Components
	)
# vim: set noet ft=cmake fenc=utf-8 ff=unix :
