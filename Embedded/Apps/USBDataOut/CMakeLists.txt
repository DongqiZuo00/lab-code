include_directories(${CMAKE_SOURCE_DIR}/src/lib/Common/)
include_directories(${CMAKE_SOURCE_DIR}/src/lib/Components/)

px4_add_module(
	MODULE Apps__USBDataOut
	COMPILE_FLAGS
        -D_MICROCONTROLLER
	MAIN usbDataOut
	STACK_MAIN 4000
	SRCS
		mainUSBOut.cpp
	DEPENDS
		platforms__common
		lib__Common
		lib__Components
	)
# vim: set noet ft=cmake fenc=utf-8 ff=unix :
