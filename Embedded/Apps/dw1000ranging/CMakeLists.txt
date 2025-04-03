include_directories(${CMAKE_SOURCE_DIR}/src/lib/Common/)
include_directories(${CMAKE_SOURCE_DIR}/src/lib/Components/)

px4_add_module(
	MODULE modules__dw1000ranging
	COMPILE_FLAGS
        -D_MICROCONTROLLER
        -fpermissive
	MAIN ranging
	STACK_MAIN 2000
	SRCS
		rangingMain.cpp
		P2PRanging.cpp
		DW1000.cpp
		DW1000Time.cpp
	DEPENDS
		lib__Common
		lib__Components
	)
# vim: set noet ft=cmake fenc=utf-8 ff=unix :
