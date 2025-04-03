include_directories(${CMAKE_SOURCE_DIR}/src/lib/Common/)
include_directories(${CMAKE_SOURCE_DIR}/src/lib/Components/)

px4_add_module(
    MODULE modules__QuadcopterLogic
    COMPILE_FLAGS
        -D_MICROCONTROLLER
        -Wno-deprecated-copy
    MAIN quad
    STACK_MAIN 4000
    SRCS
        mainQuadcopterLogic.cpp
    DEPENDS
        lib__Common
        lib__Components
    )
# vim: set noet ft=cmake fenc=utf-8 ff=unix :
