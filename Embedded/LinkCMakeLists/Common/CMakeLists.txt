include_directories(${CMAKE_SOURCE_DIR}/src/lib/Common/)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -D_MICROCONTROLLER -Wno-deprecated-copy")
file(GLOB_RECURSE LIB_SOURCES
	Common/*.cpp
)
px4_add_library( lib__Common ${LIB_SOURCES})

# target_link_libraries(lib__Common PRIVATE platforms__common)
