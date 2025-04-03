include_directories(${CMAKE_SOURCE_DIR}/src/lib/Common/)
include_directories(${CMAKE_SOURCE_DIR}/src/lib/Components/)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -D_MICROCONTROLLER -Wno-deprecated-copy -Wno-error=float-equal")
px4_add_library( lib__Components
	Components/Logic/QuadcopterLogic.cpp
	Components/Logic/KalmanFilter6DOF.cpp
	)

target_link_libraries(lib__Components PRIVATE lib__Common)
