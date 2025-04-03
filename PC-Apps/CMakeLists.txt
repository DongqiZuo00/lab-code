project(Apps CXX)

#Common includes for all Apps:
include_directories(	
	${COMMON_DIR}
	${COMPONENTS_DIR}
)

add_subdirectory(TestApp)
add_subdirectory(Simulator)
add_subdirectory(TrajectoryGeneratorExample)