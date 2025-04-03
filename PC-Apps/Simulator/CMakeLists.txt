project(Simulator CXX)

set(APPLICATION_NAME Simulator )

file(GLOB_RECURSE PROJECT_SOURCE 
	"*.cpp")

add_executable(${APPLICATION_NAME} ${PROJECT_SOURCE})
set_target_properties(${APPLICATION_NAME} PROPERTIES OUTPUT_NAME_DEBUG ${APPLICATION_NAME}D)
set_target_properties(${APPLICATION_NAME} PROPERTIES OUTPUT_NAME_RELEASE ${APPLICATION_NAME})

add_dependencies(${APPLICATION_NAME} Common Components)

target_link_libraries(${APPLICATION_NAME} 
	Common
	Components
)