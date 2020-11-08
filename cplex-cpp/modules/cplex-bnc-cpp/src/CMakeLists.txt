set(PROJECT_NAME "BnB_src2")

file(GLOB srcs "*.cpp"  "../include/*.h")

add_executable( ${PROJECT_NAME} ${srcs} )

target_include_directories( ${PROJECT_NAME} PRIVATE ${CPLEX_INCLUDE_DIRS})
target_link_libraries( ${PROJECT_NAME} PRIVATE ${CPLEX_LIBRARIES})
