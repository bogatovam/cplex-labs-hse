set(PROJECT_NAME "branch_and_price_vertex_coloring_problem")

file(GLOB srcs "*.cpp"  "../include/*.h")

add_executable( ${PROJECT_NAME} ${srcs} )

target_include_directories( ${PROJECT_NAME} PRIVATE ${CPLEX_INCLUDE_DIRS})
target_link_libraries( ${PROJECT_NAME} PRIVATE ${CPLEX_LIBRARIES})
