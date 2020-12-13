set(PROJECT_NAME "branch_and_cut_max_clique_cplex")

file(GLOB srcs "*.cpp"  "../include/*.h")

add_executable( ${PROJECT_NAME} ${srcs} )

target_include_directories( ${PROJECT_NAME} PRIVATE ${CPLEX_INCLUDE_DIRS})
target_link_libraries( ${PROJECT_NAME} PRIVATE ${CPLEX_LIBRARIES})
