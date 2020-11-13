set(PROJECT_NAME_1 "branch_and_bound_max_clique_cplex")

file(GLOB srcs "*.cpp"  "../include/*.h")

add_executable( ${PROJECT_NAME_1} ${srcs} ../include/configuration.h csv_writer.cpp ../include/csv_writer.h)

target_include_directories( ${PROJECT_NAME_1} PRIVATE ${CPLEX_INCLUDE_DIRS})
target_link_libraries( ${PROJECT_NAME_1} PRIVATE ${CPLEX_LIBRARIES})
