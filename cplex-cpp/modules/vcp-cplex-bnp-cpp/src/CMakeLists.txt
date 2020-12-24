set(PROJECT_NAME "branch_and_price_vertex_coloring_problem")

add_executable(${PROJECT_NAME}
        cplex_model.cpp
        cql_graph.cpp
        csv_writer.cpp
        local_search.cpp
        main.cpp
        shared.cpp
        vertex_coloring_solver.cpp
        ../include/vertex_coloring_solver.h
        ../include/shared.h
        ../include/utils.h
        ../include/csv_writer.h
        ../include/local_search.h
        ../include/cql_graph.h
        ../include/cplex_model.h
        ../include/constants.h main_cplex_model.cpp
        ../include/main_cplex_model.h
        slave_cplex_model.cpp
        ../include/slave_cplex_model.h)

target_include_directories(${PROJECT_NAME} PRIVATE ${CPLEX_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME} PRIVATE ${CPLEX_LIBRARIES})
