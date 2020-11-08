set(PROJECT_NAME_1 "BnB_src1")

file(GLOB srcs "*.cpp"  "../include/*.h")

add_executable( ${PROJECT_NAME_1} ${srcs} )

target_include_directories( ${PROJECT_NAME_1} PRIVATE ${CPLEX_INCLUDE_DIRS})
target_link_libraries( ${PROJECT_NAME_1} PRIVATE ${CPLEX_LIBRARIES})
