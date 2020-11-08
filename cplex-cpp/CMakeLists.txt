cmake_minimum_required(VERSION 2.8.4)

project("CPLEX")

set (CMAKE_CXX_STANDARD 17)
set(CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

include( cmake/Configure.cmake )

find_package(Cplex)

file( GLOB CPLEX_LIB  "${CPLEX_BIN_DIR}/cplex${CPLEX_WIN_VERSION}0.dll" )
file( COPY ${CPLEX_LIB} DESTINATION "${CMAKE_BINARY_DIR}/bin")

add_subdirectory(modules)