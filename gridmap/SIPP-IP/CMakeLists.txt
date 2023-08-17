project(MAPFSIPP_IP)

cmake_minimum_required (VERSION 3.10)

set(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -pthread -g -Wall -ldl")
set(CMAKE_CXX_STANDARD 14)

# set base path for cplex directory, change this path if using different version of CPLEX
set(CPLEX_BASE_DIR  "/opt/ibm/ILOG/CPLEX_Studio128")

set(CPLEX_INCLUDE_DIRS      "${CPLEX_BASE_DIR}/cplex/include")
set(CONCERT_INCLUDE_DIRS      "${CPLEX_BASE_DIR}/concert/include")
set(CPLEX_LIBRARIES         "${CPLEX_BASE_DIR}/cplex/lib/x86-64_linux/static_pic/libcplex.a")
set(ILOCPLEX_LIBRARIES         "${CPLEX_BASE_DIR}/cplex/lib/x86-64_linux/static_pic/libilocplex.a")
set(CPLEXDISTMIP_LIBRARIES         "${CPLEX_BASE_DIR}/cplex/lib/x86-64_linux/static_pic/libcplexdistmip.a")
set(CONCERT_LIBRARIES         "${CPLEX_BASE_DIR}/concert/lib/x86-64_linux/static_pic/libconcert.a")

find_package(Boost 1.49.0 REQUIRED COMPONENTS program_options system filesystem)

include_directories(
  include
)

# add all source files
file(GLOB SOURCES "src/*.cpp")


add_executable(MAPFSIPP_IP ${SOURCES})
set_property(
  TARGET MAPFSIPP_IP 
  PROPERTY COMPILE_DEFINITIONS IL_STD
)
target_include_directories(MAPFSIPP_IP PUBLIC ${Boost_INCLUDE_DIRS} ${CPLEX_INCLUDE_DIRS} ${CONCERT_INCLUDE_DIRS})
target_link_libraries(MAPFSIPP_IP ${Boost_LIBRARIES}  ${ILOCPLEX_LIBRARIES} ${CONCERT_LIBRARIES} ${CPLEX_LIBRARIES} ${CPLEXDISTMIP_LIBRARIES}  ${CMAKE_DL_LIBS})
set (target_link_options "-lilocplex -lconcert -lcplex -lm -lpthread")
