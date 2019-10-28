cmake_minimum_required(VERSION 3.12)

project("ph-pg")

set(CMAKE_CXX_STANDARD 14)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${PROJECT_SOURCE_DIR}/cmake)

option(ENABLE_CPLEX "Link to Cplex libraries" ON)

# LEMON settings
if (APPLE)
    set(LEMON_INCLUDE_DIR "/usr/local/include/" CACHE PATH "LEMON include directory")
    # set(LEMON_INCLUDE_DIR "${PROJECT_SOURCE_DIR}/third_party/lemon-1.3.1/" CACHE PATH "LEMON include directory")
    set(LEMON_INCLUDE_DIRS "${LEMON_INCLUDE_DIR}")
    set(LEMON_LIB_NAME "libemon.a")
    set(LEMON_LIBRARY "/usr/local/lib/${LEMON_LIB_NAME}" CACHE FILEPATH "LEMON library")
    # set(LEMON_LIBRARY "${PROJECT_SOURCE_DIR}/third_party/lemon-1.3.1/build/lemon/${LEMON_LIB_NAME}" CACHE FILEPATH "LEMON library")
    set(LEMON_LIBRARIES "${LEMON_LIBRARY}")
    MARK_AS_ADVANCED(LEMON_LIBRARY LEMON_INCLUDE_DIR)
    include_directories(${LEMON_INCLUDE_DIRS})
    set(LIBS ${LIBS} ${LEMON_LIBRARIES})
else()
    set(LEMON_INCLUDE_DIR "/home/diode/libraries/lemon/include" CACHE PATH "LEMON include directory")
    set(LEMON_INCLUDE_DIRS "${LEMON_INCLUDE_DIR}")
    set(LEMON_LIB_NAME "libemon.a")
    set(LEMON_LIBRARY "/home/diode/libraries/lemon/lib/${LEMON_LIB_NAME}" CACHE FILEPATH "LEMON library")
    set(LEMON_LIBRARIES "${LEMON_LIBRARY}")
    MARK_AS_ADVANCED(LEMON_LIBRARY LEMON_INCLUDE_DIR)
    include_directories(${LEMON_INCLUDE_DIRS})
    set(LIBS ${LIBS} ${LEMON_LIBRARIES})
endif()

# CPLEX settings
if (ENABLE_CPLEX)
    message(STATUS "Enable CPLEX")
    add_definitions(-DUSE_CPLEX)
    find_package(CPLEX)
    include_directories(${CPLEX_INCLUDE_DIRS})
    set(LIBS ${LIBS} ${CPLEX_LIBRARIES})
endif()

# CPLEX flags
if(APPLE)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -fPIC -DIL_STD")
else()
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -m64 -g -O0 -fPIC -fno-strict-aliasing -fexceptions -DIL_STD -lpthread -pthread -ldl")
endif()


include_directories(${PROJECT_SOURCE_DIR}/third_party/cpp_option_parser)
add_subdirectory(third_party/cpp_option_parser)
set(LIBS ${LIBS} option_parser)
set(CPP_OPTION_PARSER option_parser)

add_subdirectory(src)

