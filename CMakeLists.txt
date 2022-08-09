cmake_minimum_required(VERSION 3.16)
project(iflex_sw)

SET(MAJOR_VERSION 1)
SET(MINOR_VERSION 0)
SET(PATCH_VERSION 0)
SET(APP_VERSION "${MAJOR_VERSION}.${MINOR_VERSION}.${PATCH_VERSION}")

add_compile_options(-DAPP_VERSION="${APP_VERSION}")
add_compile_options(-DDEBUG_MODE_ENABLED)
add_compile_options(-DFMT_HEADER_ONLY)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -pthread -Wall -Wno-pointer-arith -fexceptions -fnon-call-exceptions -rdynamic -g -ljsoncpp") # -I/usr/local/ssl/include -I/usr/local/include/json causes WTF problems to occur any way I add it
find_package(Boost COMPONENTS program_options filesystem system REQUIRED)
find_package(HDF5)
#find_package(jsoncpp)
find_package(spdlog CONFIG REQUIRED)
find_package(PkgConfig REQUIRED)


add_library(Highfive INTERFACE)
target_include_directories(Highfive INTERFACE /usr/local/include/highfive)


add_library(json INTERFACE)
target_include_directories(json INTERFACE /usr/local/include/json)

include_directories(inc ${Boost_INCLUDE_DIR} ${HDF5_INCLUDE_DIR} /usr/local/ssl/include) #${JSONCPP_INCLUDE_DIRS}  /usr/local/include/json

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "bin")

set(IFLEX_TARGET_LIBRARIES ${Boost_LIBRARIES} boost_thread pthread ${HDF5_C_LIBRARIES} spdlog::spdlog json Highfive "/usr/local/lib/libjsoncpp.a" "/usr/local/ssl/lib/libcrypto.so" "/usr/local/ssl/lib/libssl.so") #${JSONCPP_LIBRARIES}

set(IFLEX_FILES
        src/main.cpp
        inc/iFlexTypes.h
        inc/INode.h
        inc/LinuxWrapper.h
        inc/Logger.h
        inc/AppConfig.h
        inc/Master.h
        inc/MasterNode.h
        inc/Session.h
        inc/NodeConfig.h
        inc/iFlexNode.h
        inc/Request.h
        inc/Response.h
        inc/TCPNode.h
        inc/RepeatingTimer.h
        inc/TimeMeter.h
        inc/iFlexNodeReader.h
        src/Logger.cpp
        src/LinuxWrapper.cpp
        src/Master.cpp
        src/MasterNode.cpp
        src/Session.cpp
        src/iFlexNode.cpp
        src/TCPNode.cpp
        )

add_executable(iflexd ${IFLEX_FILES})
target_link_libraries(iflexd ${IFLEX_TARGET_LIBRARIES}) 
