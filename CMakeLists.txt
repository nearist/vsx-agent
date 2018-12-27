cmake_minimum_required(VERSION 2.8)
project(iflex_sw)

find_package(Subversion)
if(SUBVERSION_FOUND)
    Subversion_WC_INFO(${PROJECT_SOURCE_DIR} Project)
    message("Current revision is ${Project_WC_REVISION}")
endif()

SET(MAJOR_VERSION 1)
SET(MINOR_VERSION 0)
SET(PATCH_VERSION ${Project_WC_REVISION})
SET(APP_VERSION "${MAJOR_VERSION}.${MINOR_VERSION}.${PATCH_VERSION}")

add_compile_options(-DAPP_VERSION="${APP_VERSION}")
add_compile_options(-DDEBUG_MODE_ENABLED)
add_compile_options(-DFMT_HEADER_ONLY)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -pthread -Wall -Wno-pointer-arith -fexceptions -fnon-call-exceptions -rdynamic -g")
find_package(Boost COMPONENTS program_options filesystem system REQUIRED)
find_package(HDF5)
find_package(spdlog CONFIG REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(JSONCPP jsoncpp)

include_directories(inc ${Boost_INCLUDE_DIR} ${HDF5_INCLUDE_DIR} ${JSONCPP_INCLUDE_DIRS} /opt/HighFive/include)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "bin")

set(IFLEX_TARGET_LIBRARIES ${Boost_LIBRARIES} boost_thread pthread ssl crypto ${HDF5_C_LIBRARIES} ${JSONCPP_LIBRARIES} spdlog::spdlog)

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

IF (EXISTS "${CMAKE_ROOT}/Modules/CPack.cmake")
    INCLUDE(InstallRequiredSystemLibraries)

    SET(CPACK_SET_DESTDIR "on")
    SET(CPACK_PACKAGING_INSTALL_PREFIX "/tmp")
    SET(CPACK_GENERATOR "DEB")

    SET(CPACK_PACKAGE_DESCRIPTION "iflex-server")
    SET(CPACK_PACKAGE_DESCRIPTION_SUMMARY "Nearist iFlex Server")
    SET(CPACK_PACKAGE_VENDOR "Nearist")
    SET(CPACK_PACKAGE_CONTACT "info@nearist.io")
    SET(CPACK_PACKAGE_VERSION_MAJOR "${MAJOR_VERSION}")
    SET(CPACK_PACKAGE_VERSION_MINOR "${MINOR_VERSION}")
    SET(CPACK_PACKAGE_VERSION_PATCH "${PATCH_VERSION}")
    SET(CPACK_PACKAGE_FILE_NAME "${CMAKE_PROJECT_NAME}_${APP_VERSION}")
    SET(CPACK_SOURCE_PACKAGE_FILE_NAME "${CMAKE_PROJECT_NAME}_${APP_VERSION}")

    SET(CPACK_DEBIAN_PACKAGE_SHLIBDEPS ON)

    SET(CPACK_DEBIAN_ARCHITECTURE ${CMAKE_SYSTEM_PROCESSOR})

    SET(CPACK_COMPONENTS_ALL Libraries ApplicationData)

    INSTALL(TARGETS iflexd DESTINATION opt/iflex/bin)
    INSTALL(FILES "conf/iflex.json" DESTINATION opt/iflex/etc)

    INCLUDE(CPack)

ENDIF (EXISTS "${CMAKE_ROOT}/Modules/CPack.cmake")
