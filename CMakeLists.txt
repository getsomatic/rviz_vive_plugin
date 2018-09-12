cmake_minimum_required(VERSION 2.8.3)
project(rviz_vive_plugin)

if (NOT OPENVR)
    message(FATAL_ERROR "OPENVR variable is not set")
endif ()

if (NOT PLATFORM)
    if (CMAKE_SIZEOF_VOID_P MATCHES 8)
        set(PLATFORM 64)
    else ()
        set(PLATFORM 32)
    endif ()
endif ()
message(STATUS "Compilation set for ${PLATFORM}bits architectures.")

if ((${CMAKE_CXX_COMPILER_ID} MATCHES "GNU") OR (${CMAKE_CXX_COMPILER_ID} MATCHES "Clang"))
    add_definitions(-DGNUC)

    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
    set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -Wall -Wextra -pedantic -g")
    set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O2")

    if (${PLATFORM} MATCHES 32)
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -m32")
        set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -m32")
    endif ()
elseif (CMAKE_CXX_COMPILER_ID MATCHES "MSVC")
    set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} /W2 /DEBUG")
    set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} /MP /INCREMENTAL:NO")
else ()
    message(FATAL_ERROR "Unsupported compiler '${CMAKE_CXX_COMPILER_ID}'")
endif ()

find_package(SDL2 REQUIRED)
find_package(catkin REQUIRED COMPONENTS rviz)
find_package(Qt5 ${rviz_QT_VERSION} EXACT REQUIRED Core Widgets)
find_package(OpenGL REQUIRED)

if (${CMAKE_SYSTEM_NAME} MATCHES "Linux")
    find_package(GLEW 1.11 REQUIRED)
else ()
    find_library(GLEW_LIBRARIES
        NAMES
            GLEW
            glew32
        PATHS
            ${THIRDPARTY_DIR}/glew/glew-1.11.0/lib/Release
        PATH_SUFFIXES
            osx32
            ${WINDOWS_PATH_SUFFIXES}
    )
    set(GLEW_INCLUDE_DIR ${THIRDPARTY_DIR}/glew/glew-1.11.0/include)
endif ()

set(QT_LIBRARIES Qt5::Widgets)

add_definitions(-DQT_NO_KEYWORDS)
add_definitions(-DLINUX -DPOSIX)

catkin_package()

qt5_wrap_cpp(MOC_FILES
    ./include/rviz_vive_plugin/vive_display.h
    OPTIONS -DBOOST_TT_HAS_OPERATOR_HPP_INCLUDED -DBOOST_LEXICAL_CAST_INCLUDED
)

include_directories(
    ${catkin_INCLUDE_DIRS}
    ${SDL2_INCLUDE_DIRS}
    ${OPENVR}/headers
    include
)

add_library(rviz_vive_plugin
    src/vive_display.cpp
    src/vive.cpp
    src/vive_conversations.cpp
    ${MOC_FILES}
)

target_link_libraries(rviz_vive_plugin
    ${QT_LIBRARIES}
    ${catkin_LIBRARIES}
    ${SDL2_LIBRARY}
    ${GLEW_LIBRARIES}
    ${OPENVR}/lib/linux64/libopenvr_api.so
)

install(FILES
    plugin_description.xml
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
