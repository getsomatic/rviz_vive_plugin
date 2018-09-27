cmake_minimum_required(VERSION 2.8.3)
project(rviz_vive_plugin)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")

find_package(catkin REQUIRED COMPONENTS rviz)
find_package(OpenVR REQUIRED)
find_package(SDL2 REQUIRED)
find_package(Qt5 ${rviz_QT_VERSION} EXACT REQUIRED Core Widgets)
find_package(OpenGL REQUIRED)
find_package(GLEW 1.11 REQUIRED)

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
    ${OpenVR_INCLUDE_DIRS}
    include
)

add_library(rviz_vive_plugin
    src/vive_display.cpp
    src/vive.cpp
    src/vive_conversions.cpp
    ${MOC_FILES}
)

target_link_libraries(rviz_vive_plugin
    ${QT_LIBRARIES}
    ${catkin_LIBRARIES}
    ${SDL2_LIBRARY}
    ${GLEW_LIBRARIES}
    ${OpenVR_LIBRARIES})

install(FILES
    plugin_description.xml
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
