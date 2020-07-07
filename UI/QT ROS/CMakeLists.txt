##############################################################################
# CMake
##############################################################################

cmake_minimum_required(VERSION 2.8.0)
project(gui_subscriber)
add_definitions(-std=c++11 -fPIC -Wall -Wextra)
##############################################################################
# Catkin
##############################################################################

##################
# QT ENVIRONMENT #
##################
set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON)

# qt_build provides the qt cmake glue, roscpp the comms for a default talker
find_package(catkin REQUIRED COMPONENTS
    roscpp
    sensor_msgs
    image_transport
    std_msgs
    cv_bridge
      )

find_package(OpenCV REQUIRED)
find_package(Qt5 COMPONENTS Core Gui Widgets PrintSupport)
include_directories(${catkin_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS})
# Use this to define what the package will export (e.g. libs, headers).
# Since the default here is to produce only a binary, we don't worry about
# exporting anything. 
##############################################################################
# Qt Environment
##############################################################################

# this comes from qt_build's qt-ros.cmake which is automatically 
# included via the dependency call in package.xml
#rosbuild_prepare_qt4(QtCore QtGui) # Add the appropriate components to the component list here

##############################################################################
# Sections
##############################################################################

file(GLOB QT_FORMS RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} ui/*.ui)
file(GLOB QT_RESOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} resources/*.qrc)
file(GLOB_RECURSE QT_MOC RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS include/gui_subscriber/*.hpp)

QT5_ADD_RESOURCES(QT_RESOURCES_CPP ${QT_RESOURCES})
QT5_WRAP_UI(QT_FORMS_HPP ${QT_FORMS})
QT5_WRAP_CPP(QT_MOC_HPP ${QT_MOC})

include_directories(
    ${catkin_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
    ${Qt5Core_INCLUDE_DIRS}
    ${Qt5Gui_INCLUDE_DIRS}
    ${Qt5Widgets_INCLUDE_DIRS}
    ${Qt5PrintSupport_INCLUDE_DIRS}
    )

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES gui_publisher
  CATKIN_DEPENDS roscpp std_msgs #message_generation
  DEPENDS system_lib opencv
)

###########
# SOURCES #
###########

set(QT_SOURCES
    src/main.cpp
    src/main_window.cpp
    src/qnode.cpp

    include/gui_subscriber/main_window.hpp
    include/gui_subscriber/qnode.hpp
    )


##############################################################################
# Sources
##############################################################################

#file(GLOB_RECURSE QT_SOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS src/*.cpp)

##############################################################################
# Binaries
##############################################################################

add_executable(gui_subscriber ${QT_SOURCES} ${QT_RESOURCES_CPP} ${QT_FORMS_HPP} ${QT_MOC_HPP})
target_link_libraries(gui_subscriber     
    Qt5::Core
    Qt5::Widgets
    Qt5::PrintSupport 
    ${QT_LIBRARIES}
    ${catkin_LIBRARIES} 
    ${OpenCV_LIBRARIES})
install(TARGETS gui_subscriber RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

