##############################################################################
# CMake
##############################################################################

cmake_minimum_required(VERSION 2.8.0)
project(roboy_interface)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -O0 -g")
SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/../roboy_managing_node/cmake)
include(${CMAKE_CURRENT_SOURCE_DIR}/../roboy_managing_node/cmake/options.cmake)
include_directories(
        ${OPLK_BASE_DIR}/stack/proj/linux/liboplkmndrv-pcap
        ${OPLK_BASE_DIR}/apps/common/objdicts/CiA302-4_MN
        ${CMAKE_CURRENT_SOURCE_DIR}/../roboy_powerlink/powerlink/output
        include
)

##############################################################################
# Catkin
##############################################################################

# qt_build provides the qt cmake glue, roscpp the comms for a default talker
find_package(catkin REQUIRED COMPONENTS qt_build roscpp pcl_ros roboy_communication_middleware cv_bridge
        roboy_managing_node common_utilities)
catkin_package(CATKIN_DEPENDS roboy_managing_node roboy_communication_middleware common_utilities)
include_directories(${catkin_INCLUDE_DIRS} ../common_utilities/include)


# Eigen
find_package(PkgConfig)
pkg_check_modules(EIGEN3 REQUIRED eigen3)
include_directories(${EIGEN3_INCLUDE_DIRS})

include_directories(qcustomplot)

# OpenCV
find_package(OpenCV COMPONENTS opencv_core opencv_highgui REQUIRED)

find_package(PkgConfig REQUIRED)
pkg_check_modules(tinyxml REQUIRED tinyxml)
include_directories(${tinyxml_INCLUDE_DIRS})

##############################################################################
# Qt Environment
##############################################################################

# this comes from qt_build's qt-ros.cmake which is automatically 
# included via the dependency call in package.xml
rosbuild_prepare_qt4(QtCore QtGui) # Add the appropriate components to the component list here

##############################################################################
# Sections
##############################################################################

file(GLOB QT_FORMS RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} ui/*.ui)
file(GLOB QT_RESOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} resources/*.qrc)
set(QT_MOC include/interface/main_window.hpp qcustomplot/qcustomplot.h)

QT4_ADD_RESOURCES(QT_RESOURCES_CPP ${QT_RESOURCES})
QT4_WRAP_UI(QT_FORMS_HPP ${QT_FORMS})
QT4_WRAP_CPP(QT_MOC_HPP ${QT_MOC})

##############################################################################
# Sources
##############################################################################

set(QT_SOURCES src/main_window.cpp src/main.cpp qcustomplot/qcustomplot.cpp)

##############################################################################
# Binaries
##############################################################################

add_executable(roboy_interface ${QT_SOURCES} ${QT_RESOURCES_CPP} ${QT_FORMS_HPP} ${QT_MOC_HPP} )
add_dependencies(roboy_interface roboy_communication_middleware_gencpp)
target_link_libraries(roboy_interface ${QT_LIBRARIES} ${catkin_LIBRARIES} ${PROTOBUF_LIBRARY} roboy_managing_node
        ${tinyxml_LIBRARIES} rviz_visualization)

##############################################################################
# Installation
##############################################################################
install(TARGETS roboy_interface
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(FILES
        package.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        )

