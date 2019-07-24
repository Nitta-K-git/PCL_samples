cmake_minimum_required( VERSION 3.6 )

# Create Project
project( pcl_visualizer_test )

# Set StartUp Project
# set_property( DIRECTORY PROPERTY VS_STARTUP_PROJECT "pcl_visualizer" )

if(POLICY CMP0074) # https://github.com/PointCloudLibrary/pcl/issues/2425
   cmake_policy(SET CMP0074 NEW)
endif()
# Find PCL Package
set( PCL_ROOT $ENV{PCL_ROOT} )
# find_package( PCL 1.8.0 REQUIRED )
find_package( PCL 1.9.1 REQUIRED )

# Find VTK Package
# set( VTK_DIR "$ENV{PCL_ROOT}/3rdParty/VTK" )
find_package( VTK REQUIRED )

# set(Qt5_DIR $ENV{QTDIR})
set(Qt5_DIR C:/Qt2/5.12.3/msvc2017_64)
# set(Qt5_DIR C:/Qt2/5.12.3/msvc2017_64/lib/cmake)
list(APPEND CMAKE_PREFIX_PATH ${Qt5_DIR})
find_package(Qt5 COMPONENTS Widgets REQUIRED)

add_compile_options("$<$<CXX_COMPILER_ID:MSVC>:/utf-8>")

# Set Automatic MOC
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)
if(CMAKE_VERSION VERSION_LESS "3.7.0")
    set(CMAKE_INCLUDE_CURRENT_DIR ON)
endif()

set(SOURCE_FILES
   main.cpp
   pclviewer.h
   pclviewer.cpp
   pclviewer.ui
)
# Add Executable
add_executable(${PROJECT_NAME} ${SOURCE_FILES})

# Additional Include Directories
include_directories( ${PCL_INCLUDE_DIRS} )
include_directories( ${CMAKE_CURRENT_SOURCE_DIR} )
include_directories( ${CMAKE_CURRENT_BINARY_DIR} )

# Load VTK Settings
include( ${VTK_USE_FILE} )
# Preprocessor Definitions
add_definitions( ${PCL_DEFINITIONS} )
# Additional Library Directories
link_directories( ${PCL_LIBRARY_DIRS} )
# Additional Dependencies
target_link_libraries(${PROJECT_NAME} ${PCL_LIBRARIES} ${VTK_LIBRARIES} Qt5::Widgets Qt5::Core)
