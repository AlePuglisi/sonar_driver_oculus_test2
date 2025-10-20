cmake_minimum_required(VERSION 3.5)

add_definitions(-DCONFIG_DIR_PATH=\"${CMAKE_CURRENT_SOURCE_DIR}/../config/\")

project( sonar_driver )

# Include current directory for header files
include_directories(${CMAKE_CURRENT_SOURCE_DIR})

# Add the executable
add_executable( sonar_driver 
                sonar_driver.cpp 
                OsClientCtrl.cpp
                OsDriver.cpp
)

# Link required system libraries
target_link_libraries(sonar_driver
    pthread       # for std::thread and std::mutex
)