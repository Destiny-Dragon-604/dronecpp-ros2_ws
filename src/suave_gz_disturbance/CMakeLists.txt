cmake_minimum_required(VERSION 3.10)
project(suave_gz_disturbance)

# Gazebo Harmonic plugin for wind / external disturbances in PX4 SITL.
# Skipped automatically where Gazebo is not installed (e.g. on the drone itself).
find_package(gz-sim8 QUIET)
if(NOT gz-sim8_FOUND)
  message(WARNING "gz-sim8 (Gazebo Harmonic) not found: skipping the SuaveDisturbance plugin")
  return()
endif()

find_package(gz-plugin2 REQUIRED COMPONENTS register)
find_package(gz-transport13 REQUIRED)
find_package(gz-msgs10 REQUIRED)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()

add_library(SuaveDisturbance SHARED src/SuaveDisturbance.cc)
target_link_libraries(SuaveDisturbance PRIVATE
  gz-sim8::gz-sim8
  gz-plugin2::register
  gz-transport13::gz-transport13
  gz-msgs10::gz-msgs10)

# Only link what the plugin really uses (Gazebo targets drag in ffmpeg/audio otherwise)
target_link_options(SuaveDisturbance PRIVATE "LINKER:--as-needed")

install(TARGETS SuaveDisturbance LIBRARY DESTINATION lib)
install(FILES config/server.config DESTINATION share/${PROJECT_NAME})
install(FILES package.xml DESTINATION share/${PROJECT_NAME})
