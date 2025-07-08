cmake_minimum_required(VERSION 3.5) # FetchContent is available in 3.11+
project(robot-journey)

# Generate compile_commands.json
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
#set(GRAPHICS GRAPHICS_API_OPENGL_21)
SET(CMAKE_CXX_FLAGS  "-g")
set(CMAKE_CXX_STANDARD 17) #YMConnect requires C++17.

# Dependencies
find_package(raylib 4.0.0 QUIET) # QUIET or REQUIRED
if (NOT raylib_FOUND) # If there's none, fetch and build raylib
  include(FetchContent)
  FetchContent_Declare(
    raylib
    URL https://github.com/raysan5/raylib/archive/refs/tags/4.0.0.tar.gz
  )
  FetchContent_GetProperties(raylib)
  if (NOT raylib_POPULATED) # Have we downloaded raylib yet?
    set(FETCHCONTENT_QUIET NO)
    FetchContent_Populate(raylib)
    set(BUILD_EXAMPLES OFF CACHE BOOL "" FORCE) # don't build the supplied examples
    add_subdirectory(${raylib_SOURCE_DIR} ${raylib_BINARY_DIR})
  endif()
endif()

add_library(YMConnect SHARED IMPORTED)#Import the YMConnect library into the project.
set_property(TARGET YMConnect PROPERTY IMPORTED_LOCATION "${CMAKE_SOURCE_DIR}/libYMConnect_D.so") #Set the location of the YMConnect library.

# Our Project

file(GLOB SOURCES
  *.h
  *.cpp
)

ADD_EXECUTABLE(${PROJECT_NAME}
${SOURCES}
)


target_link_libraries(${PROJECT_NAME} raylib YMConnect)

add_custom_command(
  TARGET robot-journey POST_BUILD
  COMMAND ${CMAKE_COMMAND} -E copy_directory
          ${CMAKE_CURRENT_SOURCE_DIR}/src
          ${CMAKE_CURRENT_BINARY_DIR}/src
    COMMAND ${CMAKE_COMMAND} -E copy_directory
          ${CMAKE_CURRENT_SOURCE_DIR}/dat
          ${CMAKE_CURRENT_BINARY_DIR}/dat
)