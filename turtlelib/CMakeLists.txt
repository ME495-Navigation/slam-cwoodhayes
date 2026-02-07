cmake_minimum_required(VERSION 3.20)
project(turtlelib)

set(CMAKE_CXX_COMPILER g++-14) # Don't set this in CMakeLists.txt, this is forcing your user to use this exact compiler


set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# cmake args
option(BUILD_DOCS "Build the documentation" OFF)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(Catch2 3 REQUIRED)
find_package(Doxygen)

# build the documentation
if(${DOXYGEN_FOUND} AND ${BUILD_DOCS})
    # Turn the README.md into the homepage of the doxygen docs
    set(DOXYGEN_USE_MDFILE_AS_MAINPAGE README.md)

    # Tell Doxygen where to find the documentation
    doxygen_add_docs(doxygen include/ src/ README.md ALL)

    # The documentation will be in the build/html directory
    # The main page is build/html/index.html
endif()

include(CTest)

# CTest sets BUILD_TESTING to on. To disable tests add -DBUILD_TESTING=OFF when invoking cmake
if(BUILD_TESTING)
    # Find the Unit testing framework. In this example, Catch2
    find_package(Catch2 3 REQUIRED)
    include(Catch)

    # A test is just an executable that is linked against the unit testing library
    add_executable(tests 
      test/test_geometry2d.cpp
      test/test_se2d.cpp
      test/test_svg.cpp
    )
    target_link_libraries(tests Catch2::Catch2WithMain turtlelib)

    # register the test with CTest, telling it what executable to run
    catch_discover_tests(tests)
    add_test(NAME test_all COMMAND tests)
endif()

# CREATE LIBRARIES AND EXECUTABLES
add_library(turtlelib
  src/geometry2d.cpp
  src/se2d.cpp
  src/svg.cpp
  src/svg_elements.cpp
)
target_include_directories(turtlelib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

add_executable(converter src/converter.cpp)
target_link_libraries(converter turtlelib)

add_executable(svg src/int_test_svg.cpp)
target_link_libraries(svg turtlelib)

add_executable(frame_main src/frame_main.cpp)
target_link_libraries(frame_main turtlelib)

# below is taken from matt's course notes:
# https://nu-msr.github.io/navigation/lectures/cmake_basics.html#ament-cmake-1-P-1

# install the include files by copying the whole include directory
install(DIRECTORY include/ DESTINATION include)

# Create a CMake Exported Target containing the lib and exe.
# Also create CMake Export called project_name-targets
# The CMake Export contains files that allow other CMake projects
# to find this project. It must be installed separately.
install(TARGETS turtlelib EXPORT turtlelib-targets)

# The project_name-targets created by install(TARGETS) needs to be installed.
# install(EXPORT ...) will generate a file called project_name-config.cmake
# that contains the exported targets.
# After installation this file will then be found when calling
# find_package(project_name) from another cmake project
# A user can then target_link_libraries(target project_name::library)
# to use the libraries installed here
install(EXPORT turtlelib-targets
        FILE turtlelib-config.cmake
        NAMESPACE turtlelib::
        DESTINATION lib/cmake/${PROJECT_NAME})
