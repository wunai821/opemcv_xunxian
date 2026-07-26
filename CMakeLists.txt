cmake_minimum_required(VERSION 3.16)
project(opencv_xunji LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(OpenCV 4 REQUIRED COMPONENTS core imgproc highgui videoio imgcodecs)

add_library(line_follower
    src/line_follower.cpp
    src/controller.cpp
    src/serial_port.cpp
)
target_include_directories(line_follower PUBLIC include)
target_link_libraries(line_follower PUBLIC ${OpenCV_LIBS})
target_compile_options(line_follower PRIVATE -Wall -Wextra -Wpedantic)

add_executable(opencv_xunji src/main.cpp)
target_link_libraries(opencv_xunji PRIVATE line_follower)

include(CTest)
if(BUILD_TESTING)
    add_executable(test_line_follower tests/test_line_follower.cpp)
    target_link_libraries(test_line_follower PRIVATE line_follower)
    add_test(NAME synthetic_line_test COMMAND test_line_follower)
endif()

install(TARGETS opencv_xunji RUNTIME DESTINATION bin)
install(FILES config/default.yaml DESTINATION share/opencv_xunji)
