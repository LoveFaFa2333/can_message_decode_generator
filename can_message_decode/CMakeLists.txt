cmake_minimum_required(VERSION 3.15)

project(can_message_decode)

set(CMAKE_CXX_STANDARD 14)


include_directories(
    ${PROJECT_SOURCE_DIR}/include
)


#add_library(message_decoder SHARED
#    src/can_message_decoder.cpp
#    )

#target_link_libraries(message_decoder)


add_executable(run_decode_test decode_test.cpp)

