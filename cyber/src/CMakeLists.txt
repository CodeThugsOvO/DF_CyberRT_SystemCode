project(scr VERSION 1.0.0)

add_executable(can2cyber main.cc proto/aboutcom.pb.cc)
target_link_libraries(can2cyber
    ${TARGET_NAME}
    pthread
)

add_subdirectory(common_component_example)
add_subdirectory(timer_component_example)
