get_filename_component(PROJECT_NAME ${CMAKE_CURRENT_SOURCE_DIR} NAME)
project(${PROJECT_NAME})

add_executable(${PROJECT_NAME} main.cpp)
target_link_libraries(${PROJECT_NAME} pmp_vis)

add_executable(${PROJECT_NAME}_energy energy_main.cpp)
target_link_libraries(${PROJECT_NAME}_energy pmp_vis)
