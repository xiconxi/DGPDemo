get_filename_component(PROJECT_NAME ${CMAKE_CURRENT_SOURCE_DIR} NAME)
project(${PROJECT_NAME})

find_package(CGAL)
find_package(Boost REQUIRED thread)
add_executable(${PROJECT_NAME} voronoi.cpp)
target_link_libraries(${PROJECT_NAME}  CGAL::CGAL ${Boost_LIBRARIES})
#target_link_libraries(${PROJECT_NAME} pmp_vi)
