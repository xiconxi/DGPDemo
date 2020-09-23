
include_directories(${PROJECT_SOURCE_DIR}/extern/pmp-library/external/eigen)
include_directories(${PROJECT_SOURCE_DIR}/extern/pmp-library/external/glew/include)
include_directories(${PROJECT_SOURCE_DIR}/extern/pmp-library/external/glfw/include)
include_directories(${PROJECT_SOURCE_DIR}/extern/pmp-library/external/imgui)

add_executable(mpviewer mpviewer.cpp algorithm/CPCSurfaceUnfolding.cpp)
target_link_libraries(mpviewer pmp_vis)