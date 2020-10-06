
include_directories(${PROJECT_SOURCE_DIR}/extern/pmp-library/external/eigen)
include_directories(${PROJECT_SOURCE_DIR}/extern/pmp-library/external/glew/include)
include_directories(${PROJECT_SOURCE_DIR}/extern/pmp-library/external/glfw/include)
include_directories(${PROJECT_SOURCE_DIR}/extern/pmp-library/external/imgui)

add_executable(demoviewer demoviewer.cpp algorithm/CPCSurfaceUnfolding.cpp EmbeddedCurve.cpp EmbeddedCurve.h)
target_link_libraries(demoviewer pmp_vis)