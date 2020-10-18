
include_directories(${PROJECT_SOURCE_DIR}/extern/pmp-library/external/eigen)
include_directories(${PROJECT_SOURCE_DIR}/extern/pmp-library/external/glew/include)
include_directories(${PROJECT_SOURCE_DIR}/extern/pmp-library/external/glfw/include)
include_directories(${PROJECT_SOURCE_DIR}/extern/pmp-library/external/imgui)
include_directories(${PROJECT_SOURCE_DIR}/DGP)
#add_executable(demoviewer demoviewer.cpp algorithm/CPCSurfaceUnfolding.cpp EmbeddedCurve.cpp EmbeddedCurve.h)
#target_link_libraries(demoviewer pmp_vis)


add_executable(EMTest RealizedEmbeddedCurve.cpp)
target_link_libraries(EMTest pmp_vis)

add_executable(101ManualUnfolding apps/ManuallyRotateUnfolding.cpp algorithm/CPCSurfaceUnfolding.cpp)
target_link_libraries(101ManualUnfolding pmp_vis)


add_executable(102GlobalUnfolding apps/EdgeLaplacianUnfolding.cpp)
target_link_libraries(102GlobalUnfolding pmp_vis)