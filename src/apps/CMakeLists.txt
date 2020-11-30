add_executable(EdgeLaplacianUnfolding EdgeLaplacianUnfolding.cpp)
target_link_libraries(EdgeLaplacianUnfolding pmp_vis)

add_executable(EmbeddedIsolines EmbeddedIsolines.cpp)
target_link_libraries(EmbeddedIsolines pmp_vis pmp_pupa)

add_executable(PupaPMPViewer PupaPMPViewer.cpp)
target_link_libraries(PupaPMPViewer pmp_vis pmp_pupa entityx)

if (EMSCRIPTEN)
    set_target_properties(EdgeLaplacianUnfolding PROPERTIES LINK_FLAGS "--shell-file ${PROJECT_SOURCE_DIR}/extern/pmp-library/src/apps/data/shell.html ")
    set_target_properties(EmbeddedIsolines PROPERTIES LINK_FLAGS "--shell-file ${PROJECT_SOURCE_DIR}/extern/pmp-library/src/apps/data/shell.html ")
    set_target_properties(PupaPMPViewer PROPERTIES LINK_FLAGS "--shell-file ${PROJECT_SOURCE_DIR}/extern/pmp-library/src/apps/data/shell.html ")
endif()
