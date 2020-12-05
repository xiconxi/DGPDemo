
add_library(pmp_pupa STATIC
        pupa/Common.cpp
        pupa/MinimalAreaSurface.cpp)


target_link_libraries(pmp_pupa pmp)

add_subdirectory(games102_hw6)

#add_executable(102GlobalUnfolding ../apps/EdgeLaplacianUnfolding.cpp)
#target_link_libraries(102GlobalUnfolding pmp_vis)
#
#if (EMSCRIPTEN)
#    set_target_properties(102GlobalUnfolding PROPERTIES LINK_FLAGS "--shell-file ${PROJECT_SOURCE_DIR}/extern/pmp-library/src/apps/data/shell.html ")
#endif()
