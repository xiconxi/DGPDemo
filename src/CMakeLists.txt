

add_library(pmp_pupa
        pupa/pmp/RealizedEmCurve.cpp
        )

#add_executable(102GlobalUnfolding ../apps/EdgeLaplacianUnfolding.cpp)
#target_link_libraries(102GlobalUnfolding pmp_vis)
#
#if (EMSCRIPTEN)
#    set_target_properties(102GlobalUnfolding PROPERTIES LINK_FLAGS "--shell-file ${PROJECT_SOURCE_DIR}/extern/pmp-library/src/apps/data/shell.html ")
#endif()
