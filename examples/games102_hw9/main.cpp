//
// Created by pupa on 2020/12/10.
//

#include <pmp/visualization/MeshViewer.h>
#include <pmp/algorithms/SurfaceCurvature.h>
#include <imgui.h>
#include <imgui_internal.h>
#include "SurfaceQEM.h"

using namespace pmp;


class Viewer : public MeshViewer
{
public:
    Viewer(const char* title, int width, int height);

protected:
    virtual void process_imgui();

    pmp_pupa::SurfaceQEM* simplify_{nullptr};
};

Viewer::Viewer(const char* title, int width, int height) : MeshViewer(title, width, height) {}

void Viewer::process_imgui()
{
    MeshViewer::process_imgui();

    ImGui::Spacing();
#if __EMSCRIPTEN__
    static int n = 5;
    ImGui::Combo("Files", &n, files, 6);
    ImGui::SameLine();
    if (ImGui::Button("Reload Mesh"))
    {
        load_mesh(files[n]);
        set_draw_mode("Hidden Line");
    }
#endif
    ImGui::Spacing();

    if(ImGui::Button("Simplify")) {
        if(simplify_ == nullptr)
            simplify_ = new pmp_pupa::SurfaceQEM(mesh_);

        simplify_->simplification(mesh_.n_vertices()*0.8);
        update_mesh();

        std::cout << "Simplified !!!!!!!!!! " << std::endl;
    }
}

int main(int argc, char** argv)
{
#ifndef __EMSCRIPTEN__
    Viewer window("Game102-HW9", 800, 600);
    if (argc == 2)
        window.load_mesh(argv[1]);
    return window.run();
#else
    Viewer window("Game102-HW7", 800, 600);
    window.load_mesh(argc == 2 ? argv[1] : "Nefertiti_face.obj");
    return window.run();
#endif
}
