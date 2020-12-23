//
// Created by pupa on 2020/12/10.
//

#include <pmp/visualization/MeshViewer.h>
#include <pmp/algorithms/SurfaceCurvature.h>
#include <imgui.h>
#include <imgui_internal.h>
#include "SurfaceQEMSimplification.h"

using namespace pmp;


struct VertexZSortInterface {
    size_t operator [](pmp::Vertex v) {return idx_[v];}
    double operator ()(pmp::Vertex v) {return vprio_[v];}

    pmp::VertexProperty<double> vprio_;
    pmp::VertexProperty<size_t> idx_;
};

class Viewer : public MeshViewer
{
public:
    Viewer(const char* title, int width, int height);

protected:
    virtual void process_imgui();

    pmp_pupa::SurfaceQEMSimplification
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
