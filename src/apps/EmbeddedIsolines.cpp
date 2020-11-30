//
// Created by pupa on 11/28/20.
//
#include <pupa/pmp/RealizedEmbeddedCurve.h>
#include <pupa/pmp/GeometryTools.h>
#include <pmp/visualization/MeshViewer.h>
#include <pmp/algorithms/SurfaceParameterization.h>
#include <imgui.h>

using namespace pmp;

class Viewer : public MeshViewer
{
public:
    Viewer(const char* title, int width, int height);

protected:
    virtual void process_imgui() override;
    virtual void draw(const std::string& _draw_mode) override;
};

Viewer::Viewer(const char* title, int width, int height) : MeshViewer(title, width, height) {}

void Viewer::draw(const std::string& draw_mode)
{
    MeshViewer::draw(draw_mode);
}

void Viewer::process_imgui()
{
    MeshViewer::process_imgui();

    ImGui::Spacing();
    if (ImGui::Button("Isoline"))
    {
        pmp_ext::RealizedEmbeddedCurve embeddedCurve(mesh_);
        update_mesh();
    }
    ImGui::Spacing();
}

int main(int argc, char** argv)
{
#ifndef __EMSCRIPTEN__
    Viewer window("Parametrization", 800, 600);
    if (argc == 2)
        window.load_mesh(argv[1]);
    return window.run();
#else
    Viewer window("Parametrization", 800, 600);
    window.load_mesh(argc == 2 ? argv[1] : "input.off");
    return window.run();
#endif
}
