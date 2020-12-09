//
// Created by pupa on 12/9/20.
//
#include <pmp/visualization/MeshViewer.h>
#include <pmp/algorithms/SurfaceCurvature.h>
#include <pmp/algorithms/SurfaceSmoothing.h>
#include <imgui.h>

#include "energy_smoothing.h"
#include "tangential_smoothing.h"


using namespace pmp;

class Viewer : public MeshViewer
{
public:
    Viewer(const char* title, int width, int height);

protected:
    virtual void process_imgui();

private:
    SurfaceSmoothing smoother_;
};

Viewer::Viewer(const char* title, int width, int height)
    : MeshViewer(title, width, height), smoother_(mesh_)
{
    crease_angle_ = 180.0;
    mesh_.set_ambient(0.3);
}

void Viewer::process_imgui()
{
    MeshViewer::process_imgui();

    ImGui::Spacing();
    ImGui::Spacing();

    if (ImGui::CollapsingHeader("Curvature", ImGuiTreeNodeFlags_DefaultOpen))
    {
        if (ImGui::Button("Mean Curvature"))
        {
            SurfaceCurvature analyzer(mesh_);
            analyzer.analyze_tensor(0, false);
            analyzer.mean_curvature_to_texture_coordinates();
            update_mesh();
            //            mesh_.use_cold_warm_texture();
            set_draw_mode("Texture");
        }
    }

    ImGui::Spacing();
    ImGui::Spacing();

    if (ImGui::CollapsingHeader("Energy Smoothing", ImGuiTreeNodeFlags_DefaultOpen))
    {
        static float alpha = 0.01;
        ImGui::PushItemWidth(100);
        ImGui::SliderFloat("alpha", &alpha, 0.01, 5);
        ImGui::PopItemWidth();

        if (ImGui::Button("Energy Minimize"))
        {
            pmp_pupa::SurfaceEnergySmoothing smoother(mesh_);
            smoother.implicit_smoothing(alpha);
            update_mesh();
        }
    }

    ImGui::Spacing();
    ImGui::Spacing();


    if (ImGui::CollapsingHeader("Shape optimization", ImGuiTreeNodeFlags_DefaultOpen))
    {
        static float alpha1 = 0.001;
        ImGui::PushItemWidth(100);
        ImGui::SliderFloat("alpha2", &alpha1, 0.01, 1);
        ImGui::PopItemWidth();

        if (ImGui::Button("Tangential Energy Minimize"))
        {
            pmp_pupa::SurfaceTangentialSmoothing smoother(mesh_);
            smoother.implicit_smoothing(alpha1);
            update_mesh();
        }
    }
}

int main(int argc, char** argv)
{
#ifndef __EMSCRIPTEN__
    Viewer window("Smoothing", 800, 600);
    if (argc == 2)
        window.load_mesh(argv[1]);
    return window.run();
#else
    Viewer window("Smoothing", 800, 600);
    window.load_mesh(argc == 2 ? argv[1] : "input.off");
    return window.run();
#endif
}
