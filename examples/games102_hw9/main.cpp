//
// Created by pupa on 2020/12/10.
//

#include <pmp/visualization/MeshViewer.h>
#include <pmp/algorithms/SurfaceCurvature.h>
#include <imgui.h>
#include <memory>
#include <imgui_internal.h>
#include "SurfaceQEM.h"

using namespace pmp;


class Viewer : public MeshViewer
{
public:
    Viewer(const char* title, int width, int height);

protected:
    virtual void process_imgui();

    std::shared_ptr<pmp_pupa::LODSurfaceMesh> lod_mesh_{nullptr};
};

Viewer::Viewer(const char* title, int width, int height) : MeshViewer(title, width, height) {}

static char* files[]{"armadillo.obj", "sphere.obj", "bunny.off"};

struct AnimationCtrlData{
    bool collapsing_{false};
    bool splitting_{false};
}animation_;

void Viewer::process_imgui()
{
    MeshViewer::process_imgui();

    ImGui::Spacing();
#if __EMSCRIPTEN__
    static int n = 1;
    ImGui::PushItemWidth(50);
    ImGui::Combo("Files", &n, files, 3);
    ImGui::PopItemWidth();
    ImGui::SameLine();
    if (ImGui::Button("Reload Mesh"))
    {
        load_mesh(files[n]);
        set_draw_mode("Hidden Line");
        lod_mesh_.reset((pmp_pupa::LODSurfaceMesh*)nullptr);
        animation_.collapsing_ = false;
        animation_.splitting_ = false;
    }
#endif

    ImGui::Spacing();

    if (ImGui::CollapsingHeader("LOD", ImGuiTreeNodeFlags_DefaultOpen))
    {
        static int lod_level = 5;
        if (ImGui::Button("initialize"))
        {
            auto simplify_ = new pmp_pupa::SurfaceQEM(mesh_);

            auto simplify_sequence =
                simplify_->simplification(std::max(mesh_.n_vertices() * 0.01 * 5, 5.0));
            lod_mesh_.reset(new pmp_pupa::LODSurfaceMesh(mesh_, simplify_sequence));

//            while (++(*lod_mesh_) < lod_mesh_->max_vertices())
//                ;
//            while (--(*lod_mesh_) > lod_mesh_->min_vertices())
//                ;
            mesh_.garbage_collection();
            lod_level = lod_mesh_->min_vertices();
            update_mesh();
        }

        if(lod_mesh_)
        {
            ImGui::PushItemWidth(150);
            ImGui::SliderInt("n", &lod_level, lod_mesh_->min_vertices(),
                             lod_mesh_->max_vertices());
            ImGui::PopItemWidth();
            if (ImGui::Button("Go")){
                for(size_t i = mesh_.n_vertices(); i < lod_level; i++)
                    ++(*lod_mesh_);
                for(size_t i = mesh_.n_vertices(); i > lod_level; i--)
                    --(*lod_mesh_);
                mesh_.garbage_collection();
                update_mesh();
            }
            ImGui::SameLine();
            if(ImGui::Button("Go --")) {
                animation_.collapsing_ = !animation_.collapsing_;
                animation_.splitting_ = false;
            }
            ImGui::SameLine();
            if(ImGui::Button("Go ++")) {
                animation_.collapsing_ = false;
                animation_.splitting_ = !animation_.splitting_;
            }

            size_t lod_levels = lod_mesh_->max_vertices() - lod_mesh_->min_vertices();
            if(animation_.splitting_)
            {
                for(int i = 0; i < lod_levels/60 && animation_.splitting_; i++)
                    animation_.splitting_ = ++(*lod_mesh_) < lod_mesh_->max_vertices();
                mesh_.garbage_collection();
                update_mesh();
                lod_level = mesh_.n_vertices();
            }else if (animation_.collapsing_) {
                for(int i = 0; i < lod_levels/60 && animation_.collapsing_; i++)
                    animation_.collapsing_ = --(*lod_mesh_) > lod_mesh_->min_vertices();
                mesh_.garbage_collection();
                update_mesh();
                lod_level = mesh_.n_vertices();
            }
        }
    }
}


int main(int argc, char** argv)
{

    std::cout << sizeof(pmp::Point) <<' ' << sizeof(pmp::Vertex) << ' ' <<sizeof(pmp_pupa::CollapseData) << std::endl;
#ifndef __EMSCRIPTEN__
    Viewer window("Game102-HW9", 800, 600);
    if (argc == 2)
        window.load_mesh(argv[1]);
    return window.run();
#else
    Viewer window("Game102-HW9", 800, 600);
    window.load_mesh(argc == 2 ? argv[1] : "sphere.obj");
    return window.run();
#endif
}
