// Copyright 2011-2019 the Polygon Mesh Processing Library developers.
// Distributed under a MIT-style license, see LICENSE.txt for details.

#include <pmp/visualization/MeshViewer.h>
#include <pmp/algorithms/SurfaceParameterization.h>
#include <imgui.h>

#include "algorithm/CPCSurfaceUnfolding.h"

using namespace pmp;

class Viewer : public MeshViewer
{
public:
    Viewer(const char* title, int width, int height);

protected:
    virtual void process_imgui() override;
    virtual void draw(const std::string& _draw_mode) override;
};

Viewer::Viewer(const char* title, int width, int height)
    : MeshViewer(title, width, height){}

int frame_cnt = 0, frame_total = 25*60/4;
void Viewer::draw(const std::string& draw_mode)
{
    if(mesh_.has_vertex_property("v:scalp")) {
        auto v_dq = mesh_.get_vertex_property<eigen_ext::DualQuaternion<double>>("v:dualquat");
        auto v_scalp = mesh_.get_vertex_property<Eigen::Vector3d>("v:scalp");
        frame_cnt = (frame_cnt+1)%frame_total;
        double t  = double(frame_cnt)/frame_total;
        if(t < 0.5){
            t /= 0.5;
            DualQuaternions target = DualQuaternions(
                Eigen::Quaterniond(1, 0, 0, 0), Eigen::Quaterniond(0, 0, 0, 0));
            for (auto v : mesh_.vertices())
            {
                auto dq =
                    eigen_ext::sclerp(eigen_ext::DualQuaterniond(), v_dq[v], t);
//                mesh_.position(v) =  eigen_ext::transform_point(dq, v_scalp[v]).dual().vec();
            }
        }
    }
    update_mesh();
    MeshViewer::draw(draw_mode);
}

void Viewer::process_imgui()
{
    MeshViewer::process_imgui();

    ImGui::Spacing();
    ImGui::Spacing();

    if (ImGui::Button("CPC Decomposition")) {
        CPCSurfaceUnfolding unfolding(mesh_);
        unfolding.segmentation();
        update_mesh();
    }

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
