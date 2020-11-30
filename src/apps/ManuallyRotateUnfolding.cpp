// Copyright 2011-2019 the Polygon Mesh Processing Library developers.
// Distributed under a MIT-style license, see LICENSE.txt for details.

#include <pmp/visualization/MeshViewer.h>
#include <pmp/algorithms/SurfaceParameterization.h>
#include <imgui.h>

#include "../../include/CPCSurfaceUnfolding.h"

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

int frame_cnt = 0, frame_rate = 5;
int frame_delta = 1;
void Viewer::draw(const std::string& draw_mode)
{
    if(mesh_.has_vertex_property("v:scalp")) {
        auto v_dq = mesh_.get_vertex_property<std::vector<eigen_ext::DualQuaternion<double>>>("v:dualquat");
        auto v_scalp = mesh_.get_vertex_property<Eigen::Vector3d>("v:scalp");
        int frame_total = (v_dq[pmp::Vertex(0)].size()+3)*frame_rate;
        frame_cnt += frame_delta;
        if(frame_cnt == frame_total || frame_cnt == 0)
            frame_delta *= -1;
        double motion_i = int(frame_cnt/frame_rate);
        double t  = (frame_cnt%frame_rate)/double(frame_rate);
        if(motion_i+1 < v_dq[pmp::Vertex(0)].size())
        {
            for (auto v : mesh_.vertices())
            {
                auto dq = eigen_ext::sclerp(v_dq[v][motion_i], v_dq[v][motion_i+1], t);
                mesh_.position(v) = eigen_ext::transform_point(dq, v_scalp[v]).dual().vec();
            }
        }
//        frame_cnt = (frame_cnt+1)%frame_total;
//        double t  = double(frame_cnt)/frame_total;
//        if(t < 0.5){
//            t /= 0.5;
//            for (auto v : mesh_.vertices()){
//                auto dq = eigen_ext::sclerp(eigen_ext::DualQuaterniond(), v_dq[v][0], t);
//                mesh_.position(v) =  eigen_ext::transform_point(dq, v_scalp[v]).dual().vec();
//            }
//        }
    }
    update_mesh();
    MeshViewer::draw(draw_mode);
}

void Viewer::process_imgui()
{
    MeshViewer::process_imgui();

    ImGui::Spacing();
    ImGui::Spacing();

    if (ImGui::Button("CPC Unfolding")) {
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
