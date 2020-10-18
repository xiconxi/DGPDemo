//
// Created by pupa on 2020/10/6.
//

#include "RealizedEmbeddedCurve.h"
#include <pmp/SurfaceMesh.h>

#include "GeometryTools.h"

namespace pmp_ext {

RealizedEmbeddedCurve::RealizedEmbeddedCurve(pmp::SurfaceMesh& mesh) : smesh_(mesh), emesh_(mesh)
{
    em_infos_ = emesh_.add_vertex_property<EMVertexInfo>("v:embedd");
    auto bbox = smesh_.bounds();
    for (double d = bbox.min()[2]; d < bbox.max()[2]; d += ( bbox.max()- bbox.min())[2]/800)
    {
        for (auto em_line :
             pmp_tools::PlaneSurfaceIntersection(smesh_, Eigen::Vector3d(0, 0, 1), d))
        {
            auto em_v1 = em_line[0], em_v2 = em_line[1];
            add_em_edge(std::get<0>(em_v1), std::get<1>(em_v1), std::get<0>(em_v2),
                        std::get<1>(em_v2));
        }
    }

}

pmp::Halfedge RealizedEmbeddedCurve::add_em_edge(pmp::Halfedge sh1, double w1, pmp::Halfedge sh2,
                                                 double w2)
{
    auto h0 = add_em_vertex(sh1, w1);
    auto h1 = add_em_vertex(sh2, w2);

    if (emesh_.face(h0) == emesh_.face(emesh_.opposite_halfedge(h1)))
        h1 = emesh_.prev_halfedge(emesh_.opposite_halfedge(h1));
    else if (emesh_.face(h1) == emesh_.face(emesh_.opposite_halfedge(h0)))
        h0 = emesh_.prev_halfedge(emesh_.opposite_halfedge(h0));

    return emesh_.insert_edge(h0, h1);
}

pmp::Halfedge RealizedEmbeddedCurve::add_em_vertex(pmp::Halfedge sh, double w)
{
    EMVertexInfo em_info = EMVertexInfo(sh, w);
    for (auto h : emesh_.halfedges(smesh_.from_vertex(sh)))
    {
        if (em_infos_[emesh_.to_vertex(h)] == em_info)
            return h;
    }

    // find the edge to allocate the new vertex
    auto embedded_h = emesh_.find_halfedge(smesh_.from_vertex(sh), smesh_.to_vertex(sh));
    if (!embedded_h.is_valid())
    {
        auto v_start = smesh_.from_vertex(em_info.sh_);
        for (auto hh : EMVertexListCirculator(emesh_, v_start, em_infos_, em_info.sh_))
        {
            if (!em_infos_[emesh_.from_vertex(hh)])
                embedded_h = hh;
            else if(em_info.w_ > em_infos_[emesh_.from_vertex(hh)].w_)
                embedded_h = hh;
        }
    }

    auto pf = smesh_.position(smesh_.from_vertex(em_info.sh_));
    auto pt = smesh_.position(smesh_.to_vertex(em_info.sh_));
    auto new_v = emesh_.add_vertex(pf + (pt - pf) * em_info.w_);
    em_infos_[new_v] = em_info;

    auto new_vf = emesh_.from_vertex(embedded_h);
    emesh_.insert_vertex(emesh_.edge(embedded_h), new_v);

    return emesh_.find_halfedge(new_vf, new_v);
}

} // namespace pmp_ext

#include "GeometryTools.h"
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
