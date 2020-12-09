//
// Created by pupa on 12/8/20.
//
#include <pupa/pmp/RealizedEmCurve.h>
#include <pupa/pmp/EmCurveTools.h>
#include <pmp/visualization/MeshViewer.h>
#include <imgui.h>
#include <memory>

using namespace pmp;

class Viewer : public MeshViewer
{
public:
    Viewer(const char* title, int width, int height);

    void process_imgui() override;

    void isoline_test();

    void circle_test(int n = 20);

private:
    std::shared_ptr<pmp_pupa::RealizedEmCurvePolyMesh> emCurve_;
};

Viewer::Viewer(const char* title, int width, int height) : MeshViewer(title, width, height)
{
    crease_angle_ = 180.0;
}

void Viewer::process_imgui()
{
    MeshViewer::process_imgui();
}

void Viewer::isoline_test()
{
    if (emCurve_.get() == nullptr)
        emCurve_ = std::make_shared<pmp_pupa::RealizedEmCurvePolyMesh>(mesh_);
    auto bbox = mesh_.bounds();
    for (double d = bbox.min()[2]; d < bbox.max()[2]; d += (bbox.max() - bbox.min())[2] / 8)
    {
        std::vector<pmp_pupa::EMVertexInfo> em_lines;
        for (auto em_line : pmp_pupa::PlaneSurfaceIntersection(mesh_, Eigen::Vector3d(0, 0, 1), d))
        {
            auto em_v1 = pmp_pupa::EMVertexInfo(std::get<0>(em_line[0]), std::get<1>(em_line[0]));
            auto em_v2 = pmp_pupa::EMVertexInfo(std::get<0>(em_line[1]), std::get<1>(em_line[1]));

            em_lines.push_back(em_v1);
            em_lines.push_back(em_v2);
            //            emCurve_->insert_embedded_edge(em_v1, em_v2);
        }
        for (int i = 0; i < em_lines.size(); i += 2)
        {
            emCurve_->insert_embedded_edge(em_lines[i], em_lines[i + 1]);
        }
    }
    emCurve_->mesh().write("../curve.off");
    update_mesh();
}

void Viewer::circle_test(int n)
{
    // generate quad-mesh
    for (int i = 0; i < n; i++)
        for (int j = 0; j < n; j++)
        {
            pmp::Point p(2 * i / double(n - 1) - 1.0, 2 * j / double(n - 1) - 1.0, 0);
            mesh_.add_vertex(p);
        }


    for (int i = 1; i < n; i++)
        for (int j = 1; j < n; j++)
            mesh_.add_face({pmp::Vertex(i * n + j), pmp::Vertex((i - 1) * n + j),
                            pmp::Vertex((i - 1) * n + j - 1), pmp::Vertex(i * n + j - 1)});


    std::vector<std::array<pmp_pupa::EMVertexInfo,2>> em_lines;

    if (emCurve_.get() == nullptr)
        emCurve_ = std::make_shared<pmp_pupa::RealizedEmCurvePolyMesh>(mesh_);
    // circle seam
    std::vector<pmp_pupa::EMVertexInfo> emv;
    for(auto f: mesh_.faces()){
        emv.clear();
        for(auto h: mesh_.halfedges(f)) {
            double d1 = pmp::norm(mesh_.position(mesh_.from_vertex(h)));
            double d2 = pmp::norm(mesh_.position(mesh_.to_vertex(h)));
            if((d1-0.8)*(d2-0.8) <  0)
                emv.push_back({h, 0.5});
        }
        if(emv.size() == 2)
            em_lines.push_back({emv[0], emv[1]});
    }

    for (int i = 0; i < em_lines.size(); i ++)
    {
        emCurve_->insert_embedded_edge(em_lines[i][0], em_lines[i][1]);
    }

//    mesh_.triangulate();

    mesh_.write("plane.off");
    update_mesh();
    view_all();
}

int main(int argc, char** argv)
{
#ifndef __EMSCRIPTEN__
    Viewer window("Smoothing", 800, 600);
    //    if (argc == 2)
    //        window.load_mesh(argv[1]);

    window.circle_test();
    return window.run();
#else
    Viewer window("Smoothing", 800, 600);
    window.load_mesh(argc == 2 ? argv[1] : "input.off");
    return window.run();
#endif
}
