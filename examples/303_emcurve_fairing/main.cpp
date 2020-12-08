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

    void test();

private:
    std::shared_ptr<pmp_pupa::RealizedEmCurvePolyMesh> emCurve_;
};

Viewer::Viewer(const char* title, int width, int height)
    : MeshViewer(title, width, height)
{
    crease_angle_ = 180.0;
}

void Viewer::process_imgui()
{
    MeshViewer::process_imgui();

}

void Viewer::test()
{
    if(emCurve_.get() == nullptr)
        emCurve_ = std::make_shared<pmp_pupa::RealizedEmCurvePolyMesh>(mesh_);
    auto bbox = mesh_.bounds();
    for (double d = bbox.min()[2]; d < bbox.max()[2]; d += (bbox.max() - bbox.min())[2] / 8)
    {
        for (auto em_line : pmp_pupa::PlaneSurfaceIntersection(mesh_, Eigen::Vector3d(0, 0, 1), d))
        {
            auto em_v1 = pmp_pupa::EMVertexInfo(std::get<0>(em_line[0]), std::get<1>(em_line[0]));
            auto em_v2 = pmp_pupa::EMVertexInfo(std::get<0>(em_line[1]), std::get<1>(em_line[1]));
            emCurve_->insert_embedded_edge(em_v1, em_v2);
        }
    }
    emCurve_->mesh().write("../curve.off");
}

int main(int argc, char** argv)
{
#ifndef __EMSCRIPTEN__
    Viewer window("Smoothing", 800, 600);
    if (argc == 2)
    {
        window.load_mesh(argv[1]);
        window.test();
    }
    return window.run();
#else
    Viewer window("Smoothing", 800, 600);
    window.load_mesh(argc == 2 ? argv[1] : "input.off");
    return window.run();
#endif
}
