// Copyright 2011-2019 the Polygon Mesh Processing Library developers.
// Distributed under a MIT-style license, see LICENSE.txt for details.

#include <pmp/visualization/MeshViewer.h>
#include <pmp/algorithms/SurfaceParameterization.h>
#include <imgui.h>


#include "algorithm/TriangleStripUnfolding.h"

using namespace pmp;

class Viewer : public MeshViewer
{
public:
    Viewer(const char* title, int width, int height);

protected:
    virtual void process_imgui() override;
    virtual void draw(const std::string& _draw_mode) override;
};

int n = 50;

Viewer::Viewer(const char* title, int width, int height)
    : MeshViewer(title, width, height){

    for(int i = 0; i < n; i++)
        mesh_.add_vertex(pmp::Point(1, std::sin(i / float(n) * M_PI*2), std::cos(i / float(n) * M_PI*2) ));
    for(int i = 0; i < n; i++)
        mesh_.add_vertex(pmp::Point(-1, std::sin(i / float(n) * M_PI*2), std::cos(i / float(n) * M_PI*2)));

    for(int i = 1; i < n; i++)
    {
        mesh_.add_triangle(pmp::Vertex(i-1), pmp::Vertex(i), pmp::Vertex(n+i-1));
        mesh_.add_triangle(pmp::Vertex(i), pmp::Vertex(i+n), pmp::Vertex(n+i-1));
    }
}


void Viewer::draw(const std::string& draw_mode)
{
    update_mesh();
    MeshViewer::draw(draw_mode);
}

void Viewer::process_imgui()
{
    MeshViewer::process_imgui();

    ImGui::Spacing();
    ImGui::Spacing();

    if (ImGui::Button("Edge Laplace Unfolding")) {
        auto strip_h = mesh_.find_halfedge(pmp::Vertex(n), pmp::Vertex(0));
        auto surface_strip = pmp_ext::SurfaceTriangleStrip(mesh_, strip_h);
        surface_strip.edge_laplacian_unfolding();
//        CPCSurfaceUnfolding unfolding(mesh_);
//        unfolding.segmentation();

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
