//
// Created by pupa on 2020/12/11.
//


#pragma once

#include <pmp/SurfaceMesh.h>
#include <pmp/algorithms/DifferentialGeometry.h>

#include <Eigen/Sparse>

#include <pupa/pmp/SurfaceSeamCurve.h>

namespace pmp_pupa {

class MinimalSeamCurve
{
public:
    MinimalSeamCurve(pmp_pupa::SeamedSurfaceMesh& mesh): seam_mesh_(mesh) {
        e_cotan = seam_mesh_.mesh().add_edge_property<double>("e:cotan");
        v_voronoi = seam_mesh_.mesh().add_vertex_property<double>("v:voronoi");
        initialize();
    }

    ~MinimalSeamCurve() {
        seam_mesh_.mesh().remove_edge_property(e_cotan);
        seam_mesh_.mesh().remove_vertex_property(v_voronoi);
    }


    void implicit_smoothing(double alpha = 0.01) {
        std::vector<pmp::Vertex> seam_vertices;
        auto& _mesh_ = seam_mesh_.mesh();
        for(auto v: _mesh_.vertices()) {
            if(seam_mesh_.is_seamed(v))
                seam_vertices.push_back(v);
        }
        std::cout << seam_vertices.size() << std::endl;

        for(auto v: seam_vertices) {
            for(auto h: _mesh_.halfedges(v)) {
                auto vv = _mesh_.to_vertex(h);
                if(!seam_mesh_.is_seamed(vv))
                    continue;
                std::cout << vv << '\t';
            }
            std::cout << std::endl;
        }
    }

    void initialize()
    {
        for (auto e : seam_mesh_.mesh().edges())
            e_cotan[e] = std::max(0.0, pmp::cotan_weight(seam_mesh_.mesh(), e));

        for (auto v : seam_mesh_.mesh().vertices())
            v_voronoi[v] = pmp::voronoi_area(seam_mesh_.mesh(), v);
    }

private:

    pmp_pupa::SeamedSurfaceMesh& seam_mesh_;
    pmp::EdgeProperty<double> e_cotan;
    pmp::VertexProperty<double> v_voronoi;
};

} // namespace pmp
