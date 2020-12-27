//
// Created by pupa on 12/23/20.
//

#include "SurfaceQEMSimplification.h"

namespace pmp_pupa {

SurfaceQEMSimplification::SurfaceQEMSimplification(pmp::SurfaceMesh &mesh) : mesh_(mesh) {
    vquadric_ = mesh_.add_vertex_property<Quadric>("v:quadric");
    fnormal_ = mesh_.get_face_property<pmp::Normal>("f:normal");
}

struct  EdgeQuadricDistance  {

    explicit EdgeQuadricDistance(pmp::SurfaceMesh& mesh):mesh_(mesh) {
        idx_ = mesh_.add_edge_property<std::int32_t>("e:heap_idx");
        eprio_ = mesh_.add_edge_property<double>("e:eprio_");
    }

    ~EdgeQuadricDistance(){
        mesh_.remove_edge_property(idx_);
        mesh_.remove_edge_property(eprio_);
    }

    std::int32_t& operator [](pmp::Edge e) {return idx_[e];}
    double& operator ()(pmp::Edge e) {return eprio_[e];}

private:
    pmp::EdgeProperty<double> eprio_;
    pmp::EdgeProperty<std::int32_t> idx_;
    pmp::SurfaceMesh& mesh_;
};

void SurfaceQEMSimplification::simplification(size_t n_vertex)
{
    EdgeQuadricDistance quadric_distance(mesh_);


    MinHeap<pmp::Vertex, EdgeQuadricDistance> heap(quadric_distance);

}

} // namespace pmp_pupa