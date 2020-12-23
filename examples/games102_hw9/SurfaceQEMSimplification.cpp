//
// Created by pupa on 12/23/20.
//

#include "SurfaceQEMSimplification.h"

namespace pmp_pupa {

SurfaceQEMSimplification::SurfaceQEMSimplification(pmp::SurfaceMesh &mesh) : mesh_(mesh) {
    vquadric_ = mesh_.add_vertex_property<Quadric>("v:quadric");
    equadric_ = mesh_.add_edge_property<Quadric>("e:quadric");
    fnormal_ = mesh_.get_face_property<pmp::Normal>("f:normal");
}

} // namespace pmp_pupa