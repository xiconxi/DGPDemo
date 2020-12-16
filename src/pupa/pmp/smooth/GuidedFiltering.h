//
// Created by pupa on 12/15/20.
//
#pragma once

#include <pmp/SurfaceMesh.h>
#include <pmp/algorithms/SurfaceNormals.h>

namespace pmp_pupa{
using namespace pmp;

class GuidedFiltering
{
public:
    explicit GuidedFiltering(pmp::SurfaceMesh& mesh): mesh_(mesh){
        f_normal_ = mesh_.add_face_property<pmp::Normal>("f:normal");
        pmp::SurfaceNormals::compute_face_normals(mesh);
    }



private:
    pmp::SurfaceMesh& mesh_;
    pmp::FaceProperty<pmp::Normal> f_normal_;
};

}