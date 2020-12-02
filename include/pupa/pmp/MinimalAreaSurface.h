//
// Created by pupa on 12/1/20.
//

#pragma once

#include <pmp/SurfaceMesh.h>

namespace pmp_pupa {

using namespace pmp;

class MinimalAreaSurface
{
public:
    //! Construct with mesh to be processed.
    MinimalAreaSurface(SurfaceMesh& mesh);

    // destructor
    ~MinimalAreaSurface();

    void explicit_iterate(float lambda);

    void implicit_iterate(float lambda);

    void boundary_explicit_iterate(float lambda);

private:

    SurfaceMesh& mesh_; //!< the mesh
    size_t n_inner_vertices_{0};
    size_t n_boundary_vertices_{0};

    // property handles
    EdgeProperty<double> eweight_;

    VertexProperty<int> inner_idx_;
    VertexProperty<int> boundary_idx_;
};

} // namespace pmp_pupa