//
// Created by pupa on 2020/10/6.
//

#pragma once

#include <pmp/SurfaceMesh.h>

//! provide an embedded representation for the embedded networks of curve
class EmbeddedCurve
{
public:
    explicit EmbeddedCurve(pmp::SurfaceMesh& mesh) : mesh_(mesh), em_mesh_(mesh) {}

    void add_em_vertex(pmp::Halfedge h, double weight);

    void add_em_vertex(pmp::Face f, pmp::Vector<double, 3> weight);

    //! delete all curves vertices and lines and reset the mesh to original
    void clear();

private:
    struct EmbeddedComplex
    {
        pmp::Halfedge halfedge_;
        pmp::Vector<double, 3> weight_; //!< weights for from_v, to_v, opposite_v
        enum ComplexType
        {
            None,
            VertexComplex,
            EdgeComplex,
            FaceComplex
        } complex_type_;
    };

    // embedded representations
    pmp::VertexProperty<EmbeddedComplex> embedd_;

    pmp::SurfaceMesh mesh_;
    pmp::SurfaceMesh& em_mesh_;
};
