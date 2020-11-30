//
// Created by $Pupa on 2020/10/7.
//

#pragma once

#include <pmp/SurfaceMesh.h>

namespace pmp_tools {

//!> geo-tools: base class for generate specific isoline on scalar field
class ScalarFieldSurfaceIsoline
{
public:
    ScalarFieldSurfaceIsoline(const pmp::SurfaceMesh& mesh,
                              const pmp::VertexProperty<double> values, double value)
        : mesh_(mesh), face_it_(mesh_.faces_begin()), y_(values), value_(value)
    {
        if (!is_intersected())
            this->operator++();
    }

    //! pre-increment (rotate couter-clockwise)
    ScalarFieldSurfaceIsoline& operator++()
    {
        for (++face_it_; face_it_ != mesh_.faces_end(); ++face_it_)
            if (is_intersected())
                return *this;
        return *this;
    }

    bool operator!=(const ScalarFieldSurfaceIsoline& rhs) const { return !operator==(rhs); }

    //! are two circulators equal?
    bool operator==(const ScalarFieldSurfaceIsoline& rhs) const
    {
        return (face_it_ == rhs.face_it_);
    }

    //! get the vertex the circulator refers to
    std::array<std::tuple<pmp::Halfedge, double>, 2> operator*() const
    {
        std::vector<std::tuple<pmp::Halfedge, double>> em_vertices;
        for (auto h : mesh_.halfedges(*face_it_))
        {
            double y_f = y_[mesh_.from_vertex(h)], y_t = y_[mesh_.to_vertex(h)];
            double alpha = (value_ - y_f) / (y_t - y_f);
            if (alpha >= 0 && alpha <= 1.0)
                em_vertices.push_back({h, alpha});
        }
        assert(em_vertices.size() == 2);
        return {em_vertices[0], em_vertices[1]};
    }

    // helper for C++11 range-based for-loops
    ScalarFieldSurfaceIsoline& begin() { return *this; }
    // helper for C++11 range-based for-loops
    ScalarFieldSurfaceIsoline& end()
    {
        face_it_ = mesh_.faces_end();
        return *this;
    }

private:
    bool is_intersected()
    {
        for (auto h : mesh_.halfedges(*face_it_))
            if ((y_[mesh_.from_vertex(h)] - value_) * (y_[mesh_.to_vertex(h)] - value_) < 0)
                return true;
        return false;
    }

    const pmp::SurfaceMesh& mesh_;
    pmp::VertexProperty<double> y_;
    double value_;
    pmp::SurfaceMesh::FaceIterator face_it_;
};

//!> geo-tools for intersection of plane and surface
class PlaneSurfaceIntersection : public ScalarFieldSurfaceIsoline
{
public:
    //! default constructor
    PlaneSurfaceIntersection(pmp::SurfaceMesh& mesh, Eigen::Vector3d n, double d)
        : ScalarFieldSurfaceIsoline(mesh, this->normal_dot_property(mesh, n), d), mesh_(mesh)
    {
    }

    pmp::VertexProperty<double> normal_dot_property(pmp::SurfaceMesh& mesh, pmp::Point n)
    {
        auto scalar = mesh.add_vertex_property<double>("v:normal_dot_scalar");
        for (auto v : mesh.vertices())
            scalar[v] = pmp::dot(n, mesh.position(v));
        return scalar;
    }

    ~PlaneSurfaceIntersection()
    {
        auto scalar = mesh_.get_vertex_property<double>("v:normal_dot_scalar");
        mesh_.remove_vertex_property(scalar);
    }

private:
    pmp::SurfaceMesh& mesh_;
};

} // namespace pmp_tools