//
// Created by pupa on 2020/10/6.
//

#pragma once

#include <pmp/SurfaceMesh.h>
#include <float.h>
namespace pmp_ext {

//! provide an embedded representation for the embedded networks of curve
class RealizedEmbeddedCurve
{
private:
    struct EMVertexInfo
    {
        EMVertexInfo() = default;
        EMVertexInfo(pmp::Halfedge h, double w) : sh_(h), w_(w)
        {
            if (!(sh_.idx() & 1))
                return;
            sh_ = pmp::Halfedge(h.idx() - 1);
            w_ = 1 - w;
        }
        bool operator==(const EMVertexInfo& rhs) const
        {
            return sh_ == rhs.sh_ and abs(w_ - rhs.w_) < 1e-5; //DBL_EPSILON;
        }

        bool operator^(const EMVertexInfo& rhs) const { return sh_ == rhs.sh_; }

        operator bool() const { return sh_.is_valid(); }
        double w_;
        pmp::Halfedge sh_; //!< p = from(sh_)+w_*(from(sh_)-to(sh_))
    };

    // embedded representations
    pmp::VertexProperty<EMVertexInfo> em_infos_;

    pmp::SurfaceMesh& emesh_;
    pmp::SurfaceMesh smesh_; //!< const references

public:
    //! helper class for iterating through all EMComplex using range-based
    class EMVerticesCirculator
    {
    public:
        //! default constructor
        EMVerticesCirculator(const pmp::SurfaceMesh& mesh, pmp::VertexProperty<EMVertexInfo> embedd)
            : mesh_(mesh), embedd_(embedd)
        {
            vertex_ = pmp::Vertex(0);
            if (!embedd_[vertex_])
                this->operator++();
        }

        //! pre-increment (rotate couter-clockwise)
        EMVerticesCirculator& operator++()
        {
            do
                vertex_ = pmp::Vertex(vertex_.idx() + 1);
            while (vertex_.idx() < mesh_.n_vertices() && !embedd_[vertex_]);
            return *this;
        }

        bool operator!=(const EMVerticesCirculator& rhs) const { return !operator==(rhs); }

        //! are two circulators equal?
        bool operator==(const EMVerticesCirculator& rhs) const { return (vertex_ == rhs.vertex_); }

        //! get the vertex the circulator refers to
        EMVertexInfo operator*() const { return embedd_[vertex_]; }

        // helper for C++11 range-based for-loops
        EMVerticesCirculator& begin() { return *this; }
        // helper for C++11 range-based for-loops
        EMVerticesCirculator& end()
        {
            vertex_ = pmp::Vertex(mesh_.n_vertices());
            return *this;
        }

    private:
        const pmp::SurfaceMesh& mesh_;
        pmp::VertexProperty<EMVertexInfo> embedd_;
        pmp::Vertex vertex_;
    };

    //! helper class for iterating through all EMComplex using range-based
    class EMVertexListCirculator
    {
    public:
        //! default constructor
        EMVertexListCirculator(const pmp::SurfaceMesh& mesh, pmp::Vertex v_start,
                               pmp::VertexProperty<EMVertexInfo> embedd, pmp::Halfedge embedded_h)
            : mesh_(mesh), embedd_(embedd), embedded_sh_(embedded_h)
        {
            for (auto h : mesh_.halfedges(v_start))
                if (embedd_[mesh_.to_vertex(h)].sh_ == embedded_sh_)
                {
                    halfedge_ = mesh_.opposite_halfedge(h);
                    break;
                }
            assert(!mesh_.is_triangle_mesh() and !mesh_.is_quad_mesh());
        }

        //! pre-increment (rotate couter-clockwise)
        EMVertexListCirculator& operator++()
        {
            halfedge_ = mesh_.cw_rotated_halfedge(halfedge_);
            halfedge_ = mesh_.cw_rotated_halfedge(halfedge_);
            if (embedd_[mesh_.from_vertex(halfedge_)].sh_ != embedded_sh_)
                halfedge_ = pmp::Halfedge();
            else
                halfedge_ = mesh_.opposite_halfedge(halfedge_);
            return *this;
        }

        bool operator!=(const EMVertexListCirculator& rhs) const { return !operator==(rhs); }

        //! are two circulators equal?
        bool operator==(const EMVertexListCirculator& rhs) const
        {
            return (halfedge_ == rhs.halfedge_);
        }

        //! get the vertex the circulator refers to
        pmp::Halfedge operator*() const { return mesh_.opposite_halfedge(halfedge_); }

        // helper for C++11 range-based for-loops
        EMVertexListCirculator& begin() { return *this; }
        // helper for C++11 range-based for-loops
        EMVertexListCirculator& end()
        {
            halfedge_ = pmp::Halfedge();
            return *this;
        }

    private:
        const pmp::SurfaceMesh& mesh_;
        pmp::VertexProperty<EMVertexInfo> embedd_;
        pmp::Halfedge halfedge_, embedded_sh_;
    };

    explicit RealizedEmbeddedCurve(pmp::SurfaceMesh& mesh);

    //! h is the const reference mesh's halfedge, for multiple embedded vertices on one edge.
    pmp::Halfedge add_em_vertex(pmp::Halfedge sh, double weight);

    pmp::Halfedge add_em_edge(pmp::Halfedge sh1, double w1, pmp::Halfedge sh2, double w2);

    void add_em_vertex(pmp::Face f, pmp::Vector<double, 3> weight);

    //! delete all curves vertices and lines and reset the mesh to original
    void clear();

    bool is_embedded(pmp::Vertex v) { return em_infos_[v]; }

    bool is_embedded(pmp::Halfedge h)
    {
        return is_embedded(emesh_.from_vertex(h)) and is_embedded(emesh_.to_vertex(h));
    }

    EMVerticesCirculator em_vertices() { return EMVerticesCirculator(emesh_, em_infos_); }
};

} // namespace pmp_ext
