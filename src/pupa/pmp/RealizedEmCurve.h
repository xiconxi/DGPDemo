//
// Created by pupa on 2020/10/6.
//

#pragma once

#include <pmp/SurfaceMesh.h>
#include <float.h>

namespace pmp_pupa {

struct EMVertexInfo
{
    EMVertexInfo() = default;
    EMVertexInfo(pmp::Halfedge h, double w) :  w_(w), host_h_(h)
    {
        if ((host_h_.idx() & 1) == 0)
            return;
        host_h_ = pmp::Halfedge(h.idx() - 1);
        w_ = 1 - w;
    }
    bool operator==(const EMVertexInfo& rhs) const
    {
        return host_h_ == rhs.host_h_ and abs(w_ - rhs.w_) < 1e-5; //DBL_EPSILON;
    }

    bool operator^(const EMVertexInfo& rhs) const { return host_h_ == rhs.host_h_; }

    bool is_valid() const { return host_h_.is_valid(); }

    double w_;
    pmp::Halfedge host_h_; //!< p = from(sh_)+w_*(from(sh_)-to(sh_))
};

//>! a bridge to original mesh
struct HostSurfaceMesh
{
    HostSurfaceMesh(const pmp::SurfaceMesh& mesh) : mesh_(mesh) {}

    pmp::Vertex from_vertex(const EMVertexInfo& ev) const { return mesh_.from_vertex(ev.host_h_); }

    pmp::Vertex to_vertex(const EMVertexInfo& ev) const { return mesh_.to_vertex(ev.host_h_); }

    pmp::Point point(const EMVertexInfo& ev) const
    {
        auto pf = mesh_.position(from_vertex(ev));
        auto pt = mesh_.position(to_vertex(ev));
        return pf + (pt - pf) * ev.w_;
    }

    const pmp::SurfaceMesh mesh_;
};

class RealizedEmCurvePolyMesh
{
public:

    explicit RealizedEmCurvePolyMesh(pmp::SurfaceMesh& mesh);

    pmp::Halfedge add_embedded_edge(EMVertexInfo em_v1, EMVertexInfo em_v2);

    bool is_embedded(pmp::Vertex v) { return emv_infos_[v].is_valid(); }

    bool is_allocated(const EMVertexInfo& em_v);

    pmp::Halfedge insert_embedded_edge(EMVertexInfo em_v1, EMVertexInfo em_v2);

    pmp::Halfedge hybrid_halfedge(const EMVertexInfo& em_v);

    pmp::Halfedge next_hybrid_halfedge(pmp::Halfedge hybrid_h);

    pmp::Halfedge prev_hybrid_halfedge(pmp::Halfedge hybrid_h);

    pmp::Halfedge next_seam_halfedge(pmp::Halfedge seam_h);

    pmp::Halfedge prev_seam_halfedge(pmp::Halfedge seam_h);

    pmp::SurfaceMesh& mesh() { return curve_mesh_; }

private:
    pmp::Vertex _insert_embedded_vertex(EMVertexInfo em_v1);
    pmp::Halfedge _search_hybrid(const EMVertexInfo& em_v);

    HostSurfaceMesh host_mesh_;
    pmp::SurfaceMesh& curve_mesh_;
    pmp::VertexProperty<EMVertexInfo> emv_infos_;
    pmp::EdgeProperty<bool> is_seam_;
};

class RealizedEmCurveSmoothing{
public:
    RealizedEmCurveSmoothing(RealizedEmCurvePolyMesh& mesh);

    void implicitSmoothing();

private:
    RealizedEmCurvePolyMesh& curve_mesh_;
};

} // namespace pmp_pupa