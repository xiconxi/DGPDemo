//
// Created by pupa on 2020/10/6.
//

#include <pupa/pmp/RealizedEmCurve.h>
#include <pmp/SurfaceMesh.h>

#include "EmCurveTools.h"
using namespace pmp_pupa;

RealizedEmCurvePolyMesh::RealizedEmCurvePolyMesh(pmp::SurfaceMesh& mesh)
    : host_mesh_(mesh), curve_mesh_(mesh)
{
    emv_infos_ = curve_mesh_.add_vertex_property<EMVertexInfo>("v:embedd");
    is_seam_ = curve_mesh_.add_edge_property<bool>("e:is_seam", false);
}

pmp::Halfedge RealizedEmCurvePolyMesh::hybrid_halfedge(const EMVertexInfo& em_v)
{
    pmp::Vertex host_vt = host_mesh_.to_vertex(em_v);
    for (auto h : curve_mesh_.halfedges(host_mesh_.from_vertex(em_v)))
    {
        if (!emv_infos_[curve_mesh_.to_vertex(h)].is_valid())
            continue;
        if (host_mesh_.to_vertex(emv_infos_[curve_mesh_.to_vertex(h)]) == host_vt)
            return h;
    }
    return pmp::Halfedge();
}

pmp::Halfedge RealizedEmCurvePolyMesh::next_hybrid_halfedge(pmp::Halfedge h)
{
    if (!is_embedded(curve_mesh_.to_vertex(h)))
        return pmp::Halfedge();
    return curve_mesh_.cw_rotated_halfedge(curve_mesh_.next_halfedge(h));
}

pmp::Halfedge RealizedEmCurvePolyMesh::prev_hybrid_halfedge(pmp::Halfedge h)
{
    if (!is_embedded(curve_mesh_.from_vertex(h)))
        return pmp::Halfedge();
    return curve_mesh_.ccw_rotated_halfedge(curve_mesh_.prev_halfedge(h));
}

pmp::Halfedge RealizedEmCurvePolyMesh::next_seam_halfedge(pmp::Halfedge )
{
    assert("Not implement");
}

pmp::Halfedge RealizedEmCurvePolyMesh::prev_seam_halfedge(pmp::Halfedge )
{
    assert("Not implement");
}

pmp::Halfedge RealizedEmCurvePolyMesh::insert_embedded_edge(EMVertexInfo em_v1, EMVertexInfo em_v2)
{
    pmp::Vertex v0 = _insert_embedded_vertex(em_v1);
    pmp::Vertex v1 = _insert_embedded_vertex(em_v2);

    for (auto h0 : curve_mesh_.halfedges(v0))
    {
        if (curve_mesh_.is_boundary(h0))
            h0 = curve_mesh_.opposite_halfedge(h0);
        else
            h0 = curve_mesh_.prev_halfedge(h0);
        auto h1 = curve_mesh_.next_halfedge(h0);
        for (; h1 != h0; h1 = curve_mesh_.next_halfedge(h1))
            if(curve_mesh_.to_vertex(h1) == v1)
            {
                auto new_h = curve_mesh_.insert_edge(h0, h1);
                is_seam_[curve_mesh_.edge(new_h)] = true;
                return new_h;
            }
    }
    assert("Failed to add edge");
    return pmp::Halfedge();
}

pmp::Vertex RealizedEmCurvePolyMesh::_insert_embedded_vertex(EMVertexInfo em_v)
{
    // check whether it's already inserted
    for (auto h : curve_mesh_.halfedges(host_mesh_.from_vertex(em_v)))
        if (emv_infos_[curve_mesh_.to_vertex(h)] == em_v)
            return curve_mesh_.to_vertex(h);

    // iterate all em_vertices which share the same host halfedge
    pmp::Halfedge insert_h = _search_hybrid(em_v);

    auto new_v = curve_mesh_.add_vertex(host_mesh_.point(em_v));
    emv_infos_[new_v] = em_v;
    curve_mesh_.insert_vertex(curve_mesh_.edge(insert_h), new_v);

    return new_v;
}

pmp::Halfedge RealizedEmCurvePolyMesh::_search_hybrid(const EMVertexInfo& em_v)
{
    pmp::Halfedge _h = hybrid_halfedge(em_v);
    if (!_h.is_valid())
        return em_v.host_h_;
    for (pmp::Halfedge h = _h; h.is_valid() && (_h = h).is_valid(); h = next_hybrid_halfedge(h))
    {
        if (emv_infos_[curve_mesh_.to_vertex(h)].w_ > em_v.w_)
            break;
    }
    return _h;
}

RealizedEmCurveSmoothing::RealizedEmCurveSmoothing(RealizedEmCurvePolyMesh& mesh): curve_mesh_(mesh){

}

void RealizedEmCurveSmoothing::implicitSmoothing() {

}