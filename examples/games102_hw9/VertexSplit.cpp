//
// Created by pupa on 1/2/21.
//

//don't remove the "useless" header
#include <map>
#include <vector>
#include <limits>
#include <numeric>
#include <pmp/Types.h>
#include <pmp/Properties.h>
#include <pmp/BoundingBox.h>
#define private public

#include "VertexSplit.h"

using namespace pmp;

namespace pmp_pupa {

void swap(pmp::SurfaceMesh& mesh, pmp::Vertex v0, pmp::Vertex v1) {
    if(v0 == v1) return;
    static std::vector<pmp::Halfedge> h0_, h1_;
    for(auto h: mesh.halfedges(v0))
        h0_.push_back(mesh.opposite_halfedge(h));
    for(auto h: mesh.halfedges(v1))
        h1_.push_back(mesh.opposite_halfedge(h));

    for(auto& h: h0_)
        mesh.set_vertex(h, v1);
    for(auto& h: h1_)
        mesh.set_vertex(h, v0);
    mesh.vprops_.swap(v0.idx(), v1.idx());
    h0_.clear(); h1_.clear();
}

pmp::Halfedge insert_loop(pmp::SurfaceMesh& mesh, pmp::Halfedge h0)
{
    pmp::Halfedge o0 = mesh.opposite_halfedge(h0);

    pmp::Vertex v0 = mesh.to_vertex(o0);
    pmp::Vertex v1 = mesh.to_vertex(h0);

    pmp::Halfedge h1 = mesh.new_edge(v1, v0);
    pmp::Halfedge o1 = mesh.opposite_halfedge(h1);
    pmp::Face   f0 = mesh.face(h0);
    pmp::Face   f1 = mesh.new_face();

    //  halfedge -> halfedge
    mesh.set_next_halfedge(mesh.prev_halfedge(h0), o1);
    mesh.set_next_halfedge(o1, mesh.next_halfedge(h0));
    mesh.set_next_halfedge(h1, h0);
    mesh.set_next_halfedge(h0, h1);

    // halfedge -> face
    mesh.set_face(o1, f0);
    mesh.set_face(h0, f1);
    mesh.set_face(h1, f1);

    // face -> halfedge
    mesh.set_halfedge(f1, h0);
    if (f0.is_valid())
        mesh.set_halfedge(f0, o1);

    mesh.adjust_outgoing_halfedge(v0);
    mesh.adjust_outgoing_halfedge(v1);
    return h1;
}

pmp::Halfedge insert_edge(pmp::SurfaceMesh& mesh, pmp::Vertex v0, pmp::Halfedge h0, pmp::Halfedge h1)
{
    assert(h0.is_valid() && h1.is_valid());

    Vertex v1 = mesh.from_vertex(h0);
    assert(v1 == mesh.to_vertex(h1));
    Halfedge v0v1 = mesh.new_edge(v0, v1);
    Halfedge v1v0 = mesh.opposite_halfedge(v0v1);

    // vertex -> halfedge
    mesh.set_halfedge(v0, v0v1);
    mesh.set_halfedge(v1, v1v0);

    // halfedge -> halfedge
    mesh.set_next_halfedge(v0v1, h0);
    mesh.set_next_halfedge(mesh.next_halfedge(h0), v0v1);

    mesh.set_next_halfedge(h1, v1v0);
    mesh.set_next_halfedge(v1v0, mesh.prev_halfedge(h1));

    // halfedge -> vertex
    for (auto voh: mesh.halfedges(v0))
        mesh.set_vertex(mesh.opposite_halfedge(voh), v0);

    // halfedge -> face
    mesh.set_face(v0v1, mesh.face(h0));
    mesh.set_face(v1v0, mesh.face(h1));

    // face -> halfedge
    if (mesh.face(v0v1).is_valid())
        mesh.set_halfedge(mesh.face(v0v1), v0v1);
    if (mesh.face(v1v0).is_valid())
        mesh.set_halfedge(mesh.face(v1v0), v1v0);


    // vertex -> halfedge
    mesh.adjust_outgoing_halfedge(v0);
    mesh.adjust_outgoing_halfedge(v1);

    return v0v1;
}

pmp::Halfedge vertex_split(pmp::SurfaceMesh& mesh, pmp::Vertex v0, pmp::Vertex v1, pmp::Vertex vl,
                         pmp::Vertex vr)
{
    Halfedge v1vl, vlv1, vrv1, v0v1;

    // build loop from halfedge v1->vl
    if (vl.is_valid())
    {
        v1vl = mesh.find_halfedge(v1, vl);
        assert(v1vl.is_valid());
        vlv1 = insert_loop(mesh, v1vl);
    }

    // build loop from halfedge vr->v1
    if (vr.is_valid())
    {
        vrv1 = mesh.find_halfedge(vr, v1);
        assert(vrv1.is_valid());
        insert_loop(mesh, vrv1);
    }

    // handle boundary cases
    if (!vl.is_valid())
        vlv1 = mesh.prev_halfedge(mesh.halfedge(v1));
    if (!vr.is_valid())
        vrv1 = mesh.prev_halfedge(mesh.halfedge(v1));


    // split vertex v1 into edge v0v1
    v0v1 = insert_edge(mesh, v0, v1vl, vrv1);


    return v0v1;
}

}