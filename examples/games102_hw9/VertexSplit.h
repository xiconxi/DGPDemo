//
// Created by pupa on 1/2/21.
//

#pragma once

#include <pmp/SurfaceMesh.h>

namespace pmp_pupa {


void swap(pmp::SurfaceMesh& mesh, pmp::Vertex v0, pmp::Vertex v1);

//!> insert a boundary loop, different from openmesh
pmp::Halfedge insert_loop(pmp::SurfaceMesh& mesh, pmp::Halfedge h);

pmp::Halfedge insert_edge(pmp::SurfaceMesh& mesh, pmp::Vertex v,  pmp::Halfedge h0, pmp::Halfedge h1);

pmp::Halfedge vertex_split(pmp::SurfaceMesh& mesh_, pmp::Vertex v0, pmp::Vertex v1, pmp::Vertex vl,
                         pmp::Vertex vr);

//{
//    pmp::Halfedge v1vl, vlv1, vrv1, v0v1;
//    // build loop from halfedge v1->vl
//    if (vl.is_valid())
//    {
//        v1vl = mesh_.find_halfedge(v1, vl);
//        assert(v1vl.is_valid());
//        mesh_.insert_vertex() vlv1 = insert_loop(v1vl);
//    }
//
//    // build loop from halfedge vr->v1
//    if (vr.is_valid())么撒
//    {
//        vrv1 = find_halfedge(vr, v1);
//        assert(vrv1.is_valid());
//        insert_loop(vrv1);
//    }
//
//    // handle boundary cases
//    if (!vl.is_valid())
//        vlv1 = prev_halfedge_handle(halfedge_handle(v1));
//    if (!vr.is_valid())
//        vrv1 = prev_halfedge_handle(halfedge_handle(v1));
//
//    // split vertex v1 into edge v0v1
//    v0v1 = insert_edge(v0, vlv1, vrv1);
//
//    return v0v1;
//}
}