//
// Created by pupa on 12/23/20.
//

#include "SurfaceQEM.h"
#include <pmp/algorithms/SurfaceNormals.h>

namespace pmp_pupa {

SurfaceQEM::SurfaceQEM(pmp::SurfaceMesh &mesh)
    : mesh_(mesh), quadric_distance(mesh_), heap_(quadric_distance)
{
    vquadric_ = mesh_.vertex_property<Quadric>("v:quadric");
    fnormal_ = mesh_.face_property<pmp::Normal>("f:normal");
    eoptimal_q_ = mesh_.edge_property<pmp::Point>("e:minima_q");

    pmp::SurfaceNormals::compute_face_normals(mesh_);
    for(auto v: mesh_.vertices()) {
        pmp::Point p = mesh_.position(v);
        for(auto f: mesh_.faces(v))
            vquadric_[v] += Quadric(fnormal_[f], p);
    }
}

void SurfaceQEM::simplification(size_t n_vertex)
{
    initial_quadric();
    while(heap_.size() && mesh_.n_vertices() > n_vertex) {
        auto e = heap_.pop_front();

//        std::cout << mesh_.n_vertices() << ' ' << n_vertex << ' '<< e << ' ' << quadric_distance(e)  << std::endl;

        auto h = mesh_.halfedge(e, 0);

        auto v_f = mesh_.from_vertex(h), v_t = mesh_.to_vertex(h);
        if( mesh_.is_boundary(v_f) || mesh_.is_boundary(v_t))
            continue;
        if( !mesh_.is_collapse_ok(h)) {
            if(!mesh_.is_collapse_ok(mesh_.halfedge(e, 1)))
                continue;
            h = mesh_.opposite_halfedge(h);
            std::swap(v_t, v_f);
        }
        vquadric_[v_t] += vquadric_[v_f];
        mesh_.collapse(h);
        mesh_.position(v_t) = eoptimal_q_[e];
        //post update
//        std::cout << "update: >>> ";
        for(auto h1: mesh_.halfedges(v_t)) {
            auto e1 = mesh_.edge(h1);
            std::cout << e1 << ' ';
            std::tie(eoptimal_q_[e1], quadric_distance(e1)) = find_minima(e1);
            if(heap_.is_stored(e1))
                heap_.update(e1);
        }
        std::cout << "\n finished： " << mesh_.n_edges() <<' ' << n_vertex << ' ' << mesh_.n_vertices() << std::endl;
    }
    mesh_.garbage_collection();
}


std::tuple<pmp::Point, double> SurfaceQEM::find_minima(pmp::Edge e) {
    pmp::Vertex v0 = mesh_.vertex(e, 0), v1 = mesh_.vertex(e, 1);
    auto Q = vquadric_[v0];
    Q += vquadric_[v1];
    pmp::Point  p;
    if(Q.minima(p))
        return {p, Q(p)};
    else {
        std::array<pmp::Point,3> Ps{mesh_.position(v0), mesh_.position(v1)};
        Ps[2] = (Ps[1]+Ps[0])/2;
        double errors[3]{Q(Ps[0]) ,Q(Ps[1]), Q(Ps[2]) };
        size_t min_idx = std::min_element(errors, errors+3) - errors;
        return {Ps[min_idx], errors[min_idx]};
    }
}


void SurfaceQEM::initial_quadric() {
    quadric_distance.reset_value();
    std::vector<pmp::Edge> edges;
    edges.reserve(mesh_.n_edges());

    for(auto e: mesh_.edges()) {
        std::tie(eoptimal_q_[e], quadric_distance(e)) = find_minima(e);
        edges.emplace_back(e);
    }
    heap_.reset_heap(edges);

}

} // namespace pmp_pupa