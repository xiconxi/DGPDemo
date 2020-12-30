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
        if(!is_collapse_legal(e, eoptimal_q_[e]))
            continue;
        auto h = mesh_.halfedge(e, 0);
        if( !mesh_.is_collapse_ok(h))
            h = mesh_.opposite_halfedge(h);
        if( !mesh_.is_collapse_ok(h))
            continue;

        mesh_.collapse(h);

        // post update
        auto v_f = mesh_.from_vertex(h), v_t = mesh_.to_vertex(h);
        vquadric_[v_t] += vquadric_[v_f];
        mesh_.position(v_t) = eoptimal_q_[e];

        for(auto h1: mesh_.halfedges(v_t)) {
            auto e1 = mesh_.edge(h1);
            auto f = mesh_.face(h1);
            std::tie(eoptimal_q_[e1], quadric_distance(e1)) = find_minima(e1);
            fnormal_[f] = pmp::SurfaceNormals::compute_face_normal(mesh_, f);
            if(heap_.is_stored(e1))
                heap_.update(e1);
            else
                heap_.insert(e1);
        }
    }
    mesh_.garbage_collection();

    pmp::SurfaceNormals::compute_face_normals(mesh_);

    std::cout << "N " << mesh_.n_vertices() << std::endl;
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

bool SurfaceQEM::is_collapse_legal(pmp::Edge e, pmp::Point p)
{
    auto h = mesh_.halfedge(e, 0);
    auto v_f = mesh_.from_vertex(h), v_t = mesh_.to_vertex(h);
    if (mesh_.is_boundary(v_f) || mesh_.is_boundary(v_t) )
        return false;

    pmp::Face f1 = mesh_.face(h), f2 = mesh_.face(mesh_.opposite_halfedge(h));
    // aspect_ratio  & face flip
    double aspect_ratio_before{0}, aspect_ratio_after{0};
    bool is_face_flip = false;
    for (auto f : mesh_.faces(v_f))
        aspect_ratio_before = std::max(aspect_ratio_before, aspect_ratio(f));
    for (auto f : mesh_.faces(v_t))
        aspect_ratio_before = std::max(aspect_ratio_before, aspect_ratio(f));

    std::swap(mesh_.position(v_f), p);
    for (auto f : mesh_.faces(v_f))
    {
        if (f == f1 || f == f2)
            continue;
        aspect_ratio_after = std::max(aspect_ratio_after, aspect_ratio(f));
        pmp::Normal n1 = pmp::SurfaceNormals::compute_face_normal(mesh_, f);
        is_face_flip = is_face_flip || pmp::dot(fnormal_[f], n1) < 0;
    }
    std::swap(mesh_.position(v_f), mesh_.position(v_t));
    for (auto f : mesh_.faces(v_t))
    {
        if (f == f1 || f == f2)
            continue;
        aspect_ratio_after = std::max(aspect_ratio_after, aspect_ratio(f));
        pmp::Normal n1 = pmp::SurfaceNormals::compute_face_normal(mesh_, f);
        is_face_flip = is_face_flip || pmp::dot(fnormal_[f], n1) < 0;
    }
    std::swap(mesh_.position(v_f), p);
    std::swap(mesh_.position(v_t), p);
    return !is_face_flip && aspect_ratio_after <  10 * aspect_ratio_before && aspect_ratio_after < 20 &&
           mesh_.valence(v_f) + mesh_.valence(v_t) < 16; // 6 + 6 + 4
}

double SurfaceQEM::aspect_ratio(pmp::Face f) const {
    std::vector<pmp::Point> p;
    p.reserve(3);
    for(auto v: mesh_.vertices(f))
        p.emplace_back(mesh_.position(v));
    double l = std::max(pmp::sqrnorm(p[0] - p[1]), pmp::sqrnorm(p[2] - p[1]));
    l = fmax(l,  pmp::sqrnorm(p[0] - p[2]));
    double a = pmp::norm(pmp::cross(p[2]-p[1], p[0]-p[1]));
    return l/a;
}

} // namespace pmp_pupa