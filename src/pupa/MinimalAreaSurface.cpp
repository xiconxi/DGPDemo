//
// Created by pupa on 12/2/20.
//

#include <pupa/pmp/MinimalAreaSurface.h>
#include <pmp/algorithms/DifferentialGeometry.h>

#include <Eigen/Sparse>

using namespace pmp_pupa;

MinimalAreaSurface::MinimalAreaSurface(SurfaceMesh &mesh) : mesh_(mesh)
{
    eweight_ = mesh_.add_edge_property<double>("fairing:eweight");
    inner_idx_ = mesh_.add_vertex_property<int>("fairing:inner_idx", -1);
    boundary_idx_ = mesh_.add_vertex_property<int>("fairing:boundary_idx", -1);

    for (auto v : mesh_.vertices())
        if (mesh_.is_boundary(v))
            boundary_idx_[v] = n_boundary_vertices_++;
        else
            inner_idx_[v] = n_inner_vertices_++;
}

MinimalAreaSurface::~MinimalAreaSurface()
{
    // remove properties
    mesh_.remove_edge_property(eweight_);
    mesh_.remove_vertex_property(inner_idx_);
    mesh_.remove_vertex_property(boundary_idx_);
}

void MinimalAreaSurface::explicit_iterate(float lambda)
{
    for (auto e : mesh_.edges())
        eweight_[e] = std::max(0.0, cotan_weight(mesh_, e));
    Eigen::MatrixX3d B = Eigen::MatrixX3d::Zero(n_inner_vertices_, 3);
    for (auto v : mesh_.vertices())
    {
        size_t i = inner_idx_[v];
        if (mesh_.is_boundary(v))
            continue;
        double w = 0, ww = 0;
        for (auto h : mesh_.halfedges(v))
        {
            ww += (w = eweight_[mesh_.edge(h)]);
            B.row(i) += w * Eigen::Vector3d(mesh_.position(mesh_.to_vertex(h)));
        }
        B.row(i) *= lambda / ww;
        B.row(i) += (1 - lambda) * Eigen::Vector3d(mesh_.position(v));
    }

    for (auto v : mesh_.vertices())
    {
        if (!mesh_.is_boundary(v))
            mesh_.position(v) = B.row(inner_idx_[v]);
    }
}

void MinimalAreaSurface::implicit_iterate(float lambda)
{
    lambda = 1 - lambda;
    for (auto e : mesh_.edges())
        eweight_[e] = std::max(0.0, cotan_weight(mesh_, e));
    Eigen::SparseMatrix<double> A(n_inner_vertices_, n_inner_vertices_);
    Eigen::MatrixX3d B = Eigen::MatrixX3d::Zero(n_inner_vertices_, 3);

    std::vector<Eigen::Triplet<double>> triplets;
    for (auto v : mesh_.vertices())
    {
        if (mesh_.is_boundary(v))
            continue;
        double w = 0, ww = 0;
        for (auto h : mesh_.halfedges(v))
        {
            auto vv = mesh_.to_vertex(h);
            w = eweight_[mesh_.edge(h)];
            ww += eweight_[mesh_.edge(h)];
            B.row(inner_idx_[v]) += w * Eigen::Vector3d(mesh_.position(vv)) * lambda;
            if (!mesh_.is_boundary(vv))
                triplets.emplace_back(inner_idx_[v], inner_idx_[vv], w);
            else
                B.row(inner_idx_[v]) -= w * Eigen::Vector3d(mesh_.position(vv));
        }
        triplets.emplace_back(inner_idx_[v], inner_idx_[v], -ww);

        B.row(inner_idx_[v]) -= Eigen::Vector3d(mesh_.position(v)) * ww * lambda;
    }

    // build sparse matrix from triplets
    A.setFromTriplets(triplets.begin(), triplets.end());

    // solve A*X = B
    Eigen::SparseLU<Eigen::SparseMatrix<double>> solver(A);
    Eigen::MatrixXd X = solver.solve(B);
    if (solver.info() != Eigen::Success)
    {
        std::cerr << "SurfaceParameterization: Could not solve linear system\n";
    }
    else
        for (auto v : mesh_.vertices())
        {
            if (!mesh_.is_boundary(v))
                mesh_.position(v) = X.row(inner_idx_[v]);
        }
}

void MinimalAreaSurface::boundary_explicit_iterate(float lambda)
{
    Eigen::MatrixX3d B = Eigen::MatrixX3d::Zero(n_boundary_vertices_, 3);
    for (auto v : mesh_.vertices())
    {
        size_t i = boundary_idx_[v];
        if (!mesh_.is_boundary(v))
            continue;
        double w = 0, ww = 0;
        for (auto h : mesh_.halfedges(v))
        {
            auto vv = mesh_.to_vertex(h);
            if (!mesh_.is_boundary(vv))
                continue;

            ww += (w = 1.0 / mesh_.edge_length(mesh_.edge(h)));
            B.row(i) += w * Eigen::Vector3d(mesh_.position(vv));
        }
        B.row(i) *= lambda / ww;
        B.row(i) += (1 - lambda) * Eigen::Vector3d(mesh_.position(v));
    }

    for (auto v : mesh_.vertices())
    {
        if (mesh_.is_boundary(v))
            mesh_.position(v) = B.row(boundary_idx_[v]);
    }
}
