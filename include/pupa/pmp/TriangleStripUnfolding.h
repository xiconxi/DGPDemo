//
// Created by $Pupa on 2020/10/14.
//

#pragma once

#include <pmp/SurfaceMesh.h>
#include <vector>
#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <pmp/algorithms/DifferentialGeometry.h>

namespace pmp_ext {

//!< Triangle Strip Data-Structure Helper for decomposed
class SurfaceTriangleStrip : public std::vector<pmp::Halfedge>
{
public:
    SurfaceTriangleStrip(pmp::SurfaceMesh& mesh, pmp::Halfedge h) : mesh_(mesh)
    {
        collect_inner_halfedges(h);
    }

    void edge_laplacian_unfolding()
    {
        auto n = inner_hs_.size();
        // laplace edge operator
        Eigen::SparseMatrix<double> A(n, n);
        Eigen::MatrixXd B(n, 3);

        auto idx = mesh_.add_vertex_property<int>("v:idx", -1);

        std::vector<pmp::Vertex> free_vertices;
        free_vertices.reserve(n);
        for(auto h: inner_hs_)
        {
            idx[mesh_.to_vertex(mesh_.next_halfedge(h))] = free_vertices.size();
            free_vertices.push_back(mesh_.to_vertex(mesh_.next_halfedge(h)));
        }

        std::vector<Eigen::Triplet<double>> triplets;
        Eigen::Vector3d bc;
        for(int i = 0; i < inner_hs_.size(); i++)
        {
            auto v1 = mesh_.from_vertex(inner_hs_[i]);
            auto v3 = mesh_.to_vertex(inner_hs_[i]);
            auto v4 = mesh_.to_vertex(mesh_.next_halfedge(inner_hs_[i]));
            auto v2 = mesh_.to_vertex(mesh_.next_halfedge(mesh_.opposite_halfedge(inner_hs_[i])));

            auto p1 = Eigen::Vector3d(mesh_.position(v1)), p2 = Eigen::Vector3d(mesh_.position(v2));
            auto p3 = Eigen::Vector3d(mesh_.position(v3)), p4 = Eigen::Vector3d(mesh_.position(v4));

            double cot312 = pmp::cotan(p3-p1, p2-p1);
            double cot231 = pmp::cotan(p2-p3, p1-p3);
            double cot134 = pmp::cotan(p1-p3, p4-p3);
            double cot413 = pmp::cotan(p4-p1, p3-p1);
            bc *= 0;
            if(idx[v1] >= 0)
                triplets.emplace_back(i, idx[v1], -cot231 - cot134);
            else
                bc += (-cot231-cot134)*Eigen::Vector3d(p1);

            if(idx[v2] >= 0)
                triplets.emplace_back(i, idx[v2], cot312 + cot231);
            else
                bc += (cot312+cot231)*Eigen::Vector3d(p2);

            if(idx[v3] >= 0)
                triplets.emplace_back(i, idx[v3], -cot413 - cot312);
            else
                bc += (-cot413-cot312)*Eigen::Vector3d(p3);

            if(idx[v4] >= 0)
                triplets.emplace_back(i, idx[v4], cot413 + cot134);
            else
                bc += (cot413+cot134)*Eigen::Vector3d(p4);

            B.row(i) = -bc;
        }

        // build sparse matrix from triplets
        A.setFromTriplets(triplets.begin(), triplets.end());

        // solve A*X = B
        Eigen::SparseLU<Eigen::SparseMatrix<double>> solver(A);
        Eigen::MatrixXd X = solver.solve(B);

        if (solver.info() != Eigen::Success)
            std::cerr << "SurfaceParameterization: Could not solve linear system\n";
        else
            for (auto i = 0; i < n; ++i)
                mesh_.position(free_vertices[i]) = X.row(i);
    }

private:
    void collect_inner_halfedges(pmp::Halfedge h)
    {
        assert(!mesh_.is_boundary(h));
        // to_vertex(h) == new_v
        // check whether the surface is strip-like
        while (!mesh_.is_boundary(h))
        {
            auto next_h = mesh_.opposite_halfedge(mesh_.next_halfedge(h));
            auto prev_h = mesh_.opposite_halfedge(mesh_.prev_halfedge(h));
            if (!mesh_.is_boundary(next_h))
                inner_hs_.emplace_back(h = next_h);
            else if (!mesh_.is_boundary(prev_h))
                inner_hs_.emplace_back(h = prev_h);
            else
                break;
        }
    }

    pmp::SurfaceMesh& mesh_;
    std::vector<pmp::Halfedge> inner_hs_;
};

} // namespace pmp_ext
