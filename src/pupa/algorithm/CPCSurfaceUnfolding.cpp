//
// Created by $Pupa on 2020/9/15.
//

#include <pupa/pmp/CPCSurfaceUnfolding.h>
#include <stdint.h>
#include <pmp/algorithms/SurfaceNormals.h>
#include <fstream>

typedef eigen_ext::DualQuaternion<double> DualQuatd;

void CPCSurfaceUnfolding::segmentation()
{
    std::uint16_t n = std::sqrt(mesh_.n_vertices() - 1) - 1;

    pmp::SurfaceMesh out_mesh;
    for (auto v : mesh_.vertices())
        out_mesh.add_vertex(mesh_.position(v));

    // start looping triangle-strip mode from center-axis
    for (auto h : mesh_.halfedges())
    {
        auto vf = mesh_.from_vertex(h), vt = mesh_.to_vertex(h);
        auto v_strip = mesh_.to_vertex(mesh_.next_halfedge(h));

        if (vf.idx() % n == int(n / 2) and vt.idx() % n == int(n / 2))
        {
            pmp::Halfedge h_mesh_ = h;
            for (bool strip = false; v_strip.idx() < mesh_.n_vertices() - 2; strip = !strip)
            {
                v_strip = mesh_.to_vertex(mesh_.next_halfedge(h_mesh_));
                auto nv = out_mesh.add_vertex(mesh_.position(v_strip));
                out_mesh.add_triangle(vf, vt, nv);
                if (strip)
                {
                    h_mesh_ = mesh_.opposite_halfedge(mesh_.next_halfedge(h_mesh_));
                    vf = nv;
                }
                else
                {
                    h_mesh_ = mesh_.opposite_halfedge(mesh_.prev_halfedge(h_mesh_));
                    vt = nv;
                }
            }
        }
    }

    auto e_is_axis = out_mesh.add_edge_property<bool>("E:is_axis", false);
    // post labeling for center axis
    for (auto e : out_mesh.edges())
    {
        auto vf = out_mesh.vertex(e, 0), vt = out_mesh.vertex(e, 1);
        if (vf.idx() >= mesh_.n_vertices() || vt.idx() >= mesh_.n_vertices())
            continue;
        if (vf.idx() % n == int(n / 2) and vt.idx() % n == int(n / 2))
            e_is_axis[e] = true;
    }
    for (auto v : out_mesh.vertices())
        if (out_mesh.is_isolated(v))
            out_mesh.delete_vertex(v);
    out_mesh.garbage_collection();
    out_mesh.write("segment.obj");

    pmp::SurfaceNormals::compute_face_normals(out_mesh);

    std::vector<pmp::Halfedge> axis_lists;
    for (auto v : out_mesh.vertices())
    {
        std::vector<pmp::Halfedge> axis_tmp;
        for (auto h : out_mesh.halfedges(v))
            if (e_is_axis[out_mesh.edge(h)])
                axis_tmp.emplace_back(h);
        if (axis_tmp.size() == 1)
        {
            axis_lists.emplace_back(axis_tmp[0]);
            while (axis_lists.size() != n + 1)
            {
                auto h = *axis_lists.rbegin();
                for (auto hh : out_mesh.halfedges(out_mesh.to_vertex(h)))
                {
                    if (!e_is_axis[out_mesh.edge(hh)])
                        continue;
                    if (out_mesh.to_vertex(hh) != out_mesh.from_vertex(h))
                        axis_lists.emplace_back(hh);
                }
            }
            break;
        }
    }
    out_mesh.remove_edge_property(e_is_axis);

    auto v_dq = out_mesh.add_vertex_property<std::vector<DualQuatd>>("v:dualquat");
    auto v_scalp = out_mesh.add_vertex_property<Eigen::Vector3d>("v:scalp");
    for (auto v : out_mesh.vertices())
    {
        v_scalp[v] = out_mesh.position(v);
        v_dq[v].emplace_back(DualQuatd());
    }

    unfolding_latitude(out_mesh, axis_lists);
    unfolding_longitude(out_mesh, axis_lists);

    auto bbox = out_mesh.bounds();
    for (auto v : out_mesh.vertices())
        out_mesh.position(v) -= bbox.center();
    export_to_svg(out_mesh, "cpc.svg");

    mesh_ = out_mesh;

    out_mesh.write("unfolded.obj");
}

void CPCSurfaceUnfolding::iterative_unfolding() {}

void CPCSurfaceUnfolding::unfolding_longitude(pmp::SurfaceMesh &mesh,
                                              std::vector<pmp::Halfedge> &axis)
{
    pmp::SurfaceNormals::compute_face_normals(mesh);
    auto v_dq = mesh.get_vertex_property<std::vector<DualQuatd>>("v:dualquat");
    //    auto fnormals = mesh.get_face_property<pmp::Normal>("f:normal");
    for (auto h1 : axis)
    {
        for (auto v : mesh.vertices())
            v_dq[v].emplace_back(*v_dq[v].rbegin());
        auto h2 = mesh.opposite_halfedge(h1);
        Eigen::Vector3d fn(-1, 0, 0);
        unfolding_strip(mesh, h1, fn);
        unfolding_strip(mesh, h2, fn);
    }
}

// unfolding the triangle-strip with specific axis
void CPCSurfaceUnfolding::unfolding_strip(pmp::SurfaceMesh &mesh, pmp::Halfedge h,
                                          Eigen::Vector3d target_fn)
{
    auto fnormals = mesh.get_face_property<pmp::Normal>("f:normal");
    Eigen::Vector3d p_axis = Eigen::Vector3d(mesh.position(mesh.to_vertex(h)));
    Eigen::Vector3d curr_fn = fnormals[mesh.face(h)];
    auto axis = curr_fn.cross(target_fn).normalized();
    double angle = std::acos(target_fn.dot(curr_fn));
    auto dualquat = eigen_ext::convert_to_dualquat(axis, p_axis.cross(axis), angle, 0.0);
    dualquat = dualquat.normalized();
    rotate_strip(mesh, h, dualquat.normalized());
    for (auto next : {mesh.next_halfedge(h), mesh.prev_halfedge(h)})
    {
        if (!mesh.is_boundary(mesh.opposite_halfedge(next)))
            unfolding_strip(mesh, mesh.opposite_halfedge(next), target_fn);
    }
}

// unfolding the triangle-strip with specific axis
void CPCSurfaceUnfolding::rotate_strip(pmp::SurfaceMesh &mesh, pmp::Halfedge h, DualQuatd dualquat)
{
    auto V_dq = mesh.get_vertex_property<std::vector<DualQuatd>>("v:dualquat");
    for (auto hh = h; !mesh.is_boundary(hh);)
    {
        auto v = mesh.to_vertex(mesh.next_halfedge(hh));
        auto p = Eigen::Vector3d(mesh.position(v));
        mesh.position(v) = eigen_ext::transform_point(dualquat, p).dual().vec();
        *V_dq[v].rbegin() = (dualquat * *V_dq[v].rbegin()).normalized();
        if (!mesh.is_boundary(mesh.opposite_halfedge(mesh.next_halfedge(hh))))
            hh = mesh.opposite_halfedge(mesh.next_halfedge(hh));
        else
            hh = mesh.opposite_halfedge(mesh.prev_halfedge(hh));
    }
    pmp::SurfaceNormals::compute_face_normals(mesh);
}

void CPCSurfaceUnfolding::unfolding_latitude(pmp::SurfaceMesh &mesh,
                                             std::vector<pmp::Halfedge> &axis)
{
    auto v_dq = mesh.get_vertex_property<std::vector<DualQuatd>>("v:dualquat");
    for (int i = 0; i < axis.size(); i++)
    {
        auto p1 = Eigen::Vector3d(mesh.position(mesh.from_vertex(axis[i])));
        auto p2 = Eigen::Vector3d(mesh.position(mesh.to_vertex(axis[i])));
        auto p0 = i == 0 ? p1 - Eigen::Vector3d(0, 0, 1)
                         : Eigen::Vector3d(mesh.position(mesh.from_vertex(axis[i - 1])));
        for (auto v : mesh.vertices())
            v_dq[v].emplace_back(*v_dq[v].rbegin());
        for (int j = i; j < axis.size(); j++)
        {
            auto v = mesh.to_vertex(axis[j]);
            auto axis_v = (p2 - p1).cross(p1 - p0).normalized();
            double angle = std::acos((p1 - p0).normalized().dot((p2 - p1).normalized()));
            auto dualquat = eigen_ext::convert_to_dualquat(axis_v, p1.cross(axis_v), angle, 0.0);
            dualquat = dualquat.normalized();
            mesh.position(v) =
                eigen_ext::transform_point(dualquat, Eigen::Vector3d(mesh.position(v)))
                    .dual()
                    .vec();
            *v_dq[v].rbegin() = (dualquat * *v_dq[v].rbegin()).normalized();

            rotate_strip(mesh, axis[j], dualquat);
            rotate_strip(mesh, mesh.opposite_halfedge(axis[j]), dualquat);
        }
    }
    pmp::SurfaceNormals::compute_face_normals(mesh);
}

void CPCSurfaceUnfolding::export_to_svg(pmp::SurfaceMesh &mesh, std::string file_path)
{
    int width = 2100, height = 2970;
    std::string a4_svg_header =
        "<svg width=\"2100\" height=\"2970\" viewBox=\"0 0 2100 2970\" "
        "xmlns=\"http://www.w3.org/2000/svg\" >\n";
    auto bbox = mesh.bounds();
    double scale = width * 0.95 / (bbox.max()[1] - bbox.min()[1]);

    std::string polygon = "<polygon points=\"";
    for (auto h : mesh.halfedges())
    {
        if (!mesh.is_boundary(h))
            continue;
        for (auto hh = h;;)
        {
            auto v = mesh.position(mesh.from_vertex(hh));
            polygon += std::to_string(v[1] * scale + width / 2) + "," +
                       std::to_string(v[2] * scale + height / 2) + " ";
            hh = mesh.next_halfedge(hh);
            if (hh == h)
                break;
        }
        polygon += "\" fill=\"#c6c602\" stroke=\"black\" stroke-width=\"8\"/></svg>";
        break;
    }

    std::ofstream file(file_path);

    file << a4_svg_header << polygon;

    file.close();
}