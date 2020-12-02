//
// Created by pupa on 12/2/20.
//

#pragma once

#include <pmp/SurfaceMesh.h>
#include <pmp/visualization/TrackballViewer.h>
#include <pmp/algorithms/SurfaceNormals.h>
#include "MeshPrimitive.h"
#include "GLMaterial.h"

#include <memory>
#include <imgui.h>

namespace pupa_vis {

struct MeshViewerData;

template <int _cols>
Eigen::Matrix<float, -1, _cols> vertex_property(
    pmp::SurfaceMesh& mesh_, pmp::VertexProperty<pmp::Matrix<float, _cols, 1>> vprob)
{
    Eigen::MatrixXf V(mesh_.n_vertices(), _cols);
    for (auto v : mesh_.vertices())
        V.row(v.idx()) = Eigen::Vector<float, _cols>(vprob[v]);
    return V;
}

class MeshViewer : public pmp::TrackballViewer
{
public:
    //! constructor
    MeshViewer(const char* title, int width, int height, bool showgui = true)
        : TrackballViewer(title, width, height, showgui)
    {
//        glfwWindowHint(GLFW_DOUBLEBUFFER, GL_FALSE);
        // setup draw modes
        clear_draw_modes();
        add_draw_mode("Points");
        add_draw_mode("Hidden Line");
        add_draw_mode("Smooth Shading");
        // add help items
        add_help_item("Backspace", "Reload mesh", 3);
#ifndef __EMSCRIPTEN__
        add_help_item("W", "Write mesh to 'output.off'", 4);
#endif
    }

    //! destructor
    virtual ~MeshViewer() = default;

    //! load a mesh from file \p filename
    virtual bool load_mesh(const char* filename)
    {
        // load mesh
        if (mesh_.read(filename))
        {
            update_mesh(true);
            view_all();
            // print mesh statistic
            std::cout << "Load " << filename << ": " << mesh_.n_vertices() << " vertices, "
                      << mesh_.n_faces() << " faces\n";
            filename_ = filename;
            return true;
        }

        std::cerr << "Failed to read mesh from " << filename << " !" << std::endl;
        return false;
    }

    void update_mesh(bool update_normal = false)
    {
        pmp::BoundingBox bb = mesh_.bounds();
        center_ = (pmp::vec3)bb.center();
        radius_ = 0.5f * bb.size();

        gl_mesh_data_.vertex_array_.bind();

        auto V = vertex_property(mesh_, mesh_.vertex_property<pmp::Point>("v:point"));
        gl_mesh_data_.position_.update_float_buffer(V.transpose());

        pmp::SurfaceNormals::compute_vertex_normals(mesh_);
        auto VN = vertex_property(mesh_, mesh_.vertex_property<pmp::Normal>("v:normal"));
        if (update_normal)
            gl_mesh_data_.vnormal_.update_float_buffer(VN.transpose());

        Eigen::Matrix<std::uint32_t, 3, -1> F(3, mesh_.n_faces());
        for (auto f : mesh_.faces())
        {
            int f_i = 0;
            for (auto v : mesh_.vertices(f))
                F(f_i++, f.idx()) = v.idx();
        }
        gl_mesh_data_.triangles_.update_element_buffer(F);

        Eigen::Matrix<std::uint32_t, 2, -1> E(2, mesh_.n_edges());
        for (auto e : mesh_.edges())
        {
            E(0, e.idx()) = mesh_.from_vertex(mesh_.halfedge(e, 0)).idx();
            E(1, e.idx()) = mesh_.from_vertex(mesh_.halfedge(e, 1)).idx();
        }
        gl_mesh_data_.edges_.update_element_buffer(E);

        gl_mesh_data_.vertex_array_.unbind();
        glCheckError();
    }

    //! draw the scene in different draw modes
    virtual void draw(const std::string& draw_mode) override
    {
        phong_material_.use(projection_matrix_, modelview_matrix_);
        gl_mesh_data_.gl_draw(GLMeshVertexArray::draw_triangles, gl_mesh_data_.triangles_);
        phong_material_.shader_.set_uniform("front_color", pmp::vec3(0.1, 0.1, 0.1));
        phong_material_.shader_.set_uniform("back_color", pmp::vec3(0.1, 0.1, 0.1));
        phong_material_.shader_.set_uniform("use_lighting", false);
        gl_mesh_data_.gl_draw(GLMeshVertexArray::draw_lines, gl_mesh_data_.edges_);

        phong_material_.disable();
    }

    //! handle ImGUI interface
    void process_imgui() override;
    //    {
    //        if (ImGui::CollapsingHeader("Mesh Info", ImGuiTreeNodeFlags_DefaultOpen))
    //        {
    //            // output mesh statistics
    //            ImGui::BulletText("%d vertices", (int)mesh_.n_vertices());
    //            ImGui::BulletText("%d edges", (int)mesh_.n_edges());
    //            ImGui::BulletText("%d faces", (int)mesh_.n_faces());
    //        }
    //    }
    void do_processing() override;

private:
    pmp::SurfaceMesh mesh_;
    std::string filename_; //!< the current file

    PhongMaterial phong_material_;
    GLMeshPrimitive gl_mesh_data_;
    std::shared_ptr<MeshViewerData> data_;
};

} // namespace pupa_vis
