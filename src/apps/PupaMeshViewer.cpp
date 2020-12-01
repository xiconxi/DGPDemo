// Copyright 2011-2020 the Polygon Mesh Processing Library developers.
// Distributed under a MIT-style license, see LICENSE.txt for details.

#include "pmp/visualization/TrackballViewer.h"
#include <imgui.h>

#include <pmp/algorithms/SurfaceNormals.h>
#include <pupa/vis/MeshPrimitive.h>
#include <pupa/vis/GLMaterial.h>

class PupaMeshViewer : public pmp::TrackballViewer
{
public:
    //! constructor
    PupaMeshViewer(const char* title, int width, int height, bool showgui = true)
        : TrackballViewer(title, width, height, showgui)
    {
        // setup draw modes
        clear_draw_modes();
        add_draw_mode("Points");
        add_draw_mode("Hidden Line");
        add_draw_mode("Smooth Shading");
        add_draw_mode("Texture");
        set_draw_mode("Smooth Shading");

        // add help items
        add_help_item("Backspace", "Reload mesh", 3);
#ifndef __EMSCRIPTEN__
        add_help_item("W", "Write mesh to 'output.off'", 4);
#endif
    }

    //! destructor
    virtual ~PupaMeshViewer() = default;

    //! load a mesh from file \p filename
    virtual bool load_mesh(const char* filename){
        // load mesh
        if (mesh_.read(filename))
        {
            update_mesh();

            // print mesh statistic
            std::cout << "Load " << filename << ": " << mesh_.n_vertices()
                      << " vertices, " << mesh_.n_faces() << " faces\n";

            filename_ = filename;
            return true;
        }

        std::cerr << "Failed to read mesh from " << filename << " !" << std::endl;
        return false;
    }


    virtual void update_mesh(){
        pmp::BoundingBox bb = mesh_.bounds();
        set_scene((pmp::vec3)bb.center(), 0.5 * bb.size());

        surface_mesh_data_.vertex_array_.bind();

        Eigen::Matrix3Xf V = vertex_property(mesh_, mesh_.vertex_property<pmp::Point>("v:point")).transpose();
        surface_mesh_data_.position_.update_float_buffer(V);

        pmp::SurfaceNormals::compute_vertex_normals(mesh_);
        Eigen::Matrix3Xf VN = vertex_property(mesh_, mesh_.vertex_property<pmp::Normal>("v:normal")).transpose();
        surface_mesh_data_.vnormal_.update_float_buffer(VN);

        Eigen::Matrix<std::uint32_t, 3, -1> F(3, mesh_.n_faces());
        for(auto f: mesh_.faces()){
            int f_i = 0;
            for(auto v: mesh_.vertices(f))
                F(f_i++, f.idx()) = v.idx();
        }
        surface_mesh_data_.triangles_.update_element_buffer(F);
        surface_mesh_data_.vertex_array_.unbind();
        glCheckError();
    }


    //! draw the scene in different draw modes
    virtual void draw(const std::string& draw_mode) override {
        phong_material_.use(projection_matrix_, modelview_matrix_);
        surface_mesh_data_.gl_draw();
        phong_material_.disable();
    }

    //! handle ImGUI interface
    virtual void process_imgui() override {
        if (ImGui::CollapsingHeader("Mesh Info", ImGuiTreeNodeFlags_DefaultOpen))
        {
            // output mesh statistics
            ImGui::BulletText("%d vertices", (int)mesh_.n_vertices());
            ImGui::BulletText("%d edges", (int)mesh_.n_edges());
            ImGui::BulletText("%d faces", (int)mesh_.n_faces());

            // control crease angle
            ImGui::PushItemWidth(100);
        }
    }

protected:
    pmp::SurfaceMesh mesh_;
    std::string filename_; //!< the current file


    PhongMaterial phong_material_;
    SurfaceGLMeshData surface_mesh_data_;
};



int main(int argc, char** argv)
{
    PupaMeshViewer viewer("Pupa MeshViewer", 800, 600);
    glCheckError();
    viewer.load_mesh(argv[1]);

    viewer.run();
}
