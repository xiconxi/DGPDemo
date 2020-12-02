//
// Created by pupa on 12/1/20.
//
#pragma once
#include <pmp/visualization/Shader.h>
#include <pmp/visualization/PhongShader.h>
#include <pmp/Types.h>

namespace pupa_vis {

struct GLMeshData;

struct Material
{
public:
    pmp::Shader shader_;

    virtual void use(const pmp::mat4& projection, const pmp::mat4& modelview) = 0;

    void disable() { shader_.disable(); }
};

class PhongMaterial : public Material
{
public:
    //! material properties
    pmp::vec3 front_color_, back_color_;
    float ambient_, diffuse_, specular_, shininess_, alpha_;

    explicit PhongMaterial()
    {
        front_color_ = pmp::vec3(0.6, 0.6, 0.6);
        back_color_ = pmp::vec3(0.5, 0.0, 0.0);
        ambient_ = 0.1;
        diffuse_ = 0.8;
        specular_ = 0.6;
        shininess_ = 100.0;
        alpha_ = 1.0;
    }

    void use(const pmp::mat4& projection, const pmp::mat4& modelview) override
    {
        if (!shader_.is_valid())
            shader_.source(phong_vshader, phong_fshader);
        // setup matrices
        pmp::mat4 mv_matrix = modelview;
        pmp::mat4 mvp_matrix = projection * modelview;
        pmp::mat3 n_matrix = inverse(transpose(linear_part(mv_matrix)));

        // setup shader
        shader_.use();
        shader_.set_uniform("modelview_projection_matrix", mvp_matrix);
        shader_.set_uniform("modelview_matrix", mv_matrix);
        shader_.set_uniform("normal_matrix", n_matrix);
        shader_.set_uniform("point_size", 5.0f);
        shader_.set_uniform("light1", pmp::vec3(1.0, 1.0, 1.0));
        shader_.set_uniform("light2", pmp::vec3(-1.0, 1.0, 1.0));
        shader_.set_uniform("front_color", front_color_);
        shader_.set_uniform("back_color", back_color_);
        shader_.set_uniform("ambient", ambient_);
        shader_.set_uniform("diffuse", diffuse_);
        shader_.set_uniform("specular", specular_);
        shader_.set_uniform("shininess", shininess_);
        shader_.set_uniform("alpha", alpha_);
        shader_.set_uniform("use_lighting", true);
        shader_.set_uniform("use_texture", false);
        shader_.set_uniform("use_srgb", false);
        shader_.set_uniform("show_texture_layout", false);
    }
};

//class MatCapMaterial;

} // namespace pupa_vis
