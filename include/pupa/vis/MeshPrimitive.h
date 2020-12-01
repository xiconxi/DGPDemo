//
// Created by pupa on 12/1/20.
//

#pragma once

#include "GLVertexBuffer.h"
#include <pmp/Types.h>

struct GLMeshData
{
public:
    VertexArray vertex_array_;
    virtual void gl_draw()  = 0;
};

template <int _cols> Eigen::Matrix<float, -1, _cols>
    vertex_property(pmp::SurfaceMesh& mesh_, pmp::VertexProperty<pmp::Matrix<float, _cols, 1> > vprob)
{
    Eigen::MatrixXf V(mesh_.n_vertices(), _cols);
    for (auto v : mesh_.vertices())
        V.row(v.idx()) = Eigen::Vector<float, _cols>(vprob[v]);
    return V;
}


class SurfaceGLMeshData : public GLMeshData
{
public:
    VertexBuffer position_{0, 3};
    VertexBuffer vnormal_{1, 3};
//    VertexBuffer vtex_{2, 2};
    ElementBuffer triangles_;

    explicit SurfaceGLMeshData() = default;

    void gl_draw()  override
    {
        vertex_array_.bind();
        triangles_.draw_call(GL_TRIANGLES);
        vertex_array_.unbind();
    }
};

class ColorSurfaceGLMeshData;
class TextureSurfaceGLMeshData;
class ColorGLMeshData;
