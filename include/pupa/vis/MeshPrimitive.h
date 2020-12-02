//
// Created by pupa on 12/1/20.
//

#pragma once

#include "GLVertexBuffer.h"
#include <pmp/Types.h>
#include <memory>

namespace pupa_vis {

struct GLMeshVertexArray
{
public:
    typedef std::function<void(const ElementBuffer& buffer)> DrawCallType;

    VertexArray vertex_array_;

    void gl_draw(const DrawCallType& draw_call, const ElementBuffer& buffer)
    {
        vertex_array_.bind();
        draw_call(buffer);
        vertex_array_.unbind();
    }

    static void draw_triangles(const ElementBuffer& buffer) {
        glDepthRange(0.01, 1.0);
        buffer.draw_call(GL_TRIANGLES);
    }

    static void draw_lines(const ElementBuffer& buffer)
    {
        glDepthRange(0.0, 1.0);
        glDepthFunc(GL_LEQUAL);
        buffer.draw_call(GL_LINES);
        glDepthFunc(GL_LESS);
    }

    static void draw_points(const ElementBuffer& buffer)
    {
        glDepthRange(0.0, 1.0);
        glDepthFunc(GL_LEQUAL);
        buffer.draw_call(GL_POINTS);
        glDepthFunc(GL_LESS);
    }
};


class GLMeshPrimitive : public GLMeshVertexArray
{
public:
    VertexBuffer position_{0, 3};
    VertexBuffer vnormal_{1, 3};
    ElementBuffer triangles_;
    ElementBuffer edges_;

    explicit GLMeshPrimitive() = default;
};
//
//class ColorGLMeshPrimitive;
//class TextureGLMeshPrimitive;
//class ColorGLMeshData;

} // namespace pupa_vis