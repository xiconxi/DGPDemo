//
// Created by pupa on 11/30/20.
//

#pragma once

#include <pmp/visualization/TrackballViewer.h>
#include <Eigen/Dense>

/*
 OpenGL
 Component:
    Shader     VertexArray     DrawCall
 Entity:
    Point      Line            Surface
 */

struct GLVertexArrayComponent
{
    GLuint vertex_array_object_;
    GLuint vertex_buffer_;
    GLuint normal_buffer_;

    virtual void bind() = 0;
    virtual void unbind() = 0;
};

struct GLDrawCallComponent
{
    virtual void drawcall(GLsizei count, GLsizei mode = GL_TRIANGLES)
    {
        glDrawArrays(GL_TRIANGLES, 0, count);
    }
};

void create_surface_mesh(const Eigen::MatrixX3f& V, const Eigen::MatrixX3i& F,
                         const Eigen::MatrixX3f& VN)
{

}
