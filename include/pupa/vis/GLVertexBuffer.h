//
// Created by pupa on 12/1/20.
//

#pragma once

#include <pmp/visualization/GL.h>
#include <Eigen/Dense>

/*
Mesh Geometric Data
*/

//!> VertexBuffer
class VertexBuffer
{
private:
    GLuint buffer_{0};
    GLuint attrib_index_, attrib_cols_;

public:
    explicit VertexBuffer(GLuint attrib_index, GLuint cols)
        : attrib_index_(attrib_index), attrib_cols_(cols)
    {
    }
    ~VertexBuffer() { glDeleteBuffers(1, &buffer_); }

    void update_float_buffer(const Eigen::MatrixXf& Vprop)
    {
        if (buffer_ == 0)
            glGenBuffers(1, &buffer_);
        glBindBuffer(GL_ARRAY_BUFFER, buffer_);
        glBufferData(GL_ARRAY_BUFFER, Vprop.size() * sizeof(std::float_t), Vprop.data(),
                     GL_STATIC_DRAW);
        glVertexAttribPointer(attrib_index_, attrib_cols_, GL_FLOAT, GL_FALSE, 0, nullptr);
        glEnableVertexAttribArray(attrib_index_);
    }
};

class ElementBuffer
{
private:
    GLuint buffer_{0};
    GLsizei len_;

public:
    explicit ElementBuffer() {}
    ~ElementBuffer() { glDeleteBuffers(1, &buffer_); }

    void update_element_buffer(const Eigen::Matrix<std::uint32_t, -1, -1>& Iprops)
    {
        if (buffer_ == 0)
            glGenBuffers(1, &buffer_);
        len_ = Iprops.size();
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, buffer_);
        glBufferData(GL_ELEMENT_ARRAY_BUFFER, len_ * sizeof(std::uint32_t), Iprops.data(),
                     GL_STATIC_DRAW);
    }

    void draw_call(GLenum mode) const
    {
        //        glDrawArrays(GL_POINTS, 0, 5884);
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, buffer_);
        glDrawElements(mode, len_, GL_UNSIGNED_INT, nullptr);
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
    }
};

class VertexArray
{
private:
    GLuint vertex_array_object_{0};

public:
    explicit VertexArray() {}

    ~VertexArray() { glDeleteVertexArrays(1, &vertex_array_object_); }

    void bind()
    {
        if (vertex_array_object_ == 0)
            glGenVertexArrays(1, &vertex_array_object_);
        glBindVertexArray(vertex_array_object_);
    }

    void unbind() const { glBindVertexArray(0); }
};
