//
// Created by $Pupa on 2020/9/15.
//

#ifndef PMP_CPCSURFACEUNFOLDING_H
#define PMP_CPCSURFACEUNFOLDING_H
#include "pmp/SurfaceMesh.h"
#include <stdint.h>


#include "dualquat/dualquat.h"



class CPCSurfaceUnfolding{
public:
    CPCSurfaceUnfolding(pmp::SurfaceMesh& mesh): mesh_(mesh){}

    void segmentation();

    void iterative_unfolding();

    void unfolding_strip(pmp::SurfaceMesh& mesh, pmp::Halfedge h, Eigen::Vector3d fn);

    void rotate_strip(pmp::SurfaceMesh &mesh, pmp::Halfedge h, eigen_ext::DualQuaternion<double> dualquat);

    void unfolding_latitude(pmp::SurfaceMesh& mesh, std::vector<pmp::Halfedge>& axis);

    void unfolding_longitude(pmp::SurfaceMesh& mesh, std::vector<pmp::Halfedge>& axis);

    void export_to_svg(pmp::SurfaceMesh& mesh, std::string file_path);

private:

    pmp::SurfaceMesh& mesh_;
};

#endif //PMP_CPCSURFACEUNFOLDING_H
