//
// Created by $Pupa on 2020/9/15.
//

#ifndef PMP_CPCSURFACEUNFOLDING_H
#define PMP_CPCSURFACEUNFOLDING_H
#include "pmp/SurfaceMesh.h"
#include <stdint.h>

//#include "DualQuaternion.h"

#include "dualquat/dualquat.h"

Eigen::Quaterniond operator + (Eigen::Quaterniond q1, Eigen::Quaterniond q2);
Eigen::Quaterniond operator -(Eigen::Quaterniond q);
Eigen::Quaterniond operator - (Eigen::Quaterniond q1, Eigen::Quaterniond q2);
Eigen::Quaterniond operator * (double k, Eigen::Quaterniond q1) ;

struct DualQuaternions
{
    DualQuaternions():r(1, 0, 0, 0), d(0, 0, 0, 0) {}
    static DualQuaternions AxisRotation(Eigen::Quaterniond Q, Eigen::Vector3d A);

    DualQuaternions(Eigen::Quaterniond Qr, Eigen::Quaterniond Qd):r(Qr), d(Qd) {}

    DualQuaternions operator * (DualQuaternions dualQuat){
        return DualQuaternions(r*dualQuat.r, r*dualQuat.d + d*dualQuat.r);
    }

    DualQuaternions operator * (double t){
        return DualQuaternions(t*r, t*d);
    }

    Eigen::Vector3d tranform(Eigen::Vector3d v) {
        auto V = Eigen::Quaterniond(0, v[0], v[1], v[2]);
        auto dq = DualQuaternions(Eigen::Quaterniond(1, 0, 0, 0), V);
        return ((*this) * dq * this->conjugate()).d.vec();
    }

    DualQuaternions conjugate(){
        return DualQuaternions(r.conjugate(), -d.conjugate());
    }

    DualQuaternions sclerp(double t, DualQuaternions dualquat){
        return DualQuaternions(r.slerp(t, dualquat.r), d.slerp(t, dualquat.d));
    }

private:
    Eigen::Quaterniond r, d;
};


class CPCSurfaceUnfolding{
public:
    CPCSurfaceUnfolding(pmp::SurfaceMesh& mesh): mesh_(mesh){}

    void segmentation();

    void unfolding_strip(pmp::SurfaceMesh& mesh, pmp::Halfedge h, Eigen::Vector3d fn);

    void rotate_strip(pmp::SurfaceMesh &mesh, pmp::Halfedge h, eigen_ext::DualQuaternion<double> dualquat);

    void unfolding_axis(pmp::SurfaceMesh& mesh, std::vector<pmp::Halfedge>& axis);

    void export_to_svg(pmp::SurfaceMesh& mesh, std::string file_path);

private:

    pmp::SurfaceMesh& mesh_;
};

#endif //PMP_CPCSURFACEUNFOLDING_H
