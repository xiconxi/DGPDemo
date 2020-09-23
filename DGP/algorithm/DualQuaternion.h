//
// Created by $Pupa on 2020/9/20.
//

#ifndef PMP_DualQuaternion_H
#define PMP_DualQuaternion_H

#include <type_traits>
#include <Eigen/Geometry>

template <typename T>
class DualQuaternion
{
public:
    using value_type = T;
    DualQuaternion()
        : real_(Eigen::Quaternion<T>::Identity()),
          dual_(Eigen::Quaternion<T>::Coefficients::Zero())
    {}
    DualQuaternion(Eigen::Vector3<T> v)
        : real_(Eigen::Quaternion<T>::Identity()),
          dual_(Eigen::Quaternion<T>(T(0), v.x(), v.y(), v.z()))
    {}
    
    explicit DualQuaternion(const Eigen::Quaternion<T>& real, const Eigen::Quaternion<T>& dual)
        :real_(real), dual_(dual)
    {}

    DualQuaternion(Eigen::Vector3<T> l, Eigen::Vector3<T> m)
        :DualQuaternion(Eigen::Quaternion<T>(0, l.x(), l.y(), l.z()),
                        Eigen::Quaternion<T>(0, m.x(), m.y(), m.z()))
    {}

    Eigen::Quaternion<T>& real(){return real_;}
    Eigen::Quaternion<T>& dual(){return dual_;}

    DualQuaternion<T>& operator += (DualQuaternion<T> rhs)
    {
        real_ = real_.coeffs() + rhs.real_.coeffs();
        dual_ = dual_.coeffs() + rhs.dual_.coeffs();
        return *this;
    }

    DualQuaternion<T>& operator -= (DualQuaternion<T> rhs)
    {
        real_ = real_.coeffs() - rhs.real_.coeffs();
        dual_ = dual_.coeffs() - rhs.dual_.coeffs();
        return *this;
    }

    DualQuaternion<T>& operator *= (DualQuaternion<T> rhs) 
    {
        auto temp = real_;
        real_ = temp * rhs.real_;
        dual_ = (temp * rhs.dual()).coeffs() + (dual_ * rhs.real()).coeffs();
        return *this;
    }

    DualQuaternion<T> operator * (DualQuaternion<T> rhs)
    {
        DualQuaternion<T> res;
        res.real_ = this->real_ * rhs.real_;
        res.dual_ = (this->real_ * rhs.dual()).coeffs() + (dual_ * rhs.real()).coeffs();
        return res;
    }

    DualQuaternion<T> total_conjugate()
    {
        return quaternion_conjugate().dual_conjugate();
    }

    DualQuaternion<T> dual_conjugate()
    {
        auto tmp = *this;
        tmp.dual_.coeffs() = -tmp.dual_.coeffs();
        return tmp;
    }

    DualQuaternion<T> quaternion_conjugate()
    {
        return DualQuaternion<T>(real_.conjugate(), dual_.conjugate());
    }


    DualQuaternion<T> sclerp(DualQuaternion<T>& rhs, double t) {

    }

    DualQuaternion<T> transform_point(Eigen::Vector3<T> p) {
        return (*this)*DualQuaternion<T>(p)*this->total_conjugate();
    }

private:
    Eigen::Quaternion<T> real_, dual_;
};

template <class T>
DualQuaternion<T>
convert_to_dualquat(Eigen::Vector3<T> l, Eigen::Vector3<T> m, T theta, T d)
{
    DualQuaternion<T> tmp;
    tmp.real() = Eigen::Quaternion<T>(Eigen::AngleAxis<T>(theta, l));
    tmp.dual().w() = -d/2*std::sin(theta/2);
    tmp.dual().vec() = std::sin(theta/2)*m + d/2*std::cos(theta/2)*l;
    return tmp;
}

template <class T>
DualQuaternion<T>
convert_to_dualquat(Eigen::Quaternion<T> q, Eigen::Vector3<T> m)
{
    DualQuaternion<T> tmp;
    tmp.real() = q;
    tmp.dual().vec() = q.vec().norm()*m;
    return tmp;
}


#endif //PMP_DualQuaternion_H
