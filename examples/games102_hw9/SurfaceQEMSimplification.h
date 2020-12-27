//
// Created by pupa on 12/23/20.
//

#pragma once
#include <pmp/SurfaceMesh.h>
#include <Eigen/Dense>

namespace pmp_pupa {

struct Quadric
{
    Quadric() : metric(Eigen::Matrix4d::Zero()) {}
    explicit Quadric(pmp::Normal n, pmp::Point p)
    {
        *this = Quadric(n[0], n[1], n[2], -pmp::dot(n, p));
    }
    explicit Quadric(double a, double b, double c, double d)
    {
        metric = Eigen::Vector4d(a, b, c, d) * Eigen::RowVector4d(a, b, c, d);
    }
    Eigen::Matrix4d metric;
};

template <class HeapEntry, class SortInterface>
class MinHeap : protected std::vector<HeapEntry>
{
public:
    explicit MinHeap(SortInterface& interface) : interface_(interface) {}

    void update(HeapEntry e) {
        size_t i = interface_[e];
        up_heap(i);
        down_heap(i);
    }

    bool is_stored(HeapEntry e) { return interface_[e] != -1; }

    void insert(HeapEntry e) {
        this->push_back(e);
        up_heap(this->size() - 1);
    }

    void remove(HeapEntry e) {
        size_t i = interface_[e];
        set(i, *this->rbegin());
        this->resize(this->size() - 1);
        if(i == this->size() ) return ;
        interface_[e] = -1;
        update(entry(i));
    }

    HeapEntry pop_front() {
        HeapEntry e = entry(0);
        remove(e);
        return e;
    }

    size_t size() { return ((std::vector<HeapEntry>*)this)->size(); }

    void print() {
        for(int i = 0 ; i < this->size(); i++) {
            std::cout << entry(i) << ':' << interface_(entry(i)) << '\t';
        }
        std::cout << std::endl;
    }

private:
    inline size_t parent(size_t i) { return (i - 1) >> 1; }
    inline size_t left(size_t i) { return (i << 1) + 1; }
    inline size_t right(size_t i) { return (i << 1) + 2; }
    inline HeapEntry& entry(size_t i) { return this->operator[](i); }
    inline void set(size_t i, HeapEntry& item_) { interface_[entry(i) = item_] = i; }

    void up_heap(size_t i) {
        HeapEntry e = entry(i);
        for(;i > 0 && interface_(e) < interface_(entry(parent(i))); i = parent(i))
            set(i, entry(parent(i)));
        set(i, e);
    }

    void down_heap(size_t i) {
        HeapEntry e = entry(i);
        size_t s = this->size();
        for(size_t child; left(i) < s; i = child)
        {
            child = left(i);
            if (child + 1 < s && interface_(entry(child + 1)) < interface_(entry(child)))
                child++;

            if (interface_(entry(child)) > interface_(e))
                break;
            set(i, entry(child));
        }
        set(i, e);
    }
    SortInterface& interface_;
};

class SurfaceQEMSimplification
{
public:
    explicit SurfaceQEMSimplification(pmp::SurfaceMesh& mesh);

    void simplification(size_t n_vertex) ;

private:
    pmp::SurfaceMesh& mesh_;
    pmp::VertexProperty<Quadric> vquadric_;
    pmp::FaceProperty<pmp::Normal> fnormal_;
};

} // namespace pmp_pupa