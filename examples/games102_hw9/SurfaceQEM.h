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

    bool minima(pmp::Point& p) const {
        auto m = metric;
        m.row(3) = Eigen::RowVector4d(0, 0, 0, 1);
        if(std::abs(m.determinant()) < 1e-5)
            return false;

        Eigen::Vector4d v = m.fullPivHouseholderQr().solve(m.row(3).transpose());
        p = pmp::Point(v[0], v[1], v[2]);
        return true;
    }

    double operator () (const pmp::Point& p) const {
        Eigen::Vector4d v = (Eigen::Vector4d() << p[0], p[1], p[2], 1).finished();
        return v.transpose() * metric * v;
    }

    void operator +=(Quadric rhs) { metric += rhs.metric; }

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

    void reset_heap(std::vector<HeapEntry>& es) {
        this->clear();
        for(size_t i = 0; i < es.size(); i++)
        {
            this->push_back(es[i]);
            interface_[es[i]] = i;
        }
        for(size_t i = es.size()/2; i < es.size(); i++)
            up_heap(i);
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

    inline void swap(size_t i, size_t j) {
        HeapEntry _i = entry(i), _j = entry(j);
        set(i, _j);
        set(j, _i);
    }

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

struct  EdgeQuadricDistance  {

    explicit EdgeQuadricDistance(pmp::SurfaceMesh& mesh):mesh_(mesh) {
        idx_ = mesh_.edge_property<std::int32_t>("e:heap_idx", -1);
        eprio_ = mesh_.edge_property<double>("e:eprio_", 1e5);
    }

    ~EdgeQuadricDistance(){
        mesh_.remove_edge_property(idx_);
        mesh_.remove_edge_property(eprio_);
    }

    std::int32_t& operator [](pmp::Edge e) {return idx_[e];}
    double& operator ()(pmp::Edge e) {return eprio_[e];}

    void reset_value() {
        std::fill(idx_.vector().begin(), idx_.vector().end(), -1);
        std::fill(eprio_.vector().begin(), eprio_.vector().end(), 1e5);
    }

private:
    pmp::EdgeProperty<double> eprio_;
    pmp::EdgeProperty<std::int32_t> idx_;
    pmp::SurfaceMesh& mesh_;
};

struct CollapseData {
    pmp::Vertex v_f, v_t, v_l, v_r;
    pmp::Point p_f, p_t, p_target;
};

class SurfaceQEM
{
public:
    explicit SurfaceQEM(pmp::SurfaceMesh& mesh);

    ~SurfaceQEM();

    std::vector<CollapseData> simplification(size_t n_vertex) ;

private:

    std::tuple<pmp::Point, double> find_minima(pmp::Edge e);

    void initial_quadric();

    bool is_collapse_legal(pmp::Edge e, pmp::Point p);

    double aspect_ratio(pmp::Face f) const;

    pmp::SurfaceMesh& mesh_;
    pmp::VertexProperty<Quadric> vquadric_;
    pmp::EdgeProperty<pmp::Point> eoptimal_q_;
    pmp::FaceProperty<pmp::Normal> fnormal_;

    EdgeQuadricDistance quadric_distance;
    MinHeap<pmp::Edge, EdgeQuadricDistance> heap_;
};




class LODSurfaceMesh{
public:
    explicit LODSurfaceMesh(pmp::SurfaceMesh& mesh) : mesh_(mesh) { it_ = sequence_.rbegin(); }
    explicit LODSurfaceMesh(pmp::SurfaceMesh& mesh, std::vector<CollapseData>& collapse_data)
        : sequence_(collapse_data), mesh_(mesh){
        it_ = sequence_.rbegin();
    }

    void push_back(CollapseData& data)
    {
        sequence_.push_back(data);
        it_ = ++sequence_.rbegin();
    }

    std::vector<CollapseData>& sequence() { return sequence_; }

    // inverse decimation
    size_t operator ++ ();

    // decimation
    size_t operator -- ();

    size_t max_vertices() { return mesh_.n_vertices() + std::distance(it_, sequence_.rend()); }

    size_t min_vertices() { return mesh_.n_vertices() - std::distance(sequence_.rbegin(), it_); }

private:

    pmp::SurfaceMesh& mesh_;
    std::vector<CollapseData> sequence_;
    std::vector<CollapseData>::reverse_iterator it_;
};


} // namespace pmp_pupa