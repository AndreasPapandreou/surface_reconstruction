#ifndef LAB0_KDTREE2_H
#define LAB0_KDTREE2_H

#include <VVRScene/mesh.h>
#include "opencv2/opencv.hpp"

typedef std::vector<float4> VecArray4;

/**
 * A node of a KD-Tree
 */
struct KDNode2
{
    float4 split_point;
    int axis;
    int level;
    KDNode2 *child_left;
    KDNode2 *child_right;
    KDNode2() : child_left(NULL), child_right(NULL) {}
    ~KDNode2() { delete child_left; delete child_right; }
};

/**
 * KD-Tree wrapper. Holds a ptr to tree root.
 */
class KDTree2
{
public:
    KDTree2(VecArray4 &pts);
    ~KDTree2();
    int depth() const { return m_depth; }
    const KDNode2* root() const { return m_root; }
    const VecArray4 &pts;

public:
    static int makeNode(KDNode2 *node, VecArray4 &pts, const int level);
    static void getNodesOfLevel(KDNode2 *node, std::vector<KDNode2*> &nodes, int level);
    void kNearest(const int k, const float4& test_pt, const KDNode2* root, const KDNode2 **knn, float *, float &weight);
    float Distance2(const float4 p1, const float4 p2, float &weight);

public:
    KDNode2 *m_root;
    int m_depth;
};

/**
 * Function object to compare 2 3D-vecs in the specified axis.
 */
struct VecComparator2 {
    unsigned axis;
    VecComparator2(unsigned axis) : axis(axis % 4) {}
    virtual inline bool operator() (const float4& v1, const float4& v2) {
        return (v1.ptr()[axis] < v2.ptr()[axis]);
    }
};

#endif //LAB0_KDTREE2_H


