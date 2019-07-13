#ifndef LAB0_KDTREE_H
#define LAB0_KDTREE_H

#include <VVRScene/mesh.h>
#include "opencv2/opencv.hpp"
#include <numeric>      // std::iota

/**
 * A node of a KD-Tree
 */
struct KDNode
{
    vec split_point;
    int axis;
    int level;
    KDNode *child_left;
    KDNode *child_right;
    KDNode() : child_left(NULL), child_right(NULL) {}
    ~KDNode() { delete child_left; delete child_right; }
};

/**
 * A node of a KD-Tree
 */
//struct KDNode
//{
//    vec split_point;
//    int axis;
//    int level;
//    int index; ///test
//    KDNode *child_left;
//    KDNode *child_right;
//    KDNode() : child_left(NULL), child_right(NULL) {}
//    ~KDNode() { delete child_left; delete child_right; }
//};

/**
 * KD-Tree wrapper. Holds a ptr to tree root.
 */
class KDTree
{
public:
    KDTree(VecArray &pts);
    ~KDTree();
    int depth() const { return m_depth; }
    const KDNode* root() const { return m_root; }
    const VecArray &pts;

public:
    static int makeNode(KDNode *node, VecArray &pts, const int level);
//    static int makeNode(KDNode *node, VecArray &pts, std::vector<int> indices, const int level);
    static void getNodesOfLevel(KDNode *node, std::vector<KDNode*> &nodes, int level);
    void kNearest(const int k, const vec& test_pt, const KDNode* root, const KDNode **knn, float *best_dist);
    static std::vector<size_t> sort_indexes(const std::vector<float3> &v, const int axis);
//    float findMinRec(KDNode* root, int d, unsigned depth);
//    float findMin(KDNode* root, int d);
//    float min(float v1, float v2, float v3);
//    float min(float v1, float v2);
    KDNode* deleteNodeRec(KDNode *root, vec point, int depth);
    KDNode* deleteNode(KDNode *root, vec point);
    KDNode* minNode(KDNode *x, KDNode *y, KDNode *z, int d);
    KDNode* findMinRec(KDNode* root, int d, unsigned depth);
    KDNode* findMin(KDNode* root, int d);

public:
    KDNode *m_root;
    int m_depth;
};

/**
 * Function object to compare 2 3D-vecs in the specified axis.
 */
struct VecComparator {
    unsigned axis;
    VecComparator(unsigned axis) : axis(axis % 3) {}
    virtual inline bool operator() (const vec& v1, const vec& v2) {
        return (v1.ptr()[axis] < v2.ptr()[axis]);
    }
};

#endif //LAB0_KDTREE_H


