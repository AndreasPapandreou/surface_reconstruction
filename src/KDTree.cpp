#include "KDTree.h"

#define DIMENSIONS 3

KDTree::KDTree(VecArray &pts)
        : pts(pts)
{
    const float t = vvr::getSeconds();
    m_root = new KDNode();
    m_depth = makeNode(m_root, pts, 0);
    const float KDTree_construction_time = vvr::getSeconds() - t;
    echo(KDTree_construction_time);
    echo(m_depth);
}

KDTree::~KDTree()
{
    const float t = vvr::getSeconds();
    delete m_root;
    const float KDTree_destruction_time = vvr::getSeconds() - t;
    echo(KDTree_destruction_time);
}

int KDTree::makeNode(KDNode *node, VecArray &pts, const int level)
{
    //! Sort along the appropriate axis, find median point and split.
    const int axis = level % DIMENSIONS;
    std::sort(pts.begin(), pts.end(), VecComparator(axis));
    const int i_median = pts.size() / 2;

    //! Set node members
    node->level = level;
    node->axis = axis;
    node->split_point = pts[i_median];

    //! Continue recursively or stop.
    if (pts.size() <= 1)
    {
        return level;
    }
    else
    {
        int level_left = 0;
        int level_right = 0;
        VecArray pts_left(pts.begin(), pts.begin() + i_median);
        VecArray pts_right(pts.begin() + i_median + 1, pts.end());

        if (!pts_left.empty())
        {
            node->child_left = new KDNode();
            level_left = makeNode(node->child_left, pts_left, level + 1);

        }
        if (!pts_right.empty())
        {
            node->child_right = new KDNode();
            level_right = makeNode(node->child_right, pts_right, level + 1);
        }

        int max_level = std::max(level_left, level_right);
        return max_level;
    }
}

void KDTree::getNodesOfLevel(KDNode *node, std::vector<KDNode*> &nodes, int level)
{
    if (!level)
    {
        nodes.push_back(node);
    }
    else
    {
        if (node->child_left) getNodesOfLevel(node->child_left, nodes, level - 1);
        if (node->child_right) getNodesOfLevel(node->child_right, nodes, level - 1);
    }
}

void KDTree::kNearest(const int k, const vec &test_pt, const KDNode *root, const KDNode **knn, float *best_dist) {
    if(!root) return;

    //!Distance
    const float d = test_pt.Distance(root->split_point);
    const float d_split = root->split_point.ptr()[root->axis] - test_pt.ptr()[root->axis];
    const bool right_of_split = d_split <= 0;

    if (*(knn+k) == NULL || d < *best_dist) {

        bool flag{true};
        for (int j=0; j<=k; j++) {
            if (*(knn+j) == root) {
                flag = false;
            }
        }

        if (flag) {
            *best_dist = d;
            *(knn+k) = root;
        }
    }

    // searching
    kNearest(k, test_pt, right_of_split ? root->child_right : root->child_left, knn, best_dist);

    // pruning
    if (SQUARE(d_split) >= *best_dist) return;

    kNearest(k, test_pt, right_of_split ? root->child_left : root->child_right, knn, best_dist);
}
