#include "KDTree2.h"

#define DIMENSIONS 4

KDTree2::KDTree2(VecArray4 &pts)
        : pts(pts)
{
    const float t = vvr::getSeconds();
    m_root = new KDNode2();
    m_depth = makeNode(m_root, pts, 0);
    const float KDTree_construction_time = vvr::getSeconds() - t;
    echo(KDTree_construction_time);
    echo(m_depth);
}

KDTree2::~KDTree2()
{
    const float t = vvr::getSeconds();
    delete m_root;
    const float KDTree_destruction_time = vvr::getSeconds() - t;
    echo(KDTree_destruction_time);
}

int KDTree2::makeNode(KDNode2 *node, VecArray4 &pts, const int level)
{
    //! Sort along the appropriate axis, find median point and split.
    const int axis = level % DIMENSIONS;
    std::sort(pts.begin(), pts.end(), VecComparator2(axis));
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
        VecArray4 pts_left(pts.begin(), pts.begin() + i_median);
        VecArray4 pts_right(pts.begin() + i_median + 1, pts.end());

        if (!pts_left.empty())
        {
            node->child_left = new KDNode2();
            level_left = makeNode(node->child_left, pts_left, level + 1);

        }
        if (!pts_right.empty())
        {
            node->child_right = new KDNode2();
            level_right = makeNode(node->child_right, pts_right, level + 1);
        }

        int max_level = std::max(level_left, level_right);
        return max_level;
    }
}

void KDTree2::getNodesOfLevel(KDNode2 *node, std::vector<KDNode2*> &nodes, int level)
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

void KDTree2::kNearest(const int k, const float4 &test_pt, const KDNode2 *root, const KDNode2 **knn, float *best_dist, float &weight) {
    if(!root) return;

    //!Distance
//    const float d = test_pt.Distance(root->split_point);
    const float d = Distance2(test_pt, root->split_point, weight);
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
    kNearest(k, test_pt, right_of_split ? root->child_right : root->child_left, knn, best_dist, weight);

    // pruning
    if (SQUARE(d_split) >= *best_dist) return;

    kNearest(k, test_pt, right_of_split ? root->child_left : root->child_right, knn, best_dist, weight);
}

float KDTree2::Distance2(const float4 p1, const float4 p2, float &weight) {
//    float weight = 0.05f;
    return static_cast<float>(sqrt(pow(p1.x - p2.x, 2) + pow(p1.y - p2.y, 2) + pow(p1.z - p2.z, 2) + weight * pow(p1.w - p2.w, 2)));
}
