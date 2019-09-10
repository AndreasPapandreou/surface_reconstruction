#include "kdtree.h"
#include "utils.h"
#include <algorithm>

using namespace vvr;
using namespace std;
using namespace math;

KDTree::KDTree(VecArray &pts, int dimensions)
    : m_DIM(dimensions)
    , pts(pts)
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
    const int axis = level % m_DIM;
    std::sort(pts.begin(), pts.end(), VecComparator(axis));
    const int i_median = pts.size() / 2;

    //! Set node members
    node->level = level;
    node->axis = axis;
    node->split_point = pts[i_median];
    node->aabb.SetFrom(&pts[0], pts.size());

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

std::vector<KDNode*> KDTree::getNodesOfLevel(const int level)
{
    std::vector<KDNode*> nodes;
    if (!m_root) return nodes;
    getNodesOfLevel(m_root, nodes, level);
    return nodes;
}
