#include "KDTree.h"

#define DIMENSIONS 3

KDTree::KDTree(VecArray &pts)
        : pts(pts)
{
    const float t = vvr::getSeconds();
    m_root = new KDNode();

    std::vector<int> indices;
    for(int i=0; i<pts.size(); i++)
        indices.emplace_back(i);
    m_depth = makeNode(m_root, pts, 0);
//    m_depth = makeNode(m_root, pts, indices, 0);

    const float KDTree_construction_time = vvr::getSeconds() - t;
    echo(KDTree_construction_time);
    echo(m_depth);
}

KDTree::~KDTree()
{
    const float t = vvr::getSeconds();
    delete m_root;
    const float KDTree_destruction_time = vvr::getSeconds() - t;
//    echo(KDTree_destruction_time);
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

//int KDTree::makeNode(KDNode *node, VecArray &pts, std::vector<int> old_indices, const int level)
//{
//
//    std::vector<int> sorted_indices;
//
//    //! Sort along the appropriate axis, find median point and split.
//    const int axis = level % DIMENSIONS;
//
//    VecArray old_pts = pts;
//    VecArray new_pts = pts;
//
//    std::vector<size_t>  tmp = sort_indexes(pts, axis);
//
//    const int i_median = pts.size() / 2;
//
//    int s{0};
//    for (auto i:  tmp) {
//        if(s == pts.size()/2) {
//            node->split_point = pts[i];
//        }
//        sorted_indices.emplace_back(old_indices[i]);
//        new_pts[s] = pts[i];
//        s++;
//    }
//
//    node->index = sorted_indices[i_median];
//
//    //! Set node members
//    node->level = level;
//    node->axis = axis;
//
//    //! Continue recursively or stop.
//    if (pts.size() <= 1)
//    {
//        return level;
//    }
//    else
//    {
//        int level_left = 0;
//        int level_right = 0;
//        VecArray pts_left(new_pts.begin(), new_pts.begin() + i_median);
//        VecArray pts_right(new_pts.begin() + i_median + 1, new_pts.end());
//
//        if (!pts_left.empty())
//        {
//            node->child_left = new KDNode();
//
//            std::vector<int> new_indices;
//            for(int i=0; i<sorted_indices.size(); i++) {
//                if(i < i_median)
//                    new_indices.emplace_back(sorted_indices[i]);
//            }
//
//            level_left = makeNode(node->child_left, pts_left, new_indices, level + 1);
//
//        }
//        if (!pts_right.empty())
//        {
//            node->child_right = new KDNode();
//
//            std::vector<int> new_indices;
//            for(int i=0; i<sorted_indices.size(); i++) {
//                if(i > i_median)
//                    new_indices.emplace_back(sorted_indices[i]);
//            }
//
//            level_right = makeNode(node->child_right, pts_right, new_indices, level + 1);
//        }
//
//        int max_level = std::max(level_left, level_right);
//        return max_level;
//    }
//}

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
//    const float d = test_pt.Distance(root->split_point);
    const float d = test_pt.DistanceSq(root->split_point);
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
//    if ((d_split) >= *best_dist) return;

    kNearest(k, test_pt, right_of_split ? root->child_left : root->child_right, knn, best_dist);
}

std::vector<size_t> KDTree::sort_indexes(const std::vector<float3> &v, const int axis) {
    // initialize original index locations
    std::vector<size_t> idx(v.size());
    iota(idx.begin(), idx.end(), 0);

    if(axis == 0) {
        sort(idx.begin(), idx.end(),
             [&v](size_t i1, size_t i2) {return v[i1].ptr()[0] < v[i2].ptr()[0];});
    }
    else if(axis == 1){
        sort(idx.begin(), idx.end(),
             [&v](size_t i1, size_t i2) {return v[i1].ptr()[1] < v[i2].ptr()[1];});
    }
    else {
        sort(idx.begin(), idx.end(),
             [&v](size_t i1, size_t i2) {return v[i1].ptr()[2] < v[i2].ptr()[2];});
    }

    return idx;
}

//// Recursively finds minimum of d'th dimension in KD tree
//// The parameter depth is used to determine current axis.
//float KDTree::findMinRec(KDNode* root, int d, unsigned depth)
//{
//    // Base cases
//    if (!root)
//        return FLOAT_INF;
//
//    // Current dimension is computed using current depth and total
//    // dimensions (k)
//    unsigned cd = depth % DIMENSIONS;
//
//    // Compare point with root with respect to cd (Current dimension)
//    if (cd == d) {
//        if (root->child_left == NULL) {
//            return root->split_point.ptr()[d];
//        }
//        return min(root->split_point.ptr()[d], findMinRec(root->child_left, d, depth + 1));
//    }
//
//    // If current dimension is different then minimum can be anywhere
//    // in this subtree
//    return min(root->split_point.ptr()[d],
//               findMinRec(root->child_left, d, depth + 1),
//               findMinRec(root->child_right, d, depth + 1));
//}
//
//// A wrapper over findMinRec(). Returns minimum of d'th dimension
//float KDTree::findMin(KDNode* root, int d)
//{
//    // Pass current level or depth as 0
//    return findMinRec(root, d, 0);
//}
//
//float KDTree::min(float v1, float v2, float v3) {
//    if ((v1 <= v2) && (v1 <= v3))
//        return v1;
//    if ((v2 <= v1) && (v2 <= v3))
//        return v2;
//    if ((v3 <= v1) && (v3 <= v2))
//        return v3;
//}
//
//float KDTree::min(float v1, float v2) {
//    if (v1 <= v2)
//        return v1;
//    else
//        return v2;
//}

// A utility function to find minimum of three integers
KDNode *KDTree::minNode(KDNode *x, KDNode *y, KDNode *z, int d)
{
    KDNode *res = x;
    if (y != NULL && y->split_point.ptr()[d] < res->split_point.ptr()[d])
        res = y;
    if (z != NULL && z->split_point.ptr()[d] < res->split_point.ptr()[d])
        res = z;
    return res;
}

// Recursively finds minimum of d'th dimension in KD tree
// The parameter depth is used to determine current axis.
KDNode* KDTree::findMinRec(KDNode* root, int d, unsigned depth)
{
    // Base cases
    if (root == NULL)
        return NULL;

    // Current dimension is computed using current depth and total
    // dimensions (k)
    unsigned cd = depth % DIMENSIONS;

    // Compare point with root with respect to cd (Current dimension)
    if (cd == d)
    {
        if (root->child_left == NULL)
            return root;
        return findMinRec(root->child_left, d, depth+1);
    }

    // If current dimension is different then minimum can be anywhere
    // in this subtree
    return minNode(root,
                   findMinRec(root->child_left, d, depth+1),
                   findMinRec(root->child_right, d, depth+1), d);
}

// A wrapper over findMinRec(). Returns minimum of d'th dimension
KDNode* KDTree::findMin(KDNode* root, int d)
{
    // Pass current level or depth as 0
    return findMinRec(root, d, 0);
}

// A utility method to determine if two Points are same
// in K Dimensional space
bool arePointsSame(int point1[], int point2[])
{
    // Compare individual pointinate values
    for (int i = 0; i < DIMENSIONS; ++i)
        if (point1[i] != point2[i])
            return false;

    return true;
}

// Copies point p2 to p1
void copyPoint(int p1[], int p2[])
{
    for (int i=0; i<DIMENSIONS; i++)
        p1[i] = p2[i];
}

// Function to delete a given point 'point[]' from tree with root
// as 'root'.  depth is current depth and passed as 0 initially.
// Returns root of the modified tree.
KDNode* KDTree::deleteNodeRec(KDNode *root, vec point, int depth)
{
//    std::cout << "in deleteNodeRec, root = " << root->split_point << std::endl;

    // Given point is not present
    if (root == NULL) {
//        std::cout << "root == null" << std::endl;
        return NULL;
    }

    // Find dimension of current node
    int cd = depth % DIMENSIONS;

    // If the point to be deleted is present at root
//    if (arePointsSame(root->point, point))
//    std::cout << "check for same points, between " << root->split_point << " and " << point << std::endl;
//    std::cout << "x : " << (root->split_point.x == point.x) << std::endl;
//    std::cout << "y : " << (root->split_point.y == point.y) << std::endl;
//    std::cout << "z : " << (root->split_point.z == point.z) << std::endl;

    if ( (root->split_point.x == point.x) && (root->split_point.y == point.y) && (root->split_point.z == point.z))
    {
//        std::cout << "same" << std::endl;
//        std::cout << 1 << std::endl;
        // 2.b) If right child is not NULL
        if (root->child_right != NULL)
        {
//            std::cout << 2 << std::endl;
            // Find minimum of root's dimension in right subtree
            KDNode *min = findMin(root->child_right, cd);
//            std::cout << 3 << std::endl;

            // Copy the minimum to root
//            copyPoint(root->point, min->point);
            root->split_point = min->split_point;
//            std::cout << 4 << std::endl;

//            std::cout << "new root = " << root->split_point << std::endl;
//            std::cout << "child_right = " << root->child_right->split_point << std::endl;

            // Recursively delete the minimum
            root->child_right = deleteNodeRec(root->child_right, min->split_point, depth+1);
//            std::cout << 5 << std::endl;
        }
        else if (root->child_left != NULL) // same as above
        {
//            std::cout << 6 << std::endl;
            KDNode *min = findMin(root->child_left, cd);
//            copyPoint(root->point, min->point);
//            std::cout << 7 << std::endl;
//            std::cout << "old root  = " << root->split_point << std::endl;
            root->split_point = min->split_point;
//            std::cout << "min = " << min->split_point << std::endl;
//            std::cout << "new root  = " << root->split_point << std::endl;
//            std::cout << 8 << std::endl;

//            root->child_right = deleteNodeRec(root->child_left, min->split_point, depth+1); /// old
            root->child_left = deleteNodeRec(root->child_left, min->split_point, depth+1);
//            std::cout << 9 << std::endl;
        }
        else // If node to be deleted is leaf node
        {
//            std::cout << 10 << std::endl;
//            std::cout << "root for deletion = " << root->split_point << std::endl;
            delete root;
            return NULL;
        }
        return root;
    }

//    std::cout << "cd = " << cd << std::endl;
//    std::cout << "root = " << root->split_point << std::endl;
//    std::cout << "point = " << point << std::endl;

    // 2) If current node doesn't contain point, search downward
    if (cd == 0) {
//        if (point.x < root->split_point.ptr()[0])
        if (point.x < root->split_point.x)
            root->child_left = deleteNodeRec(root->child_left, point, depth+1);
        else
            root->child_right = deleteNodeRec(root->child_right, point, depth+1);
    }
    else if (cd == 1) {
//        if (point.y < root->split_point.ptr()[1])
        if (point.y < root->split_point.y)
            root->child_left = deleteNodeRec(root->child_left, point, depth+1);
        else
            root->child_right = deleteNodeRec(root->child_right, point, depth+1);
    }
    else {
//        if (point.z < root->split_point.ptr()[2])
        if (point.z < root->split_point.z)
            root->child_left = deleteNodeRec(root->child_left, point, depth+1);
        else
            root->child_right = deleteNodeRec(root->child_right, point, depth+1);
    }
    return root;
}

// Function to delete a given point from K D Tree with 'root'
KDNode* KDTree::deleteNode(KDNode *root, vec point)
{
    // Pass depth as 0
//    std::cout << "in deleteNode, root = " << root->split_point << std::endl;
    return deleteNodeRec(root, point, 0);
}