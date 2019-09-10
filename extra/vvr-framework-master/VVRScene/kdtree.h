#include <MathGeoLib.h>

namespace vvr {

    /**
     * A node of a KD-Tree
     */
    struct KDNode
    {
        vec split_point;
        int axis;
        int level;
        AABB aabb;
        KDNode *child_left;
        KDNode *child_right;
        KDNode() : child_left(NULL), child_right(NULL) {}
        ~KDNode() { delete child_left; delete child_right; }
    };

    /**
     * KD-Tree wrapper. Holds a ptr to tree root.
     */
    class KDTree
    {
    public:
        KDTree(VecArray &pts, int dimensions = 3);
        ~KDTree();
        std::vector<KDNode*> getNodesOfLevel(int level);
        int depth() const { return m_depth; }
        const KDNode* root() const { return m_root; }
        const VecArray &pts;

    private:
        int makeNode(KDNode *node, VecArray &pts, const int level);
        void getNodesOfLevel(KDNode *node, std::vector<KDNode*> &nodes, int level);

    private:
        int m_DIM;
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

}
