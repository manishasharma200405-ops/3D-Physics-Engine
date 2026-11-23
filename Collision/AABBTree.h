#pragma once
// Dynamic bounding-volume tree used as the broad phase.
// Leaves store "fat" AABBs so slowly moving bodies rarely need re-insertion.
#include <vector>

#include "Core/EngineMath.h"

namespace eng {

class AABBTree {
public:
    static constexpr int kNull = -1;

    explicit AABBTree(float margin = 0.1f) : margin_(margin) {}

    // Returns a proxy id.  `userData` is handed back from queries (we store the body index).
    int createProxy(const AABB& tight, int userData);
    void destroyProxy(int proxy);

    // Call every step.  Re-inserts only if the tight box left its fat box.
    // `displacement` predicts motion so the fat box extends in the direction of travel.
    bool moveProxy(int proxy, const AABB& tight, const Vec3& displacement);

    const AABB& fatAABB(int proxy) const { return nodes_[proxy].box; }
    int userData(int proxy) const { return nodes_[proxy].userData; }

    // Calls cb(proxyId) for every leaf whose fat box overlaps `box`.
    template <class F>
    void query(const AABB& box, F&& cb) const {
        if (root_ == kNull) return;
        std::vector<int> stack;
        stack.push_back(root_);
        while (!stack.empty()) {
            int n = stack.back();
            stack.pop_back();
            const Node& node = nodes_[n];
            if (!node.box.overlaps(box)) continue;
            if (node.isLeaf()) cb(n);
            else { stack.push_back(node.child1); stack.push_back(node.child2); }
        }
    }

    int height() const { return root_ == kNull ? 0 : nodes_[root_].height; }
    int proxyCount() const { return proxyCount_; }
    void clear();

private:
    struct Node {
        AABB box;
        int parent = kNull;   // doubles as the free-list link
        int child1 = kNull;
        int child2 = kNull;
        int height = 0;
        int userData = -1;
        bool isLeaf() const { return child1 == kNull; }
    };

    int allocNode();
    void freeNode(int n);
    void insertLeaf(int leaf);
    void removeLeaf(int leaf);

    std::vector<Node> nodes_;
    int root_ = kNull;
    int freeList_ = kNull;
    int proxyCount_ = 0;
    float margin_;
};

}  // namespace eng
