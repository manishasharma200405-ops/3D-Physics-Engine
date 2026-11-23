#include "Collision/AABBTree.h"

#include <algorithm>

namespace eng {

int AABBTree::allocNode() {
    if (freeList_ == kNull) {
        nodes_.emplace_back();
        return (int)nodes_.size() - 1;
    }
    int n = freeList_;
    freeList_ = nodes_[n].parent;
    nodes_[n] = Node();
    return n;
}

void AABBTree::freeNode(int n) {
    nodes_[n] = Node();
    nodes_[n].parent = freeList_;
    freeList_ = n;
}

int AABBTree::createProxy(const AABB& tight, int userData) {
    int p = allocNode();
    nodes_[p].box = tight.fattened(margin_);
    nodes_[p].userData = userData;
    nodes_[p].height = 0;
    insertLeaf(p);
    proxyCount_++;
    return p;
}

void AABBTree::destroyProxy(int proxy) {
    removeLeaf(proxy);
    freeNode(proxy);
    proxyCount_--;
}

bool AABBTree::moveProxy(int proxy, const AABB& tight, const Vec3& displacement) {
    if (nodes_[proxy].box.contains(tight)) return false;

    removeLeaf(proxy);
    AABB fat = tight.fattened(margin_);
    // Stretch the box along the direction of motion.
    Vec3 d = displacement * 4.f;
    if (d.x < 0.f) fat.min.x += d.x; else fat.max.x += d.x;
    if (d.y < 0.f) fat.min.y += d.y; else fat.max.y += d.y;
    if (d.z < 0.f) fat.min.z += d.z; else fat.max.z += d.z;
    nodes_[proxy].box = fat;
    insertLeaf(proxy);
    return true;
}

void AABBTree::insertLeaf(int leaf) {
    nodes_[leaf].parent = kNull;
    if (root_ == kNull) {
        root_ = leaf;
        return;
    }

    const AABB leafBox = nodes_[leaf].box;

    // Descend, choosing the child that increases total surface area the least.
    int index = root_;
    while (!nodes_[index].isLeaf()) {
        int c1 = nodes_[index].child1;
        int c2 = nodes_[index].child2;

        float area = nodes_[index].box.surfaceArea();
        float combinedArea = AABB::merge(nodes_[index].box, leafBox).surfaceArea();
        float cost = 2.f * combinedArea;               // cost of making a new parent here
        float inherit = 2.f * (combinedArea - area);   // cost pushed down to children

        auto childCost = [&](int c) {
            AABB merged = AABB::merge(leafBox, nodes_[c].box);
            if (nodes_[c].isLeaf()) return merged.surfaceArea() + inherit;
            return merged.surfaceArea() - nodes_[c].box.surfaceArea() + inherit;
        };
        float cost1 = childCost(c1);
        float cost2 = childCost(c2);

        if (cost < cost1 && cost < cost2) break;
        index = cost1 < cost2 ? c1 : c2;
    }

    int sibling = index;
    int oldParent = nodes_[sibling].parent;
    int newParent = allocNode();  // may reallocate: use indices, never references, from here on
    nodes_[newParent].parent = oldParent;
    nodes_[newParent].box = AABB::merge(leafBox, nodes_[sibling].box);
    nodes_[newParent].height = nodes_[sibling].height + 1;
    nodes_[newParent].child1 = sibling;
    nodes_[newParent].child2 = leaf;
    nodes_[sibling].parent = newParent;
    nodes_[leaf].parent = newParent;

    if (oldParent != kNull) {
        if (nodes_[oldParent].child1 == sibling) nodes_[oldParent].child1 = newParent;
        else nodes_[oldParent].child2 = newParent;
    } else {
        root_ = newParent;
    }

    // Refit ancestors.
    index = nodes_[leaf].parent;
    while (index != kNull) {
        int c1 = nodes_[index].child1, c2 = nodes_[index].child2;
        nodes_[index].box = AABB::merge(nodes_[c1].box, nodes_[c2].box);
        nodes_[index].height = 1 + std::max(nodes_[c1].height, nodes_[c2].height);
        index = nodes_[index].parent;
    }
}

void AABBTree::removeLeaf(int leaf) {
    if (leaf == root_) {
        root_ = kNull;
        return;
    }

    int parent = nodes_[leaf].parent;
    int grand = nodes_[parent].parent;
    int sibling = nodes_[parent].child1 == leaf ? nodes_[parent].child2 : nodes_[parent].child1;

    if (grand != kNull) {
        if (nodes_[grand].child1 == parent) nodes_[grand].child1 = sibling;
        else nodes_[grand].child2 = sibling;
        nodes_[sibling].parent = grand;
        freeNode(parent);

        int index = grand;
        while (index != kNull) {
            int c1 = nodes_[index].child1, c2 = nodes_[index].child2;
            nodes_[index].box = AABB::merge(nodes_[c1].box, nodes_[c2].box);
            nodes_[index].height = 1 + std::max(nodes_[c1].height, nodes_[c2].height);
            index = nodes_[index].parent;
        }
    } else {
        root_ = sibling;
        nodes_[sibling].parent = kNull;
        freeNode(parent);
    }
    nodes_[leaf].parent = kNull;
}

void AABBTree::clear() {
    nodes_.clear();
    root_ = kNull;
    freeList_ = kNull;
    proxyCount_ = 0;
}

}  // namespace eng
