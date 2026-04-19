#include "MXPBD/DynamicAABBTree.h"

namespace MXPBD {
    void DynamicAABBTree::Clear()
    {
        nodes.clear();
        root = UINT32_MAX;
        freeList = UINT32_MAX;
        nodeCount = 0;
    }

    std::uint32_t DynamicAABBTree::InsertLeaf(const std::uint32_t objIdx, const AABB& leafAABB)
    {
        const std::uint32_t leafNode = AllocateNode();
        nodes[leafNode].aabb = leafAABB;
        nodes[leafNode].objIdx = objIdx;
        nodes[leafNode].height = 0;

        if (root == UINT32_MAX)
        {
            root = leafNode;
            return leafNode;
        }

        std::uint32_t bestSibling = root;
        while (!nodes[bestSibling].IsLeaf())
        {
            const std::uint32_t left = nodes[bestSibling].left;
            const std::uint32_t right = nodes[bestSibling].right;

            const float area = nodes[bestSibling].aabb.SurfaceArea();
            const AABB combined = nodes[bestSibling].aabb.Merge(leafAABB);
            const float combinedArea = combined.SurfaceArea();

            const float cost = 2.0f * combinedArea;
            const float inheritanceCost = 2.0f * (combinedArea - area);

            float costLeft = inheritanceCost + nodes[left].aabb.Merge(leafAABB).SurfaceArea();
            if (!nodes[left].IsLeaf())
                costLeft -= nodes[left].aabb.SurfaceArea();
            float costRight = inheritanceCost + nodes[right].aabb.Merge(leafAABB).SurfaceArea();
            if (!nodes[right].IsLeaf())
                costRight -= nodes[right].aabb.SurfaceArea();
            if (cost < costLeft && cost < costRight)
                break;
            bestSibling = (costLeft < costRight) ? left : right;
        }

        const std::uint32_t oldParent = nodes[bestSibling].parent;
        const std::uint32_t newParent = AllocateNode();

        nodes[newParent].parent = oldParent;
        nodes[newParent].aabb = leafAABB.Merge(nodes[bestSibling].aabb);
        nodes[newParent].height = nodes[bestSibling].height + 1;

        if (oldParent != UINT32_MAX)
        {
            if (nodes[oldParent].left == bestSibling)
                nodes[oldParent].left = newParent;
            else
                nodes[oldParent].right = newParent;
        }
        else
        {
            root = newParent;
        }

        nodes[newParent].left = bestSibling;
        nodes[newParent].right = leafNode;
        nodes[bestSibling].parent = newParent;
        nodes[leafNode].parent = newParent;

        SyncHierarchy(nodes[leafNode].parent);
        return leafNode;
    }

    void DynamicAABBTree::RemoveLeaf(const std::uint32_t leafNode)
    {
        if (leafNode == root)
        {
            root = UINT32_MAX;
            FreeNode(leafNode);
            return;
        }

        const std::uint32_t parent = nodes[leafNode].parent;
        const std::uint32_t grandParent = nodes[parent].parent;
        const std::uint32_t sibling = (nodes[parent].left == leafNode) ? nodes[parent].right : nodes[parent].left;

        if (grandParent != UINT32_MAX)
        {
            if (nodes[grandParent].left == parent)
                nodes[grandParent].left = sibling;
            else
                nodes[grandParent].right = sibling;
            nodes[sibling].parent = grandParent;
            FreeNode(parent);
            SyncHierarchy(grandParent);
        }
        else
        {
            root = sibling;
            nodes[sibling].parent = UINT32_MAX;
            FreeNode(parent);
        }
        FreeNode(leafNode);
    }

    std::uint32_t DynamicAABBTree::UpdateLeaf(const std::uint32_t leafNode, const AABB& newAABB)
    {
        if (nodes[leafNode].aabb.IsContains(newAABB))
            return leafNode;
        const std::uint32_t currentObjIdx = nodes[leafNode].objIdx;
        if (currentObjIdx == UINT32_MAX)
            return leafNode;
        RemoveLeaf(leafNode);
        AABB fatAABB = newAABB;
        fatAABB.Fatten();
        return InsertLeaf(currentObjIdx, fatAABB);
    }

    void DynamicAABBTree::QueryPairs(const std::uint32_t queryObjIdx, const AABB& queryAABB, std::vector<AABBPair>& outPairs) const
    {
        if (queryObjIdx == UINT32_MAX || root == UINT32_MAX)
            return;

        std::vector<std::uint32_t> stack;
        stack.reserve(256);
        stack.push_back(root);

        while (!stack.empty())
        {
            const std::uint32_t nodeId = stack.back();
            stack.pop_back();

            const TreeNode& node = nodes[nodeId];
            if (node.aabb.Overlaps(queryAABB))
            {
                if (node.IsLeaf())
                {
                    if (node.objIdx != UINT32_MAX && queryObjIdx < node.objIdx)
                    {
                        outPairs.push_back({queryObjIdx, node.objIdx});
                    }
                }
                else
                {
                    stack.push_back(node.left);
                    stack.push_back(node.right);
                }
            }
        }
    }

    std::uint32_t DynamicAABBTree::AllocateNode()
    {
        if (freeList == UINT32_MAX)
        {
            const std::uint32_t newIdx = static_cast<std::uint32_t>(nodes.size());
            nodes.push_back(TreeNode());
            return newIdx;
        }
        const std::uint32_t nodeId = freeList;
        freeList = nodes[nodeId].left;
        nodes[nodeId] = TreeNode();
        return nodeId;
    }

    void DynamicAABBTree::FreeNode(std::uint32_t nodeId)
    {
        nodes[nodeId].left = freeList;
        nodes[nodeId].height = -1;
        freeList = nodeId;
    }

    void DynamicAABBTree::SyncHierarchy(std::uint32_t index)
    {
        while (index != UINT32_MAX)
        {
            index = Rebalance(index);

            const std::uint32_t left = nodes[index].left;
            const std::uint32_t right = nodes[index].right;
            nodes[index].height = 1 + std::max(nodes[left].height, nodes[right].height);
            nodes[index].aabb = nodes[left].aabb.Merge(nodes[right].aabb);
            index = nodes[index].parent;
        }
    }

    std::uint32_t DynamicAABBTree::Rebalance(std::uint32_t iA)
    {
        if (iA == UINT32_MAX || nodes[iA].IsLeaf() || nodes[iA].height < 2)
        {
            return iA;
        }
        const std::uint32_t iB = nodes[iA].left;
        const std::uint32_t iC = nodes[iA].right;
        const std::int32_t balance = nodes[iC].height - nodes[iB].height;
        if (balance > 1)
        {
            const std::uint32_t iF = nodes[iC].left;
            const std::uint32_t iG = nodes[iC].right;

            nodes[iC].left = iA;
            nodes[iC].parent = nodes[iA].parent;
            nodes[iA].parent = iC;

            if (nodes[iC].parent != UINT32_MAX)
            {
                if (nodes[nodes[iC].parent].left == iA)
                    nodes[nodes[iC].parent].left = iC;
                else
                    nodes[nodes[iC].parent].right = iC;
            }
            else
            {
                root = iC;
            }

            if (nodes[iF].height > nodes[iG].height)
            {
                nodes[iC].right = iF;
                nodes[iA].right = iG;
                nodes[iG].parent = iA;
                nodes[iA].aabb = nodes[iB].aabb.Merge(nodes[iG].aabb);
                nodes[iC].aabb = nodes[iA].aabb.Merge(nodes[iF].aabb);

                nodes[iA].height = 1 + std::max(nodes[iB].height, nodes[iG].height);
                nodes[iC].height = 1 + std::max(nodes[iA].height, nodes[iF].height);
            }
            else
            {
                nodes[iC].right = iG;
                nodes[iA].right = iF;
                nodes[iF].parent = iA;
                nodes[iA].aabb = nodes[iB].aabb.Merge(nodes[iF].aabb);
                nodes[iC].aabb = nodes[iA].aabb.Merge(nodes[iG].aabb);

                nodes[iA].height = 1 + std::max(nodes[iB].height, nodes[iF].height);
                nodes[iC].height = 1 + std::max(nodes[iA].height, nodes[iG].height);
            }

            return iC;
        }

        if (balance < -1)
        {
            const std::uint32_t iD = nodes[iB].left;
            const std::uint32_t iE = nodes[iB].right;

            nodes[iB].left = iA;
            nodes[iB].parent = nodes[iA].parent;
            nodes[iA].parent = iB;

            if (nodes[iB].parent != UINT32_MAX)
            {
                if (nodes[nodes[iB].parent].left == iA)
                    nodes[nodes[iB].parent].left = iB;
                else
                    nodes[nodes[iB].parent].right = iB;
            }
            else
            {
                root = iB;
            }

            if (nodes[iD].height > nodes[iE].height)
            {
                nodes[iB].right = iD;
                nodes[iA].left = iE;
                nodes[iE].parent = iA;
                nodes[iA].aabb = nodes[iC].aabb.Merge(nodes[iE].aabb);
                nodes[iB].aabb = nodes[iA].aabb.Merge(nodes[iD].aabb);

                nodes[iA].height = 1 + std::max(nodes[iC].height, nodes[iE].height);
                nodes[iB].height = 1 + std::max(nodes[iA].height, nodes[iD].height);
            }
            else
            {
                nodes[iB].right = iE;
                nodes[iA].left = iD;
                nodes[iD].parent = iA;
                nodes[iA].aabb = nodes[iC].aabb.Merge(nodes[iD].aabb);
                nodes[iB].aabb = nodes[iA].aabb.Merge(nodes[iE].aabb);

                nodes[iA].height = 1 + std::max(nodes[iC].height, nodes[iD].height);
                nodes[iB].height = 1 + std::max(nodes[iA].height, nodes[iE].height);
            }

            return iB;
        }

        return iA;
    }
}