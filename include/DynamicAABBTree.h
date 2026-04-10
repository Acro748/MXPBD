#pragma once

namespace MXPBD {
    constexpr float AABB_MARGIN = 1.0f;

    struct AABB {
        float minX = 0.0f, minY = 0.0f, minZ = 0.0f;
        float maxX = 0.0f, maxY = 0.0f, maxZ = 0.0f;

        inline bool Overlaps(const AABB& b) const {
            if (maxX < b.minX || minX > b.maxX)
                return false;
            if (maxY < b.minY || minY > b.maxY)
                return false;
            if (maxZ < b.minZ || minZ > b.maxZ)
                return false;
            return true;
        }

        inline AABB Merge(const AABB& b) const {
            return {
                std::min(minX, b.minX), std::min(minY, b.minY), std::min(minZ, b.minZ),
                std::max(maxX, b.maxX), std::max(maxY, b.maxY), std::max(maxZ, b.maxZ)};
        }

        inline float SurfaceArea() const {
            float dX = maxX - minX;
            float dY = maxY - minY;
            float dZ = maxZ - minZ;
            return 2.0f * (dX * dY + dY * dZ + dZ * dX);
        }

        inline void Fatten() {
            minX -= AABB_MARGIN;
            minY -= AABB_MARGIN;
            minZ -= AABB_MARGIN;
            maxX += AABB_MARGIN;
            maxY += AABB_MARGIN;
            maxZ += AABB_MARGIN;
        }

        inline AABB GetWorldAABB(const Vector& pos, const Quaternion& rot, const float scale) const {
            const DirectX::XMVECTOR vMin = DirectX::XMVectorSet(minX, minY, minZ, 0.0f);
            const DirectX::XMVECTOR vMax = DirectX::XMVectorSet(maxX, maxY, maxZ, 0.0f);

            const DirectX::XMVECTOR centerLocal = DirectX::XMVectorScale(DirectX::XMVectorAdd(vMax, vMin), 0.5f);
            const DirectX::XMVECTOR extentsLocal = DirectX::XMVectorScale(DirectX::XMVectorSubtract(vMax, vMin), 0.5f * scale);

            const DirectX::XMMATRIX R = DirectX::XMMatrixRotationQuaternion(rot);
            const DirectX::XMVECTOR centerWorld = DirectX::XMVectorAdd(pos, XMVector3TransformNormal(DirectX::XMVectorScale(centerLocal, scale), R));
            const DirectX::XMVECTOR absR0 = DirectX::XMVectorAbs(R.r[0]);
            const DirectX::XMVECTOR absR1 = DirectX::XMVectorAbs(R.r[1]);
            const DirectX::XMVECTOR absR2 = DirectX::XMVectorAbs(R.r[2]);

            const DirectX::XMVECTOR ex = DirectX::XMVectorSplatX(extentsLocal);
            const DirectX::XMVECTOR ey = DirectX::XMVectorSplatY(extentsLocal);
            const DirectX::XMVECTOR ez = DirectX::XMVectorSplatZ(extentsLocal);

            const DirectX::XMVECTOR extentsWorld = DirectX::XMVectorAdd(
                DirectX::XMVectorAdd(DirectX::XMVectorMultiply(ex, absR0), DirectX::XMVectorMultiply(ey, absR1)),
                DirectX::XMVectorMultiply(ez, absR2));

            const DirectX::XMVECTOR wMin = DirectX::XMVectorSubtract(centerWorld, extentsWorld);
            const DirectX::XMVECTOR wMax = DirectX::XMVectorAdd(centerWorld, extentsWorld);

            DirectX::XMFLOAT3 fMin, fMax;
            DirectX::XMStoreFloat3(&fMin, wMin);
            DirectX::XMStoreFloat3(&fMax, wMax);

            return {fMin.x, fMin.y, fMin.z, fMax.x, fMax.y, fMax.z};
        }
    };

    struct AABBPair {
        std::uint32_t objIdxA = UINT32_MAX;
        std::uint32_t objIdxB = UINT32_MAX;
    };

    class DynamicAABBTree {
    public:
        DynamicAABBTree() { nodes.reserve(256); }

        void Clear();
        std::uint32_t InsertLeaf(const std::uint32_t objIdx, const AABB& leafAABB);
        void RemoveLeaf(const std::uint32_t leafNode);
        std::uint32_t UpdateLeaf(const std::uint32_t leafNode, const AABB& newAABB);
        void QueryPairs(const std::uint32_t queryObjIdx, const AABB& queryAABB, std::vector<AABBPair>& outPairs) const;

    private:
        struct TreeNode {
            AABB aabb;
            std::uint32_t parent = UINT32_MAX;
            std::uint32_t left = UINT32_MAX;
            std::uint32_t right = UINT32_MAX;

            std::uint32_t objIdx = UINT32_MAX;
            std::int32_t height = -1;

            inline bool IsLeaf() const { return left == UINT32_MAX; }
        };

        std::vector<TreeNode> nodes;
        std::uint32_t root = UINT32_MAX;
        std::uint32_t freeList = UINT32_MAX;
        std::uint32_t nodeCount = 0;

        std::uint32_t AllocateNode();
        void FreeNode(const std::uint32_t nodeId);
        void SyncHierarchy(const std::uint32_t index);
        std::uint32_t Rebalance(std::uint32_t iA);
    };
}