#pragma once

namespace MXPBD {
    struct AABB {
        AABB() : min(vInf), max(vNegInf) {}
        AABB(const float a_min_x, const float a_min_y, const float a_min_z, const float a_max_x, const float a_max_y, const float a_max_z) : min(DirectX::XMVectorSet(a_min_x, a_min_y, a_min_z, 0.0f)), max(DirectX::XMVectorSet(a_max_x, a_max_y, a_max_z, 0.0f)) {}
        AABB(const Vector& a_min, const Vector& a_max) : min(a_min), max(a_max) {}

        Vector min = vZero;
        Vector max = vZero;

        [[nodiscard]] inline bool Overlaps(const AABB& b) const {
            const Vector fail1 = DirectX::XMVectorGreater(min, b.max);
            const Vector fail2 = DirectX::XMVectorLess(max, b.min);
            const Vector fail = DirectX::XMVectorOrInt(fail1, fail2);
            return DirectX::XMComparisonAllTrue(DirectX::XMVector3EqualIntR(fail, DirectX::XMVectorFalseInt()));
        }

        [[nodiscard]] inline AABB Merge(const AABB& b) const {
            return {DirectX::XMVectorMin(min, b.min), DirectX::XMVectorMax(max, b.max)};
        }

        [[nodiscard]] inline float SurfaceArea() const {
            const Vector d = DirectX::XMVectorSubtract(max, min);
            const Vector dSwizzled = DirectX::XMVectorSwizzle<DirectX::XM_SWIZZLE_Y, DirectX::XM_SWIZZLE_Z, DirectX::XM_SWIZZLE_X, DirectX::XM_SWIZZLE_W>(d);
            Vector area = DirectX::XMVector3Dot(d, dSwizzled);
            area = DirectX::XMVectorAdd(area, area);
            return DirectX::XMVectorGetX(area);
        }

        [[nodiscard]] inline Vector GetCenter() const {
            return DirectX::XMVectorMultiply(DirectX::XMVectorAdd(max, min), vHalf);
        }

        [[nodiscard]] inline Vector GetExtents(const float scale = 1.0f) const {
            return DirectX::XMVectorScale(DirectX::XMVectorSubtract(max, min), 0.5f * scale);
        }

        inline void Fatten(const float margin = 1.0f) {
            const Vector vMargin = DirectX::XMVectorSetW(DirectX::XMVectorReplicate(margin), 0.0f);
            min = DirectX::XMVectorSubtract(min, vMargin);
            max = DirectX::XMVectorAdd(max, vMargin);
        }

        [[nodiscard]] inline AABB GetWorldAABB(const Vector& pos, const Quaternion& rot, const float scale) const {
            const Vector centerLocal = GetCenter();
            const Vector extentsLocal = GetExtents(scale);

            const DirectX::XMMATRIX R = DirectX::XMMatrixRotationQuaternion(rot);
            const Vector centerWorld = DirectX::XMVectorAdd(pos, XMVector3TransformNormal(DirectX::XMVectorScale(centerLocal, scale), R));
            const Vector absR0 = DirectX::XMVectorAbs(R.r[0]);
            const Vector absR1 = DirectX::XMVectorAbs(R.r[1]);
            const Vector absR2 = DirectX::XMVectorAbs(R.r[2]);

            const Vector ex = DirectX::XMVectorSplatX(extentsLocal);
            const Vector ey = DirectX::XMVectorSplatY(extentsLocal);
            const Vector ez = DirectX::XMVectorSplatZ(extentsLocal);

            const Vector extentsWorld = DirectX::XMVectorAdd(
                DirectX::XMVectorAdd(DirectX::XMVectorMultiply(ex, absR0), DirectX::XMVectorMultiply(ey, absR1)),
                DirectX::XMVectorMultiply(ez, absR2));

            const Vector wMin = DirectX::XMVectorSubtract(centerWorld, extentsWorld);
            const Vector wMax = DirectX::XMVectorAdd(centerWorld, extentsWorld);
            return {wMin, wMax};
        }

        [[nodiscard]] inline bool IsContains(const AABB& b) const {
            const Vector minCheck = DirectX::XMVectorLessOrEqual(min, b.min);
            const Vector maxCheck = DirectX::XMVectorGreaterOrEqual(max, b.max);
            const Vector bothCheck = DirectX::XMVectorAndInt(minCheck, maxCheck);
            return DirectX::XMComparisonAllTrue(DirectX::XMVector3EqualIntR(bothCheck, DirectX::XMVectorTrueInt()));
        }

        [[nodiscard]] inline bool IsInvalid() const {
            if (DirectX::XMVector3Equal(min, vInf))
                return true;
            const Vector badMin = DirectX::XMVectorOrInt(DirectX::XMVectorIsNaN(min), DirectX::XMVectorIsInfinite(min));
            const Vector badMax = DirectX::XMVectorOrInt(DirectX::XMVectorIsNaN(max), DirectX::XMVectorIsInfinite(max));
            const Vector isExploded = DirectX::XMVectorOrInt(badMin, badMax);
            return !DirectX::XMComparisonAllTrue(DirectX::XMVector3EqualIntR(isExploded, DirectX::XMVectorFalseInt()));
        }
    };

    struct AABBPair {
        std::uint32_t objIdxA = UINT32_MAX;
        std::uint32_t objIdxB = UINT32_MAX;

        std::uint32_t beginA = 0;
        std::uint32_t endA = 0;
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