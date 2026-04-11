#pragma once

#include <ankerl/unordered_dense.h>
#include <tbb/task_scheduler_observer.h>
namespace MXPBD {
    class TBB_CoreMasking : public tbb::task_scheduler_observer {
        DWORD_PTR mask;

    public:
        TBB_CoreMasking(tbb::task_arena& arena, DWORD_PTR m)
            : tbb::task_scheduler_observer(arena), mask(m) {
            observe(true);
        }

        void on_scheduler_entry(bool worker) override {
            if (worker) {
                static thread_local bool pinned = false;
                if (!pinned) {
                    SetThreadAffinityMask(GetCurrentThread(), mask);
                    pinned = true;
                }
            }
        }
    };

    class TBB_ThreadPool {
    public:
        TBB_ThreadPool() = delete;
        TBB_ThreadPool(std::uint32_t a_threadSize, std::uint64_t a_coreMask)
            : workers(std::make_unique<tbb::task_arena>(a_threadSize)) {
            if (a_coreMask != 0)
                observer = std::make_unique<TBB_CoreMasking>(*workers, a_coreMask);
        }

        template <typename F>
        void Execute(F&& f) {
            workers->execute(std::forward<F>(f));
        }

        template <typename F>
        void Enqueue(F&& f) {
            workers->enqueue(std::forward<F>(f));
        }

        std::int32_t GetThreadSize() const { return workers->max_concurrency(); }

    private:
        std::unique_ptr<tbb::task_arena> workers;
        std::unique_ptr<TBB_CoreMasking> observer;
    };

    class XPBDWorld {
    public:
        XPBDWorld(std::uint8_t iteration, float gridSize);

        enum RootType {
            skeleton,
            cloth,
            headpart,
            none,
            total
        };

        void AddPhysics(RE::TESObjectREFR* object, RE::NiNode* rootNode, const RootType rootType, const PhysicsInput& input);
        void UpdatePhysicsSetting(RE::TESObjectREFR* object, const PhysicsInput& input, bool reset = true);
        void Reset();
        void Reset(RE::TESObjectREFR* object);
        void RemovePhysics(const RE::FormID objectID);
        void RemovePhysics(RE::TESObjectREFR* object, const RootType rootType, const std::uint32_t bipedSlot);
        void TogglePhysics(const RE::FormID objectID, bool isDisable);

        void RunPhysicsWorld(const float deltaTime);

    private:
        mutable std::mutex lock;
        const std::uint8_t ITERATION_MAX = 5;
        const float GRID_SIZE = 20.0f;
        bool orderDirty = false;
        std::uint64_t currentFrame = 0;
        std::unique_ptr<TBB_ThreadPool> threadPool;
        std::unordered_set<RE::FormID> resetObjects;

        struct CleanObject {
            std::uint32_t objIdx = UINT32_MAX;
            std::uint32_t rootIdx = UINT32_MAX;

            bool operator==(const CleanObject& co) const {
                return objIdx == co.objIdx && rootIdx == co.rootIdx;
            }
        };
        struct CleanObjectHash {
            std::size_t operator()(const CleanObject& co) const {
                return static_cast<std::uint64_t>(co.objIdx) << 32 | co.rootIdx;
            }
        };

        struct ObjectDatas {
            std::vector<RE::FormID> objectID;
            struct Root { // ri
                RootType type;
                std::uint32_t bipedSlot;
                bool operator==(const Root& other) const {
                    return type == other.type && bipedSlot == other.bipedSlot;
                }
            };
            std::vector<std::vector<Root>> roots;                // skeleton, armor
            std::vector<Vector> beforeWorldPos;
            std::vector<Vector> velocity;
            std::vector<Vector> acceleration;
            std::vector<AABB> boundingAABB;
            std::vector<std::uint8_t> isDisable;                 // just physics disable
        };
        ObjectDatas objectDatas;

        using BatchLaneIndex = std::uint32_t;
        inline std::uint32_t GetBatchIdx(BatchLaneIndex i) { return i >> 3; };
        inline std::uint8_t GetLaneIdx(BatchLaneIndex i) { return i & 0x7; };
        inline BatchLaneIndex SetBatchLaneIdx(std::uint32_t batchIdx, std::uint8_t laneIdx) { return (batchIdx << 3) | (laneIdx & 0x7); };

        struct PhysicsBones {
            std::uint32_t numBones = 0;

            // 3 DOF
            std::vector<Vector> pos;                // world
            std::vector<Vector> pred; // world
            std::vector<Vector> vel;

            // 6 DOF
            std::vector<std::uint8_t> advancedRotation;
            std::vector<Quaternion> rot;
            std::vector<Quaternion> prevRot;
            std::vector<Quaternion> predRot;
            std::vector<Vector> angVel;
            std::vector<float> invInertia;

            // setting
            std::vector<float> damping;
            std::vector<float> inertiaScale;
            std::vector<float> rotRatio;
            std::vector<Vector> gravity;
            std::vector<Vector> offset;
            std::vector<float> invMass;

            // fake rotation setting
            std::vector<float> linearRotTorque;

            // collision
            std::vector<float> colMargin;
            std::vector<float> colFriction;
            std::vector<float> colComp;

            struct alignas(64) FrictionCache {
                float nx = 0.0f, ny = 0.0f, nz = 0.0f;
                float depth = 0.0f;
            };
            std::vector<FrictionCache> frictionCache;

            // info
            std::vector<RE::NiPointer<RE::NiAVObject>> node;
            std::vector<std::string> particleName;                  // if not particle, empty
            std::vector<std::uint8_t> isParticle;
            std::vector<std::uint32_t> particleDepth;
            std::vector<std::uint32_t> parentBoneIdx;
            std::vector<std::uint32_t> objIdx;                      // ObjectData
            std::vector<std::uint32_t> rootIdx;                     // Root
            std::vector<std::uint32_t> depth;

            std::vector<float> worldScale;
            std::vector<RE::NiMatrix3> worldRot;
            std::vector<RE::NiPoint3> orgLocalPos;                  // backup
            std::vector<RE::NiMatrix3> orgLocalRot;                 // backup
        };
        PhysicsBones physicsBones;
        std::vector<std::uint32_t> physicsBonesRoots;               // root for physics only
        std::vector<std::uint32_t> physicsBonesOrder;
        std::vector<std::uint32_t> physicsBonesGroup;               // physicsBonesOrder index by objIdx

        struct Constraints {
            std::uint32_t numConstraints = 0;
            std::vector<std::uint32_t> boneIdx;         // PhysicsBone
            std::vector<std::uint32_t> objIdx;
            std::vector<std::uint32_t> rootIdx;
            std::vector<std::uint32_t> colorGraph;

            std::vector<std::uint8_t> numAnchors;       // 0-3
            std::vector<std::uint32_t> anchIdx;         // padding by ANCHOR_MAX
            std::vector<float> restLen;                 // padding by ANCHOR_MAX
            std::vector<float> compSquish;              // padding by ANCHOR_MAX
            std::vector<float> compStretch;             // padding by ANCHOR_MAX
            std::vector<float> squishLimit;             // padding by ANCHOR_MAX
            std::vector<float> stretchLimit;            // padding by ANCHOR_MAX
            std::vector<float> angularLimit;            // padding by ANCHOR_MAX
            std::vector<Vector> restDirLocal;           // padding by ANCHOR_MAX
            std::vector<float> squishDamping;                 // padding by ANCHOR_MAX
            std::vector<float> stretchDamping;                 // padding by ANCHOR_MAX
            std::vector<float> lambda;                  // padding by ANCHOR_MAX
        };
        Constraints constraints;
        std::vector<std::uint32_t> constraintsOrder;
        std::vector<std::uint32_t> constraintsGroup;    // constraintsOrder index by objIdx
        std::vector<std::uint32_t> constraintsColorGroup;    // constraintsOrder index by objIdx

        struct AngularConstraints {
            std::uint32_t numConstraints = 0;
            std::vector<std::uint32_t> boneIdx;         // PhysicsBone
            std::vector<std::uint32_t> objIdx;
            std::vector<std::uint32_t> rootIdx;
            std::vector<std::uint32_t> colorGraph;

            std::vector<std::uint8_t> numAnchors;       // 0-3
            std::vector<std::uint32_t> anchIdx;         // padding by ANCHOR_MAX
            std::vector<Quaternion> restRot;      // padding by ANCHOR_MAX
            std::vector<float> comp;                    // padding by ANCHOR_MAX
            std::vector<float> limit;                    // padding by ANCHOR_MAX
            std::vector<float> damping;                    // padding by ANCHOR_MAX
            std::vector<float> lambda;                  // padding by ANCHOR_MAX
        };
        AngularConstraints angularConstraints;
        std::vector<std::uint32_t> angularConstraintsOrder;
        std::vector<std::uint32_t> angularConstraintsGroup;
        std::vector<std::uint32_t> angularConstraintsColorGroup;

        struct ContactManifold {
            DirectX::XMVECTOR normal = EmptyVector;
            struct ContactPoint {
                DirectX::XMVECTOR localPointA = EmptyVector;
                DirectX::XMVECTOR localPointB = EmptyVector;
                float depth = 0.0f;
            };
            ContactPoint points[4];
            std::uint8_t pointCount = 0;
        };
        struct Colliders {
            std::uint32_t numColliders = 0;
            std::vector<std::uint32_t> boneIdx;             // PhysicsBone
            std::vector<std::uint32_t> objIdx;
            std::vector<std::uint32_t> rootIdx;

            std::vector<std::uint8_t> noCollideCount;    // Same as numColliders
            std::vector<std::uint32_t> noCollideBoneIdx; // padding by NOCOLLIDE_MAX

            // convexHull
            std::vector<std::uint8_t> vertexCount;
            std::vector<std::uint8_t> edgeCount;
            std::vector<std::uint8_t> faceCount;
            struct alignas(32) ConvexHullDataBatch {
                float vX[COL_VERTEX_MAX], vY[COL_VERTEX_MAX], vZ[COL_VERTEX_MAX];
                float eX[COL_EDGE_MAX], eY[COL_EDGE_MAX], eZ[COL_EDGE_MAX];
                float fX[COL_FACE_MAX], fY[COL_FACE_MAX], fZ[COL_FACE_MAX];
            };
            std::vector<ConvexHullDataBatch> convexHullData;
            std::vector<AABB> boundingAABB;
            struct ConvexHullCache {
                float ax = 0.0f, ay = 0.0f, az = 0.0f;
                std::uint64_t lastFrame = 0;
                ContactManifold persistentManifold;
            };
            using CacheKey = std::uint64_t;
            tbb::concurrent_unordered_map<CacheKey, ConvexHullCache> convexHullCache;
            std::size_t collideMaxObserved = 0;

            struct alignas(64) CollisionCache {
                float nx = 0.0f, ny = 0.0f, nz = 0.0f;
                float dtx = 0.0f, dty = 0.0f, dtz = 0.0f;
                float maxDisp = 0.0f;
                std::uint32_t count = 0;
            };
            std::vector<CollisionCache> collisionCache;

            std::vector<float> boundingSphere;
        };
        Colliders colliders;
        std::vector<std::uint32_t> collidersOrder;
        std::vector<std::uint32_t> collidersGroup;

        inline Colliders::CacheKey GetCacheKey(std::uint32_t a, std::uint32_t b) const {
            return (static_cast<std::uint64_t>(std::min(a, b)) << 32) | std::max(a, b);
        }

        struct LocalSpatialHash {
            std::uint32_t tableSize = 1009;
            float invGridSize = 0.1f;

            std::vector<std::uint32_t> cellStart;
            std::vector<std::uint32_t> cellCount;
            std::vector<std::uint32_t> entries;

            inline void Init(const std::uint32_t totalColliders, const float newGridSize) {
                invGridSize = 1.0f / newGridSize;
                cellStart.resize(tableSize + 1, 0);
                cellCount.resize(tableSize, 0);
                entries.resize(totalColliders * 2);
                std::fill(cellCount.begin(), cellCount.end(), 0);
            }
            inline std::uint32_t HashWorldCoordsHigh(const DirectX::XMVECTOR& p) const {
                using namespace DirectX;
                XMVECTOR v = XMVectorScale(p, invGridSize);
                v = XMVectorFloor(XMVectorAdd(v, XMVectorReplicate(0.25f)));
                XMINT3 i3;
                XMStoreSInt3(&i3, v);
                return XXH32(&i3, sizeof(i3), 0) % tableSize;
            }
            inline std::uint32_t HashWorldCoordsLow(const DirectX::XMVECTOR& p) const {
                using namespace DirectX;
                XMVECTOR v = XMVectorScale(p, invGridSize);
                v = XMVectorFloor(XMVectorSubtract(v, XMVectorReplicate(0.25f)));
                XMINT3 i3;
                XMStoreSInt3(&i3, v);
                return XXH32(&i3, sizeof(i3), 0) % tableSize;
            }
        };
        std::vector<LocalSpatialHash> objectHashes;

        DynamicAABBTree globalAABBTree;
        std::vector<std::uint32_t> objIdxToTreeNodeIdx;

        void UpdateObjectAcceleration(const float deltaTime);
        void PrefetchBoneDatas();
        void UpdateGlobalAABBTree();
        void PredictBones(const float deltaTime);
        void CreateLocalSpatialHash();
        void SolveConstraints(const float deltaTime);
        void SolveCollisions(const float deltaTime);
        void UpdateBoneVelocity(const float deltaTime);
        void ApplyToSkyrim(const float deltaTime);

        AABB GetObjectAABB(const std::uint32_t objIdx) const;
        std::vector<AABBPair> GetAABBPairs(const std::uint32_t objIdx);
        bool CheckCollisionConvexHullVSConvexHull(const std::uint32_t coiA, const std::uint32_t coiB, ContactManifold& outManifold);

        std::uint32_t AllocateBone();
        void ReserveBone(std::uint32_t n);
        std::uint32_t AllocateConstraint();
        void ReserveConstraint(std::uint32_t n);
        std::uint32_t AllocateAngularConstraint();
        void ReserveAngularConstraint(std::uint32_t n);
        std::uint32_t AllocateCollider();
        void ReserveCollider(std::uint32_t n);

        void BuildConstraintColorGraph();

        void Reset(const RE::FormID objectID);
        void RemoveCollider(RE::TESObjectREFR* object, const ObjectDatas::Root& targetRoot);
        void CleanPhysics(const std::unordered_set<CleanObject, CleanObjectHash>& cleanList);
        void ReorderMaps();

        void ForceUpdateTransforms(RE::NiAVObject* node, bool isDirty) const;
    };
} // namespace MXPBD
