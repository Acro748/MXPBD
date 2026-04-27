#pragma once

namespace MXPBD {
    class XPBDWorld {
    public:
        XPBDWorld();

        enum RootType {
            skeleton,
            facegen,
            cloth,
            weapon,
            collider,
            none,
            total
        };

        void AddPhysics(RE::TESObjectREFR* object, RE::NiNode* rootNode, const RootType rootType, const PhysicsInput& input);
        void UpdatePhysicsSetting(RE::TESObjectREFR* object, const PhysicsInput& input, bool reset = true);
        void ResetAll();
        void Reset(RE::TESObjectREFR* object);
        void RemovePhysics(const RE::FormID objectID);
        void RemovePhysics(RE::TESObjectREFR* object, const RootType rootType, const std::uint32_t bipedSlot);
        void TogglePhysics(const RE::FormID objectID, bool isDisable);
        void RunPhysicsWorld(const float deltaTime);

        inline void SetIteration(const std::uint32_t iteration) {
            ITERATION_MAX = iteration;
        }
        inline void SetGridSize(const float smallGridSize, const float largeGridSize) {
            SMALL_GRID_SIZE = smallGridSize;
            LARGE_GRID_SIZE = largeGridSize;
        }
        inline void SetRotationClampSpeed(const float rotationClampSpeed) {
            ROTATION_CLAMP = ROTATION_CLAMP_DEFAULT * rotationClampSpeed;
        }
        inline void SetCollisionConvergence(const float collisionConvergence) {
            COL_CONVERGENCE = collisionConvergence;
        }
        inline void SetGroundDetectRange(const float groundDetectRange) {
            GROUND_DETECT_RANGE = groundDetectRange;
            groundRayFrom = DirectX::XMVectorSet(0.0f, 0.0f, GROUND_DETECT_RANGE, 0.0f);
        }
        inline void SetGroundDetectQuality(const std::uint32_t groundDetectQuality) {
            GROUND_DETECT_QUALITY = groundDetectQuality;
        }
        inline void SetCullingDistance(const float cullingDistance) {
            CULLING_DISTANCE_SQ = DirectX::XMVectorReplicate(cullingDistance * cullingDistance);
        }
        inline void SetColliderHashTableSize(const std::uint8_t colliderHashTableSize) {
            COL_HASH_TABLE_SIZE = (1 << colliderHashTableSize);
        }

    private:
        mutable std::mutex lock;

        std::uint32_t ITERATION_MAX = 5;
        float SMALL_GRID_SIZE = 30.0f;
        float LARGE_GRID_SIZE = 100.0f;
        float ROTATION_CLAMP = 0.2f;
        float COL_CONVERGENCE = 0.1f;
        float GROUND_DETECT_RANGE = 25.0f;
        std::uint32_t GROUND_DETECT_QUALITY = 30;
        Vector CULLING_DISTANCE_SQ = DirectX::XMVectorReplicate(std::powf(100.0f * InverseScale_skyrimUnit, 2.0f));
        std::uint32_t COL_HASH_TABLE_SIZE = 1 << 10;

        Vector groundRayFrom = DirectX::XMVectorSet(0.0f, 0.0f, GROUND_DETECT_RANGE, 0.0f);
        const Vector groundRayTo = DirectX::XMVectorSet(0.0f, 0.0f, -5.0f, 0.0f);

        bool orderDirty = false;
        bool isNeedColorGraphUpdate = false;

        std::uint64_t currentFrame = 0;
        std::unique_ptr<TBB_ThreadPool> threadPool;
        float objectAccelerationTime = 0.0f;
        float timeAccumulator = 0.0f;

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
            struct Root {
                RootType type;
                std::uint32_t bipedSlot;
                bool operator==(const Root& other) const {
                    return type == other.type && bipedSlot == other.bipedSlot;
                }
            };
            std::vector<std::vector<Root>> roots;                // skeleton, armor
            std::vector<Vector> prevWorldPos;
            std::vector<Quaternion> prevNPCWorldRot;
            std::vector<Quaternion> targetNPCWorldRot;
            std::vector<RE::NiNode*> npcNode;
            std::vector<RE::bhkWorld*> bhkWorld;
            std::vector<Vector> velocity;
            std::vector<Vector> acceleration;
            std::vector<AABB> boundingAABB;
            std::vector<std::uint8_t> isDisable;                 // just physics disable
            std::vector<std::uint8_t> isDisableByToggle;                 // just physics disable
        };
        ObjectDatas objectDatas;

        struct PhysicsBones {
            std::uint32_t numBones = 0;

            // 3 DOF
            std::vector<Vector> pos;                // world
            std::vector<Vector> prevPos;                // world
            std::vector<Vector> pred; // world
            std::vector<Vector> vel;

            // 6 DOF
            std::vector<std::uint8_t> advancedRotation;
            std::vector<Quaternion> rot;
            std::vector<Quaternion> prevRot;
            std::vector<Quaternion> predRot;
            std::vector<Quaternion> backupRot;
            std::vector<Vector> angVel;
            std::vector<float> invInertia;

            // setting
            std::vector<float> damping;
            std::vector<float> inertiaScale;
            std::vector<float> restitution;
            std::vector<float> rotationRatio; // only for 3 DOF
            std::vector<Vector> gravity;
            std::vector<Vector> offset;
            std::vector<float> invMass;

            std::vector<float> restPoseLimit;
            std::vector<float> restPoseCompliance;
            std::vector<float> restPoseLambda;

            std::vector<float> restPoseAngularLimit;
            std::vector<float> restPoseAngularCompliance;
            std::vector<float> restPoseAngularLambda;

            // fake rotation setting
            std::vector<float> linearRotTorque; // only for 6 DOF

            // collision
            std::vector<float> collisionMargin;
            std::vector<float> collisionShrink;
            std::vector<float> collisionFriction;
            std::vector<float> collisionRotationBias;
            std::vector<float> collisionCompliance;

            struct CollisionCache {
                Vector n = vZero;
                Vector p = vZero;
                float totalDepth = 0.0f;
                float maxDepth = 0.0f;
            };
            std::vector<CollisionCache> collisionCache;

            struct FrictionCache {
                Vector n = vZero;
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
        std::vector<std::uint32_t> physicsBonesOrder;
        std::vector<std::uint32_t> physicsBonesRoots;               // root for physics only
        std::vector<std::uint32_t> physicsBonesGroup;               // physicsBonesOrder index by objIdx
        std::unique_ptr<tbb::spin_mutex[]> physicsBonesLock;

        struct Constraints {
            std::uint32_t numConstraints = 0;
            std::vector<std::uint32_t> boneIdx;         // PhysicsBone
            std::vector<std::uint32_t> objIdx;
            std::vector<std::uint32_t> rootIdx;
            std::vector<std::uint32_t> colorGraph;

            std::vector<std::uint8_t> numAnchors;       // 0-3
            struct alignas(16) AnchorData {
                Vector restDirLocal = vZero;

                std::uint32_t anchIdx = UINT32_MAX;
                float restLen = 0.0f;       
                float complianceSquish = 0.0f;
                float complianceStretch = 0.0f;

                float squishLimit = 0.0f;
                float stretchLimit = 0.0f;
                float angularLimit = 0.0f;
                float squishDamping = 0.0f;

                float stretchDamping = 0.0f;
                float lambda = 0.0f;
            };
            std::vector<AnchorData> anchData;         // padding by ANCHOR_MAX
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
            struct alignas(16) AnchorData {
                Quaternion restRot = vZero;

                std::uint32_t anchIdx = UINT32_MAX;
                float compliance = 0.0f;
                float limit = 0.0f;
                float damping = 0.0f;

                float lambda = 0.0f;
            };
            std::vector<AnchorData> anchData;      // padding by ANCHOR_MAX
        };
        AngularConstraints angularConstraints;
        std::vector<std::uint32_t> angularConstraintsOrder;
        std::vector<std::uint32_t> angularConstraintsGroup;
        std::vector<std::uint32_t> angularConstraintsColorGroup;

        struct ContactManifold {
            DirectX::XMVECTOR normal = vZero;
            struct ContactPoint {
                DirectX::XMVECTOR localPointA = vZero;
                DirectX::XMVECTOR localPointB = vZero;
                float depth = 0.0f;
            };
            ContactPoint points[4];
            std::uint32_t pointCount = 0;
        };
        struct Colliders { // usually one collider per bone is recommended, but can be multiple for convexhull (e.g. furniture)
            std::uint32_t numColliders = 0;
            std::vector<std::uint32_t> boneIdx;             // PhysicsBone
            std::vector<std::uint32_t> objIdx;              // ObjectData
            std::vector<std::uint32_t> rootIdx;             // Root

            std::vector<std::uint8_t> noCollideCount;    // Same as numColliders
            std::vector<std::uint32_t> noCollideBoneIdx; // padding by NOCOLLIDE_MAX

            // convexHull
            std::vector<ConvexHullDataBatch> convexHullData;
            std::vector<AABB> boundingAABB;

            std::vector<float> boundingSphere;
            std::vector<Vector> boundingSphereCenter;
        };
        Colliders colliders;
        std::vector<std::uint32_t> collidersOrder;
        std::vector<std::uint32_t> collidersLeafs; // no child bones only
        std::vector<std::uint32_t> collidersRoots;
        std::vector<std::uint32_t> collidersGroup;

        struct ConvexHullCache {
            Vector axis = vZero;
            std::uint64_t lastFrame = 0;
            ContactManifold persistentManifold;
        };
        using CacheKey = std::uint64_t;
        tbb::concurrent_unordered_map<CacheKey, ConvexHullCache> convexHullCache;
        std::uint32_t collideMaxObserved = 0;
        std::uint32_t expectedCollisionCount = 0;

        struct ManifoldCache {
            std::uint32_t coiA;
            std::uint32_t coiB;
            ContactManifold manifold;
        };
        std::vector<ManifoldCache> manifoldCache;
        std::uint32_t manifoldCacheCount = 0;

        struct GroundCache {
            float height = -FLT_MAX;
            Vector normal = vZero;
            bool hasHit = false;
            bool isCast = false;
        };
        std::vector<GroundCache> groundCache;
        inline CacheKey GetCacheKey(std::uint32_t a, std::uint32_t b) const {
            return (static_cast<std::uint64_t>(std::min(a, b)) << 32) | std::max(a, b);
        }

        struct LocalSpatialHash {
            float invGridSize = 0.1f;
            Vector vInvGridSize = DirectX::XMVectorReplicate(invGridSize);
            std::uint32_t hashTableSize = 1 << 10;

            std::vector<std::uint32_t> cell;
            std::vector<std::uint32_t> cellCount;
            std::vector<std::uint32_t> entries;

            inline void Init(const std::uint32_t totalColliders, const float newGridSize, const std::uint32_t newHashTableSize) {
                invGridSize = 1.0f / newGridSize;
                vInvGridSize = DirectX::XMVectorReplicate(invGridSize);
                hashTableSize = newHashTableSize;
                cell.resize(hashTableSize + 1, 0);
                cellCount.resize(hashTableSize, 0);
                entries.resize(totalColliders * 2);
                std::fill(cellCount.begin(), cellCount.end(), 0);
            }

            inline std::uint32_t HashWorldCoordsHigh(const Vector& p) const {
                const Vector v = DirectX::XMVectorFloor(DirectX::XMVectorMultiplyAdd(p, vInvGridSize, vFloorHigh));
                const __m128i vi = _mm_cvttps_epi32(v);
                const __m128i hashed = _mm_mullo_epi32(vi, hash_primes);
                const std::uint32_t x = _mm_extract_epi32(hashed, 0);
                const std::uint32_t y = _mm_extract_epi32(hashed, 1);
                const std::uint32_t z = _mm_extract_epi32(hashed, 2);
                return (x ^ y ^ z) & (hashTableSize - 1);
            }
            inline std::uint32_t HashWorldCoordsLow(const Vector& p) const {
                const Vector v = DirectX::XMVectorFloor(DirectX::XMVectorMultiplyAdd(p, vInvGridSize, vFloorLow));
                const __m128i vi = _mm_cvttps_epi32(v);
                const __m128i hashed = _mm_mullo_epi32(vi, hash_primes);
                const std::uint32_t x = _mm_extract_epi32(hashed, 0);
                const std::uint32_t y = _mm_extract_epi32(hashed, 1);
                const std::uint32_t z = _mm_extract_epi32(hashed, 2);
                return (x ^ y ^ z) & (hashTableSize - 1);
            }
        };
        std::vector<LocalSpatialHash> objectHashesSmall;
        std::vector<LocalSpatialHash> objectHashesLarge;

        DynamicAABBTree globalAABBTree;
        std::vector<std::uint32_t> objIdxToTreeNodeIdx;

        class GroundHitCollector : public RE::hkpRayHitCollector {
        public:
            void AddRayHit(const RE::hkpCdBody& a_body, const RE::hkpShapeRayCastCollectorOutput& a_hitInfo) override {
                if (rayHit.hitFraction <= a_hitInfo.hitFraction)
                    return;
                const RE::hkpCdBody* hkpCdBody = &a_body;
                while (hkpCdBody->parent != nullptr) {
                    hkpCdBody = hkpCdBody->parent;
                }
                const RE::hkpCollidable* collidable = static_cast<const RE::hkpCollidable*>(hkpCdBody);
                const RE::COL_LAYER layer = collidable->GetCollisionLayer();

                if (layer != RE::COL_LAYER::kStatic &&
                    layer != RE::COL_LAYER::kGround &&
                    layer != RE::COL_LAYER::kTerrain &&
                    layer != RE::COL_LAYER::kStairHelper) {
                    return;
                }
                rayHit.rootCollidable = collidable;
                rayHit.hitFraction = a_hitInfo.hitFraction;
                rayHit.normal = a_hitInfo.normal;
                earlyOutHitFraction = a_hitInfo.hitFraction;
            }
            RE::hkpWorldRayCastOutput rayHit;
        };

        void UpdateObjectData(const float deltaTime);
        void ClampObjectRotation();
        void PrefetchBoneDatas();
        void UpdateGlobalAABBTree();
        void ObjectCulling();
        void PredictBones(const float deltaTime);
        void CreateLocalSpatialHash();
        void GenerateCollisionManifolds();
        void GenerateGroundCache();
        void SolveCachedCollisions(const float deltaTime);
        void SolveCachedGroundCollisions(const float deltaTime);
        void SolveConstraints(const float deltaTime, const bool initLambda);
        void SolveRestPoseConstraints(const float deltaTime, const bool initLambda);
        void UpdateBoneVelocity(const float deltaTime);
        void ApplyToSkyrim();

        AABB GetObjectAABB(const std::uint32_t objIdx) const;
        std::vector<AABBPair> GetAABBPairs(const std::uint32_t objIdx);

        bool ConvexHullvsConvexHull(const std::uint32_t coiA, const std::uint32_t coiB, ContactManifold& outManifold);

        std::uint32_t AllocateBone();
        void ReserveBone(std::uint32_t n);
        std::uint32_t AllocateConstraint();
        void ReserveConstraint(std::uint32_t n);
        std::uint32_t AllocateAngularConstraint();
        void ReserveAngularConstraint(std::uint32_t n);
        std::uint32_t AllocateCollider();
        void ReserveCollider(std::uint32_t n);

        void BuildConstraintColorGraph();

        void ResetBone(const std::uint32_t bi);
        void Reset(const RE::FormID objectID);
        void RemoveCollider(RE::TESObjectREFR* object, const ObjectDatas::Root& targetRoot);
        void CleanPhysics(const std::unordered_set<CleanObject, CleanObjectHash>& cleanList);
        void ReorderMaps();

        void UpdateChildTreeData(RE::NiNode* node) const;
        void UpdateChildTreeWorldTransforms(RE::NiNode* node) const;
    };
} // namespace MXPBD
