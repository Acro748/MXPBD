#pragma once

namespace MXPBD {
    class XPBDWorld {
    public:
        XPBDWorld();

        enum RootType {
            kNone,
            kSkeleton,
            kFacegen,
            kCloth,
            kWeapon,
            kStatic,
            kCollider,
            total
        };

        void AddPhysics(RE::TESObjectREFR* object, RE::NiNode* rootNode, const RootType rootType, const PhysicsInput& input);
        void UpdatePhysicsSetting(RE::TESObjectREFR* object, const PhysicsInput& input, bool reset = true);
        void ResetAll();
        void Reset(RE::TESObjectREFR* object);
        void RemovePhysics(const RE::FormID objectID);
        void RemovePhysics(const RE::FormID objectID, const RootType rootType, const std::uint32_t bipedSlot);
        void TogglePhysics(const RE::FormID objectID, bool disable);
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
            GROUND_DETECT_RANGE = groundDetectRange * InverseScale_skyrimUnit;
            groundRayFrom = DirectX::XMVectorSet(0.0f, 0.0f, GROUND_DETECT_RANGE, 0.0f);
        }
        inline void SetGroundDetectQuality(const std::uint8_t groundDetectQuality) {
            GROUND_DETECT_QUALITY = (1u << groundDetectQuality);
        }

        inline void SetWindDetectRange(const float windDetectRange) {
            WIND_DETECT_RANGE = windDetectRange * InverseScale_skyrimUnit;
        }
        inline void SetWindDetectQuality(const std::uint8_t windDetectRangeQuality) {
            WIND_DETECT_QUALITY = (1u << windDetectRangeQuality);
        }

        inline void SetColliderHashTableSize(const std::uint8_t colliderHashTableSize) {
            COL_HASH_TABLE_SIZE = (1u << colliderHashTableSize);
        }
        inline void SetCullingDistance(const float cullingDistance) {
            CULLING_DISTANCE_SQ = (cullingDistance * InverseScale_skyrimUnit) * (cullingDistance * InverseScale_skyrimUnit);
            INV_CULLING_DISTANCE_SQ = reciprocal(CULLING_DISTANCE_SQ);
        }
        inline void SetCollisionQualityByDistance(const bool collisionQualityByDistanceEnable) {
            COL_QUALITY_BY_LOD = collisionQualityByDistanceEnable;
        }

        inline void SetWind(const float newWindSpeed, const float newWindAngle) {
            windSpeed = newWindSpeed * InverseScale_skyrimUnit * 1000.0f;
            windAngle = newWindAngle;
        }

    private:
        mutable std::mutex lock;

        std::uint32_t ITERATION_MAX = 5;
        float SMALL_GRID_SIZE = 30.0f;
        float LARGE_GRID_SIZE = 100.0f;
        float ROTATION_CLAMP = 0.2f;
        float COL_CONVERGENCE = 0.25f;

        float GROUND_DETECT_RANGE = 0.15f * InverseScale_skyrimUnit;
        std::uint32_t GROUND_DETECT_QUALITY = 1 << 5;
        Vector groundRayFrom = DirectX::XMVectorSet(0.0f, 0.0f, GROUND_DETECT_RANGE, 0.0f);
        const Vector groundRayTo = DirectX::XMVectorSet(0.0f, 0.0f, -5.0f, 0.0f);

        float WIND_DETECT_RANGE = 20.0f * InverseScale_skyrimUnit;
        std::uint32_t WIND_DETECT_QUALITY = 1 << 5;

        float CULLING_DISTANCE_SQ = (100.0f * InverseScale_skyrimUnit) * (100.0f * InverseScale_skyrimUnit);
        float INV_CULLING_DISTANCE_SQ = reciprocal(CULLING_DISTANCE_SQ);
        bool COL_QUALITY_BY_LOD = true;
        std::uint32_t COL_HASH_TABLE_SIZE = 1 << 10;

        bool orderDirty = false;
        bool isNeedColorGraphUpdate = false;

        std::uint64_t currentFrame = 0;
        std::unique_ptr<TBB_ThreadPool> threadPool;
        float objectAccelerationTime = 0.0f;
        float timeAccumulator = 0.0f;

        float windSpeed = 0.0f;
        float windAngle = 0.0f;

        std::uint64_t totalColCandidates = 0;
        std::uint64_t colCandidatesStackCount = 0;

        struct RemoveData {
            std::uint32_t objIdx = UINT32_MAX;
            std::uint32_t rootIdx = UINT32_MAX;

            bool operator==(const RemoveData& co) const {
                return objIdx == co.objIdx && rootIdx == co.rootIdx;
            }
        };
        struct RemoveDataHash {
            std::size_t operator()(const RemoveData& co) const {
                return static_cast<std::uint64_t>(co.objIdx) << 32 | co.rootIdx;
            }
        };
        using RemoveDataList = std::unordered_set<RemoveData, RemoveDataHash>;

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
            std::vector<std::uint8_t> isStatic;
            std::vector<float> windMultiplier;
            std::vector<std::uint8_t> maxManifoldPoints;         // collision quality by LOD / 1 - 4
            std::vector<std::uint8_t> isDisable;                 // physics disable by culling or trigger
            std::vector<std::uint8_t> isDisableByToggle;         // physics disable by trigger
            std::vector<std::uint64_t> randState;
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
            std::vector<float> rotationBlendFactor; // only for 3 DOF
            std::vector<Vector> gravity;
            std::vector<Vector> offset;
            std::vector<float> invMass;
            std::vector<float> windFactor;

            // restPose
            std::vector<float> restPoseLimit;
            std::vector<float> restPoseCompliance;
            std::vector<float> restPoseLambda;
            std::vector<float> restPoseAngularLimit;
            std::vector<float> restPoseAngularCompliance;
            std::vector<float> restPoseAngularLambda;

            // fake rotation / only for 6 DOF
            std::vector<DirectX::XMMATRIX> linearRotTorque;

            // collision
            std::vector<float> collisionMargin;
            std::vector<float> collisionShrink;
            std::vector<float> collisionFriction;
            std::vector<float> collisionRotationBias;
            std::vector<float> collisionCompliance;
            std::vector<std::uint32_t> layerGroup;
            std::vector<std::uint32_t> collideLayer;

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

            std::vector<Vector> prevNodeWorldPos;
            std::vector<Vector> targetNodeWorldPos;
            std::vector<Quaternion> prevNodeWorldRot;
            std::vector<Quaternion> targetNodeWorldRot;

            std::vector<float> orgWorldScale;
            std::vector<Vector> orgLocalPos;     // backup
            std::vector<Quaternion> orgLocalRot;   // backup
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

        void UpdateObjectData(const float deltaTime);
        void ClampObjectRotation();
        void PrefetchBoneDatas(const float alpha, const bool isFirstStep);
        void UpdateGlobalAABBTree();
        void ObjectCulling();
        void UpdateWindStrength();
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

        bool ConvexHullvsConvexHull(const std::uint32_t ciA, const std::uint32_t ciB, ContactManifold& outManifold);

        std::uint32_t AllocateObject(RE::TESObjectREFR* object);
        std::uint32_t AllocateRoot(const std::uint32_t objIdx, const ObjectDatas::Root& rootData);

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
        void RemovePhysics(const RemoveDataList& cleanList);
        void ReorderMaps();

        void UpdateChildTreeData(RE::NiNode* node) const;
        void UpdateChildTreeWorldTransforms(RE::NiNode* node) const;

        inline bool IsDisable(const std::uint32_t objIdx) const {
            return objectDatas.roots.size() < objIdx || objectDatas.isDisable[objIdx];
        };

        // physicsBone idx
        inline bool IsCollide(const std::uint32_t targetLayerGroupBoneIdx, const std::uint32_t ownCollideLayerBoneIdx) const {
            return (physicsBones.layerGroup[targetLayerGroupBoneIdx] & physicsBones.collideLayer[ownCollideLayerBoneIdx]) != 0;
        };
        inline bool IsCollideGround(const std::uint32_t ownCollideLayerBoneIdx) const {
            return (CollisionLayer::kGround & physicsBones.collideLayer[ownCollideLayerBoneIdx]) != 0;
        };
    };
} // namespace MXPBD
