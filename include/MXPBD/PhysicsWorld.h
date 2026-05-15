#pragma once

namespace MXPBD {
    class XPBDWorld {
    public:
        void SetThreads(std::int32_t subtractThreadCount);

        enum RootType : std::uint8_t {
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
        void UpdatePhysicsSetting(RE::TESObjectREFR* object, const PhysicsInput& input, bool reset = false);
        void ResetAll();
        void Reset(RE::TESObjectREFR* object);
        void RemovePhysics(const RE::FormID objectID);
        void RemovePhysics(const RE::FormID objectID, const RootType rootType, const std::uint32_t bipedSlot);
        void TogglePhysics(const RE::FormID objectID, bool disable);
        void RunPhysicsWorld(const float deltaTime);
        void RunPhysicsWorldAsync(const float deltaTime);
        void WaitForPhysicsWorldAsync();

        inline void SetGridSize(const float smallGridSize, const float largeGridSize) {
            WaitForPhysicsWorldAsync();
            SMALL_GRID_SIZE = smallGridSize;
            LARGE_GRID_SIZE = largeGridSize;
        }

        inline void SetRotationClampSpeed(const float rotationClampSpeed) {
            WaitForPhysicsWorldAsync();
            ROTATION_CLAMP = ROTATION_CLAMP_DEFAULT * rotationClampSpeed;
        }

        inline void SetGroundDetectRange(const float groundDetectRange) {
            WaitForPhysicsWorldAsync();
            GROUND_DETECT_RANGE = groundDetectRange * SkyrimWorldUnitInverse;
            groundRayFrom = DirectX::XMVectorSet(0.0f, 0.0f, GROUND_DETECT_RANGE, 0.0f);
        }
        inline void SetGroundDetectQuality(const std::uint8_t groundDetectQuality) {
            WaitForPhysicsWorldAsync();
            GROUND_DETECT_QUALITY = (1u << groundDetectQuality);
        }

        inline void SetWindMultiplier(const float newWindMultiplier) {
            WaitForPhysicsWorldAsync();
            WIND_MULTIPLIER = SkyrimWorldScaleInverse * newWindMultiplier;
        }
        inline void SetWindDetectRange(const float windDetectRange) {
            WaitForPhysicsWorldAsync();
            WIND_DETECT_RANGE = windDetectRange * SkyrimWorldUnitInverse;
        }
        inline void SetWindDetectQuality(const std::uint8_t windDetectRangeQuality) {
            WaitForPhysicsWorldAsync();
            WIND_DETECT_QUALITY = (1u << windDetectRangeQuality);
        }
        inline void SetWind(const float newWindSpeed, const float newWindAngle) {
            WaitForPhysicsWorldAsync();
            windSpeed = newWindSpeed;
            windAngle = newWindAngle;
        }

        inline void SetColliderHashTableSize(const std::uint8_t colliderHashTableSize) {
            COL_HASH_TABLE_SIZE = (1u << colliderHashTableSize);
        }
        inline void SetCullingDistance(const float cullingDistance) {
            CULLING_DISTANCE_SQ = (cullingDistance * SkyrimWorldScaleInverse) * (cullingDistance * SkyrimWorldScaleInverse);
            INV_CULLING_DISTANCE_SQ = reciprocal(CULLING_DISTANCE_SQ);
        }
        inline void SetCollisionQualityByDistance(const bool collisionQualityByDistanceEnable) {
            COL_QUALITY_BY_LOD = collisionQualityByDistanceEnable;
        }
    private:
        mutable std::mutex lock;

        float SMALL_GRID_SIZE = 30.0f;
        float LARGE_GRID_SIZE = 100.0f;
        float ROTATION_CLAMP = 0.2f;

        float GROUND_DETECT_RANGE = 0.15f * SkyrimWorldUnitInverse;
        Vector groundRayFrom = DirectX::XMVectorSet(0.0f, 0.0f, GROUND_DETECT_RANGE, 0.0f);
        const Vector groundRayTo = DirectX::XMVectorSet(0.0f, 0.0f, -0.05f * SkyrimWorldUnitInverse, 0.0f);
        std::uint32_t GROUND_DETECT_QUALITY = 1 << 5;

        float WIND_MULTIPLIER = SkyrimWorldScaleInverse * 100.0f;
        float WIND_DETECT_RANGE = 20.0f * SkyrimWorldScaleInverse;
        std::uint32_t WIND_DETECT_QUALITY = 1 << 5;

        float CULLING_DISTANCE_SQ = (100.0f * SkyrimWorldScaleInverse) * (100.0f * SkyrimWorldScaleInverse);
        float INV_CULLING_DISTANCE_SQ = reciprocal(CULLING_DISTANCE_SQ);
        bool COL_QUALITY_BY_LOD = true;
        std::uint32_t COL_HASH_TABLE_SIZE = 1 << 10;

        bool isTaskLoading = false;
        bool orderDirty = false;
        bool isNeedColorGraphUpdate = false;
        bool isResetBones = false;

        std::uint64_t currentFrame = 0;
        std::unique_ptr<TBB_ThreadPool> threadPool;
        std::unique_ptr<TBB_ThreadPool> threadPoolAsync;
        tbb::task_group backGroundTask;
        float objectAccelerationTime = 0.0f;
        float timeAccumulator = 0.0f;
        std::uint32_t preCalcStepCount = 0;

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
            std::vector<Vector> deltaWorldPos;
            std::vector<Quaternion> deltaWorldRot;
            std::vector<RE::bhkWorld*> bhkWorld;
            std::vector<Vector> velocity;
            std::vector<Vector> acceleration;
            std::vector<AABB> boundingAABB;
            std::vector<std::uint8_t> isStatic;
            std::vector<Vector> windMultiplier;
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
            std::vector<Vector> predPos; // world
            std::vector<Vector> posVel;

            // 6 DOF
            std::vector<std::uint8_t> advancedRotation;
            std::vector<Quaternion> rot;
            std::vector<Quaternion> prevRot;
            std::vector<Quaternion> predRot;
            std::vector<Quaternion> backupRot;
            std::vector<Vector> angVel;

            // setting
            std::vector<Vector> dampingPositive;
            std::vector<Vector> dampingNegative;
            std::vector<Vector> angularDampingPositive;
            std::vector<Vector> angularDampingNegative;
            std::vector<Vector> limitPositive;
            std::vector<Vector> limitNegative;
            std::vector<Vector> angularLimitPositive;
            std::vector<Vector> angularLimitNegative;
            std::vector<Vector> inertiaPositive;
            std::vector<Vector> inertiaNegative;
            std::vector<Vector> invAngularInertiaPositive;
            std::vector<Vector> invAngularInertiaNegative;
            std::vector<Vector> inertiaCorrectionPositive;
            std::vector<Vector> inertiaCorrectionNegative;
            std::vector<float> angularBlendFactor;
            std::vector<Vector> gravity;
            std::vector<Vector> offset;
            std::vector<float> invMass;
            std::vector<Vector> windFactor;
            std::vector<float> physicsScale;

            // deformation
            std::vector<Vector> deformMax;
            std::vector<Vector> deformMin;
            std::vector<Vector> deformVolumePreservation;
            std::vector<Vector> deformSquishSensitivity;
            std::vector<Vector> deformStretchSensitivity;
            std::vector<Vector> deformBulgeSensitivity;
            std::vector<Vector> deformSquishStiffness;
            std::vector<Vector> deformStretchStiffness;
            std::vector<Vector> deformSquishDamping;
            std::vector<Vector> deformStretchDamping;

            std::vector<Matrix> deformScale;
            std::vector<Matrix> deformScaleCache;
            std::vector<Matrix> deformVelocityScale;
            std::vector<std::uint32_t> deformCount;

            // animDrive
            std::vector<Vector> animDriveCompliance;
            std::vector<float> animDriveLambda;
            std::vector<Vector> animDriveAngularCompliance;
            std::vector<float> animDriveAngularLambda;

            // fake rotation / only for 6 DOF
            std::vector<Matrix> linearRotTorque;

            // collision
            std::vector<float> collisionMargin;
            std::vector<float> collisionShrink;
            std::vector<float> collisionFriction;
            std::vector<float> collisionCompliance;
            std::vector<float> collisionRestitution;
            std::vector<std::uint32_t> layerGroup;
            std::vector<std::uint32_t> collideLayer;

            struct CollideCache {
                Vector normal = vZero;
                float depth = -1.0f;
            };
            std::vector<CollideCache> frictionCache;
            std::vector<CollideCache> deformCache;

            struct GroundCache {
                float height = -FLT_MAX;
                Vector normal = vZero;
                bool hasHit = false;
                bool isCast = false;
            };
            std::vector<GroundCache> groundCache;

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

            std::vector<Quaternion> alignRot;
            std::vector<Quaternion> invAlignRot;

            std::vector<float> orgWorldScale;
            std::vector<Vector> orgLocalPos;     // backup
            std::vector<Quaternion> orgLocalRot;   // backup
        };
        PhysicsBones physicsBones;
        std::vector<std::uint32_t> physicsBonesOrder;
        std::vector<std::uint32_t> physicsBonesRoots;               // root for physics only
        std::vector<std::uint32_t> physicsBonesGroup;               // physicsBonesOrder index by objIdx
        std::unique_ptr<tbb::spin_mutex[]> physicsBonesLock;

        struct DistanceConstraints {
            std::uint32_t numConstraints = 0;
            std::vector<std::uint32_t> boneIdx;         // PhysicsBone
            std::vector<std::uint32_t> objIdx;
            std::vector<std::uint32_t> rootIdx;
            std::vector<std::uint32_t> colorGraph;

            std::vector<std::uint8_t> numAnchors;       // 0-3
            struct alignas(16) AnchorData {
                std::uint32_t anchIdx = UINT32_MAX;
                float restLen = 0.0f;
                float complianceSquish = 0.0f;
                float complianceStretch = 0.0f;

                float squishMargin = 0.0f;
                float stretchMargin = 0.0f;
                float squishDamping = 0.0f;
                float stretchDamping = 0.0f;

                float lambda = 0.0f;
            };
            std::vector<AnchorData> anchData;         // padding by ANCHOR_MAX
        };
        DistanceConstraints distanceConstraints;
        std::vector<std::uint32_t> distanceConstraintsOrder;
        std::vector<std::uint32_t> distanceConstraintsGroup;
        std::vector<std::uint32_t> distanceConstraintsColorGroup;

        struct AngularConstraints {
            std::uint32_t numConstraints = 0;
            std::vector<std::uint32_t> boneIdx;         // PhysicsBone
            std::vector<std::uint32_t> objIdx;
            std::vector<std::uint32_t> rootIdx;
            std::vector<std::uint32_t> colorGraph;

            std::vector<std::uint8_t> numAnchors;       // 0-3
            struct alignas(16) AnchorData {
                std::uint32_t anchIdx = UINT32_MAX;

                Quaternion restRot = vZero;

                Vector compliancePositive = vZero;
                Vector complianceNegative = vZero;

                Vector marginPositive = vZero;
                Vector marginNegative = vZero;

                Vector dampingPositive = vZero;
                Vector dampingNegative = vZero;

                float lambda = 0.0f;
            };
            std::vector<AnchorData> anchData;      // padding by ANCHOR_MAX
        };
        AngularConstraints angularConstraints;
        std::vector<std::uint32_t> angularConstraintsOrder;
        std::vector<std::uint32_t> angularConstraintsGroup;
        std::vector<std::uint32_t> angularConstraintsColorGroup;

        struct DeformConstraints {
            std::uint32_t numConstraints = 0;
            std::vector<std::uint32_t> boneIdx; // PhysicsBone
            std::vector<std::uint32_t> objIdx;
            std::vector<std::uint32_t> rootIdx;
            std::vector<std::uint8_t> numAnchors; // 0-3
            struct alignas(16) AnchorData {
                std::uint32_t anchIdx = UINT32_MAX;
                float restLen = 0.0f;
                Quaternion restRot = vZero;

                Vector squishWeight = vOne;
                Vector stretchWeight = vOne;
                Vector bulgeWeight = vOne;
            };
            std::vector<AnchorData> anchData; // padding by ANCHOR_MAX
        };
        DeformConstraints deformConstraints;
        std::vector<std::uint32_t> deformConstraintsOrder;
        std::vector<std::uint32_t> deformConstraintsGroup;

        struct ShapeMatchingConstraints {
            std::uint32_t numConstraints = 0;

            std::vector<std::uint32_t> objIdx; // constraint index
            std::vector<std::uint32_t> rootIdx; // constraint index

            struct ClusterData {
                std::uint32_t offset = 0;
                std::uint32_t size = 0;
                Vector compliancePositive = vZero;
                Vector complianceNegative = vZero;
                Vector inertiaScale = vZero;
            };
            std::vector<ClusterData> cluster; // constraint index

            std::vector<std::uint32_t> boneIdx; // cluster index
            std::vector<Vector> restRelativePos; // cluster index
            std::vector<float> boneMass; // cluster index
        };
        ShapeMatchingConstraints shapeMatchingConstraints;
        std::vector<std::uint32_t> shapeMatchingConstraintsOrder;
        std::vector<std::uint32_t> shapeMatchingConstraintsGroup;

        struct ContactManifold {
            Vector normal = vZero;
            struct ContactPoint {
                Vector localPointA = vZero;
                Vector localPointB = vZero;
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

            std::vector<AABB> boundingAABB;
            std::vector<float> boundingSphere;
            std::vector<Vector> boundingSphereCenter;

            std::vector<std::uint8_t> colliderType;
            std::vector<ConvexHullDataBatch> convexHullData;
            std::vector<SphereData> sphereData;
        };
        Colliders colliders;
        std::vector<std::uint32_t> collidersOrder;
        std::vector<std::uint32_t> collidersLeafs; // no child bones only
        std::vector<std::uint32_t> collidersRoots;
        std::vector<std::uint32_t> collidersGroup;

        using CacheKey = std::uint64_t;
        inline CacheKey GetCacheKey(std::uint32_t a, std::uint32_t b) const {
            return (static_cast<std::uint64_t>(std::min(a, b)) << 32) | std::max(a, b);
        }
        struct ConvexHullCache {
            Vector axis = vZero;
            std::uint64_t lastFrame = 0;
            ContactManifold persistentManifold;
        };
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

        struct LocalSpatialHash {
            float invGridSize = 0.1f;
            Vector vInvGridSize = DirectX::XMVectorReplicate(invGridSize);
            std::uint32_t hashTableSize = 1 << 10;

            std::vector<std::uint32_t> cell;
            std::vector<std::uint32_t> cellCount;
            std::vector<std::uint32_t> entries;

            inline void Init(const std::uint32_t totalColliders, const float newGridSize, const std::uint32_t newHashTableSize) {
                invGridSize = reciprocal(newGridSize);
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
        void ClampObjectRotation(const float stepCount);
        void PrefetchBoneDatas();
        void InterpolateBoneDatas(const float alpha, const float nextAlpha);
        void UpdateGlobalAABBTree();
        void ObjectCulling();
        void UpdateWindStrength();
        void PredictBones(const float deltaTime);
        void CreateLocalSpatialHash();
        void GenerateCollisionManifolds();
        void GenerateGroundCache(const float stepCount);
        void SolveCachedCollisions(const float deltaTime);
        void SolveCachedGroundCollisions(const float deltaTime);
        void SolveDistanceConstraints(const float invDeltaTimeSq, const bool initLambda);
        void SolveAngularConstraints(const float invDeltaTimeSq, const bool initLambda);
        void SolveConstraints(const float deltaTime, const bool initLambda);
        void SolveAnimDrive(const float deltaTime, const bool initLambda);
        void SolveShapeMatchingConstraint(const float deltaTime);
        void SolveDeformConstraint(const float deltaTime);
        void UpdateBoneVelocity(const float deltaTime);
        void ApplyToSkyrim(const bool syncFrame);

        AABB GetObjectAABB(const std::uint32_t objIdx) const;
        std::vector<AABBPair> GetAABBPairs(const std::uint32_t objIdx);

        bool ConvexHullvsConvexHull(const std::uint32_t coiA, const std::uint32_t coiB, ContactManifold& outManifold);
        bool ConvexHullvsSphere(const std::uint32_t coiHull, const std::uint32_t coiSphere, ContactManifold& outManifold);
        bool SpherevsSphere(const std::uint32_t coiA, const std::uint32_t coiB, ContactManifold& outManifold);

        std::uint32_t AllocateObject(RE::TESObjectREFR* object);
        std::uint32_t AllocateRoot(const std::uint32_t objIdx, const ObjectDatas::Root& rootData);

        std::uint32_t AllocateBone();
        void ReserveBone(std::uint32_t n);
        std::uint32_t AllocateDistanceConstraint();
        void ReserveDistanceConstraint(std::uint32_t n);
        std::uint32_t AllocateAngularConstraint();
        void ReserveAngularConstraint(std::uint32_t n);
        std::uint32_t AllocateDeformConstraint();
        void ReserveDeformConstraint(std::uint32_t n);
        std::uint32_t AllocateShapeMatchingConstraint();
        void ReserveShapeMatchingConstraint(std::uint32_t n);
        std::uint32_t AllocateCollider();
        void ReserveCollider(std::uint32_t n);

        void BuildConstraintColorGraph();

        void ResetBone(const std::uint32_t bi);
        void ResetParticleBone(const std::uint32_t bi);
        void Reset(const RE::FormID objectID);
        void RemoveCollider(RE::TESObjectREFR* object, const ObjectDatas::Root& targetRoot);
        void RemovePhysics(const RemoveDataList& cleanList);
        void ReorderMaps();

        void UpdateChildTreeData(RE::NiNode* node, RE::NiUpdateData ::Flag flag) const;
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

        inline RE::NiNode* GetNPCNodeByObjectIndex(const std::uint32_t objIdx) const {
            return GetNPCNode(GetREFR(objectDatas.objectID[objIdx]));
        }

        void SetBone(const std::uint32_t bi, const PhysicsInput::Bone& bone);
        void SetDistanceConstraint(const std::uint32_t ai, const PhysicsInput::DistanceConstraint::AnchorData& anchData, const float physicsScale);
        void SetAngularConstraint(const std::uint32_t ai, const PhysicsInput::AngularConstraint::AnchorData& anchData, const float physicsScale);
        void SetDeformConstraint(const std::uint32_t ai, const PhysicsInput::DeformConstraint::AnchorData& anchData, const float physicsScale);

        class TimeProfiler {
        public:
            TimeProfiler() = delete;
            TimeProfiler(const std::string& a_funcName) : funcName(a_funcName) {};
            void Start() {
                timeStart = std::chrono::high_resolution_clock::now();
            }
            void End(const XPBDWorld* world) {
                const auto timeEnd = std::chrono::high_resolution_clock::now();
                nsSum += std::chrono::duration_cast<std::chrono::nanoseconds>(timeEnd - timeStart).count();
                timeCount++;
                if (timeCount >= 1000) {
                    const auto ms = (nsSum * 0.001f) * ns2ms;
                    logger::debug("{} time: {:.3f}ms ({} bones / {} distanceConstrants / {} angularConstrants / {} deformConstraints / {} colliders)", funcName, ms,
                                  world->physicsBones.numBones, world->distanceConstraints.numConstraints, world->angularConstraints.numConstraints, world->deformConstraints.numConstraints, world->colliders.numColliders);
                    nsSum = 0;
                    timeCount = 0;
                }
            }

        private:
            std::string funcName = "";
            double nsSum = 0.0;
            std::uint32_t timeCount = 0;
            std::chrono::steady_clock::time_point timeStart;
        };
    };
} // namespace MXPBD
