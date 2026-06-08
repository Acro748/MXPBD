#pragma once

namespace MXPBD {
    class XPBDWorld {
    public:
        XPBDWorld();

        using RootType = Internal::RootType;

        void SetThreads(std::int32_t subtractThreadCount);

        void AddPhysics(RE::TESObjectREFR* object, RE::NiNode* rootNode, const RootType rootType, const PhysicsInput& input);
        void AddCollider(RE::TESObjectREFR* object, const PhysicsInput& input);
        void AddDriver(RE::TESObjectREFR* object, const RootType rootType, const DriverInput& input);
        void UpdatePhysicsSetting(RE::TESObjectREFR* object, const PhysicsInput& input, bool reset = false);
        void ResetAll();
        void Reset(RE::TESObjectREFR* object);
        void RemovePhysics(const RE::FormID objectID);
        void RemovePhysics(const RE::FormID objectID, const RootType rootType, const std::uint32_t bipedSlot);
        void RemoveDriver(const RE::FormID objectID);
        void RemoveDriver(const RE::FormID objectID, const RootType rootType, const std::uint32_t bipedSlot);
        void TogglePhysics(const RE::FormID objectID, bool disable);
        void RunPhysicsWorld(const float deltaTime);
        void RunPhysicsWorldAsync(const float deltaTime);
        void WaitForPhysicsWorldAsync();

        inline void SetDebugMode(const bool debugMode) {
            WaitForPhysicsWorldAsync();
            DEBUG_MODE = debugMode;
        }

        inline void SetGridSize(const float smallGridSize, const float largeGridSize) {
            WaitForPhysicsWorldAsync();
            SMALL_GRID_SIZE = smallGridSize;
            LARGE_GRID_SIZE = largeGridSize;
        }

        inline void SetRotationClampSpeed(const float rotationClampSpeed) {
            WaitForPhysicsWorldAsync();
            ROTATION_CLAMP = ROTATION_CLAMP_DEFAULT * rotationClampSpeed;
        }

        inline void SetCollisionConvergence(const float collisionConvergence) {
            WaitForPhysicsWorldAsync();
            COL_CONVERGENCE = collisionConvergence;
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
            WIND_DETECT_RANGE = DirectX::XMVectorNegate(DirectX::XMVectorReplicate(windDetectRange * SkyrimWorldUnitInverse));
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

        XPBDDriver* GetDriver() const { return driver.get(); };
    private:
        mutable std::mutex lock;

        bool DEBUG_MODE = false;

        float SMALL_GRID_SIZE = 30.0f;
        float LARGE_GRID_SIZE = 100.0f;
        float ROTATION_CLAMP = 0.2f;
        float COL_CONVERGENCE = 0.6f;

        float GROUND_DETECT_RANGE = 0.15f * SkyrimWorldUnitInverse;
        Vector groundRayFrom = DirectX::XMVectorSet(0.0f, 0.0f, GROUND_DETECT_RANGE, 0.0f);
        const Vector groundRayTo = DirectX::XMVectorSet(0.0f, 0.0f, GROUND_DETECT_RANGE, 0.0f);
        std::uint32_t GROUND_DETECT_QUALITY = 1 << 5;

        float WIND_MULTIPLIER = SkyrimWorldScaleInverse * 100.0f;
        Vector WIND_DETECT_RANGE = DirectX::XMVectorNegate(DirectX::XMVectorReplicate(20.0f * SkyrimWorldScaleInverse));
        std::uint32_t WIND_DETECT_QUALITY = 1 << 5;

        float CULLING_DISTANCE_SQ = (100.0f * SkyrimWorldScaleInverse) * (100.0f * SkyrimWorldScaleInverse);
        float INV_CULLING_DISTANCE_SQ = reciprocal(CULLING_DISTANCE_SQ);
        bool COL_QUALITY_BY_LOD = true;
        std::uint32_t COL_HASH_TABLE_SIZE = 1 << 10;

        bool isTaskLoading = false;
        bool orderDirty = false;
        bool colorGraphDirty = false;

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

        Internal::ObjectDatas objectDatas;

        Internal::PhysicsBones physicsBones;
        Internal::BoneNameCache boneNameToIdx;
        std::vector<std::uint32_t> physicsBonesOrder;
        std::vector<std::uint32_t> physicsBonesRoots; // root for physics only
        std::vector<std::uint32_t> physicsBonesGroup; // physicsBonesOrder index by objIdx
        std::unique_ptr<tbb::spin_mutex[]> physicsBonesLock;

        Internal::DistanceConstraints distanceConstraints;
        std::vector<std::uint32_t> distanceConstraintsOrder;
        std::vector<std::uint32_t> distanceConstraintsGroup;
        std::vector<std::uint32_t> distanceConstraintsColorGroup;

        Internal::AngularConstraints angularConstraints;
        std::vector<std::uint32_t> angularConstraintsOrder;
        std::vector<std::uint32_t> angularConstraintsGroup;
        std::vector<std::uint32_t> angularConstraintsColorGroup;

        Internal::ConeConstraints coneConstraints;
        std::vector<std::uint32_t> coneConstraintsOrder;
        std::vector<std::uint32_t> coneConstraintsGroup;
        std::vector<std::uint32_t> coneConstraintsColorGroup;

        Internal::DeformConstraints deformConstraints;
        std::vector<std::uint32_t> deformConstraintsOrder;
        std::vector<std::uint32_t> deformConstraintsGroup;

        Internal::ShapeMatchingConstraints shapeMatchingConstraints;
        std::vector<std::uint32_t> shapeMatchingConstraintsOrder;
        std::vector<std::uint32_t> shapeMatchingConstraintsGroup;

        Internal::Colliders colliders;
        std::vector<std::uint32_t> collidersOrder;
        std::vector<std::uint32_t> collidersLeafs; // no child bones only
        std::vector<std::uint32_t> collidersRoots;
        std::vector<std::uint32_t> collidersGroup;

        tbb::concurrent_unordered_map<Internal::CacheKey, Internal::ConvexHullCache> convexHullCache;
        std::uint32_t collideMaxObserved = 0;
        std::uint32_t expectedCollisionCount = 0;

        std::vector<Internal::ManifoldCache> manifoldCache;
        std::uint32_t manifoldCacheCount = 0;

        std::vector<Internal::LocalSpatialHash> objectHashesSmall;
        std::vector<Internal::LocalSpatialHash> objectHashesLarge;

        DynamicAABBTree globalAABBTree;
        std::vector<std::uint32_t> objIdxToTreeNodeIdx;

        friend class XPBDDriver;
        std::unique_ptr<XPBDDriver> driver;
        inline Internal::PhysicsWorldContext GetContext() {
            Internal::PhysicsWorldContext ctx;
            ctx.threadPool = threadPool.get();
            ctx.objectDatas = &objectDatas;
            ctx.physicsBones = &physicsBones;
            ctx.physicsBonesLock = physicsBonesLock.get();
            ctx.boneNameToIdx = &boneNameToIdx;
            return ctx;
        };

        void UpdateObjectData(const float deltaTime);
        void ClampObjectRotation(const float deltaTime, const float stepCount);
        void PrefetchBoneDatas();
        void InterpolateBoneDatas(const float alpha, const float nextAlpha);
        void UpdateGlobalAABBTree();
        void ObjectCulling();
        void UpdateWindStrength();
        void PredictBones(const float deltaTime);
        void RunDriver(const float deltaTime);
        void CreateLocalSpatialHash();
        void GenerateCollisionManifolds();
        void GenerateGroundCache(const float stepCount);
        void SolveCachedCollisions(const float deltaTime);
        void SolveCachedGroundCollisions(const float deltaTime);
        void SolveDistanceConstraints(const float deltaTime, const float invDeltaTimeSq, const bool initLambda);
        void SolveAngularConstraints(const float deltaTime, const float invDeltaTimeSq, const bool initLambda);
        void SolveConeConstraints(const float deltaTime, const float invDeltaTimeSq, const bool initLambda);
        void SolveConstraints(const float deltaTime, const bool initLambda);
        void UpdateProceduralRotations();
        void SolveAnimDrive(const float deltaTime, const bool initLambda);
        void SolveShapeMatchingConstraint(const float deltaTime);
        void SolveDeformConstraint(const float deltaTime);
        void UpdateBoneVelocity(const float deltaTime);
        void ApplyToSkyrim(const bool syncFrame);

        AABB GetObjectAABB(const std::uint32_t objIdx) const;
        std::vector<AABBPair> GetAABBPairs(const std::uint32_t objIdx);

        bool ConvexHullvsConvexHull(const std::uint32_t coiA, const std::uint32_t coiB, Internal::ContactManifold& outManifold);
        bool ConvexHullvsSphere(const std::uint32_t coiHull, const std::uint32_t coiSphere, Internal::ContactManifold& outManifold);
        bool SpherevsSphere(const std::uint32_t coiA, const std::uint32_t coiB, Internal::ContactManifold& outManifold);

        std::uint32_t FindObject(RE::TESObjectREFR* object) const;
        std::uint32_t AllocateObject(RE::TESObjectREFR* object);
        std::uint32_t FindRoot(const std::uint32_t objIdx, const Internal::ObjectDatas::Root& rootData) const;
        std::uint32_t AllocateRoot(const std::uint32_t objIdx, const Internal::ObjectDatas::Root& rootData);

        std::uint32_t AllocateBone();
        void ReserveBone(std::uint32_t n);
        std::uint32_t AllocateDistanceConstraint();
        void ReserveDistanceConstraint(std::uint32_t n);
        std::uint32_t AllocateAngularConstraint();
        void ReserveAngularConstraint(std::uint32_t n);
        std::uint32_t AllocateConeConstraint();
        void ReserveConeConstraint(std::uint32_t n);
        std::uint32_t AllocateDeformConstraint();
        void ReserveDeformConstraint(std::uint32_t n);
        std::uint32_t AllocateShapeMatchingConstraint();
        std::uint32_t AllocateShapeMatchingConstraintCluster(std::uint32_t ci);
        void ReserveShapeMatchingConstraint(std::uint32_t n);
        void ReserveShapeMatchingConstraintCluster(std::uint32_t n);
        std::uint32_t AllocateCollider();
        void ReserveCollider(std::uint32_t n);

        void BuildConstraintColorGraph();

        void CheckUpdate();

        void ResetBone(const std::uint32_t bi);
        void ResetParticleBone(const std::uint32_t bi);
        void Reset(const std::uint32_t oi);
        void RemoveCollider(RE::TESObjectREFR* object, const Internal::ObjectDatas::Root& targetRoot);
        void RemovePhysics(const Internal::RemoveDataList& removeList);
        void ReorderMaps();

        void UpdateChildTreeData(RE::NiNode* node, RE::NiUpdateData ::Flag flag) const;
        void UpdateChildTreeWorldTransforms(RE::NiNode* node) const;

        inline bool IsDisable(const std::uint32_t objIdx) const {
            return objectDatas.isDisable.size() <= objIdx || objectDatas.isDisable[objIdx];
        };
        inline bool IsSkippedFrame(const std::uint32_t boneIdx) const {
            return physicsBones.lastFrame.size() <= boneIdx || (physicsBones.lastFrame[boneIdx] != (currentFrame - 1u) && physicsBones.lastFrame[boneIdx] != currentFrame);
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
        void SetDistanceConstraint(const std::uint32_t ci, const PhysicsInput::DistanceConstraint& data, const float physicsScale);
        void SetAngularConstraint(const std::uint32_t ci, const PhysicsInput::AngularConstraint& data, const float physicsScale);
        void SetConeConstraint(const std::uint32_t ci, const PhysicsInput::ConeConstraint& data, const float physicsScale);
        void SetDeformConstraint(const std::uint32_t ci, const PhysicsInput::DeformConstraint& data, const float physicsScale);
        void SetShapeMatchingConstraint(const std::uint32_t ci, const PhysicsInput::ShapeMatchingConstraint& data, const float physicsScale);

        void LoggingTimeProfiler(const std::string& funcName, const double ms) const;
    };
} // namespace MXPBD
