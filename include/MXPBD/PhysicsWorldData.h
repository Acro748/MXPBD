#pragma once

namespace MXPBD {
    namespace Internal {
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

        struct ObjectDatas {
            std::vector<RE::FormID> objectID;
            struct Root {
                RootType type;
                std::uint32_t bipedSlot;
                bool operator==(const Root& other) const {
                    return type == other.type && bipedSlot == other.bipedSlot;
                }
            };
            std::vector<std::vector<Root>> roots; // skeleton, armor
            std::vector<RE::NiPointer<RE::NiNode>> npcNode;
            std::vector<Vector> prevWorldPos;
            std::vector<Quaternion> prevNPCWorldRot;
            std::vector<Quaternion> targetNPCWorldRot;
            std::vector<Vector> deltaWorldPos;
            std::vector<Quaternion> deltaWorldRot;
            std::vector<Vector> omegaWorldRot;
            std::vector<RE::bhkWorld*> bhkWorld;
            std::vector<Vector> velocity;
            std::vector<Vector> acceleration;
            std::vector<AABB> boundingAABB;
            std::vector<std::uint8_t> isStatic;
            std::vector<Vector> windMultiplier;
            std::vector<std::uint8_t> maxManifoldPoints; // collision quality by LOD / 1 - 4
            std::vector<std::uint8_t> isDisable;         // physics disable by culling or trigger
            std::vector<std::uint8_t> isDisableByToggle; // physics disable by trigger
            std::vector<std::uint64_t> randState;
        };

        struct PhysicsBones {
            std::uint32_t numBones = 0;

            // 3 DOF
            std::vector<Vector> pos;     // world
            std::vector<Vector> prevPos; // world
            std::vector<Vector> predPos; // world
            std::vector<Vector> backupPos; // world
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
            std::vector<Vector> inertiaPositive;
            std::vector<Vector> inertiaNegative;
            std::vector<Vector> invAngularInertiaPositive;
            std::vector<Vector> invAngularInertiaNegative;
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

            std::vector<Vector> centerOfMass;
            std::vector<Vector> dynamicLinearOffset;
            std::vector<Quaternion> dynamicAngularOffset;

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
            std::vector<std::string> particleName; // if not particle, empty
            std::vector<std::uint8_t> isParticle;
            std::vector<std::uint32_t> particleDepth;
            std::vector<std::uint32_t> parentBoneIdx;
            std::vector<std::uint32_t> objIdx;  // ObjectData
            std::vector<std::uint32_t> rootIdx; // Root
            std::vector<std::uint32_t> depth;

            std::vector<Vector> prevNodeWorldPos;
            std::vector<Vector> targetNodeWorldPos;
            std::vector<Quaternion> prevNodeWorldRot;
            std::vector<Quaternion> targetNodeWorldRot;

            std::vector<Quaternion> alignRot;
            std::vector<Quaternion> invAlignRot;

            std::vector<float> orgWorldScale;
            std::vector<Vector> orgLocalPos;     // backup
            std::vector<Quaternion> orgLocalRot; // backup

            std::vector<std::uint64_t> lastFrame;
        };

        struct DistanceConstraints {
            std::uint32_t numConstraints = 0;
            std::vector<std::uint32_t> boneIdx; // PhysicsBone
            std::vector<std::uint32_t> anchIdx; // PhysicsBone
            std::vector<std::uint32_t> objIdx;
            std::vector<std::uint32_t> rootIdx;
            std::vector<std::uint32_t> colorGraph;

            std::vector<float> restLen;
            std::vector<float> complianceSquish;
            std::vector<float> complianceStretch;
            std::vector<float> squishMargin;
            std::vector<float> stretchMargin;
            std::vector<float> squishLimit;
            std::vector<float> stretchLimit;
            std::vector<float> squishDamping;
            std::vector<float> stretchDamping;

            std::vector<float> lambda;
        };

        struct AngularConstraints {
            std::uint32_t numConstraints = 0;
            std::vector<std::uint32_t> boneIdx; // PhysicsBone
            std::vector<std::uint32_t> anchIdx; // PhysicsBone
            std::vector<std::uint32_t> objIdx;
            std::vector<std::uint32_t> rootIdx;
            std::vector<std::uint32_t> colorGraph;

            std::vector<Quaternion> restRot;

            std::vector<Vector> compliancePositive;
            std::vector<Vector> complianceNegative;
            std::vector<Vector> marginPositive;
            std::vector<Vector> marginNegative;
            std::vector<Vector> limitPositive;
            std::vector<Vector> limitNegative;
            std::vector<Vector> dampingPositive;
            std::vector<Vector> dampingNegative;

            std::vector<float> lambda;
        };

        struct ConeConstraints {
            std::uint32_t numConstraints = 0;
            std::vector<std::uint32_t> boneIdx; // PhysicsBone
            std::vector<std::uint32_t> anchIdx; // PhysicsBone
            std::vector<std::uint32_t> objIdx;
            std::vector<std::uint32_t> rootIdx;
            std::vector<std::uint32_t> colorGraph;

            std::vector<Vector> restDir;
            std::vector<Quaternion> alignRot;
            std::vector<Quaternion> invAlignRot;

            std::vector<Vector> compliancePositive;
            std::vector<Vector> complianceNegative;
            std::vector<Vector> marginPositive;
            std::vector<Vector> marginNegative;
            std::vector<Vector> limitPositive;
            std::vector<Vector> limitNegative;
            std::vector<Vector> dampingPositive;
            std::vector<Vector> dampingNegative;

            std::vector<float> lambda;
        };

        struct DeformConstraints {
            std::uint32_t numConstraints = 0;
            std::vector<std::uint32_t> boneIdx; // PhysicsBone
            std::vector<std::uint32_t> anchIdx; // PhysicsBone
            std::vector<std::uint32_t> objIdx;
            std::vector<std::uint32_t> rootIdx;

            std::vector<float> restLen;
            std::vector<Quaternion> restRot;

            std::vector<Vector> squishWeight;
            std::vector<Vector> stretchWeight;
            std::vector<Vector> bulgeWeight;
        };

        struct ShapeMatchingConstraints {
            std::uint32_t numConstraints = 0;

            std::vector<std::uint32_t> objIdx;  // constraint index
            std::vector<std::uint32_t> rootIdx; // constraint index

            struct ClusterData {
                std::uint32_t offset = 0;
                std::uint32_t size = 0;
                Vector compliancePositive = vZero;
                Vector complianceNegative = vZero;
                Vector inertiaScale = vZero;
            };
            std::vector<ClusterData> cluster; // constraint index

            std::vector<std::uint32_t> boneIdx;  // cluster index
            std::vector<float> boneMass;         // cluster index
            std::vector<Vector> restRelativePos; // cluster index
        };

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
        struct Colliders { // usually one collider per bone is recommended, but can be multiple colliders (e.g. furniture)
            std::uint32_t numColliders = 0;
            std::vector<std::uint32_t> boneIdx; // PhysicsBone
            std::vector<std::uint32_t> objIdx;  // ObjectData
            std::vector<std::uint32_t> rootIdx; // Root

            std::vector<std::uint8_t> noCollideCount;    // Same as numColliders
            std::vector<std::uint32_t> noCollideBoneIdx; // padding by NOCOLLIDE_MAX

            std::vector<AABB> boundingAABB;
            std::vector<float> boundingSphere;
            std::vector<Vector> boundingSphereCenter;

            std::vector<std::uint8_t> colliderType;
            std::vector<ConvexHullDataBatch> convexHullData;
            std::vector<SphereData> sphereData;
        };

        using CacheKey = std::uint64_t;
        inline CacheKey GetCacheKey(std::uint32_t a, std::uint32_t b) {
            return (static_cast<std::uint64_t>(std::min(a, b)) << 32) | std::max(a, b);
        }
        struct ConvexHullCache {
            Vector axis = vZero;
            std::uint64_t lastFrame = 0;
            ContactManifold persistentManifold;
        };

        struct ManifoldCache {
            std::uint32_t coiA;
            std::uint32_t coiB;
            ContactManifold manifold;
        };

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

        using BoneNameCache = std::vector<std::unordered_map<std::string, std::uint32_t>>;
        struct PhysicsWorldContext {
            TBB_ThreadPool* threadPool;
            ObjectDatas* objectDatas;
            PhysicsBones* physicsBones;
            tbb::spin_mutex* physicsBonesLock;
            BoneNameCache* boneNameToIdx;
        };

        class TimeProfiler {
        public:
            TimeProfiler() = delete;
            TimeProfiler(const std::string& a_funcName) : funcName(a_funcName) {};
            void Start() {
                timeStart = std::chrono::high_resolution_clock::now();
            }
            template <typename LogFunc>
            void End(LogFunc&& logCallback) {
                const auto timeEnd = std::chrono::high_resolution_clock::now();
                nsSum += std::chrono::duration_cast<std::chrono::nanoseconds>(timeEnd - timeStart).count();
                timeCount++;
                if (timeCount >= 1000) {
                    const auto ms = (nsSum * 0.001f) * ns2ms;
                    logCallback(funcName, ms);
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
    } // namespace Internal

    namespace Driver {
        struct DirectionPosOffsets {
            std::uint32_t numOffsets = 0;
            std::vector<std::string> driverName;
            std::vector<std::uint32_t> srcBoneIdx;
            std::vector<std::uint32_t> dstBoneIdx;
            std::vector<std::uint8_t> isWorldOffset;
            std::vector<Vector> baseDirection;
            std::vector<Vector> targetDirection;
            std::vector<Vector> minOffset;
            std::vector<Vector> maxOffset;
        };
        struct DirectionRotOffsets {
            std::uint32_t numOffsets = 0;
            std::vector<std::string> driverName;
            std::vector<std::uint32_t> srcBoneIdx;
            std::vector<std::uint32_t> dstBoneIdx;
            std::vector<std::uint8_t> isWorldOffset;
            std::vector<Vector> direct;
            std::vector<Quaternion> minOffset;
            std::vector<Quaternion> maxOffset;
        };
        struct VelocityPosOffsets {
            std::uint32_t numOffsets = 0;
            std::vector<std::uint32_t> srcBoneIdx;
            std::vector<std::uint32_t> dstBoneIdx;
            std::vector<std::uint8_t> isWorldOffset;
            std::vector<Vector> velocity;
            std::vector<Quaternion> minOffset;
            std::vector<Quaternion> maxOffset;
        };
        struct VelocityRotOffsets {
            std::uint32_t numOffsets = 0;
            std::vector<std::uint32_t> srcBoneIdx;
            std::vector<std::uint32_t> dstBoneIdx;
            std::vector<std::uint8_t> isWorldOffset;
            std::vector<Vector> velocity;
            std::vector<Quaternion> minOffset;
            std::vector<Quaternion> maxOffset;
        };
    } // namespace Driver
} // namespace MXPBD