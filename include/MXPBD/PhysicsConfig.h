#pragma once

namespace MXPBD {
    using RenameStringMap = std::unordered_map<std::string, std::string>;
    struct PhysicsInput {
        struct Info {
            bool isSMPConfig = false;
            std::string inputPath = "";
        };
        std::vector<Info> infos;
        std::uint32_t bipedSlot = 0;

        struct Bone {
            float mass = 0.0f;
            RE::NiPoint3 dampingPositive = pZero;
            RE::NiPoint3 dampingNegative = pZero;
            RE::NiPoint3 angularDampingPositive = pZero;
            RE::NiPoint3 angularDampingNegative = pZero;
            RE::NiPoint3 inertiaPositive = pZero;
            RE::NiPoint3 inertiaNegative = pZero;
            RE::NiPoint3 angularInertiaPositive = pZero;
            RE::NiPoint3 angularInertiaNegative = pZero;
            RE::NiPoint3 inertiaCorrectionPositive = pZero;
            RE::NiPoint3 inertiaCorrectionNegative = pZero;
            float angularBlendFactor = 0.5f;
            float gravity = 1.0f;
            RE::NiPoint3 windFactor = pZero;

            RE::NiPoint3 deformMax = pOne;
            RE::NiPoint3 deformMin = pOne;
            RE::NiPoint3 deformVolumePreservation = pOne;
            RE::NiPoint3 deformSquishSensitivity = pZero;
            RE::NiPoint3 deformStretchSensitivity = pZero;
            RE::NiPoint3 deformBulgeSensitivity = pZero;
            RE::NiPoint3 deformSquishStiffness = pZero;
            RE::NiPoint3 deformStretchStiffness = pZero;
            RE::NiPoint3 deformSquishDamping = pZero;
            RE::NiPoint3 deformStretchDamping = pZero;

            RE::NiPoint3 animDriveCompliance = pZero;
            RE::NiPoint3 animDriveAngularCompliance = pZero;

            RE::NiPoint3 linearRotTorque[3] = {pZero, pZero, pZero};

            RE::NiPoint3 centerOfMass = pZero;

            RE::NiPoint3 offset = pZero;

            std::uint8_t colliderType = ColliderType::kConvexHull;
            float collisionMargin = 0.5f;
            float collisionFriction = 0.0f;
            float collisionCompliance = 0.0001f;
            float collisionRestitution = 0.0f;
            std::uint32_t collisionLayerGroup = 0;
            std::uint32_t collisionCollideLayer = CollisionLayer::kAllLayer;

            // particle
            std::uint8_t isParticle = 0;
            std::string parentBoneName = "";

            // volume scale
            bool enableDynamicVolume = false;
            float currentVolume = 1.5f;
            float volumeMin = 0.0f;
            float volumeMax = 2.0f;
            float physicsScaleMin = 0.0f;
            float physicsScaleMax = 1.0f;
            bool clampPhysicsScale = true;
            inline float GetPhysicsScaleByVolume() const {
                if (!enableDynamicVolume)
                    return 1.0f;
                float physicsScaleByVolume = 1.0f;
                if (std::abs(volumeMax - volumeMin) > Epsilon) {
                    float t = (currentVolume - volumeMin) / (volumeMax - volumeMin);
                    if (clampPhysicsScale)
                        t = std::clamp(t, 0.0f, 1.0f);
                    physicsScaleByVolume = std::lerp(physicsScaleMin, physicsScaleMax, t);
                    if (!clampPhysicsScale)
                        physicsScaleByVolume = std::max(0.0f, physicsScaleByVolume);
                } 
                else
                    physicsScaleByVolume = (currentVolume >= volumeMax) ? physicsScaleMax : physicsScaleMin;
                return physicsScaleByVolume;
            };
        };
        std::unordered_map<std::string, Bone> bones; // boneName, bone

        struct DistanceConstraint {
            std::string boneNameA = "";
            std::string boneNameB = "";
            float complianceSquish = 0.0f;
            float complianceStretch = 0.0f;
            float squishMargin = 0.0f;
            float stretchMargin = 0.0f;
            float squishLimit = 0.0f;
            float stretchLimit = 0.0f;
            float squishDamping = 0.0f;
            float stretchDamping = 0.0f;
        };
        std::vector<DistanceConstraint> distanceConstraints;

        struct AngularConstraint {
            std::string boneNameA = "";
            std::string boneNameB = "";
            RE::NiPoint3 compliancePositive = pZero;
            RE::NiPoint3 complianceNegative = pZero;
            RE::NiPoint3 marginPositive = pZero;
            RE::NiPoint3 marginNegative = pZero;
            RE::NiPoint3 limitPositive = pZero;
            RE::NiPoint3 limitNegative = pZero;
            RE::NiPoint3 dampingPositive = pZero;
            RE::NiPoint3 dampingNegative = pZero;
        };
        std::vector<AngularConstraint> angularConstraints;

        struct ConeConstraint {
            std::string boneNameA = "";
            std::string boneNameB = "";
            RE::NiPoint3 compliancePositive = pZero;
            RE::NiPoint3 complianceNegative = pZero;
            RE::NiPoint3 marginPositive = pZero;
            RE::NiPoint3 marginNegative = pZero;
            RE::NiPoint3 limitPositive = pZero;
            RE::NiPoint3 limitNegative = pZero;
            RE::NiPoint3 dampingPositive = pZero;
            RE::NiPoint3 dampingNegative = pZero;
        };
        std::vector<ConeConstraint> coneConstraints;

        struct DeformConstraint {
            std::string boneNameA = "";
            std::string boneNameB = "";
            RE::NiPoint3 squishWeight = pZero;
            RE::NiPoint3 stretchWeight = pZero;
            RE::NiPoint3 bulgeWeight = pZero;
        };
        std::vector<DeformConstraint> deformConstraints;

        struct ShapeMatchingConstraint {
            std::vector<std::string> bones;
            RE::NiPoint3 compliancePositive = pZero;
            RE::NiPoint3 complianceNegative = pZero;
            RE::NiPoint3 inertiaScale = pZero;
        };
        std::vector<ShapeMatchingConstraint> shapeMatchingConstraints;

        struct Colliders {
            struct Collider {
                std::string boneName = "";
                std::uint8_t colliderType = ColliderType::kConvexHull;
                ConvexHullDataBatch convexHullData = {};
                SphereData sphereData = {};
            };
            std::vector<Collider> datas;
            std::unordered_map<std::string, std::unordered_set<std::string>> noCollideBones;
        };
        Colliders colliders;

        struct ClothCluster {
            std::vector<std::string> rootList;
            std::vector<std::vector<std::string>> chainList;
            float distanceHorizontalMultiplier = 1.0f;
            float distanceDiagonalMultiplier = 0.0f;
            float angularHorizontalMultiplier = 1.0f;
            float gradientCurve = 2.5f;
            struct Setting {
                Bone bone;
                DistanceConstraint distanceConstraint;
                AngularConstraint angularConstraint;
                ConeConstraint coneConstraint;
                DeformConstraint deformConstraint;
            };
            Setting root;
            Setting tip;

            std::int32_t StiffnessChainCount = 0;
            float StiffnessStartCompliance = 3.0f;
            float StiffnessEndCompliance = 30.0f;
        };
        std::vector<ClothCluster> clothClusters;

        std::uint32_t defaultCollisionLayerGroup = 0;
        std::uint32_t defaultCollisionCollideLayer = CollisionLayer::kAllLayer;

        inline void merge(const PhysicsInput& other) {
            infos.append_range(other.infos);
            bones.insert_range(other.bones);
            distanceConstraints.append_range(other.distanceConstraints);
            angularConstraints.append_range(other.angularConstraints);
            deformConstraints.append_range(other.deformConstraints);
            for (const auto& noColBones : other.colliders.noCollideBones) {
                colliders.noCollideBones[noColBones.first].insert(noColBones.second.begin(), noColBones.second.end());
            }
            clothClusters.append_range(other.clothClusters);
        };
        inline bool boneEmpty() const {
            return bones.empty();
        }
        inline bool constraintEmpty() const {
            return distanceConstraints.empty() && angularConstraints.empty() && coneConstraints.empty() && deformConstraints.empty() && clothClusters.empty();
        }
        inline bool colliderEmpty() const {
            return colliders.datas.empty() && colliders.noCollideBones.empty();
        }
        inline bool empty() const {
            return infos.empty() && boneEmpty() && constraintEmpty() && colliderEmpty();
        }
        inline void clear() {
            bones.clear();
            distanceConstraints.clear();
            angularConstraints.clear();
            coneConstraints.clear();
            deformConstraints.clear();
            colliders.datas.clear();
            colliders.noCollideBones.clear();
            clothClusters.clear();
        }
    };

    struct DriverInput {
        std::uint32_t bipedSlot = 0;
        struct DirectionOffset {
            std::string srcBoneName = "";
            std::string dstBoneName = "";
            bool isWorldOffset = true;
            RE::NiPoint3 baseDirection = pZero;
            RE::NiPoint3 targetDirection = pZero;
            RE::NiPoint3 minOffset = pZero;
            RE::NiPoint3 maxOffset = pZero;
        };
        std::unordered_map<std::string, std::vector<DirectionOffset>> directionPosOffsets; // driverName, DirectionOffset
        std::unordered_map<std::string, std::vector<DirectionOffset>> directionRotOffsets; // driverName, DirectionOffset

        inline void merge(const DriverInput& other) {
            directionPosOffsets.insert_range(other.directionPosOffsets);
            directionRotOffsets.insert_range(other.directionRotOffsets);
        }
    };

    class PhysicsConfigReader {
    public:
        [[nodiscard]] static PhysicsConfigReader& GetSingleton() {
            static PhysicsConfigReader instance;
            return instance;
        }

        using DistanceConstraintData = PhysicsInput::DistanceConstraint;
        using AngularConstraintData = PhysicsInput::AngularConstraint;
        using ConeConstraintData = PhysicsInput::ConeConstraint;
        using DeformConstraintData = PhysicsInput::DeformConstraint;
        using ShapeMatchingConstraintData = PhysicsInput::ShapeMatchingConstraint;

        void CreateParentPhysicsBone(RE::NiNode* rootNode, PhysicsInput& input) const;
        void CreateParentConstraint(RE::NiNode* rootNode, PhysicsInput& input) const;
        void CreateOriginalConstraint(RE::NiNode* rootNode, PhysicsInput& input) const;
        void CreateVolumeConstraint(RE::NiNode* rootNode, PhysicsInput& input, const std::vector<RawCollider>& a_mergedConvexHulls) const;
        void SetCenterPhysicsBone(RE::NiNode* rootNode, PhysicsInput& input, const std::vector<RawCollider>& a_mergedConvexHulls) const;
        void UpdateVolume(RE::NiNode* rootNode, PhysicsInput& input, const std::vector<RawCollider>& a_mergedConvexHulls) const;
        void CreateClothCluster(RE::NiNode* rootNode, PhysicsInput& input, const std::vector<RawCollider>& a_mergedConvexHulls) const;
        void CreateProperties(RE::NiNode* rootNode, PhysicsInput& input, const std::vector<RawCollider>& a_mergedConvexHulls) const;
        void CreateProperties(RE::NiNode* rootNode, PhysicsInput& input, const std::vector<RawColliderData>& a_rawConvexHullDatas) const;

        void AssignDefaultCollisionLayerGroup(const std::uint32_t collisionLayerGroup, PhysicsInput& input);

        bool GetPhysicsInput(const std::string& file, PhysicsInput& input) const;
        bool GetPhysicsInput(tinyxml2::XMLElement* root, const std::string& file, PhysicsInput& input) const;

        bool GetDriverInput(const std::string& file, DriverInput& input) const;
        bool GetDriverInput(tinyxml2::XMLElement* root, const std::string& file, DriverInput& input) const;

        struct SMPMigration {
            PhysicsInput::Bone defaultSMPBone;
            DistanceConstraintData defaultSMPDistanceCons;
            AngularConstraintData defaultSMPAngularCons;
            ConeConstraintData defaultSMPConeCons;
            DeformConstraintData defaultSMPDeformCons;
            PhysicsInput::ClothCluster defaultClothCluster;
        };
        SMPMigration smpMigration;
        void SetDefaultSMPConfig(const std::string& file);
        bool ConvertSMPConfig(const std::string& file, PhysicsInput& input) const;
        bool ConvertSMPConfig(tinyxml2::XMLElement* root, const std::string& file, PhysicsInput& input) const;

        void FixBoneName(PhysicsInput& input, const RenameStringMap& map) const;

    private:
        inline bool IsEmptyChar(const char* c) const {
            return !c || c[0] == '\0';
        }

        void GetBoneData(const Mus::lString& rootElemName, tinyxml2::XMLElement* rootElem, PhysicsInput::Bone& boneData) const;
        void GetDistanceConstraint(const Mus::lString& rootElemName, tinyxml2::XMLElement* elem, DistanceConstraintData& consData) const;
        void GetAngularConstraint(const Mus::lString& rootElemName, tinyxml2::XMLElement* elem, AngularConstraintData& consData) const;
        void GetConeConstraint(const Mus::lString& rootElemName, tinyxml2::XMLElement* elem, ConeConstraintData& consData) const;
        void GetDeformConstraint(const Mus::lString& rootElemName, tinyxml2::XMLElement* elem, DeformConstraintData& consData) const;
        void GetShapeMatchingConstraint(const Mus::lString& rootElemName, tinyxml2::XMLElement* elem, ShapeMatchingConstraintData& consData) const;
        void GetGenericConstraint(tinyxml2::XMLElement* rootElem, DistanceConstraintData* distConsData, AngularConstraintData* angConsData, ConeConstraintData* coneConsData, DeformConstraintData* deformConsData) const;
        void GetClothCluster(tinyxml2::XMLElement* rootElem, PhysicsInput::ClothCluster& clothCluster) const;

        void LoggingBone(const std::string& file, const std::string_view name, const PhysicsInput::Bone& bone) const;
        void LoggingDistanceConstraint(const std::string& file, const std::string_view A, const std::string_view B, const DistanceConstraintData& cons) const;
        void LoggingAngularConstraint(const std::string& file, const std::string_view A, const std::string_view B, const AngularConstraintData& cons) const;
        void LoggingConeConstraint(const std::string& file, const std::string_view A, const std::string_view B, const ConeConstraintData& cons) const;
        void LoggingDeformConstraint(const std::string& file, const std::string_view A, const std::string_view B, const DeformConstraintData& cons) const;
        void LoggingShapeMatchingConstraint(const std::string& file, const ShapeMatchingConstraintData& cons) const;
        void LoggingClothCluster(const std::string& file, PhysicsInput::ClothCluster& clothCluster) const;
    };

} // namespace MXPBD