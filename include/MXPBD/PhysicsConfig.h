#pragma once

namespace MXPBD {
    enum CollisionLayer : std::uint32_t {
        kSkeleton = 1 << 0,
        kHead = 1 << 1,
        kRigidBody = 1 << 2,
        kSoftBody = 1 << 3,
        kGenitals = 1 << 4,
        kBody = kHead | kRigidBody | kSoftBody | kGenitals,
        kHair = 1 << 5,
        kCloth = 1 << 6,
        kSkirt = 1 << 7,
        kCape = 1 << 8,
        kOutfit = kCloth | kSkirt | kCape,
        kWing = 1 << 9,
        kEars = 1 << 10,
        kTail = 1 << 11,
        kWeapon = 1 << 12,
        kGround = 1 << 15,
        kStatic = 1 << 16,
        kEnvironment = kGround | kStatic,
        kMisc1 = 1 << 17,
        kMisc2 = 1 << 18,
        kMisc3 = 1 << 19,
        kMisc4 = 1 << 20,
        kMisc5 = 1 << 21,
        kMisc6 = 1 << 22,
        kMisc7 = 1 << 23,
        kMisc8 = 1 << 24,
        kMisc9 = 1 << 25,
        kMisc10 = 1 << 26,
        kMisc11 = 1 << 27,
        kMisc12 = 1 << 28,
        kMisc13 = 1 << 29,
        kMisc14 = 1 << 30,
        kMisc15 = 1 << 31,
        kAllLayer = ~0u
    };
    const std::unordered_map<Mus::lString, std::uint32_t>& GetCollisionLayerEnum();
    std::uint32_t GetStringAsBitMask(const Mus::lString& str);

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
            RE::NiPoint3 limitPositive = pHalfInf;
            RE::NiPoint3 limitNegative = pHalfInf;
            RE::NiPoint3 angularLimitPositive = pRotInf;
            RE::NiPoint3 angularLimitNegative = pRotInf;
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

        struct Constraint {
            struct AnchorData {
                std::string anchorBoneName = "";
                float complianceSquish = 0.0f;
                float complianceStretch = 0.0f;
                float squishMargin = 0.0f;
                float stretchMargin = 0.0f;
                float squishDamping = 0.0f;
                float stretchDamping = 0.0f;
                float scaleSquish = 1.0f;
                float scaleStretch = 1.0f;
            };
            std::vector<AnchorData> anchors;
        };
        std::unordered_map<std::string, Constraint> constraints; // boneName, Constraint

        struct AngularConstraint {
            struct AnchorData {
                std::string anchorBoneName = "";
                RE::NiPoint3 compliancePositive = pZero;
                RE::NiPoint3 complianceNegative = pZero;
                RE::NiPoint3 marginPositive = pZero;
                RE::NiPoint3 marginNegative = pZero;
                RE::NiPoint3 dampingPositive = pZero;
                RE::NiPoint3 dampingNegative = pZero;
            };
            std::vector<AnchorData> anchors;
        };
        std::unordered_map<std::string, AngularConstraint> angularConstraints; // boneName, AngularConstraint

        struct DeformConstraint {
            struct AnchorData {
                std::string anchorBoneName = "";
                RE::NiPoint3 squishWeight = pZero;
                RE::NiPoint3 stretchWeight = pZero;
                RE::NiPoint3 bulgeWeight = pZero;
            };
            std::vector<AnchorData> anchors;
        };
        std::unordered_map<std::string, DeformConstraint> deformConstraints; // boneName, DeformConstraint

        struct Colliders {
            struct Collider {
                std::string boneName = "";
                std::uint8_t colliderType = 0;
                ConvexHullDataBatch convexHullData = {};
                SphereData sphereData;
            };
            std::vector<Collider> datas;
            NearBones noCollideBones;
        };
        Colliders colliders;

        inline void merge(const PhysicsInput& other) {
            infos.append_range(other.infos);
            bones.insert_range(other.bones);
            constraints.insert_range(other.constraints);
            angularConstraints.insert_range(other.angularConstraints);
            deformConstraints.insert_range(other.deformConstraints);
            for (const auto& noColBones : other.colliders.noCollideBones) {
                colliders.noCollideBones[noColBones.first].insert(noColBones.second.begin(), noColBones.second.end());
            }
        };
    };
    class PhysicsConfigReader {
    public:
        [[nodiscard]] static PhysicsConfigReader& GetSingleton() {
            static PhysicsConfigReader instance;
            return instance;
        }

        using ConstraintData = PhysicsInput::Constraint::AnchorData;
        using AngularConstraintData = PhysicsInput::AngularConstraint::AnchorData;
        using DeformConstraintData = PhysicsInput::DeformConstraint::AnchorData;

        void CreateParentPhysicsBone(RE::NiNode* rootNode, PhysicsInput& input) const;
        void CreateParentConstraint(RE::NiNode* rootNode, PhysicsInput& input) const;
        void CreateOriginalConstraint(RE::NiNode* rootNode, PhysicsInput& input) const;
        void CreateVolumeConstraint(RE::NiNode* rootNode, PhysicsInput& input, const std::vector<RawConvexHull>& a_mergedConvexHulls) const;
        void UpdateVolume(RE::NiNode* rootNode, PhysicsInput& input, const std::vector<RawConvexHull>& a_mergedConvexHulls) const;
        void CreateProperties(RE::NiNode* rootNode, PhysicsInput& input, const std::vector<RawConvexHull>& a_mergedConvexHulls) const;
        void CreateProperties(RE::NiNode* rootNode, PhysicsInput& input, const std::vector<RawConvexHullData>& a_rawConvexHullDatas) const;

        void AssignDefaultCollisionLayerGroup(const std::uint32_t collisionLayerGroup, PhysicsInput& input);

        bool GetPhysicsInput(const std::string& file, PhysicsInput& input) const;
        bool GetPhysicsInput(tinyxml2::XMLElement* root, const std::string& file, PhysicsInput& input) const;

        struct SMPMigration {
            PhysicsInput::Bone defaultSMPBone;
            ConstraintData defaultSMPLinearCons;
            AngularConstraintData defaultSMPAngularCons;
            std::int32_t StiffnessChainCount = 2;
            float StiffnessStartCompliance = 3.0f;
            float StiffnessEndCompliance = 30.0f;
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
        void GetLinearConstraint(const Mus::lString& rootElemName, tinyxml2::XMLElement* elem, ConstraintData& consData) const;
        void GetAngularConstraint(const Mus::lString& rootElemName, tinyxml2::XMLElement* elem, AngularConstraintData& consData) const;
        void GetDeformConstraint(const Mus::lString& rootElemName, tinyxml2::XMLElement* elem, DeformConstraintData& consData) const;

        void BoneLogging(const std::string& file, const std::string_view name, const PhysicsInput::Bone& bone) const;
        void LinearConstraintLogging(const std::string& file, const std::string_view A, const std::string_view B, std::uint32_t anchIdx, const ConstraintData& cons) const;
        void AngularConstraintLogging(const std::string& file, const std::string_view A, const std::string_view B, std::uint32_t anchIdx, const AngularConstraintData& cons) const;
        void DeformConstraintLogging(const std::string& file, const std::string_view A, const std::string_view B, std::uint32_t anchIdx, const DeformConstraintData& cons) const;
    };
} // namespace MXPBD