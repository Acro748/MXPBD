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
            float damping = 0.0f;
            float inertiaScale = 0.0f;
            float restitution = 0.0f;
            float rotationBlendFactor = 0.1f;
            float gravity = 1.0f;
            float windFactor = 0.0f;

            float restPoseLimit = 0.0f;
            float restPoseCompliance = 0.0f;
            float restPoseAngularLimit = 0.0f;
            float restPoseAngularCompliance = 0.0f;

            RE::NiPoint3 linearRotTorque[3] = {pZero, pZero, pZero};

            RE::NiPoint3 offset = pZero;

            float collisionMargin = 0.5f;
            float collisionFriction = 0.0f;
            float collisionRotationBias = 0.05f;
            float collisionCompliance = 0.0001f;
            std::uint32_t collisionLayerGroup = 0;
            std::uint32_t collisionCollideLayer = CollisionLayer::kAllLayer;

            // particle
            std::uint8_t isParticle = 0;
            std::string parentBoneName = "";
        };
        std::unordered_map<std::string, Bone> bones; // boneName, bone

        struct Constraint {
            std::vector<std::string> anchorBoneNames;
            std::vector<float> complianceSquish;
            std::vector<float> complianceStretch;
            std::vector<float> squishLimit;
            std::vector<float> stretchLimit;
            std::vector<float> angularLimit;
            std::vector<float> squishDamping;
            std::vector<float> stretchDamping;
        };
        std::unordered_map<std::string, Constraint> constraints; // boneName, Constraint

        struct AngularConstraint {
            std::vector<std::string> anchorBoneNames;
            std::vector<float> compliance;
            std::vector<float> limit;
            std::vector<float> damping;
        };
        std::unordered_map<std::string, AngularConstraint> angularConstraints; // boneName, AngularConstraint

        struct ConvexHullColliders {
            std::unordered_map<std::string, ConvexHullDataBatch> colliders; // boneName, convexHull
            NearBones noCollideBones;
        };
        ConvexHullColliders convexHullColliders;
    };
    class PhysicsConfigReader {
    public:
        [[nodiscard]] static PhysicsConfigReader& GetSingleton() {
            static PhysicsConfigReader instance;
            return instance;
        }

        void CreateParent(RE::NiNode* rootNode, PhysicsInput& input) const;
        void CreateOriginal(RE::NiNode* rootNode, PhysicsInput& input) const;
        void CreateVolume(RE::NiNode* rootNode, PhysicsInput& input, const std::vector<RawConvexHullData>& a_rawConvexHullDatas) const;
        void CreateProperties(RE::NiNode* rootNode, PhysicsInput& input, const std::vector<RawConvexHullData>& a_rawConvexHullDatas) const;

        void AssignDefaultCollisionLayerGroup(const std::uint32_t collisionLayerGroup, PhysicsInput& input);

        struct ConstraintData {
            float complianceSquish = 0.0f;
            float complianceStretch = 0.0f;
            float squishLimit = 0.0f;
            float stretchLimit = 0.0f;
            float angularLimit = 0.0f;
            float squishDamping = 0.0f;
            float stretchDamping = 0.0f;
        };
        struct AngularConstraintData {
            float compliance = 0.0f;
            float limit = 0.0f;
            float damping = 0.0f;
        };

        bool GetPhysicsInput(const std::string& file, PhysicsInput& input) const;
        bool GetPhysicsInput(tinyxml2::XMLElement* root, const std::string& file, PhysicsInput& input) const;

        PhysicsInput::Bone defaultSMPBone;
        ConstraintData defaultSMPLinearCons;
        AngularConstraintData defaultSMPAngularCons;
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

        void BoneLogging(const std::string& file, const std::string_view name, const PhysicsInput::Bone& bone) const;
        void LinearConstraintLogging(const std::string& file, const std::string_view A, const std::string_view B, std::uint32_t anchIdx, const ConstraintData& cons) const;
        void AngularConstraintLogging(const std::string& file, const std::string_view A, const std::string_view B, std::uint32_t anchIdx, const AngularConstraintData& cons) const;
    };
} // namespace MXPBD