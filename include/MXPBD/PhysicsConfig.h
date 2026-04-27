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
            float damping = 0.0f;
            float inertiaScale = 0.0f;
            float restitution = 0.0f;
            float rotationRatio = 0.1f;
            float gravity = 1.0f;

            float restPoseLimit = 0.0f;
            float restPoseCompliance = 0.0f;

            float restPoseAngularLimit = 0.0f;
            float restPoseAngularCompliance = 0.0f;

            float linearRotTorque = 0.0f;

            RE::NiPoint3 offset = pZero;

            float collisionMargin = 0.5f;
            float collisionFriction = 0.0f;
            float collisionRotationBias = 0.05f;
            float collisionCompliance = 0.0001f;

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

        PhysicsInput::Bone defaultSMPBone = {
            .mass = 0.0f,
            .damping = 0.1f,
            .inertiaScale = 0.1f,
            .restitution = 0.1f,
            .rotationRatio = 0.1f,
            .gravity = 1.0f,
            .linearRotTorque = 0.0f,
            .offset = pZero,
            .collisionMargin = 0.0f,
            .collisionFriction = 0.0f,
            .collisionRotationBias = 0.1f,
            .collisionCompliance = 0.0001f,
        };
        ConstraintData defaultSMPLinearCons = {
            .complianceSquish = 0.001f,
            .complianceStretch = 0.001f,
            .squishLimit = 0.0f,
            .stretchLimit = 0.0f,
            .angularLimit = 0.0f,
            .squishDamping = 0.0f,
            .stretchDamping = 0.0f
        };
        AngularConstraintData defaultSMPAngularCons = {
            .compliance = 0.001f,
            .limit = 0.0f,
            .damping = 0.0f
        };
        void SetDefaultSMPConfig(const std::string& file);
        bool ConvertSMPConfig(const std::string& file, PhysicsInput& input) const;
        bool ConvertSMPConfig(tinyxml2::XMLElement* root, const std::string& file, PhysicsInput& input) const;

        void FixBoneName(PhysicsInput& input, const RenameStringMap& map) const;

    private:
        void GetBoneData(const std::string& elemName, tinyxml2::XMLElement* elem, PhysicsInput::Bone& boneData) const;
        void GetLinearConstraint(const std::string& elemName, tinyxml2::XMLElement* elem, ConstraintData& consData) const;
        void GetAngularConstraint(const std::string& elemName, tinyxml2::XMLElement* elem, AngularConstraintData& consData) const;

        void BoneLogging(const std::string& file, const std::string& name, const PhysicsInput::Bone& bone) const;
        void LinearConstraintLogging(const std::string& file, const std::string_view A, const std::string_view B, std::uint32_t anchIdx, const ConstraintData& cons) const;
        void AngularConstraintLogging(const std::string& file, const std::string_view A, const std::string_view B, std::uint32_t anchIdx, const AngularConstraintData& cons) const;
    };
} // namespace MXPBD