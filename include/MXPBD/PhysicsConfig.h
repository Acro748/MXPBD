#pragma once

namespace MXPBD {
    struct PhysicsInput {
        std::uint32_t bipedSlot = 0;

        struct Bone {
            float mass = 0.0f;
            float damping = 0.0f;
            float inertiaScale = 0.0f;
            float rotRatio = 0.0f;
            std::uint8_t advancedRotation = 0;
            float gravity = 1.0f;
            RE::NiPoint3 offset = EmptyPoint;
            float colMargin = 0.5f;
            float colFriction = 0.0f;
            float colComp = 0.0001f;

            float linearRotTorque = 0.0f;

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
            struct ConvexHullCollider {
                std::vector<RE::NiPoint3> vertices;
                std::vector<std::uint32_t> indices;
                std::string boneName;
            };
            std::vector<ConvexHullCollider> colliders;
            std::unordered_map<std::string, std::unordered_set<std::string>> noCollideBones;
        };
        ConvexHullColliders convexHullColliders;
    };

    PhysicsInput GeyPhysicsInput(const std::string& file);
    PhysicsInput GeyPhysicsInput(tinyxml2::XMLElement* root, const std::string& file);

    void CreateParent(RE::NiNode* rootNode, PhysicsInput& input);
    void CreateOriginal(RE::NiNode* rootNode, PhysicsInput& input);
    void CreateSoftBody(RE::NiNode* rootNode, PhysicsInput& input);
    void CreateProperties(RE::NiNode* rootNode, PhysicsInput& input);
} // namespace MXPBD