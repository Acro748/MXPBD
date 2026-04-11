#pragma once

namespace MXPBD {
    extern bool IsHDTSMPEnabled;

    class XPBDWorldSystem : 
        public Mus::IEventListener<Mus::FrameEvent>,
        public Mus::IEventListener<Mus::ArmorAttachEvent>,
        public Mus::IEventListener<Mus::FacegenNiNodeEvent>,
        public Mus::IEventListener<Mus::ArmorDetachEvent>
    {
    public:
        [[nodiscard]] static XPBDWorldSystem& GetSingleton() {
            static XPBDWorldSystem instance;
            return instance;
        }

        void Init();
        void AddPhysics(RE::TESObjectREFR* object, RE::NiNode* rootNode, const XPBDWorld::RootType rootType, const std::uint32_t bipedSlot);
        void UpdatePhysicsSetting(RE::TESObjectREFR* object, PhysicsInput input);
        void Reset() const;
        void Reset(RE::TESObjectREFR* object) const;
        void RemovePhysics(const RE::FormID objectID);
        void RemovePhysics(RE::TESObjectREFR* object, const XPBDWorld::RootType rootType, const std::uint32_t bipedSlot);
    private:
        std::unique_ptr<XPBDWorld> physicsWorld;

        void AddSkeletonPhysics(RE::TESObjectREFR* object, RE::NiNode* rootNode);
        void AddClothPhysics(RE::TESObjectREFR* object, RE::NiNode* rootNode, const std::uint32_t bipedSlot);

        struct ObjectData {
            RE::NiPointer<RE::NiAVObject> rootNode;
            XPBDWorld::RootType rootType;
            std::uint32_t bipedSlot;
            bool operator==(const ObjectData& other) const {
                return rootType == other.rootType && bipedSlot == other.bipedSlot;
            }
            PhysicsInput input;
        };
        std::unordered_map<RE::FormID, std::vector<ObjectData>> objectData;
        mutable std::mutex objectDataLock;
        void MergeNodeTree(RE::NiNode* skeleton, RE::NiNode* root);
        void MergeArmorNodeTree(RE::NiNode* skeletonRoot, RE::NiNode* armorRoot);
        void MergeFacegenNodeTree(RE::NiNode* skeletonRoot, RE::BSFaceGenNiNode* facegenRoot, RE::BSGeometry* geo);
        void MergeFacegenNodeTree(RE::NiNode* skeletonRoot, RE::BSFaceGenNiNode* facegenGeo);
        void CheckObjectState();
        bool CullingObject(RE::TESObjectREFR* object);

        std::string GetPhysicsInputPath(RE::NiNode* root);
    protected:
        void onEvent(const Mus::FrameEvent& e) override;
        void onEvent(const Mus::FacegenNiNodeEvent& e) override;
        void onEvent(const Mus::ArmorAttachEvent& e) override;
        void onEvent(const Mus::ArmorDetachEvent& e) override;
    };
}