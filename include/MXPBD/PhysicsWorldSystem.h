#pragma once

namespace MXPBD {
    extern bool IsHDTSMPEnabled;

    class XPBDWorldSystem : 
        public Mus::IEventListener<Mus::FrameEvent>,
        public Mus::IEventListener<Mus::FacegenNiNodeEvent>,
        public Mus::IEventListener<Mus::ArmorAttachEvent>,
        public Mus::IEventListener<Mus::ArmorDetachEvent>,
        public Mus::IEventListener<Mus::Load3DEvent>,
        public RE::BSTEventSink<RE::MenuOpenCloseEvent>
    {
    public:
        [[nodiscard]] static XPBDWorldSystem& GetSingleton() {
            static XPBDWorldSystem instance;
            return instance;
        }

        void Init();
        void LoadConfigOnPhysicsWorld() const;

        void AddPhysics(RE::TESObjectREFR* object, RE::NiNode* rootNode, const XPBDWorld::RootType rootType, const std::uint32_t bipedSlot, const bool isAddCollider = true);
        void UpdatePhysicsSetting(RE::TESObjectREFR* object, PhysicsInput input);
        void Reset() const;
        void Reset(RE::TESObjectREFR* object) const;
        void RemovePhysics(const RE::FormID objectID);
        void RemovePhysics(RE::TESObjectREFR* object, const XPBDWorld::RootType rootType, const std::uint32_t bipedSlot);
        void ReloadPhysics(RE::TESObjectREFR* object);
        void TogglePhysics(RE::TESObjectREFR* object, bool isDisable);
        void UpdateRawConvexHulls(RE::TESObjectREFR* object, RE::NiNode* rootNode);
    private:
        std::unique_ptr<XPBDWorld> physicsWorld;
        std::uint8_t isRaceSexMenuOpen = 0;

        void AddSkeletonPhysics(RE::TESObjectREFR* object, RE::NiNode* rootNode);
        void AddFacegenPhysics(RE::TESObjectREFR* object, RE::NiNode* rootNode);
        void AddClothPhysics(RE::TESObjectREFR* object, RE::NiNode* rootNode, const std::uint32_t bipedSlot);
        void AddColliders(RE::TESObjectREFR* object, RE::NiNode* rootNode);

        struct ObjectData {
            mutable std::mutex lock;
            RE::SEX sex = RE::SEX::kTotal;
            RE::FormID raceID = 0;
            RenameStringMap renameMap;
            struct RawData {
                RE::NiPointer<RE::NiAVObject> rootNode = nullptr;
                XPBDWorld::RootType rootType = XPBDWorld::RootType::kNone;
                std::uint32_t bipedSlot = 0;
                bool operator==(const RawData& other) const {
                    return rootType == other.rootType && bipedSlot == other.bipedSlot;
                }
                bool operator<(const RawData& other) const {
                    if (rootType != other.rootType)
                        return rootType < other.rootType;
                    return bipedSlot < other.bipedSlot;
                }
                PhysicsInput input;
                std::vector<RawConvexHullData> rawConvexHullDatas;
            };
            std::vector<RawData> rawDatas;
            inline void sortRawDatas() {
                std::ranges::sort(rawDatas, [](const ObjectData::RawData& dataA, const ObjectData::RawData& dataB) {
                    return dataA < dataB;
                });
            }
        };
        using ObjectDataPtr = std::shared_ptr<ObjectData>;
        ObjectDataPtr GetOrCreateObjectDataPtr_unsafe(RE::TESObjectREFR* object);
        ObjectDataPtr FindObjectDataPtr_unsafe(RE::TESObjectREFR* object);
        std::unordered_map<RE::FormID, ObjectDataPtr> objectDatas;
        mutable std::mutex objectDatasLock;

        bool ResetIfChanged(RE::TESObjectREFR* refr);

        const std::string_view BSFaceGenNiNodeSkinned = "BSFaceGenNiNodeSkinned";
        void MergeNodeTree(RE::NiNode* skeleton, RE::NiNode* root, const std::string& prefix, RenameStringMap& map, bool isRenameOrgTree) const;
        void MergeArmorNodeTree(RE::NiNode* skeletonRoot, RE::NiNode* armorRoot, const std::string& prefix, RenameStringMap& map) const;
        void MergeFacegenNodeTree(RE::NiNode* skeletonRoot, RE::BSFaceGenNiNode* facegenRoot, RE::BSGeometry* geo, PhysicsInput& input, RenameStringMap& map) const;
        void RemoveMergedNode(RE::NiNode* root, std::string_view prefix) const;
        void RemoveRenameMap(RenameStringMap& map, std::string_view prefix) const;

        void CheckObjectState();

        std::string GetPhysicsInputPath(RE::NiNode* root) const;
        std::string GetSMPConfigPath(RE::NiNode* root) const;
    protected:
        void onEvent(const Mus::FrameEvent& e) override;
        void onEvent(const Mus::FacegenNiNodeEvent& e) override;
        void onEvent(const Mus::ArmorAttachEvent& e) override;
        void onEvent(const Mus::ArmorDetachEvent& e) override;
        void onEvent(const Mus::Load3DEvent& e) override;

        EventResult ProcessEvent(const RE::MenuOpenCloseEvent* evn, RE::BSTEventSource<RE::MenuOpenCloseEvent>*) override;
    };
}