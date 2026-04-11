#include "MXPBD/PhysicsWorldSystem.h"

namespace MXPBD
{
    bool IsHDTSMPEnabled = false;
    void XPBDWorldSystem::Init()
    {
        physicsWorld = std::make_unique<XPBDWorld>(
            Mus::Config::GetSingleton().GetIterationMax(), 
            Mus::Config::GetSingleton().GetGridSize());

        Mus::g_frameEventDispatcher.addListener(this);
        Mus::g_facegenNiNodeEventDispatcher.addListener(this);
        Mus::g_armorAttachEventDispatcher.addListener(this);
        Mus::g_armorDetachEventDispatcher.addListener(this);
    }

    void XPBDWorldSystem::AddPhysics(RE::TESObjectREFR* object, RE::NiNode* rootNode, const XPBDWorld::RootType rootType, const std::uint32_t bipedSlot) 
    {
        if (!object)
            return;
        if (rootType == XPBDWorld::RootType::skeleton)
        {
            AddSkeletonPhysics(object, rootNode);
        }
        else
        {
            AddClothPhysics(object, rootNode, bipedSlot);
        }
    }

    void XPBDWorldSystem::UpdatePhysicsSetting(RE::TESObjectREFR* object, PhysicsInput input)
    {
        if (input.bones.empty() && input.constraints.empty())
            return;
        if (!object || !object->loadedData || !object->loadedData->data3D)
            return;
        physicsWorld->Reset(object);
        if (auto rootNode = object->loadedData->data3D->AsNode(); rootNode)
        {
            CreateProperties(rootNode, input);
        }
        physicsWorld->UpdatePhysicsSetting(object, input, false);
    }

    void XPBDWorldSystem::Reset() const
    {
        physicsWorld->Reset();
    }

    void XPBDWorldSystem::Reset(RE::TESObjectREFR* object) const
    {
        if (!object)
            return;
        {
            std::lock_guard lg(objectDataLock);
            if (objectData.count(object->formID) == 0)
                return;
        }
        physicsWorld->Reset(object);
    }

    void XPBDWorldSystem::RemovePhysics(const RE::FormID objectID)
    {
        {
            std::lock_guard lg(objectDataLock);
            if (objectData.count(objectID) == 0)
                return;
            objectData.erase(objectID);
        }
        physicsWorld->RemovePhysics(objectID);
    }

    void XPBDWorldSystem::RemovePhysics(RE::TESObjectREFR* object, const XPBDWorld::RootType rootType, const std::uint32_t bipedSlot)
    {
        if (!object)
            return;
        {
            std::lock_guard lg(objectDataLock);
            auto found = objectData.find(object->formID);
            if (found == objectData.end())
                return;
            const ObjectData newData = {.rootType = rootType, .bipedSlot = bipedSlot};
            auto it = std::find(found->second.begin(), found->second.end(), newData);
            if (it == found->second.end())
                return;
            found->second.erase(it);
        }
        physicsWorld->RemovePhysics(object, rootType, bipedSlot);
    }

    void XPBDWorldSystem::MergeNodeTree(RE::NiNode* skeleton, RE::NiNode* root)
    {
        if (!skeleton || !root)
            return;

        auto npc = skeleton->GetObjectByName("NPC");
        auto npcNode = npc ? npc->AsNode() : nullptr;
        if (!npcNode)
            return;

        auto cloneNodeTree = [&](RE::NiNode* tree) -> RE::NiNode* {
            if (!tree)
                return nullptr;
            RE::NiCloningProcess c;
            auto clone = static_cast<RE::NiNode*>(tree->CreateClone(c));
            tree->ProcessClone(c);
            return clone;
        };

        std::stack<std::pair<RE::NiNode*, RE::NiNode*>> nodeStack;
        nodeStack.push({npcNode, root});
        while (!nodeStack.empty())
        {
            auto [dst, src] = nodeStack.top();
            nodeStack.pop();

            if (!dst || !src)
                continue;

            auto& children = src->GetChildren();
            for (auto& srcChild : children)
            {
                auto srcChildNode = srcChild ? srcChild->AsNode() : nullptr;
                if (!srcChildNode)
                    continue;
                if (srcChildNode->name.empty())
                {
                    nodeStack.push({dst, srcChildNode});
                    continue;
                }
                auto dstChild = dst->GetObjectByName(srcChildNode->name.c_str());
                if (dstChild)
                {
                    auto dstChildNode = dstChild->AsNode();
                    if (dstChildNode)
                    {
                        nodeStack.push({dstChildNode, srcChildNode});
                    }
                }
                else
                {
                    auto cloneTree = cloneNodeTree(srcChildNode);
                    if (cloneTree)
                    {
                        dst->AttachChild(cloneTree);
                    }
                }
            }
        }
    }

    void XPBDWorldSystem::MergeArmorNodeTree(RE::NiNode* skeletonRoot, RE::NiNode* armorRoot)
    {
        if (!skeletonRoot || !armorRoot)
            return;

        MergeNodeTree(skeletonRoot, armorRoot);
    }

    void XPBDWorldSystem::MergeFacegenNodeTree(RE::NiNode* skeletonRoot, RE::BSFaceGenNiNode* facegenRoot, RE::BSGeometry* geo)
    {
        if (!skeletonRoot || !geo)
            return;
        RE::NiNode* root = nullptr;
        auto fmd = geo->GetExtraData<RE::BSFaceGenModelExtraData>("FMD");
        if (!fmd || !fmd->m_model || !fmd->m_model->modelMeshData)
            return;
        root = fmd->m_model->modelMeshData->faceNode.get();
        if (!root)
            return;
        auto& skinInstance = geo->skinInstance;
        if (!skinInstance || !skinInstance->skinData)
            return;
        /*if (auto headObj = root->GetObjectByName("NPC Head [Head]"); headObj)
        {
            if (auto headNode = root->AsNode(); headNode)
            {
                auto invLocal = headNode->local.Invert();
                for (auto child : headNode->GetChildren())
                {
                    child->local = invLocal * child->local;
                    headNode->DetachChild(child.get());

                }
            }
        }*/
        for (std::uint32_t i = 0; i < skinInstance->skinData->bones; ++i)
        {
            std::string boneName = "";
            if (i <= 7 && fmd && !fmd->bones[i].empty())
                boneName = fmd->bones[i].c_str();
            if (boneName.empty())
            {
                if (!skinInstance->bones[i] || skinInstance->bones[i]->name.empty())
                    continue;
                boneName = skinInstance->bones[i]->name.c_str();
            }
            auto bone = skeletonRoot->GetObjectByName(boneName.c_str());
            if (!bone)
                continue;
            skinInstance->bones[i] = bone;
            skinInstance->boneWorldTransforms[i] = &bone->world;
        }
        geo->skinInstance->rootParent = facegenRoot;
    }

    void XPBDWorldSystem::MergeFacegenNodeTree(RE::NiNode* skeletonRoot, RE::BSFaceGenNiNode* facegenNode)
    {
        if (!skeletonRoot || !facegenNode)
            return;

        auto npc = skeletonRoot->GetObjectByName("NPC");
        auto npcNode = npc ? npc->AsNode() : nullptr;
        if (!npcNode)
            return;

        RE::BSVisit::TraverseScenegraphGeometries(facegenNode, [&](RE::BSGeometry* geo) {
            if (!geo)
                return RE::BSVisit::BSVisitControl::kContinue;
            RE::NiNode* root = nullptr;
            auto fmd = static_cast<RE::BSFaceGenModelExtraData*>(geo->GetExtraData("FMD"));
            if (!fmd || !fmd->m_model || !fmd->m_model->modelMeshData)
                return RE::BSVisit::BSVisitControl::kContinue;
            root = fmd->m_model->modelMeshData->faceNode.get();
            if (!root)
                return RE::BSVisit::BSVisitControl::kContinue;
            auto& skinInstance = geo->skinInstance;
            if (!skinInstance || !skinInstance->skinData)
                return RE::BSVisit::BSVisitControl::kContinue;
            MergeNodeTree(npcNode, root);
            for (std::uint32_t i = 0; i < skinInstance->skinData->bones; ++i)
            {
                std::string boneName = "";
                if (i <= 7 && fmd && !fmd->bones[i].empty())
                    boneName = fmd->bones[i].c_str();
                if (boneName.empty())
                {
                    if (!skinInstance->bones[i] || skinInstance->bones[i]->name.empty())
                        continue;
                    boneName = skinInstance->bones[i]->name.c_str();
                }
                auto bone = npcNode->GetObjectByName(boneName.c_str());
                if (!bone)
                    continue;
                skinInstance->bones[i] = bone;
                skinInstance->boneWorldTransforms[i] = &bone->world;
            }
            geo->skinInstance->rootParent = facegenNode;
            return RE::BSVisit::BSVisitControl::kContinue;
        });
    }

    void XPBDWorldSystem::AddSkeletonPhysics(RE::TESObjectREFR* object, RE::NiNode* rootNode)
    {
        if (!object)
            return;
        if (!rootNode)
        {
            if (!object->loadedData || !object->loadedData->data3D)
                return;
            rootNode = object->loadedData->data3D->AsNode();
            if (!rootNode)
                return;
        }
        ObjectData data = {.rootType = XPBDWorld::RootType::skeleton, .bipedSlot = 0};
        {
            std::lock_guard lg(objectDataLock);
            auto it = std::find(objectData[object->formID].begin(), objectData[object->formID].end(), data);
            if (it != objectData[object->formID].end())
                return;
        }
        data.input = Mus::ConditionManager::GetSingleton().GetCondition(GetActor(object));
        data.bipedSlot = 0;
        CreateProperties(rootNode, data.input);
        {
            std::lock_guard lg(objectDataLock);
            objectData[object->formID].push_back(data);
        }
        physicsWorld->AddPhysics(object, rootNode, XPBDWorld::RootType::skeleton, data.input);
    }

    void XPBDWorldSystem::AddClothPhysics(RE::TESObjectREFR* object, RE::NiNode* rootNode, const std::uint32_t bipedSlot)
    {
        if (!object)
            return;
        if (!rootNode)
            return;
        auto geometries = GetGeometries(rootNode, bipedSlot);
        if (geometries.empty())
            return;
        PhysicsInput newInput;
        newInput.convexHullColliders = GetColliders(GetGeometryData(geometries));
        newInput.bipedSlot = bipedSlot;
        {
            ObjectData target = {.rootType = XPBDWorld::RootType::cloth, .bipedSlot = bipedSlot};
            std::lock_guard lg(objectDataLock);
            for (const auto& data : objectData[object->formID])
            {
                for (const auto& noColBones : data.input.convexHullColliders.noCollideBones)
                {
                    newInput.convexHullColliders.noCollideBones[noColBones.first].insert(noColBones.second.begin(), noColBones.second.end());
                }
                if (data != target)
                    continue;
                newInput.bones = data.input.bones;
                newInput.constraints = data.input.constraints;
                newInput.angularConstraints = data.input.angularConstraints;
            }
        }
        CreateProperties(rootNode, newInput);
        physicsWorld->AddPhysics(object, rootNode, XPBDWorld::RootType::cloth, newInput);
    }

    void XPBDWorldSystem::CheckObjectState()
    {
        std::vector<RE::FormID> removeList;
        std::unordered_map<RE::FormID, bool> cullingList;
        {
            std::lock_guard lg(objectDataLock);
            removeList.reserve(objectData.size());
            for (auto it = objectData.begin(); it != objectData.end();)
            {
                RE::TESObjectREFR* object = GetREFR(it->first);
                if (!object || !object->loadedData || !object->loadedData->data3D)
                {
                    removeList.push_back(it->first);
                    it = objectData.erase(it);
                }
                else
                {
                    cullingList[it->first] = CullingObject(object);
                    it++;
                }
            }
        }
        for (const auto& r : removeList)
        {
            physicsWorld->RemovePhysics(r);
        }
        /*for (const auto& c : cullingList)
        {
            physicsWorld->TogglePhysics(c.first, c.second);
        }*/
    }

    bool XPBDWorldSystem::CullingObject(RE::TESObjectREFR* object)
    {
        // todo
        return false;
    }

    std::string XPBDWorldSystem::GetPhysicsInputPath(RE::NiNode* root)
    {
        if (!root)
            return "";
        auto extraData = root->GetExtraData<RE::NiStringExtraData>("MXPBD");
        if (!extraData)
            return "";
        return extraData->value ? extraData->value : "";
    }

    void XPBDWorldSystem::onEvent(const Mus::FrameEvent& e)
    {
        if (Mus::IsGamePaused.load())
            return;

        CheckObjectState();

        const float deltaTime = std::min(RE::GetSecondsSinceLastFrame(), 0.1f);
        physicsWorld->RunPhysicsWorld(deltaTime);
    }

    void XPBDWorldSystem::onEvent(const Mus::FacegenNiNodeEvent& e)
    {
        if (!e.root || !e.facegenNiNode)
            return;
        logger::debug("FacegenNiNodeEvent : {:x}", e.root->GetUserData() ? e.root->GetUserData()->formID : 0);
        if (!IsHDTSMPEnabled)
            MergeFacegenNodeTree(e.root, e.facegenNiNode, e.geometry);
    }

    void XPBDWorldSystem::onEvent(const Mus::ArmorAttachEvent& e)
    {
        if (!e.actor)
            return;
        if (!e.hasAttached)
        {
            logger::debug("ArmorAttachEvent : {:x} {}", e.actor->formID, e.bipedSlot);
            PhysicsInput input;
            if (std::string physicsPath = GetPhysicsInputPath(e.armor); !physicsPath.empty())
                input = GeyPhysicsInput(physicsPath);
            if (!IsHDTSMPEnabled)
                MergeArmorNodeTree(e.skeleton, e.armor);
            std::lock_guard lg(objectDataLock);
            ObjectData data = {.rootNode = nullptr, .rootType = XPBDWorld::RootType::cloth, .bipedSlot = e.bipedSlot, .input = input};
            auto found = std::find(objectData[e.actor->formID].begin(), objectData[e.actor->formID].end(), data);
            if (found != objectData[e.actor->formID].end())
                *found = data;
            else
                objectData[e.actor->formID].push_back(data);
        }
        else
        {
            if (!e.attachedNode)
                return;
            std::lock_guard lg(objectDataLock);
            ObjectData data = {.rootNode = RE::NiPointer(e.attachedNode), .rootType = XPBDWorld::RootType::cloth, .bipedSlot = e.bipedSlot};
            auto found = std::find(objectData[e.actor->formID].begin(), objectData[e.actor->formID].end(), data);
            if (found != objectData[e.actor->formID].end())
                *found = data;
            else
                objectData[e.actor->formID].push_back(data);
        }
    }

    void XPBDWorldSystem::onEvent(const Mus::ArmorDetachEvent& e)
    {
        if (!e.hasDetached)
            return;
        if (!e.actor)
            return;
        logger::debug("ArmorDetachEvent : {:x}", e.actor->formID);
        std::vector<std::uint32_t> bipedSlotsToRemove;
        {
            std::lock_guard lg(objectDataLock);
            auto& actorData = objectData[e.actor->formID];
            for (auto it = actorData.begin(); it != actorData.end();)
            {
                if (!it->rootNode || !it->rootNode->parent)
                {
                    bipedSlotsToRemove.push_back(it->bipedSlot);
                    it = actorData.erase(it);
                }
                else
                    it++;
            }
        }
        for (const auto& bipedSlot : bipedSlotsToRemove)
        {
            physicsWorld->RemovePhysics(e.actor, XPBDWorld::RootType::cloth, bipedSlot);
        }
    }
}