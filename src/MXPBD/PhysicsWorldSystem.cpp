#include "MXPBD/PhysicsWorldSystem.h"

namespace MXPBD
{
    bool IsHDTSMPEnabled = false;
    void XPBDWorldSystem::Init()
    {
        physicsWorld = std::make_unique<XPBDWorld>(
            Mus::Config::GetSingleton().GetIterationMax(), 
            Mus::Config::GetSingleton().GetSmallGridSize(), 
            Mus::Config::GetSingleton().GetLargeGridSize(),
            GetSIMDType(false, Mus::Config::GetSingleton().GetSIMDType() + 1));

        Mus::g_frameEventDispatcher.addListener(this);
        Mus::g_facegenNiNodeEventDispatcher.addListener(this);
        Mus::g_armorAttachEventDispatcher.addListener(this);
        Mus::g_armorDetachEventDispatcher.addListener(this);
    }

    void XPBDWorldSystem::AddPhysics(RE::TESObjectREFR* object, RE::NiNode* rootNode, const XPBDWorld::RootType rootType, const std::uint32_t bipedSlot)
    {
        if (!object/* || !Mus::IsPlayer(object->formID)*/)
            return;
        UpdateRawConvexHulls(object, nullptr);
        if (rootType == XPBDWorld::RootType::skeleton)
        {
            AddSkeletonPhysics(object, rootNode);
        }
        else if (rootType == XPBDWorld::RootType::facegen)
        {
            AddFacegenPhysics(object, rootNode);
        }
        else if (rootType == XPBDWorld::RootType::cloth)
        {
            AddClothPhysics(object, rootNode, bipedSlot);
        }
        else if (rootType == XPBDWorld::RootType::weapon)
        {
        }
        AddColliders(object, rootNode);
    }

    void XPBDWorldSystem::UpdatePhysicsSetting(RE::TESObjectREFR* object, PhysicsInput input)
    {
        if (input.bones.empty() && input.constraints.empty())
            return;
        if (!object || !object->loadedData || !object->loadedData->data3D)
            return;
        auto rootNode = object->loadedData->data3D->AsNode();
        if (!rootNode)
            return;
        physicsWorld->Reset(object);
        std::vector<RawConvexHullData> rawConvexHullDatas;
        {
            ObjectData::RawData targetCollider = {.rootType = XPBDWorld::RootType::collider, .bipedSlot = 0};
            std::unique_lock ul(objectDatasLock);
            ObjectDataPtr objData = FindObjectDataPtr_unsafe(object);
            if (!objData)
                return;
            std::lock_guard lg(objData->lock);
            ul.unlock();
            auto found = std::find(objData->rawDatas.begin(), objData->rawDatas.end(), targetCollider);
            if (found != objData->rawDatas.end())
            {
                rawConvexHullDatas = found->rawConvexHullDatas;
            }
        }
        CreateProperties(rootNode, input, rawConvexHullDatas);
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
            std::lock_guard lg(objectDatasLock);
            if (objectDatas.count(object->formID) == 0)
                return;
        }
        physicsWorld->Reset(object);
    }

    void XPBDWorldSystem::RemovePhysics(const RE::FormID objectID)
    {
        {
            std::lock_guard lg(objectDatasLock);
            if (objectDatas.count(objectID) == 0)
                return;
            objectDatas.erase(objectID);
        }
        physicsWorld->RemovePhysics(objectID);
    }

    void XPBDWorldSystem::RemovePhysics(RE::TESObjectREFR* object, const XPBDWorld::RootType rootType, const std::uint32_t bipedSlot)
    {
        if (!object)
            return;

        const ObjectData::RawData target = {.rootType = rootType, .bipedSlot = bipedSlot};
        std::unique_lock ul(objectDatasLock);
        ObjectDataPtr objData = FindObjectDataPtr_unsafe(object);
        if (!objData)
            return;
        std::lock_guard lg(objData->lock);
        ul.unlock();
        auto it = std::find(objData->rawDatas.begin(), objData->rawDatas.end(), target);
        if (it == objData->rawDatas.end())
            return;
        objData->rawDatas.erase(it);
        physicsWorld->RemovePhysics(object, rootType, bipedSlot);
    }

    XPBDWorldSystem::ObjectDataPtr XPBDWorldSystem::GetOrCreateObjectDataPtr_unsafe(RE::TESObjectREFR* object)
    {
        if (!object)
            return nullptr;
        auto objIt = objectDatas.find(object->formID);
        if (objIt != objectDatas.end())
            return objIt->second;
        ObjectDataPtr objData = std::make_shared<ObjectData>();
        if (RE::Actor* actor = object->As<RE::Actor>(); actor)
        {
            if (RE::TESRace* race = actor->GetRace(); race)
            {
                objData->raceID = race->formID;
            }
        }
        if (object->data.objectReference)
        {
            if (RE::TESNPC* actorBase = object->data.objectReference->As<RE::TESNPC>(); actorBase)
            {
                objData->sex = actorBase->GetSex();
            }
        }
        objectDatas.emplace(object->formID, objData);
        return objData;
    }
    XPBDWorldSystem::ObjectDataPtr XPBDWorldSystem::FindObjectDataPtr_unsafe(RE::TESObjectREFR* object)
    {
        if (!object)
            return nullptr;
        auto objIt = objectDatas.find(object->formID);
        if (objIt != objectDatas.end())
            return objIt->second;
        return nullptr;
    }

    bool XPBDWorldSystem::ResetIfChanged(RE::TESObjectREFR* object)
    {
        if (!object)
            return false;
        bool isChanged = false;
        {
            std::unique_lock ul(objectDatasLock);
            ObjectDataPtr objData = FindObjectDataPtr_unsafe(object);
            if (!objData)
                return false;
            std::lock_guard lg(objData->lock);
            ul.unlock();
            if (RE::Actor* actor = object->As<RE::Actor>(); actor)
            {
                RE::TESRace* race = actor->GetRace();
                if ((!race && objData->raceID != 0) || (objData->raceID != race->formID))
                {
                    isChanged = true;
                    objData->raceID = race->formID;
                }
            }
            if (object->data.objectReference)
            {
                RE::TESNPC* actorBase = object->data.objectReference->As<RE::TESNPC>();
                if (actorBase)
                {
                    RE::SEX sex = actorBase->GetSex();
                    if (objData->sex != sex)
                    {
                        isChanged = true;
                        objData->sex = sex;
                    }
                }
            }
            if (isChanged)
            {
                objData->rawDatas.clear();
                physicsWorld->RemovePhysics(object->formID);
            }
        }
        return isChanged;
    }

    void XPBDWorldSystem::MergeNodeTree(RE::NiNode* skeleton, RE::NiNode* root, std::string_view prefix, RenameStringMap& map, bool isRenameOrgTree) const
    {
        if (!skeleton || !root || prefix.empty())
            return;

        auto refr = skeleton->GetUserData();
        if (!refr)
            return;

        auto npc = skeleton->GetObjectByName("NPC");
        auto npcNode = npc ? npc->AsNode() : nullptr;
        if (!npcNode)
            return;

        auto renameTree = [&](RE::NiNode* tree) -> void {
            RE::BSVisit::TraverseScenegraphObjects(tree, [&](RE::NiAVObject* obj) {
                if (!obj || obj->name.empty())
                    return RE::BSVisit::BSVisitControl::kContinue;
                std::string_view sv = obj->name.c_str();
                if (sv.starts_with(prefix))
                    return RE::BSVisit::BSVisitControl::kContinue;
                const std::string newName = std::string(prefix) + sv.data();
                map.emplace(sv, newName);
                Mus::setNiNodeName(obj->AsNode(), newName.c_str());
                return RE::BSVisit::BSVisitControl::kContinue;
            });
        };

        auto cloneNodeTree = [&](RE::NiNode* tree) -> RE::NiNode* {
            if (!tree)
                return nullptr;
            if (isRenameOrgTree)
                renameTree(tree);
            RE::NiCloningProcess c;
            c.copyType = 1;
            c.scale = RE::NiPoint3(1.0f, 1.0f, 1.0f);
            auto clone = static_cast<RE::NiNode*>(tree->CreateClone(c));
            tree->ProcessClone(c);
            if (!isRenameOrgTree)
                renameTree(clone);
            return clone;
        };

        auto merge = [&](auto&& self, RE::NiNode* dst, RE::NiNode* src) -> void {
            auto& children = src->GetChildren();
            for (std::int32_t i = static_cast<std::int32_t>(children.size()) - 1; i >= 0; --i)
            {
                auto srcChildNode = children[i] ? children[i]->AsNode() : nullptr;
                if (!srcChildNode)
                    continue;
                if (srcChildNode->name.empty())
                {
                    self(self, dst, srcChildNode);
                    continue;
                }
                if (std::string_view(srcChildNode->name.c_str()) == BSFaceGenNiNodeSkinned)
                    continue;
                auto dstChild = npcNode->GetObjectByName(srcChildNode->name);
                if (dstChild && dstChild->AsNode())
                {
                    self(self, dstChild->AsNode(), srcChildNode);
                }
                else
                {
                    logger::debug("{:x} : try attach node tree root {} on {}", refr->formID, srcChildNode->name.c_str(), dst->name.c_str());
                    auto cloneTree = cloneNodeTree(srcChildNode);
                    if (cloneTree)
                    {
                        dst->AttachChild(cloneTree);
                        logger::debug("{:x} : attached node tree root {} on {}", refr->formID, cloneTree->name.c_str(), dst->name.c_str());
                    }
                }
            }
        };
        merge(merge, npcNode, root);
    }

    void XPBDWorldSystem::MergeArmorNodeTree(RE::NiNode* skeletonRoot, RE::NiNode* armorRoot, std::string_view prefix, RenameStringMap& map) const
    {
        if (!skeletonRoot || !armorRoot)
            return;

        MergeNodeTree(skeletonRoot, armorRoot, prefix, map, true);
    }

    void XPBDWorldSystem::MergeFacegenNodeTree(RE::NiNode* skeletonRoot, RE::BSFaceGenNiNode* facegenRoot, RE::BSGeometry* geo, PhysicsInput& input, RenameStringMap& map) const
    {
        if (!skeletonRoot || !geo || !facegenRoot)
            return;

        auto refr = skeletonRoot->GetUserData();
        if (!refr || !refr->data.objectReference)
            return;

        auto npc = skeletonRoot->GetObjectByName("NPC");
        auto npcNode = npc ? npc->AsNode() : nullptr;
        if (!npcNode)
            return;

        RE::NiNode* origRootNode = nullptr;
        RE::BSGeometry* origGeom = nullptr;
        bool isFMD = false;
        auto fmd = geo->GetExtraData<RE::BSFaceGenModelExtraData>("FMD");
        if (fmd && fmd->m_model && fmd->m_model->modelMeshData)
        {
            origRootNode = fmd->m_model->modelMeshData->faceNode.get();
            isFMD = true;
            if (origRootNode)
            {
                for (auto& child : origRootNode->GetChildren())
                {
                    if (child && child->AsGeometry())
                    {
                        origGeom = child->AsGeometry();
                        break;
                    }
                }
            }
        }
        else
        {
            typedef bool (*func_t)(RE::TESNPC*, char*);
            REL::Relocation<func_t> func{RELOCATION_ID(24222, 24726)};
            auto actorBase = refr->data.objectReference->As<RE::TESNPC>();
            if (!actorBase)
                return;

            char filePath[MAX_PATH];
            if (!func(actorBase, filePath))
                return;

            RE::NiPointer<RE::NiNode> model;
            RE::BSModelDB::DBTraits::ArgsType argsType;
            RE::BSModelDB::Demand(filePath, model, argsType);

            if (model)
            {
                auto rootFadeNode = model->AsFadeNode();
                if (rootFadeNode)
                {
                    origRootNode = rootFadeNode;
                    auto obj = rootFadeNode->GetObjectByName(geo->name);
                    if (obj && obj->AsGeometry())
                    {
                        origGeom = obj->AsGeometry();
                    }
                }
            }
        }

        if (origRootNode)
        {
            if (std::string physicsPath = GetPhysicsInputPath(origRootNode); !physicsPath.empty())
                input = GetPhysicsInput(physicsPath);
            else if (std::string smpConfigPath = GetSMPConfigPath(origRootNode); !smpConfigPath.empty())
                input = ConvertSMPConfig(smpConfigPath);
        }

        auto& skinInstance = geo->skinInstance;
        if (!skinInstance || !skinInstance->skinData)
            return;

        bool hasMerged = false;
        for (std::uint32_t i = 0; i < skinInstance->skinData->bones; ++i)
        {
            std::string boneName = "";
            if (i <= 7 && fmd && !fmd->bones[i].empty())
            {
                boneName = fmd->bones[i].c_str();
            }

            if (boneName.empty())
            {
                if (origGeom && origGeom->skinInstance && origGeom->skinInstance->bones && origGeom->skinInstance->bones[i])
                {
                    boneName = origGeom->skinInstance->bones[i]->name.c_str();
                }
                else if (skinInstance->bones && skinInstance->bones[i])
                {
                    boneName = skinInstance->bones[i]->name.c_str();
                }
            }

            if (boneName.empty())
                continue;

            auto bone = npcNode->GetObjectByName(boneName);

            if (!bone)
            {
                std::string prefixedName = std::string(GetFacegenCloneNodePrefix()) + boneName;
                bone = npcNode->GetObjectByName(prefixedName);
            }

            if (!bone && !hasMerged && origRootNode)
            {
                if (!isFMD)
                {
                    auto headObj = origRootNode->GetObjectByName("NPC Head [Head]");
                    auto headNode = headObj ? headObj->AsNode() : nullptr;
                    if (headNode)
                    {
                        RE::NiTransform invTransform = headNode->local.Invert();
                        auto& children = origRootNode->GetChildren();
                        for (std::int32_t ci = static_cast<std::int32_t>(children.size()) - 1; ci >= 0; --ci)
                        {
                            auto child = children[ci];
                            auto childNode = child ? child->AsNode() : nullptr;

                            if (childNode && !npcNode->GetObjectByName(childNode->name))
                            {
                                childNode->local = invTransform * childNode->local;
                                origRootNode->DetachChild(childNode);
                                headNode->AttachChild(childNode);
                                logger::debug("{:x} : attached node {} on head node", refr->formID, childNode->name.c_str());
                            }
                        }
                    }
                }

                MergeNodeTree(skeletonRoot, origRootNode, GetFacegenCloneNodePrefix(), map, false);
                hasMerged = true;

                bone = npcNode->GetObjectByName(boneName);
                if (!bone)
                {
                    std::string prefixedName = std::string(GetFacegenCloneNodePrefix()) + boneName;
                    bone = npcNode->GetObjectByName(prefixedName);
                    if (bone)
                        map.emplace(boneName, prefixedName);
                }
            }

            if (bone && bone->AsNode())
            {
                skinInstance->bones[i] = bone->AsNode();
                skinInstance->boneWorldTransforms[i] = &bone->world;
            }
        }

        geo->skinInstance->rootParent = facegenRoot;
    }

    void XPBDWorldSystem::RemoveMergedNode(RE::NiNode* a_root, std::string_view a_prefix) const
    {
        if (!a_root || a_prefix.empty())
            return;
        auto remove = [&](auto&& self, RE::NiNode* node, std::string_view prefix) -> void {
            auto& children = node->GetChildren();
            for (std::int32_t i = static_cast<std::int32_t>(children.size()) - 1; i >= 0; --i)
            {
                auto child = children[i].get();
                if (!child)
                    continue;
                std::string_view sv = child->name.c_str();
                if (sv.starts_with(prefix))
                {
                    node->DetachChildAt(i);
                }
                else if (auto childNode = child->AsNode())
                {
                    self(self, childNode, prefix);
                }
            }
        };
        remove(remove, a_root, a_prefix);
    }

    void XPBDWorldSystem::RemoveRenameMap(RenameStringMap& map, std::string_view prefix) const 
    {
        for (auto it = map.begin(); it != map.end();)
        {
            if (it->first.starts_with(prefix))
                it = map.erase(it);
            else
                it++;
        }
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
        ObjectData::RawData data = {.rootType = XPBDWorld::RootType::skeleton, .bipedSlot = 0};
        data.input = Mus::ConditionManager::GetSingleton().GetCondition(GetActor(object));
        data.bipedSlot = 0;
        std::vector<RawConvexHullData> rawConvexHullDatas;
        {
            std::unique_lock ul(objectDatasLock);
            ObjectDataPtr objData = GetOrCreateObjectDataPtr_unsafe(object);
            if (!objData)
                return;
            std::lock_guard lg(objData->lock);
            ul.unlock();
            {
                ObjectData::RawData target = {.rootType = XPBDWorld::RootType::skeleton, .bipedSlot = 0};
                auto it = std::find(objData->rawDatas.begin(), objData->rawDatas.end(), target);
                if (it != objData->rawDatas.end())
                {
                    it->input = data.input;
                }
                else
                {
                    target.input = data.input;
                    objData->rawDatas.push_back(target);
                }
            }
            {
                ObjectData::RawData target = {.rootType = XPBDWorld::RootType::collider, .bipedSlot = 0};
                auto it = std::find(objData->rawDatas.begin(), objData->rawDatas.end(), target);
                if (it != objData->rawDatas.end())
                {
                    rawConvexHullDatas = it->rawConvexHullDatas;
                }
            }
        }
        CreateProperties(rootNode, data.input, rawConvexHullDatas);
        physicsWorld->AddPhysics(object, rootNode, XPBDWorld::RootType::skeleton, data.input);
    }

    void XPBDWorldSystem::AddFacegenPhysics(RE::TESObjectREFR* object, RE::NiNode* rootNode)
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
        PhysicsInput newInput;
        std::vector<RawConvexHullData> rawConvexHullDatas;
        {
            std::unique_lock ul(objectDatasLock);
            const ObjectDataPtr objData = GetOrCreateObjectDataPtr_unsafe(object);
            if (!objData)
                return;
            std::lock_guard lg(objData->lock);
            ul.unlock();
            {
                ObjectData::RawData target = {.rootType = XPBDWorld::RootType::facegen, .bipedSlot = 0};
                auto it = std::find(objData->rawDatas.begin(), objData->rawDatas.end(), target);
                if (it == objData->rawDatas.end())
                    return;
                newInput = it->input;
            }
            {
                ObjectData::RawData target = {.rootType = XPBDWorld::RootType::collider, .bipedSlot = 0};
                auto it = std::find(objData->rawDatas.begin(), objData->rawDatas.end(), target);
                if (it != objData->rawDatas.end())
                {
                    rawConvexHullDatas = it->rawConvexHullDatas;
                }
            }
        }
        newInput.bipedSlot = 0;
        CreateProperties(rootNode, newInput, rawConvexHullDatas);
        physicsWorld->AddPhysics(object, rootNode, XPBDWorld::RootType::cloth, newInput);
    }

    void XPBDWorldSystem::AddClothPhysics(RE::TESObjectREFR* object, RE::NiNode* rootNode, const std::uint32_t bipedSlot)
    {
        if (!object)
            return;
        if (!rootNode)
            return;
        PhysicsInput newInput;
        std::vector<RawConvexHullData> rawConvexHullDatas;
        {
            std::unique_lock ul(objectDatasLock);
            const ObjectDataPtr objData = GetOrCreateObjectDataPtr_unsafe(object);
            if (!objData)
                return;
            std::lock_guard lg(objData->lock);
            ul.unlock();
            {
                ObjectData::RawData target = {.rootType = XPBDWorld::RootType::cloth, .bipedSlot = bipedSlot};
                auto it = std::find(objData->rawDatas.begin(), objData->rawDatas.end(), target);
                if (it == objData->rawDatas.end())
                    return;
                newInput = it->input;
            }
            {
                ObjectData::RawData target = {.rootType = XPBDWorld::RootType::collider, .bipedSlot = 0};
                auto it = std::find(objData->rawDatas.begin(), objData->rawDatas.end(), target);
                if (it != objData->rawDatas.end())
                {
                    rawConvexHullDatas = it->rawConvexHullDatas;
                }
            }
        }
        newInput.bipedSlot = bipedSlot;
        CreateProperties(rootNode, newInput, rawConvexHullDatas);
        physicsWorld->AddPhysics(object, rootNode, XPBDWorld::RootType::cloth, newInput);
    }

    void XPBDWorldSystem::AddColliders(RE::TESObjectREFR* object, RE::NiNode* rootNode)
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

        PhysicsInput newInput;
        {
            std::unique_lock ul(objectDatasLock);
            const ObjectDataPtr objData = GetOrCreateObjectDataPtr_unsafe(object);
            if (!objData)
                return;
            std::lock_guard lg(objData->lock);
            ul.unlock();
            for (const auto& rawData : objData->rawDatas)
            {
                for (const auto& noColBones : rawData.input.convexHullColliders.noCollideBones)
                {
                    newInput.convexHullColliders.noCollideBones[noColBones.first].insert(noColBones.second.begin(), noColBones.second.end());
                }
            }
            const ObjectData::RawData target = {.rootType = XPBDWorld::RootType::collider, .bipedSlot = 0};
            auto it = std::find(objData->rawDatas.begin(), objData->rawDatas.end(), target);
            if (it == objData->rawDatas.end())
                return;
            std::unordered_map<std::string, PointCloud> pointClouds;
            for (const auto& rawConvexHullData : it->rawConvexHullDatas)
            {
                for (const auto& rawConvexHull : rawConvexHullData.rawConvexHulls)
                {
                    pointClouds[rawConvexHull.boneName].vertices.append_range(rawConvexHull.vertices);
                }
                for (const auto& nearBone : rawConvexHullData.nearBones)
                {
                    newInput.convexHullColliders.noCollideBones[nearBone.first].insert(nearBone.second.begin(), nearBone.second.end());
                }
            }
            for (auto& pc : pointClouds)
            {
                RawConvexHull mergedRawConvexHull;
                GenerateRawConvexHull(pc.second, mergedRawConvexHull);
                GenerateConvexHullBatch(mergedRawConvexHull, newInput.convexHullColliders.colliders[pc.first]);
            }
            it->input = newInput;
        }

        physicsWorld->AddPhysics(object, rootNode, XPBDWorld::RootType::collider, newInput);
    }

    void XPBDWorldSystem::UpdateRawConvexHulls(RE::TESObjectREFR* object, RE::NiNode* rootNode)
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

        std::unique_lock ul(objectDatasLock);
        const ObjectDataPtr objData = GetOrCreateObjectDataPtr_unsafe(object);
        if (!objData)
            return;
        std::lock_guard lg(objData->lock);
        ul.unlock();

        ObjectData::RawData target = {.rootType = XPBDWorld::RootType::collider, .bipedSlot = 0};
        std::vector<RE::BSGeometry*> geometries = GetGeometries(rootNode);
        {
            auto it = std::find(objData->rawDatas.begin(), objData->rawDatas.end(), target);
            if (it != objData->rawDatas.end())
            {
                for (auto rawIt = it->rawConvexHullDatas.begin(); rawIt != it->rawConvexHullDatas.end();)
                {
                    auto geoIt = std::find(geometries.begin(), geometries.end(), rawIt->geometry);
                    if (geoIt != geometries.end())
                    {
                        geometries.erase(geoIt);
                        rawIt++;
                    }
                    else
                    {
                        rawIt = it->rawConvexHullDatas.erase(rawIt);
                    }
                }
            }
        }
        if (geometries.empty())
            return;
        std::vector<RawConvexHullData> newRawConvexHullDatas;
        newRawConvexHullDatas.reserve(geometries.size());
        for (auto& geo : geometries)
        {
            RawConvexHullData newRawConvexHullData;
            newRawConvexHullData.geometry = geo;

            BoneVertexData boneVertData = GetGeometryData(geo);
            newRawConvexHullData.nearBones = boneVertData.nearBones;

            auto pointClouds = ConvertPointClouds(boneVertData);
            newRawConvexHullData.rawConvexHulls.reserve(pointClouds.size());
            for (auto& pc : pointClouds)
            {
                RawConvexHull newRawConvexHull;
                GenerateRawConvexHull(pc, newRawConvexHull);
                newRawConvexHullData.rawConvexHulls.push_back(newRawConvexHull);

                // std::string fileName = "Data\\SKSE\\Plugins\\MXPBD\\" + std::to_string(object->formID) + "\\" + "_" + geo->name.c_str() + "_" + pc.boneName + ".obj";
                // writeWaveformOBJ(fileName, std::string(geo->name.c_str()) + "_" + pc.boneName, newRawConvexHull.vertices, newRawConvexHull.indices);
            }
            newRawConvexHullDatas.push_back(std::move(newRawConvexHullData));
        }

        auto it = std::find(objData->rawDatas.begin(), objData->rawDatas.end(), target);
        if (it != objData->rawDatas.end())
        {
            it->rawConvexHullDatas.append_range(newRawConvexHullDatas);
        }
        else
        {
            target.bipedSlot = 0;
            target.rawConvexHullDatas = std::move(newRawConvexHullDatas);
            objData->rawDatas.push_back(target);
        }
    }

    void XPBDWorldSystem::CheckObjectState()
    {
        std::vector<RE::FormID> removeList;
        std::unordered_map<RE::FormID, bool> cullingList;
        {
            std::lock_guard lg(objectDatasLock);
            removeList.reserve(objectDatas.size());
            for (auto it = objectDatas.begin(); it != objectDatas.end();)
            {
                RE::TESObjectREFR* object = GetREFR(it->first);
                if (!object || !object->loadedData || !object->loadedData->data3D)
                {
                    removeList.push_back(it->first);
                    it = objectDatas.erase(it);
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

    std::string XPBDWorldSystem::GetPhysicsInputPath(RE::NiNode* root) const
    {
        if (!root)
            return "";
        auto extraData = root->GetExtraData<RE::NiStringExtraData>("MXPBD");
        if (!extraData)
            return "";
        return extraData->value ? extraData->value : "";
    }

    std::string XPBDWorldSystem::GetSMPConfigPath(RE::NiNode* root) const
    {
        return "";
        if (!root)
            return "";
        auto extraData = root->GetExtraData<RE::NiStringExtraData>("HDT Skinned Mesh Physics Object");
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
        if (!IsHDTSMPEnabled)
        {
            auto object = e.root->GetUserData();
            if (!object)
                return;
            ResetIfChanged(object);
            ObjectData::RawData data = {.rootNode = RE::NiPointer(e.facegenNiNode), .rootType = XPBDWorld::RootType::facegen, .bipedSlot = 0};
            bool isNeedRemove = false;
            {
                std::unique_lock ul(objectDatasLock);
                const ObjectDataPtr objData = GetOrCreateObjectDataPtr_unsafe(object);
                if (!objData)
                    return;
                std::lock_guard lg(objData->lock);
                ul.unlock();
                auto it = std::find(objData->rawDatas.begin(), objData->rawDatas.end(), data);
                if (it != objData->rawDatas.end())
                {
                    if (it->rootNode != data.rootNode)
                    {
                        *it = data;
                        isNeedRemove = true;
                        RemoveRenameMap(objData->renameMap, GetFacegenCloneNodePrefix());
                    }
                }
                else
                {
                    objData->rawDatas.push_back(data);
                }
            }
            if (isNeedRemove)
            {
                RemoveMergedNode(e.root, GetFacegenCloneNodePrefix());
                physicsWorld->RemovePhysics(object, XPBDWorld::RootType::facegen, 0);
            }

            PhysicsInput newInput = {};
            RenameStringMap renameMap = {};
            MergeFacegenNodeTree(e.root, e.facegenNiNode, e.geometry, newInput, renameMap);

            {
                std::unique_lock ul(objectDatasLock);
                const ObjectDataPtr objData = GetOrCreateObjectDataPtr_unsafe(object);
                if (!objData)
                    return;
                std::lock_guard lg(objData->lock);
                ul.unlock();
                objData->renameMap.insert(renameMap.begin(), renameMap.end());
                FixBoneName(newInput, objData->renameMap);
                auto it = std::find(objData->rawDatas.begin(), objData->rawDatas.end(), data);
                if (it != objData->rawDatas.end())
                {
                    it->input.bones.insert_range(newInput.bones);
                    it->input.constraints.insert_range(newInput.constraints);
                    it->input.angularConstraints.insert_range(newInput.angularConstraints);
                    for (const auto& noColBones : newInput.convexHullColliders.noCollideBones)
                    {
                        it->input.convexHullColliders.noCollideBones[noColBones.first].insert(noColBones.second.begin(), noColBones.second.end());
                    }
                }
            }
        }
    }

    void XPBDWorldSystem::onEvent(const Mus::ArmorAttachEvent& e)
    {
        if (!e.actor || !e.skeleton)
            return;
        ResetIfChanged(e.actor);
        if (!e.hasAttached)
        {
            if (!e.armor)
                return;
            PhysicsInput input;
            RenameStringMap renameMap;
            if (std::string physicsPath = GetPhysicsInputPath(e.armor); !physicsPath.empty())
                input = GetPhysicsInput(physicsPath);
            else if (std::string smpConfigPath = GetSMPConfigPath(e.armor); !smpConfigPath.empty())
                input = ConvertSMPConfig(smpConfigPath);
            if (!IsHDTSMPEnabled)
                MergeArmorNodeTree(e.skeleton, e.armor, GetArmorCloneNodePrefix(e.bipedSlot), renameMap);
            FixBoneName(input, renameMap);

            std::unique_lock ul(objectDatasLock);
            const ObjectDataPtr objData = GetOrCreateObjectDataPtr_unsafe(e.actor);
            if (!objData)
                return;
            std::lock_guard lg(objData->lock);
            ul.unlock();
            objData->renameMap.insert(renameMap.begin(), renameMap.end());
            ObjectData::RawData data = {.rootNode = nullptr, .rootType = XPBDWorld::RootType::cloth, .bipedSlot = e.bipedSlot, .input = input};
            auto it = std::find(objData->rawDatas.begin(), objData->rawDatas.end(), data);
            if (it != objData->rawDatas.end())
                *it = data;
            else
                objData->rawDatas.push_back(data);
        }
        else
        {
            if (!e.attachedNode)
                return;
            std::unique_lock ul(objectDatasLock);
            const ObjectDataPtr objData = GetOrCreateObjectDataPtr_unsafe(e.actor);
            if (!objData)
                return;
            std::lock_guard lg(objData->lock);
            ul.unlock();
            ObjectData::RawData data = {.rootNode = RE::NiPointer(e.attachedNode), .rootType = XPBDWorld::RootType::cloth, .bipedSlot = e.bipedSlot};
            auto it = std::find(objData->rawDatas.begin(), objData->rawDatas.end(), data);
            if (it != objData->rawDatas.end())
                it->rootNode = data.rootNode;
            else
                objData->rawDatas.push_back(data);
        }
    }

    void XPBDWorldSystem::onEvent(const Mus::ArmorDetachEvent& e)
    {
        if (!e.hasDetached)
            return;
        if (!e.actor)
            return;
        ResetIfChanged(e.actor);
        std::vector<std::uint32_t> bipedSlotsToRemove;
        {
            std::unique_lock ul(objectDatasLock);
            const ObjectDataPtr objData = GetOrCreateObjectDataPtr_unsafe(e.actor);
            if (!objData)
                return;
            std::lock_guard lg(objData->lock);
            ul.unlock();
            for (auto it = objData->rawDatas.begin(); it != objData->rawDatas.end();)
            {
                if (it->rootType != XPBDWorld::RootType::cloth)
                {
                    it++;
                    continue;
                }
                if (!it->rootNode || !it->rootNode->parent)
                {
                    bipedSlotsToRemove.push_back(it->bipedSlot);
                    it = objData->rawDatas.erase(it);
                    RemoveRenameMap(objData->renameMap, GetArmorCloneNodePrefix(it->bipedSlot));
                }
                else
                    it++;
            }
            auto skeletonNode = e.actor->loadedData && e.actor->loadedData->data3D ? e.actor->loadedData->data3D->AsNode() : nullptr;
            for (const auto& bipedSlot : bipedSlotsToRemove)
            {
                if (skeletonNode)
                    RemoveMergedNode(skeletonNode, GetArmorCloneNodePrefix(bipedSlot));
                physicsWorld->RemovePhysics(e.actor, XPBDWorld::RootType::cloth, bipedSlot);
            }
        }
    }

    EventResult XPBDWorldSystem::ProcessEvent(const RE::MenuOpenCloseEvent* evn, RE::BSTEventSource<RE::MenuOpenCloseEvent>*)
    {
        if (!evn)
            return EventResult::kContinue;
        if (evn->menuName == "RaceSexMenu")
        {
            if (evn->opening)
            {

            }
            else
            {

            }
        }
        return EventResult::kContinue;
    }
}
