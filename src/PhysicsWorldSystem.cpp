#include "PhysicsWorldSystem.h"

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

    void XPBDWorldSystem::CreateParent(RE::NiNode* rootNode, PhysicsInput& input)
    {
        if (!rootNode)
            return;
        constexpr std::string parConsName = "__PARENT__";
        std::vector<std::pair<std::string, PhysicsInput::Bone>> newPhysicsBones;
        for (auto& cons : input.constraints)
        {
            if (cons.second.anchorBoneNames.empty())
                continue;
            auto found = std::find(cons.second.anchorBoneNames.begin(), cons.second.anchorBoneNames.end(), parConsName);
            if (found == cons.second.anchorBoneNames.end())
                continue;
            auto obj = rootNode->GetObjectByName(cons.first);
            if (!obj || !obj->parent || obj->parent->name.empty())
                continue;
            *found = obj->parent->name.c_str();
            PhysicsInput::Bone newParentBone;
            newParentBone.mass = 0.0f;
            newPhysicsBones.emplace_back(obj->parent->name.c_str(), newParentBone);
        }
        for (auto& cons : input.angularConstraints)
        {
            if (cons.second.anchorBoneNames.empty())
                continue;
            auto found = std::find(cons.second.anchorBoneNames.begin(), cons.second.anchorBoneNames.end(), parConsName);
            if (found == cons.second.anchorBoneNames.end())
                continue;
            auto obj = rootNode->GetObjectByName(cons.first);
            if (!obj || !obj->parent || obj->parent->name.empty())
                continue;
            *found = obj->parent->name.c_str();
            newPhysicsBones.emplace_back(obj->parent->name.c_str(), PhysicsInput::Bone());
        }
        for (const auto& newBone : newPhysicsBones)
        {
            if (input.bones.find(newBone.first) != input.bones.end())
                continue;
            input.bones[newBone.first] = newBone.second;
        }
    }

    void XPBDWorldSystem::CreateOriginal(RE::NiNode* rootNode, PhysicsInput& input)
    {
        if (!rootNode)
            return;
        constexpr std::string orgConsName = "__ORIGINAL__";
        std::vector<std::pair<std::string, PhysicsInput::Bone>> newPhysicsBones;
        for (auto& cons : input.constraints)
        {
            if (cons.second.anchorBoneNames.empty())
                continue;
            auto found = std::find(cons.second.anchorBoneNames.begin(), cons.second.anchorBoneNames.end(), orgConsName);
            if (found == cons.second.anchorBoneNames.end())
                continue;
            auto obj = rootNode->GetObjectByName(cons.first);
            if (!obj || !obj->parent || obj->parent->name.empty())
                continue;
            const std::string particleBoneName = cons.first + orgConsName;
            const std::string parentName = obj->parent->name.c_str();
            *found = particleBoneName;

            PhysicsInput::Bone newParticleBone;
            newParticleBone.mass = 0.0f;
            newParticleBone.parentBoneName = parentName;
            newParticleBone.isParticle = 1;
            newParticleBone.offset = obj->local.translate;
            newPhysicsBones.emplace_back(particleBoneName, newParticleBone);
            newPhysicsBones.emplace_back(parentName, PhysicsInput::Bone());
        }
        for (auto& cons : input.angularConstraints)
        {
            if (cons.second.anchorBoneNames.empty())
                continue;
            auto found = std::find(cons.second.anchorBoneNames.begin(), cons.second.anchorBoneNames.end(), orgConsName);
            if (found == cons.second.anchorBoneNames.end())
                continue;
            auto obj = rootNode->GetObjectByName(cons.first);
            if (!obj || !obj->parent || obj->parent->name.empty())
                continue;
            const std::string particleBoneName = cons.first + orgConsName;
            const std::string parentName = obj->parent->name.c_str();
            *found = particleBoneName;

            PhysicsInput::Bone newParticleBone;
            newParticleBone.mass = 0.0f;
            newParticleBone.parentBoneName = parentName;
            newParticleBone.isParticle = 1;
            newParticleBone.offset = obj->local.translate;
            newPhysicsBones.emplace_back(particleBoneName, newParticleBone);
            newPhysicsBones.emplace_back(parentName, PhysicsInput::Bone());
        }
        for (const auto& newBone : newPhysicsBones)
        {
            if (input.bones.find(newBone.first) != input.bones.end())
                continue;
            input.bones[newBone.first] = newBone.second;
        }
    }

    void XPBDWorldSystem::CreateSoftBody(RE::NiNode* rootNode, PhysicsInput& input)
    {
        if (!rootNode)
            return;
        constexpr std::string softBodyName = "__SOFTBODY__";
        bool isSoftBody = false;
        for (auto& cons : input.constraints)
        {
            if (cons.second.anchorBoneNames.empty())
                continue;
            if (std::find(cons.second.anchorBoneNames.begin(), cons.second.anchorBoneNames.end(), softBodyName) == cons.second.anchorBoneNames.end())
                continue;
            isSoftBody = true;
            break;
        }
        if (!isSoftBody)
            return;

        auto user = rootNode->GetUserData();
        if (!user)
            return;
        const std::string userID = Mus::SetHex(user->formID, false);
        auto geoDatas = GetGeometryData(GetGeometries(rootNode));
        std::vector<std::pair<std::string, PhysicsInput::Bone>> newPhysicsBones;
        for (auto& cons : input.constraints)
        {
            if (cons.second.anchorBoneNames.empty())
                continue;
            for (std::uint8_t anchIdx = 0; anchIdx < ANCHOR_MAX; ++anchIdx)
            {
                if (cons.second.anchorBoneNames.size() <= anchIdx)
                    break;
                if (cons.second.anchorBoneNames[anchIdx] != softBodyName)
                    continue;

                auto found = geoDatas.boneVertexData.find(cons.first);
                if (found == geoDatas.boneVertexData.end() || found->second.empty())
                    continue;
                auto node = rootNode->GetObjectByName(cons.first);
                if (!node || !node->parent || node->parent->name.empty())
                    continue;

                const std::string parentName = node->parent->name.c_str();

                const float* vertData = reinterpret_cast<const float*>(found->second.data());
                std::vector<RE::NiPoint3> vertices;
                std::vector<std::uint32_t> indices;
                GenerateConvexHull(vertData, found->second.size(), vertices, indices, userID + "_" + cons.first);

                std::vector<std::uint32_t> selectedIdx;
                const std::uint8_t requiredAnchors = ANCHOR_MAX - anchIdx;
                const std::uint8_t maxToPick = std::min<std::uint8_t>(requiredAnchors, vertices.size());
                if (maxToPick > 0)
                {
                    RE::NiPoint3 center(0.0f, 0.0f, 0.0f);
                    for (const auto& v : vertices)
                        center += v;
                    center *= (1.0f / vertices.size());

                    std::uint32_t bestV = 0;
                    float maxDistSq = -1.0f;
                    for (std::uint32_t v = 0; v < vertices.size(); ++v)
                    {
                        float dSq = (vertices[v] - center).SqrLength();
                        if (dSq > maxDistSq)
                        {
                            maxDistSq = dSq;
                            bestV = v;
                        }
                    }
                    selectedIdx.push_back(bestV);

                    for (std::uint8_t k = 1; k < maxToPick; ++k)
                    {
                        bestV = 0;
                        maxDistSq = -1.0f;

                        for (std::uint32_t v = 0; v < vertices.size(); ++v)
                        {
                            if (std::find(selectedIdx.begin(), selectedIdx.end(), v) != selectedIdx.end())
                                continue;

                            float minDistToSelected = FLT_MAX;
                            for (std::uint32_t sel : selectedIdx)
                            {
                                float dSq = (vertices[v] - vertices[sel]).SqrLength();
                                if (dSq < minDistToSelected)
                                    minDistToSelected = dSq;
                            }

                            if (minDistToSelected > maxDistSq)
                            {
                                maxDistSq = minDistToSelected;
                                bestV = v;
                            }
                        }
                        selectedIdx.push_back(bestV);
                    }
                }

                cons.second.anchorBoneNames.resize(ANCHOR_MAX);
                cons.second.complianceStretch.resize(ANCHOR_MAX);
                cons.second.complianceSquish.resize(ANCHOR_MAX);
                cons.second.squishLimit.resize(ANCHOR_MAX);
                cons.second.stretchLimit.resize(ANCHOR_MAX);
                cons.second.angularLimit.resize(ANCHOR_MAX);
                cons.second.squishDamping.resize(ANCHOR_MAX);
                cons.second.stretchDamping.resize(ANCHOR_MAX);
                std::vector<std::string> anchorNames(ANCHOR_MAX);
                for (std::uint8_t i = 0; i < anchIdx; ++i)
                {
                    anchorNames[i] = cons.second.anchorBoneNames[i];
                }
                for (std::uint8_t i = anchIdx; i < ANCHOR_MAX; ++i)
                {
                    const std::uint32_t vi = selectedIdx.empty() ? 0 : selectedIdx[std::min<std::size_t>(i - anchIdx, selectedIdx.size() - 1)];
                    const RE::NiPoint3 pSpacePos = node->local.translate + (node->local.rotate * vertices[vi]);
                    std::string particleBoneName = cons.first + softBodyName + std::to_string(i);

                    PhysicsInput::Bone particleBone;
                    particleBone.mass = 0.0f;
                    particleBone.isParticle = 1;
                    particleBone.parentBoneName = parentName;
                    particleBone.offset = pSpacePos;

                    newPhysicsBones.emplace_back(particleBoneName, particleBone);
                    cons.second.anchorBoneNames[i] = particleBoneName;
                    cons.second.complianceStretch[i] = cons.second.complianceStretch[anchIdx];
                    cons.second.complianceSquish[i] = cons.second.complianceSquish[anchIdx];
                    cons.second.squishLimit[i] = cons.second.squishLimit[anchIdx];
                    cons.second.stretchLimit[i] = cons.second.stretchLimit[anchIdx];
                    cons.second.angularLimit[i] = cons.second.angularLimit[anchIdx];
                    cons.second.squishDamping[i] = cons.second.squishDamping[anchIdx];
                    cons.second.stretchDamping[i] = cons.second.stretchDamping[anchIdx];

                    logger::debug("{} : added particle for {} on {}", particleBoneName, cons.first, parentName);
                }
                newPhysicsBones.emplace_back(parentName, PhysicsInput::Bone());
                break;
            }
        }
        for (auto& cons : input.angularConstraints)
        {
            if (cons.second.anchorBoneNames.empty())
                continue;
            for (std::uint8_t anchIdx = 0; anchIdx < ANCHOR_MAX; ++anchIdx)
            {
                if (cons.second.anchorBoneNames.size() <= anchIdx)
                    break;
                if (cons.second.anchorBoneNames[anchIdx] != softBodyName)
                    continue;

                auto found = geoDatas.boneVertexData.find(cons.first);
                if (found == geoDatas.boneVertexData.end() || found->second.empty())
                    continue;
                auto node = rootNode->GetObjectByName(cons.first);
                if (!node || !node->parent || node->parent->name.empty())
                    continue;

                const std::string parentName = node->parent->name.c_str();

                const float* vertData = reinterpret_cast<const float*>(found->second.data());
                std::vector<RE::NiPoint3> vertices;
                std::vector<std::uint32_t> indices;
                GenerateConvexHull(vertData, found->second.size(), vertices, indices, userID + "_" + cons.first);

                std::vector<std::uint32_t> selectedIdx;
                const std::uint8_t requiredAnchors = ANCHOR_MAX - anchIdx;
                const std::uint8_t maxToPick = std::min<std::uint8_t>(requiredAnchors, vertices.size());
                if (maxToPick > 0)
                {
                    RE::NiPoint3 center(0.0f, 0.0f, 0.0f);
                    for (const auto& v : vertices)
                        center += v;
                    center *= (1.0f / vertices.size());

                    std::uint32_t bestV = 0;
                    float maxDistSq = -1.0f;
                    for (std::uint32_t v = 0; v < vertices.size(); ++v)
                    {
                        float dSq = (vertices[v] - center).SqrLength();
                        if (dSq > maxDistSq)
                        {
                            maxDistSq = dSq;
                            bestV = v;
                        }
                    }
                    selectedIdx.push_back(bestV);

                    for (std::uint8_t k = 1; k < maxToPick; ++k)
                    {
                        bestV = 0;
                        maxDistSq = -1.0f;

                        for (std::uint32_t v = 0; v < vertices.size(); ++v)
                        {
                            if (std::find(selectedIdx.begin(), selectedIdx.end(), v) != selectedIdx.end())
                                continue;

                            float minDistToSelected = FLT_MAX;
                            for (std::uint32_t sel : selectedIdx)
                            {
                                float dSq = (vertices[v] - vertices[sel]).SqrLength();
                                if (dSq < minDistToSelected)
                                    minDistToSelected = dSq;
                            }

                            if (minDistToSelected > maxDistSq)
                            {
                                maxDistSq = minDistToSelected;
                                bestV = v;
                            }
                        }
                        selectedIdx.push_back(bestV);
                    }
                }

                cons.second.anchorBoneNames.resize(ANCHOR_MAX);
                cons.second.compliance.resize(ANCHOR_MAX);
                cons.second.limit.resize(ANCHOR_MAX);
                cons.second.damping.resize(ANCHOR_MAX);
                std::vector<std::string> anchorNames(ANCHOR_MAX);
                for (std::uint8_t i = 0; i < anchIdx; ++i)
                {
                    anchorNames[i] = cons.second.anchorBoneNames[i];
                }
                for (std::uint8_t i = anchIdx; i < ANCHOR_MAX; ++i)
                {
                    const std::uint32_t vi = selectedIdx.empty() ? 0 : selectedIdx[std::min<std::size_t>(i - anchIdx, selectedIdx.size() - 1)];
                    const RE::NiPoint3 pSpacePos = node->local.translate + (node->local.rotate * vertices[vi]);
                    std::string particleBoneName = cons.first + softBodyName + std::to_string(i);

                    PhysicsInput::Bone particleBone;
                    particleBone.mass = 0.0f;
                    particleBone.isParticle = 1;
                    particleBone.parentBoneName = parentName;
                    particleBone.offset = pSpacePos;

                    newPhysicsBones.emplace_back(particleBoneName, particleBone);
                    cons.second.anchorBoneNames[i] = particleBoneName;
                    cons.second.compliance[i] = cons.second.compliance[anchIdx];
                    cons.second.limit[i] = cons.second.limit[anchIdx];
                    cons.second.damping[i] = cons.second.damping[anchIdx];

                    logger::debug("{} : added particle for {} on {}", particleBoneName, cons.first, parentName);
                }
                newPhysicsBones.emplace_back(parentName, PhysicsInput::Bone());
                break;
            }
        }
        for (const auto& newBone : newPhysicsBones)
        {
            if (input.bones.find(newBone.first) != input.bones.end())
                continue;
            input.bones[newBone.first] = newBone.second;
        }
    }

    void XPBDWorldSystem::CreateProperties(RE::NiNode* rootNode, PhysicsInput& input)
    {
        if (input.bones.empty() || input.constraints.empty())
            return;
        CreateParent(rootNode, input);
        CreateOriginal(rootNode, input);
        CreateSoftBody(rootNode, input);
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

    PhysicsInput XPBDWorldSystem::GeyPhysicsInput(const std::string& file) const
    {
        PhysicsInput input;
        tinyxml2::XMLDocument doc;
        const auto error = doc.LoadFile(file.c_str());
        switch (error)
        {
        case tinyxml2::XML_SUCCESS:
            break;
        case tinyxml2::XML_ERROR_FILE_NOT_FOUND:
        case tinyxml2::XML_ERROR_FILE_COULD_NOT_BE_OPENED:
        case tinyxml2::XML_ERROR_FILE_READ_ERROR:
            logger::error("{} : Unable to open the file ({})", file, std::to_underlying(error));
            return input;
            break;
        default:
            logger::error("{} : The file's xml format is invalid ({})", file, std::to_underlying(error));
            return input;
            break;
        };

        tinyxml2::XMLElement* root = doc.RootElement();
        return GeyPhysicsInput(root, file);
    }

    PhysicsInput XPBDWorldSystem::GeyPhysicsInput(tinyxml2::XMLElement* root, const std::string& file) const
    {
        PhysicsInput input;
        if (!root)
            return input;
        {
            if (tinyxml2::XMLElement* Bones = root->FirstChildElement("Bones"); Bones)
            {
                for (tinyxml2::XMLElement* Bone = Bones->FirstChildElement("Bone");
                     Bone != nullptr;
                     Bone = Bone->NextSiblingElement("Bone"))
                {
                    const char* name = Bone->Attribute("name");
                    if (!name)
                        continue;
                    MXPBD::PhysicsInput::Bone boneSetting;
                    std::uint32_t isParticle = 0;
                    Bone->QueryUnsignedAttribute("particle", &isParticle);
                    boneSetting.isParticle = isParticle;
                    if (tinyxml2::XMLElement* Physics = Bone->FirstChildElement("Physics"); Physics)
                    {
                        Physics->QueryFloatAttribute("mass", &boneSetting.mass);
                        Physics->QueryFloatAttribute("damping", &boneSetting.damping);
                        Physics->QueryFloatAttribute("inertiaScale", &boneSetting.inertiaScale);
                        Physics->QueryFloatAttribute("rotRatio", &boneSetting.rotRatio);
                        Physics->QueryFloatAttribute("gravity", &boneSetting.gravity);
                        Physics->QueryFloatAttribute("linearRotTorque", &boneSetting.linearRotTorque);
                        logger::debug("{} : bone physics {} - mass {} / damping {} / inertiaScale {} / rotRatio {} / gravity {} / linearRotTorque {}", file, name, boneSetting.mass, boneSetting.damping, boneSetting.inertiaScale, boneSetting.rotRatio, boneSetting.gravity, boneSetting.linearRotTorque);
                    }
                    if (tinyxml2::XMLElement* Offset = Bone->FirstChildElement("Offset"); Offset)
                    {
                        Offset->QueryFloatAttribute("posX", &boneSetting.offset.x);
                        Offset->QueryFloatAttribute("posY", &boneSetting.offset.y);
                        Offset->QueryFloatAttribute("posZ", &boneSetting.offset.z);
                        const char* pTarget = Offset->Attribute("target");
                        if (pTarget && isParticle)
                            boneSetting.parentBoneName = pTarget;
                        logger::debug("{} : bone offset {} - pos {}{}{}{}", file, name, boneSetting.offset, isParticle ? " /" : "", isParticle ? " target " : "", isParticle ? boneSetting.parentBoneName : "");
                    }
                    if (tinyxml2::XMLElement* Collider = Bone->FirstChildElement("Collider"); Collider)
                    {
                        float colComp = boneSetting.colComp;
                        Collider->QueryFloatAttribute("margin", &boneSetting.colMargin);
                        Collider->QueryFloatAttribute("friction", &boneSetting.colFriction);
                        Collider->QueryFloatAttribute("softness", &colComp);
                        logger::debug("{} : bone collider {} - margin {} / friction {} / softness {}", file, name, boneSetting.colMargin, boneSetting.colFriction, colComp);
                        boneSetting.colComp = colComp * COMPLIANCE_SCALE;
                    }
                    if (isParticle && boneSetting.parentBoneName.empty())
                        continue;
                    input.bones.emplace(name, boneSetting);
                }
                for (tinyxml2::XMLElement* NoCollide = Bones->FirstChildElement("NoCollide");
                     NoCollide != nullptr;
                     NoCollide = NoCollide->NextSiblingElement("NoCollide"))
                {
                    const char* aBoneName = NoCollide->Attribute("A");
                    const char* bBoneName = NoCollide->Attribute("B");
                    if (!aBoneName || !bBoneName)
                        continue;
                    input.convexHullColliders.noCollideBones[aBoneName].insert(bBoneName);
                    input.convexHullColliders.noCollideBones[bBoneName].insert(aBoneName);
                    logger::debug("{} : bone no collide {} - {}", file, aBoneName, bBoneName);
                }
            }
        }
        {
            if (tinyxml2::XMLElement* Constraints = root->FirstChildElement("Constraints"); Constraints)
            {
                for (tinyxml2::XMLElement* Linear = Constraints->FirstChildElement("Linear");
                     Linear != nullptr;
                     Linear = Linear->NextSiblingElement("Linear"))
                {
                    const char* target = Linear->Attribute("target");
                    if (!target)
                        continue;
                    auto it = input.bones.find(target);
                    if (it == input.bones.end())
                        continue;
                    MXPBD::PhysicsInput::Constraint cons;
                    std::uint8_t anchorIdx = 0;
                    for (tinyxml2::XMLElement* Anchor = Linear->FirstChildElement("Anchor");
                         Anchor != nullptr;
                         Anchor = Anchor->NextSiblingElement("Anchor"))
                    {
                        if (anchorIdx >= 4)
                            break;
                        const char* name = Anchor->Attribute("name");
                        if (!name)
                            continue;
                        cons.anchorBoneNames.push_back(name);
                        float complianceSquish = 0.0f, complianceStretch = 0.0f, squishLimit = 0.0f, stretchLimit = 0.0f, angularLimit = 0.0f, squishDamping = 0.0f, stretchDamping = 0.0f;
                        if (tinyxml2::XMLElement* Compliance = Anchor->FirstChildElement("Compliance"); Compliance)
                        {
                            Compliance->QueryFloatAttribute("squish", &complianceSquish);
                            Compliance->QueryFloatAttribute("stretch", &complianceStretch);
                        }
                        if (tinyxml2::XMLElement* Limit = Anchor->FirstChildElement("Limit"); Limit)
                        {
                            Limit->QueryFloatAttribute("squish", &squishLimit);
                            Limit->QueryFloatAttribute("stretch", &stretchLimit);
                            Limit->QueryFloatAttribute("angular", &angularLimit);
                        }
                        if (tinyxml2::XMLElement* Damping = Anchor->FirstChildElement("Damping"); Damping)
                        {
                            Damping->QueryFloatAttribute("squish", &squishDamping);
                            Damping->QueryFloatAttribute("stretch", &stretchDamping);
                        }
                        cons.complianceSquish.push_back(complianceSquish * COMPLIANCE_SCALE);
                        cons.complianceStretch.push_back(complianceStretch * COMPLIANCE_SCALE);
                        cons.squishLimit.push_back(squishLimit);
                        cons.stretchLimit.push_back(stretchLimit);
                        cons.angularLimit.push_back(DirectX::XMConvertToRadians(angularLimit));
                        cons.squishDamping.push_back(squishDamping);
                        cons.stretchDamping.push_back(stretchDamping);
                        logger::debug("{} : bone add anchor {}({}|{}) - complianceSquish {} / complianceStretch {} / squishLimit {} / stretchLimit {} / angularLimit {} / squishDamping {} / stretchDamping {}", file, target, name, anchorIdx, complianceSquish, complianceStretch, squishLimit, stretchLimit, angularLimit, squishDamping, stretchDamping);
                        anchorIdx++;
                    }
                    input.constraints.emplace(target, cons);
                }

                for (tinyxml2::XMLElement* Angular = Constraints->FirstChildElement("Angular");
                     Angular != nullptr;
                     Angular = Angular->NextSiblingElement("Angular"))
                {
                    const char* target = Angular->Attribute("target");
                    if (!target)
                        continue;
                    auto it = input.bones.find(target);
                    if (it == input.bones.end())
                        continue;
                    MXPBD::PhysicsInput::AngularConstraint angCons;
                    std::uint8_t anchorIdx = 0;
                    for (tinyxml2::XMLElement* Anchor = Angular->FirstChildElement("Anchor");
                         Anchor != nullptr;
                         Anchor = Anchor->NextSiblingElement("Anchor"))
                    {
                        if (anchorIdx >= 4)
                            break;
                        const char* name = Anchor->Attribute("name");
                        if (!name)
                            continue;
                        angCons.anchorBoneNames.push_back(name);
                        float compliance = 0.0f, limit = 0.0f, damping = 0.0f;
                        if (tinyxml2::XMLElement* Compliance = Anchor->FirstChildElement("Compliance"); Compliance)
                        {
                            Compliance->QueryFloatAttribute("value", &compliance);
                        }
                        if (tinyxml2::XMLElement* Limit = Anchor->FirstChildElement("Limit"); Limit)
                        {
                            Limit->QueryFloatAttribute("value", &limit);
                        }
                        if (tinyxml2::XMLElement* Damping = Anchor->FirstChildElement("Damping"); Damping)
                        {
                            Damping->QueryFloatAttribute("value", &damping);
                        }
                        angCons.compliance.push_back(compliance * COMPLIANCE_SCALE);
                        angCons.limit.push_back(DirectX::XMConvertToRadians(limit));
                        angCons.damping.push_back(damping);
                        logger::debug("{} : bone add anchor {}({}|{}) - compliance {} / limit {} / damping {}", file, target, name, anchorIdx, compliance, limit, damping);
                        anchorIdx++;
                    }
                    it->second.advancedRotation = 1;
                    input.angularConstraints.emplace(target, angCons);
                }
            }
        }
        return input;
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