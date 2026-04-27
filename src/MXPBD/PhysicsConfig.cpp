#include "MXPBD/PhysicsConfig.h"

namespace MXPBD
{
    void PhysicsConfigReader::CreateParent(RE::NiNode* rootNode, PhysicsInput& input) const
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
            std::string_view parentNodeName = obj->parent->name.c_str();
            logger::debug("{} : parent constraint found {}", cons.first, parentNodeName);
            *found = parentNodeName;
            PhysicsInput::Bone newParentBone;
            newParentBone.mass = 0.0f;
            newPhysicsBones.emplace_back(parentNodeName, newParentBone);
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
            std::string_view parentNodeName = obj->parent->name.c_str();
            logger::debug("{} : parent angular constraint found {}", cons.first, parentNodeName);
            *found = parentNodeName;
            newPhysicsBones.emplace_back(parentNodeName, PhysicsInput::Bone());
        }
        for (const auto& newBone : newPhysicsBones)
        {
            if (input.bones.find(newBone.first) != input.bones.end())
                continue;
            input.bones[newBone.first] = newBone.second;
        }
    }

    void PhysicsConfigReader::CreateOriginal(RE::NiNode* rootNode, PhysicsInput& input) const
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
            *found = particleBoneName;
            std::string_view parentName = obj->parent->name.c_str();
            logger::debug("{} : original constraint found {}", cons.first, parentName);

            PhysicsInput::Bone newParticleBone;
            newParticleBone.mass = 0.0f;
            newParticleBone.parentBoneName = parentName;
            newParticleBone.isParticle = 1;
            newParticleBone.offset = obj->local.translate;
            newPhysicsBones.emplace_back(parentName, PhysicsInput::Bone());
            newPhysicsBones.emplace_back(particleBoneName, newParticleBone);
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
            *found = particleBoneName;
            std::string_view parentName = obj->parent->name.c_str();
            logger::debug("{} : original angular constraint found {}", cons.first, parentName);

            PhysicsInput::Bone newParticleBone;
            newParticleBone.mass = 0.0f;
            newParticleBone.parentBoneName = parentName;
            newParticleBone.isParticle = 1;
            newParticleBone.offset = obj->local.translate;
            newPhysicsBones.emplace_back(parentName, PhysicsInput::Bone());
            newPhysicsBones.emplace_back(particleBoneName, newParticleBone);
        }
        for (const auto& newBone : newPhysicsBones)
        {
            if (input.bones.find(newBone.first) != input.bones.end())
                continue;
            input.bones[newBone.first] = newBone.second;
        }
    }

    void PhysicsConfigReader::CreateVolume(RE::NiNode* rootNode, PhysicsInput& input, const std::vector<RawConvexHullData>& a_rawConvexHullDatas) const
    {
        if (!rootNode)
            return;
        constexpr std::string volumeName = "__VOLUME__";
        bool isVolume = false;
        for (auto& cons : input.constraints)
        {
            if (cons.second.anchorBoneNames.empty())
                continue;
            if (std::find(cons.second.anchorBoneNames.begin(), cons.second.anchorBoneNames.end(), volumeName) == cons.second.anchorBoneNames.end())
                continue;
            isVolume = true;
            break;
        }
        if (!isVolume)
            return;

        auto user = rootNode->GetUserData();
        if (!user)
            return;

        std::unordered_map<std::string, ConvexHullDataBatch> convexHullDataBatches;
        {
            std::unordered_map<std::string, PointCloud> pointClouds;
            for (const auto& rawConvexHullData : a_rawConvexHullDatas)
            {
                for (const auto& rawConvexHull : rawConvexHullData.rawConvexHulls)
                {
                    pointClouds[rawConvexHull.boneName].vertices.append_range(rawConvexHull.vertices);
                }
            }
            for (auto& pc : pointClouds)
            {
                RawConvexHull mergedRawConvexHull;
                ConvexHullDataBatch newConvexHullDataBatch;
                GenerateRawConvexHull(pc.second, mergedRawConvexHull);
                GenerateConvexHullBatch(mergedRawConvexHull, newConvexHullDataBatch);
                convexHullDataBatches[pc.first] = newConvexHullDataBatch;
            }
        }

        std::vector<std::pair<std::string, PhysicsInput::Bone>> newPhysicsBones;
        for (auto& cons : input.constraints)
        {
            if (cons.second.anchorBoneNames.empty())
                continue;
            for (std::uint8_t anchIdx = 0; anchIdx < ANCHOR_MAX; ++anchIdx)
            {
                if (cons.second.anchorBoneNames.size() <= anchIdx)
                    break;
                if (cons.second.anchorBoneNames[anchIdx] != volumeName)
                    continue;

                auto chdbIt = convexHullDataBatches.find(cons.first);
                if (chdbIt == convexHullDataBatches.end())
                {
                    logger::error("{:x} : Unable to get mesh shape for {}. so set __ORIGINAL__", user->formID, cons.first);
                    cons.second.anchorBoneNames[anchIdx] = "__ORIGINAL__";
                    continue;
                }

                auto obj = rootNode->GetObjectByName(cons.first);
                if (!obj || !obj->parent || obj->parent->name.empty())
                {
                    logger::error("{:x} : Unable to get parent node for {}", user->formID, cons.first);
                    continue;
                }

                std::string_view parentName = obj->parent->name.c_str();

                std::vector<std::uint32_t> selectedIdx;
                const std::uint8_t requiredAnchors = ANCHOR_MAX - anchIdx;
                const std::uint8_t maxToPick = std::min<std::uint8_t>(requiredAnchors, COL_VERTEX_MAX);
                if (maxToPick > 0)
                {
                    AABB aabb = AABB();
                    for (std::uint8_t v = 0; v < COL_VERTEX_MAX; ++v)
                    {
                        const Vector p = DirectX::XMVectorSet(chdbIt->second.vX[v], chdbIt->second.vY[v], chdbIt->second.vZ[v], 0.0f);
                        const AABB vAABB(p, p);
                        aabb = aabb.Merge(vAABB);
                    }
                    const Vector center = DirectX::XMVectorMultiply(DirectX::XMVectorAdd(aabb.min, aabb.max), vHalf);

                    std::uint32_t bestV = 0;
                    Vector maxDistSqV = DirectX::XMVectorReplicate(-1.0f);
                    for (std::uint32_t v = 0; v < COL_VERTEX_MAX; ++v)
                    {
                        const Vector p = DirectX::XMVectorSet(chdbIt->second.vX[v], chdbIt->second.vY[v], chdbIt->second.vZ[v], 0.0f);
                        const Vector dSq = DirectX::XMVector3LengthSq(DirectX::XMVectorSubtract(p, center));
                        if (DirectX::XMVector3Less(maxDistSqV, dSq))
                        {
                            maxDistSqV = dSq;
                            bestV = v;
                        }
                    }
                    selectedIdx.push_back(bestV);

                    for (std::uint8_t k = 1; k < maxToPick; ++k)
                    {
                        bestV = 0;
                        float maxDistSq = -1.0f;

                        for (std::uint32_t v = 0; v < COL_VERTEX_MAX; ++v)
                        {
                            if (std::find(selectedIdx.begin(), selectedIdx.end(), v) != selectedIdx.end())
                                continue;

                            float minDistToSelected = FLT_MAX;
                            for (std::uint32_t sel : selectedIdx)
                            {
                                float dSq = (RE::NiPoint3(chdbIt->second.vX[v], chdbIt->second.vY[v], chdbIt->second.vZ[v]) - RE::NiPoint3(chdbIt->second.vX[sel], chdbIt->second.vY[sel], chdbIt->second.vZ[sel])).SqrLength();
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
                    const RE::NiPoint3 pSpacePos = obj->local.translate + (obj->local.rotate * RE::NiPoint3(chdbIt->second.vX[vi], chdbIt->second.vY[vi], chdbIt->second.vZ[vi]));
                    std::string particleBoneName = cons.first + volumeName + std::to_string(i);

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
                if (cons.second.anchorBoneNames[anchIdx] != volumeName)
                    continue;

                auto chdbIt = convexHullDataBatches.find(cons.first);
                if (chdbIt == convexHullDataBatches.end())
                {
                    logger::error("{:x} : Unable to get mesh shape for {}. so set __ORIGINAL__", user->formID, cons.first);
                    cons.second.anchorBoneNames[anchIdx] = "__ORIGINAL__";
                    continue;
                }

                auto obj = rootNode->GetObjectByName(cons.first);
                if (!obj || !obj->parent || obj->parent->name.empty())
                {
                    logger::error("{:x} : Unable to get parent node for {}", user->formID, cons.first);
                    continue;
                }

                std::string_view parentName = obj->parent->name.c_str();

                std::vector<std::uint32_t> selectedIdx;
                const std::uint8_t requiredAnchors = ANCHOR_MAX - anchIdx;
                const std::uint8_t maxToPick = std::min<std::uint8_t>(requiredAnchors, COL_VERTEX_MAX);
                if (maxToPick > 0)
                {
                    AABB aabb = AABB();
                    for (std::uint8_t v = 0; v < COL_VERTEX_MAX; ++v)
                    {
                        const Vector p = DirectX::XMVectorSet(chdbIt->second.vX[v], chdbIt->second.vY[v], chdbIt->second.vZ[v], 0.0f);
                        const AABB vAABB(p, p);
                        aabb = aabb.Merge(vAABB);
                    }
                    const Vector center = DirectX::XMVectorMultiply(DirectX::XMVectorAdd(aabb.min, aabb.max), vHalf);

                    std::uint32_t bestV = 0;
                    Vector maxDistSq = vNegOne;
                    for (std::uint32_t v = 0; v < COL_VERTEX_MAX; ++v)
                    {
                        const Vector p = DirectX::XMVectorSet(chdbIt->second.vX[v], chdbIt->second.vY[v], chdbIt->second.vZ[v], 0.0f);
                        const Vector dSq = DirectX::XMVector3LengthSq(DirectX::XMVectorSubtract(p, center));
                        if (DirectX::XMVector3Less(maxDistSq, dSq))
                        {
                            maxDistSq = dSq;
                            bestV = v;
                        }
                    }
                    selectedIdx.push_back(bestV);

                    for (std::uint8_t k = 1; k < maxToPick; ++k)
                    {
                        bestV = 0;
                        maxDistSq = vNegOne;

                        for (std::uint32_t v = 0; v < COL_VERTEX_MAX; ++v)
                        {
                            if (std::find(selectedIdx.begin(), selectedIdx.end(), v) != selectedIdx.end())
                                continue;

                            Vector minDistToSelected = vInf;
                            for (std::uint32_t sel : selectedIdx)
                            {
                                const Vector av = DirectX::XMVectorSet(chdbIt->second.vX[v], chdbIt->second.vY[v], chdbIt->second.vZ[v], 0.0f);
                                const Vector bv = DirectX::XMVectorSet(chdbIt->second.vX[sel], chdbIt->second.vY[sel], chdbIt->second.vZ[sel], 0.0f);
                                const Vector dSq = DirectX::XMVector3LengthSq(DirectX::XMVectorSubtract(av, bv));
                                if (DirectX::XMVector3Less(dSq, minDistToSelected))
                                    minDistToSelected = dSq;
                            }

                            if (DirectX::XMVector3Less(maxDistSq, minDistToSelected))
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
                    const RE::NiPoint3 pSpacePos = obj->local.translate + (obj->local.rotate * RE::NiPoint3(chdbIt->second.vX[vi], chdbIt->second.vY[vi], chdbIt->second.vZ[vi]));
                    std::string particleBoneName = cons.first + volumeName + std::to_string(i);

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

    void PhysicsConfigReader::CreateProperties(RE::NiNode* rootNode, PhysicsInput& input, const std::vector<RawConvexHullData>& a_rawConvexHullDatas) const
    {
        if (input.bones.empty() || input.constraints.empty())
            return;
        CreateParent(rootNode, input);
        CreateVolume(rootNode, input, a_rawConvexHullDatas);
        CreateOriginal(rootNode, input);
    }

    bool PhysicsConfigReader::GetPhysicsInput(const std::string& file, PhysicsInput& input) const
    {
        tinyxml2::XMLDocument doc;
        std::string lfile = file;
        std::transform(lfile.begin(), lfile.end(), lfile.begin(), ::tolower);
        if (!lfile.starts_with("data\\"))
            lfile = "data\\" + lfile;
        const auto error = doc.LoadFile(lfile.c_str());
        switch (error)
        {
        case tinyxml2::XML_SUCCESS:
            break;
        case tinyxml2::XML_ERROR_FILE_NOT_FOUND:
        case tinyxml2::XML_ERROR_FILE_COULD_NOT_BE_OPENED:
        case tinyxml2::XML_ERROR_FILE_READ_ERROR:
            logger::error("{} : Unable to open the file ({})", file, std::to_underlying(error));
            return false;
            break;
        default:
            logger::error("{} : The file's xml format is invalid ({})", file, std::to_underlying(error));
            return false;
            break;
        };

        tinyxml2::XMLElement* root = doc.RootElement();
        return GetPhysicsInput(root, file, input);
    }

    bool PhysicsConfigReader::GetPhysicsInput(tinyxml2::XMLElement* root, const std::string& file, PhysicsInput& input) const
    {
        if (!root)
        {
            logger::error("{} : Unable to load root", file);
            return false;
        }
        input.infos.push_back({false, file});

        PhysicsInput::Bone defaultBone;
        ConstraintData defaultLinearConstraint;
        AngularConstraintData defaultAngularConstraint;

        tinyxml2::XMLElement* elem = root->FirstChildElement();
        while (elem)
        {
            std::string elemName = elem->Name();
            std::transform(elemName.begin(), elemName.end(), elemName.begin(), ::tolower);
            if (elemName == "bone-default")
            {
                tinyxml2::XMLElement* boneDefaultElem = elem->FirstChildElement();
                while (boneDefaultElem)
                {
                    std::string boneDefaultElemName = boneDefaultElem->Name();
                    std::transform(boneDefaultElemName.begin(), boneDefaultElemName.end(), boneDefaultElemName.begin(), ::tolower);
                    GetBoneData(boneDefaultElemName, boneDefaultElem, defaultBone);
                    boneDefaultElem = boneDefaultElem->NextSiblingElement();
                }
            }
            else if (elemName == "bone")
            {
                const char* cName = elem->Attribute("name");
                if (!cName)
                {
                    elem = elem->NextSiblingElement();
                    continue;
                }
                std::string name = cName;
                if (name.empty())
                {
                    elem = elem->NextSiblingElement();
                    continue;
                }

                PhysicsInput::Bone newBone = defaultBone;
                tinyxml2::XMLElement* boneElem = elem->FirstChildElement();
                while (boneElem)
                {
                    std::string boneElemName = boneElem->Name();
                    std::transform(boneElemName.begin(), boneElemName.end(), boneElemName.begin(), ::tolower);
                    GetBoneData(boneElemName, boneElem, newBone);
                    boneElem = boneElem->NextSiblingElement();
                }
                BoneLogging(file, name, newBone);
                newBone.restPoseCompliance *= COMPLIANCE_SCALE;
                newBone.restPoseAngularLimit = DirectX::XMConvertToRadians(newBone.restPoseAngularLimit);
                newBone.restPoseAngularCompliance *= COMPLIANCE_SCALE;
                newBone.collisionCompliance *= COMPLIANCE_SCALE;
                input.bones.emplace(name, newBone);
            }
            else if (elemName == "no-collide" || elemName == "nocollide")
            {
                const char* aBoneName = elem->Attribute("A");
                const char* bBoneName = elem->Attribute("B");
                if (!aBoneName || !bBoneName)
                {
                    elem = elem->NextSiblingElement();
                    continue;
                }
                input.convexHullColliders.noCollideBones[aBoneName].insert(bBoneName);
                logger::debug("{} : bone no collide {} - {}", file, aBoneName, bBoneName);
            }
            else if (elemName == "linear-constraint-default")
            {
                tinyxml2::XMLElement* consElem = elem->FirstChildElement();
                while (consElem)
                {
                    std::string consElemName = consElem->Name();
                    std::transform(consElemName.begin(), consElemName.end(), consElemName.begin(), ::tolower);
                    GetLinearConstraint(consElemName, consElem, defaultLinearConstraint);
                    consElem = consElem->NextSiblingElement();
                }
            }
            else if (elemName == "linear-constraint")
            {
                const char* A = elem->Attribute("A");
                const char* B = elem->Attribute("B");
                if (!A || !B)
                {
                    elem = elem->NextSiblingElement();
                    continue;
                }
                ConstraintData newCons = defaultLinearConstraint;
                tinyxml2::XMLElement* consElem = elem->FirstChildElement();
                while (consElem)
                {
                    std::string consElemName = consElem->Name();
                    std::transform(consElemName.begin(), consElemName.end(), consElemName.begin(), ::tolower);
                    GetLinearConstraint(consElemName, consElem, newCons);
                    consElem = consElem->NextSiblingElement();
                }
                auto& cons = input.constraints[A];
                std::uint8_t anchIdx = static_cast<std::uint8_t>(cons.anchorBoneNames.size());
                if (anchIdx >= ANCHOR_MAX)
                {
                    elem = elem->NextSiblingElement();
                    continue;
                }
                cons.anchorBoneNames.push_back(B);
                cons.complianceSquish.push_back(newCons.complianceSquish * COMPLIANCE_SCALE);
                cons.complianceStretch.push_back(newCons.complianceStretch * COMPLIANCE_SCALE);
                cons.squishLimit.push_back(newCons.squishLimit);
                cons.stretchLimit.push_back(newCons.stretchLimit);
                cons.angularLimit.push_back(DirectX::XMConvertToRadians(newCons.angularLimit));
                cons.squishDamping.push_back(newCons.squishDamping);
                cons.stretchDamping.push_back(newCons.stretchDamping);
                LinearConstraintLogging(file, A, B, anchIdx, newCons);
            }
            else if (elemName == "angular-constraint-default")
            {
                tinyxml2::XMLElement* consElem = elem->FirstChildElement();
                while (consElem)
                {
                    std::string consElemName = consElem->Name();
                    std::transform(consElemName.begin(), consElemName.end(), consElemName.begin(), ::tolower);
                    GetAngularConstraint(consElemName, consElem, defaultAngularConstraint);
                    consElem = consElem->NextSiblingElement();
                }
            }
            else if (elemName == "angular-constraint")
            {
                const char* A = elem->Attribute("A");
                const char* B = elem->Attribute("B");
                if (!A || !B)
                {
                    elem = elem->NextSiblingElement();
                    continue;
                }
                AngularConstraintData newCons = defaultAngularConstraint;
                tinyxml2::XMLElement* consElem = elem->FirstChildElement();
                while (consElem)
                {
                    std::string consElemName = consElem->Name();
                    std::transform(consElemName.begin(), consElemName.end(), consElemName.begin(), ::tolower);
                    GetAngularConstraint(consElemName, consElem, newCons);
                    consElem = consElem->NextSiblingElement();
                }
                auto& cons = input.angularConstraints[A];
                std::uint8_t anchIdx = static_cast<std::uint8_t>(cons.anchorBoneNames.size());
                if (anchIdx >= ANCHOR_MAX)
                {
                    elem = elem->NextSiblingElement();
                    continue;
                }
                cons.anchorBoneNames.push_back(B);
                cons.compliance.push_back(newCons.compliance * COMPLIANCE_SCALE);
                cons.limit.push_back(DirectX::XMConvertToRadians(newCons.limit));
                cons.damping.push_back(newCons.damping);
                AngularConstraintLogging(file, A, B, anchIdx, newCons);
            }
            elem = elem->NextSiblingElement();
        }
        return true;
    }

    void PhysicsConfigReader::PhysicsConfigReader::SetDefaultSMPConfig(const std::string& file)
    {
        tinyxml2::XMLDocument doc;
        std::string lfile = file;
        std::transform(lfile.begin(), lfile.end(), lfile.begin(), ::tolower);
        if (!lfile.starts_with("data\\"))
            lfile = "data\\" + lfile;
        const auto error = doc.LoadFile(lfile.c_str());
        switch (error)
        {
        case tinyxml2::XML_SUCCESS:
            break;
        case tinyxml2::XML_ERROR_FILE_NOT_FOUND:
        case tinyxml2::XML_ERROR_FILE_COULD_NOT_BE_OPENED:
        case tinyxml2::XML_ERROR_FILE_READ_ERROR:
            logger::error("{} : Unable to open the file ({})", file, std::to_underlying(error));
            return;
            break;
        default:
            logger::error("{} : The file's xml format is invalid ({})", file, std::to_underlying(error));
            return;
            break;
        };

        tinyxml2::XMLElement* root = doc.RootElement();


        tinyxml2::XMLElement* elem = root->FirstChildElement();
        while (elem)
        {
            std::string elemName = elem->Name();
            std::transform(elemName.begin(), elemName.end(), elemName.begin(), ::tolower);
            if (elemName == "bone-default")
            {
                tinyxml2::XMLElement* boneDefaultElem = elem->FirstChildElement();
                while (boneDefaultElem)
                {
                    std::string boneDefaultElemName = boneDefaultElem->Name();
                    std::transform(boneDefaultElemName.begin(), boneDefaultElemName.end(), boneDefaultElemName.begin(), ::tolower);
                    GetBoneData(boneDefaultElemName, boneDefaultElem, defaultSMPBone);
                    boneDefaultElem = boneDefaultElem->NextSiblingElement();
                }
            }
            else if (elemName == "linear-constraint-default")
            {
                tinyxml2::XMLElement* consElem = elem->FirstChildElement();
                while (consElem)
                {
                    std::string consElemName = consElem->Name();
                    std::transform(consElemName.begin(), consElemName.end(), consElemName.begin(), ::tolower);
                    GetLinearConstraint(consElemName, consElem, defaultSMPLinearCons);
                    consElem = consElem->NextSiblingElement();
                }
            }
            else if (elemName == "angular-constraint-default")
            {
                tinyxml2::XMLElement* consElem = elem->FirstChildElement();
                while (consElem)
                {
                    std::string consElemName = consElem->Name();
                    std::transform(consElemName.begin(), consElemName.end(), consElemName.begin(), ::tolower);
                    GetAngularConstraint(consElemName, consElem, defaultSMPAngularCons);
                    consElem = consElem->NextSiblingElement();
                }
            }
            elem = elem->NextSiblingElement();
        }
    }
    bool PhysicsConfigReader::ConvertSMPConfig(const std::string& file, PhysicsInput& input) const
    {
        tinyxml2::XMLDocument doc;
        std::string lfile = file;
        std::transform(lfile.begin(), lfile.end(), lfile.begin(), ::tolower);
        if (!lfile.starts_with("data\\"))
            lfile = "data\\" + lfile;
        const auto error = doc.LoadFile(lfile.c_str());
        switch (error)
        {
        case tinyxml2::XML_SUCCESS:
            break;
        case tinyxml2::XML_ERROR_FILE_NOT_FOUND:
        case tinyxml2::XML_ERROR_FILE_COULD_NOT_BE_OPENED:
        case tinyxml2::XML_ERROR_FILE_READ_ERROR:
            logger::error("{} : Unable to open the file ({})", file, std::to_underlying(error));
            return false;
            break;
        default:
            logger::error("{} : The file's xml format is invalid ({})", file, std::to_underlying(error));
            return false;
            break;
        };

        tinyxml2::XMLElement* root = doc.RootElement();
        return ConvertSMPConfig(root, file, input);
    }
    bool PhysicsConfigReader::ConvertSMPConfig(tinyxml2::XMLElement* root, const std::string& file, PhysicsInput& input) const
    {
        if (!root)
        {
            logger::error("{} : Unable to load root", file);
            return false;
        }
        input.infos.push_back({true, file});

        PhysicsInput::Bone defaultBone = defaultSMPBone;
        defaultBone.mass = 0.0f;
        defaultBone.damping = 0.0f;
        defaultBone.gravity = 1.0f;
        defaultBone.collisionFriction = 0.0f;
        ConstraintData defaultLinearConstraint = defaultSMPLinearCons;
        AngularConstraintData defaultAngularConstraint = defaultSMPAngularCons;

        auto constraintAdd = [&](const char* A, const char* B) {
            {
                auto& cons = input.constraints[A];
                std::uint8_t anchIdx = static_cast<std::uint8_t>(cons.anchorBoneNames.size());
                if (anchIdx < ANCHOR_MAX)
                {
                    ConstraintData newCons = defaultLinearConstraint;
                    cons.anchorBoneNames.push_back(B);
                    cons.complianceSquish.push_back(newCons.complianceSquish * COMPLIANCE_SCALE);
                    cons.complianceStretch.push_back(newCons.complianceStretch * COMPLIANCE_SCALE);
                    cons.squishLimit.push_back(newCons.squishLimit);
                    cons.stretchLimit.push_back(newCons.stretchLimit);
                    cons.angularLimit.push_back(newCons.angularLimit);
                    cons.squishDamping.push_back(newCons.squishDamping);
                    cons.stretchDamping.push_back(newCons.stretchDamping);
                    LinearConstraintLogging(file, A, B, anchIdx, newCons);
                }
            }
            {
                auto& cons = input.angularConstraints[A];
                std::uint8_t anchIdx = static_cast<std::uint8_t>(cons.anchorBoneNames.size());
                if (anchIdx < ANCHOR_MAX)
                {
                    AngularConstraintData newCons = defaultAngularConstraint;
                    cons.anchorBoneNames.push_back(B);
                    cons.compliance.push_back(newCons.compliance * COMPLIANCE_SCALE);
                    cons.limit.push_back(newCons.limit);
                    cons.damping.push_back(newCons.damping);
                    AngularConstraintLogging(file, A, B, anchIdx, newCons);
                }
            }
        };

        tinyxml2::XMLElement* elem = root->FirstChildElement();
        while (elem)
        {
            const std::string elemName = elem->Name();
            if (elemName == "bone-default")
            {
                if (tinyxml2::XMLElement* massElem = elem->FirstChildElement("mass"); massElem)
                {
                    massElem->QueryFloatText(&defaultBone.mass);
                    defaultBone.mass *= defaultSMPBone.mass;
                }
                if (tinyxml2::XMLElement* dampingElem = elem->FirstChildElement("linearDamping"); dampingElem)
                {
                    dampingElem->QueryFloatText(&defaultBone.damping);
                    defaultBone.damping *= defaultSMPBone.damping;
                }
                if (tinyxml2::XMLElement* gravityElem = elem->FirstChildElement("gravity-factor"); gravityElem)
                {
                    gravityElem->QueryFloatText(&defaultBone.gravity);
                    defaultBone.gravity *= defaultSMPBone.gravity;
                }
                if (tinyxml2::XMLElement* frictionElem = elem->FirstChildElement("friction"); frictionElem)
                {
                    frictionElem->QueryFloatText(&defaultBone.collisionFriction);
                    defaultBone.collisionFriction *= defaultSMPBone.collisionFriction;
                }
            }
            else if (elemName == "bone")
            {
                const char* cName = elem->Attribute("name");
                if (!cName)
                {
                    elem = elem->NextSiblingElement();
                    continue;
                }
                std::string name = cName;
                if (name.empty())
                {
                    elem = elem->NextSiblingElement();
                    continue;
                }

                PhysicsInput::Bone newBone = defaultBone;
                BoneLogging(file, name, newBone);
                newBone.restPoseCompliance *= COMPLIANCE_SCALE;
                newBone.restPoseAngularLimit = DirectX::XMConvertToRadians(newBone.restPoseAngularLimit);
                newBone.restPoseAngularCompliance *= COMPLIANCE_SCALE;
                newBone.collisionCompliance *= COMPLIANCE_SCALE;
                input.bones.emplace(name, newBone);
            }
            else if (elemName == "generic-constraint-default")
            {
            }
            else if (elemName == "constraint-group")
            {
                tinyxml2::XMLElement* consElem = elem->FirstChildElement();
                while (consElem)
                {
                    const std::string consElemName = consElem->Name();
                    if (consElemName == "generic-constraint")
                    {
                        const char* A = consElem->Attribute("bodyA");
                        const char* B = consElem->Attribute("bodyB");
                        if (!A || !B)
                        {
                            consElem = consElem->NextSiblingElement();
                            continue;
                        }
                        constraintAdd(A, B);
                    }
                    consElem = consElem->NextSiblingElement();
                }
            }
            else if (elemName == "generic-constraint")
            {
                const char* A = elem->Attribute("bodyA");
                const char* B = elem->Attribute("bodyB");
                if (!A || !B)
                {
                    elem = elem->NextSiblingElement();
                    continue;
                }
                constraintAdd(A, B);
            }
            elem = elem->NextSiblingElement();
        }

        std::unordered_set<std::string> visited;
        auto AddStiffness = [&](auto&& self, const std::string& boneName) -> float {
            if (!visited.insert(boneName).second)
                return 0.0001f;

            auto it = input.constraints.find(boneName);
            if (it == input.constraints.end() || it->second.anchorBoneNames.empty())
            {
                visited.erase(boneName);
                return 0.0f;
            }

            auto& cons = it->second;
            for (std::uint32_t anchorIdx = 0; anchorIdx < cons.anchorBoneNames.size(); ++anchorIdx)
            {
                if (cons.anchorBoneNames[anchorIdx] == "__ORIGINAL__")
                {
                    visited.erase(boneName);
                    return cons.complianceStretch[anchorIdx];
                }
            }

            float parentCompliance = 0.0f;
            for (const std::string& anchorName : cons.anchorBoneNames)
            {
                if (anchorName != "__ORIGINAL__" && anchorName != "__PARENT__" && anchorName != "__VOLUME__")
                {
                    parentCompliance = self(self, anchorName);
                    break;
                }
            }

            if (parentCompliance >= 0.0099f)
            {
                visited.erase(boneName);
                return parentCompliance;
            }

            float currentCompliance = 0.0001f;
            if (parentCompliance > 0.00001f)
                currentCompliance = parentCompliance * 10.0f;
            std::uint32_t anchIdx = static_cast<std::uint32_t>(cons.anchorBoneNames.size());
            if (anchIdx < ANCHOR_MAX)
            {
                cons.anchorBoneNames.push_back("__ORIGINAL__");
                cons.complianceSquish.push_back(0.001f);
                cons.complianceStretch.push_back(currentCompliance);
                cons.squishLimit.push_back(0);
                cons.stretchLimit.push_back(0);
                cons.angularLimit.push_back(0);
                cons.squishDamping.push_back(0);
                cons.stretchDamping.push_back(0);
                logger::debug("{} : bone add anchor {}({}|{}) - complianceStretch {}", file, boneName, "__ORIGINAL__", anchIdx, currentCompliance);
            }

            visited.erase(boneName);
            return currentCompliance;
        };

        for (auto& cons : input.constraints)
        {
            AddStiffness(AddStiffness, cons.first);
        }
        return true;
    }

    void PhysicsConfigReader::FixBoneName(PhysicsInput& input, const RenameStringMap& map) const
    {
        PhysicsInput fixedInput;
        fixedInput.infos = input.infos;
        for (auto& bone : input.bones)
        {
            if (!bone.second.parentBoneName.empty())
            {
                auto it = map.find(bone.second.parentBoneName);
                if (it != map.end())
                {
                    logger::debug("renamed parent {} => {} for physics bone {}", bone.second.parentBoneName, it->second, bone.first);
                    bone.second.parentBoneName = it->second;
                }
            }
            auto it = map.find(bone.first);
            if (it != map.end())
            {
                logger::debug("renamed physics bone {} => {}", bone.first, it->second);
                fixedInput.bones[it->second] = std::move(bone.second);
            }
            else
                fixedInput.bones[bone.first] = std::move(bone.second);
        }
        for (auto& cons : input.constraints)
        {
            for (auto& anch : cons.second.anchorBoneNames)
            {
                auto it = map.find(anch);
                if (it != map.end())
                {
                    logger::debug("renamed anchor {} => {} for constraint {}", anch, it->second, cons.first);
                    anch = it->second;
                }
            }
            auto it = map.find(cons.first);
            if (it != map.end())
            {
                logger::debug("renamed constraint {} => {}", cons.first, it->second);
                fixedInput.constraints[it->second] = std::move(cons.second);
            }
            else
                fixedInput.constraints[cons.first] = std::move(cons.second);
        }
        for (auto& cons : input.angularConstraints)
        {
            for (auto& anch : cons.second.anchorBoneNames)
            {
                auto it = map.find(anch);
                if (it != map.end())
                {
                    logger::debug("renamed anchor {} => {} for constraint {}", anch, it->second, cons.first);
                    anch = it->second;
                }
            }
            auto it = map.find(cons.first);
            if (it != map.end())
            {
                logger::debug("renamed angular constraint {} => {}", cons.first, it->second);
                fixedInput.angularConstraints[it->second] = std::move(cons.second);
            }
            else
                fixedInput.angularConstraints[cons.first] = std::move(cons.second);
        }
        input = std::move(fixedInput);
    }

    
    void PhysicsConfigReader::GetBoneData(const std::string& elemName, tinyxml2::XMLElement* elem, PhysicsInput::Bone& boneData) const
    {
        if (elemName == "physics")
        {
            elem->QueryFloatAttribute("mass", &boneData.mass);
            elem->QueryFloatAttribute("damping", &boneData.damping);
            elem->QueryFloatAttribute("inertiaScale", &boneData.inertiaScale);
            elem->QueryFloatAttribute("restitution", &boneData.restitution);
            elem->QueryFloatAttribute("rotationRatio", &boneData.rotationRatio);
            elem->QueryFloatAttribute("gravity", &boneData.gravity);
            elem->QueryFloatAttribute("linearRotTorque", &boneData.linearRotTorque);
        }
        else if (elemName == "restpose")
        {
            elem->QueryFloatAttribute("limit", &boneData.restPoseLimit);
            elem->QueryFloatAttribute("compliance", &boneData.restPoseCompliance);
            elem->QueryFloatAttribute("angularLimit", &boneData.restPoseLimit);
            elem->QueryFloatAttribute("angularCompliance", &boneData.restPoseCompliance);
        }
        else if (elemName == "offset")
        {
            elem->QueryFloatAttribute("posX", &boneData.offset.x);
            elem->QueryFloatAttribute("posY", &boneData.offset.y);
            elem->QueryFloatAttribute("posZ", &boneData.offset.z);
            const char* pTarget = elem->Attribute("target");
            if (pTarget)
            {
                boneData.parentBoneName = pTarget;
                boneData.isParticle = 1;
            }
        }
        else if (elemName == "collider")
        {
            elem->QueryFloatAttribute("margin", &boneData.collisionMargin);
            elem->QueryFloatAttribute("friction", &boneData.collisionFriction);
            elem->QueryFloatAttribute("rotationBias", &boneData.collisionRotationBias);
            elem->QueryFloatAttribute("softness", &boneData.collisionCompliance);
        }
    };

    void PhysicsConfigReader::GetLinearConstraint(const std::string& elemName, tinyxml2::XMLElement* elem, ConstraintData& consData) const
    {
        if (elemName == "compliance")
        {
            elem->QueryFloatAttribute("squish", &consData.complianceSquish);
            elem->QueryFloatAttribute("stretch", &consData.complianceStretch);
        }
        else if (elemName == "limit")
        {
            elem->QueryFloatAttribute("squish", &consData.squishLimit);
            elem->QueryFloatAttribute("stretch", &consData.stretchLimit);
            elem->QueryFloatAttribute("angular", &consData.angularLimit);
        }
        else if (elemName == "damping")
        {
            elem->QueryFloatAttribute("squish", &consData.squishDamping);
            elem->QueryFloatAttribute("stretch", &consData.stretchDamping);
        }
    };

    void PhysicsConfigReader::GetAngularConstraint(const std::string& elemName, tinyxml2::XMLElement* elem, AngularConstraintData& consData) const
    {
        if (elemName == "compliance")
        {
            elem->QueryFloatAttribute("value", &consData.compliance);
        }
        else if (elemName == "limit")
        {
            elem->QueryFloatAttribute("value", &consData.limit);
        }
        else if (elemName == "damping")
        {
            elem->QueryFloatAttribute("value", &consData.damping);
        }
    };

    void PhysicsConfigReader::BoneLogging(const std::string& file, const std::string& name, const PhysicsInput::Bone& bone) const
    {
        logger::debug("{} : bone physics {} - mass {} / damping {} / inertiaScale {} / restitution {} / rotationRatio {} / gravity {} / linearRotTorque {}", file, name, bone.mass, bone.damping, bone.inertiaScale, bone.restitution, bone.rotationRatio, bone.gravity, bone.linearRotTorque);
        logger::debug("{} : bone offset {} - pos {}{}{}{}", file, name, bone.offset, bone.isParticle ? " /" : "", bone.isParticle ? " target " : "", bone.isParticle ? bone.parentBoneName : "");
        logger::debug("{} : bone restPose {} - limit {} / compliance {} / angularLimit {} / angularCompliance {}", file, name, bone.restPoseLimit, bone.restPoseCompliance, bone.restPoseAngularLimit, bone.restPoseAngularCompliance);
        logger::debug("{} : bone collider {} - margin {} / friction {} / rotationBias {} / softness {}", file, name, bone.collisionMargin, bone.collisionFriction, bone.collisionRotationBias, bone.collisionCompliance);             
    }

    void PhysicsConfigReader::LinearConstraintLogging(const std::string& file, const std::string_view A, const std::string_view B, std::uint32_t anchIdx, const ConstraintData& cons) const
    {
        logger::debug("{} : bone add anchor {}({}|{}) - complianceSquish {} / complianceStretch {} / squishLimit {} / stretchLimit {} / angularLimit {} / squishDamping {} / stretchDamping {}", file, A, B, anchIdx, cons.complianceSquish, cons.complianceStretch, cons.squishLimit, cons.stretchLimit, cons.angularLimit, cons.squishDamping, cons.stretchDamping);         
    }

    void PhysicsConfigReader::AngularConstraintLogging(const std::string& file, const std::string_view A, const std::string_view B, std::uint32_t anchIdx, const AngularConstraintData& cons) const
    {
        logger::debug("{} : bone add anchor {}({}|{}) - compliance {} / limit {} / damping {}", file, A, B, anchIdx, cons.compliance, cons.limit, cons.damping);     
    }
}