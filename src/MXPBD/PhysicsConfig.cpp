#include "MXPBD/PhysicsConfig.h"

namespace MXPBD
{
    void CreateParent(RE::NiNode* rootNode, PhysicsInput& input)
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

    void CreateOriginal(RE::NiNode* rootNode, PhysicsInput& input)
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
            *found = particleBoneName;
            std::string_view parentName = obj->parent->name.c_str();
            logger::debug("{} : original angular constraint found {}", cons.first, parentName);

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

    void CreateVolume(RE::NiNode* rootNode, PhysicsInput& input, const std::vector<RawConvexHullData>& a_rawConvexHullDatas)
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
                    continue;

                auto obj = rootNode->GetObjectByName(cons.first);
                if (!obj || !obj->parent || obj->parent->name.empty())
                    continue;

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
                    continue;

                auto obj = rootNode->GetObjectByName(cons.first);
                if (!obj || !obj->parent || obj->parent->name.empty())
                    continue;

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

    void CreateProperties(RE::NiNode* rootNode, PhysicsInput& input, const std::vector<RawConvexHullData>& a_rawConvexHullDatas)
    {
        if (input.bones.empty() || input.constraints.empty())
            return;
        CreateParent(rootNode, input);
        CreateOriginal(rootNode, input);
        CreateVolume(rootNode, input, a_rawConvexHullDatas);
    }

    bool GetPhysicsInput(const std::string& file, PhysicsInput& input)
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

    bool GetPhysicsInput(tinyxml2::XMLElement* root, const std::string& file, PhysicsInput& input)
    {
        if (!root)
        {
            logger::error("{} : Unable to load root", file);
            return false;
        }
        input.infos.push_back({false, file});

        PhysicsInput::Bone defaultBone;
        struct ConstraintData {
            float complianceSquish = 0.0f;
            float complianceStretch = 0.0f;
            float squishLimit = 0.0f;
            float stretchLimit = 0.0f;
            float angularLimit = 0.0f;
            float squishDamping = 0.0f;
            float stretchDamping = 0.0f;
        };
        ConstraintData defaultLinearConstraint;
        struct AngularConstraintData {
            float compliance = 0.0f;
            float limit = 0.0f;
            float damping = 0.0f;
        };
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
                    if (boneDefaultElemName == "physics")
                    {
                        boneDefaultElem->QueryFloatAttribute("mass", &defaultBone.mass);
                        boneDefaultElem->QueryFloatAttribute("damping", &defaultBone.damping);
                        boneDefaultElem->QueryFloatAttribute("inertiaScale", &defaultBone.inertiaScale);
                        boneDefaultElem->QueryFloatAttribute("restitution", &defaultBone.restitution);
                        boneDefaultElem->QueryFloatAttribute("rotationRatio", &defaultBone.rotationRatio);
                        boneDefaultElem->QueryFloatAttribute("gravity", &defaultBone.gravity);
                        boneDefaultElem->QueryFloatAttribute("linearRotTorque", &defaultBone.linearRotTorque);
                    }
                    else if (boneDefaultElemName == "offset")
                    {
                        boneDefaultElem->QueryFloatAttribute("posX", &defaultBone.offset.x);
                        boneDefaultElem->QueryFloatAttribute("posY", &defaultBone.offset.y);
                        boneDefaultElem->QueryFloatAttribute("posZ", &defaultBone.offset.z);
                        const char* pTarget = boneDefaultElem->Attribute("target");
                        if (pTarget)
                        {
                            defaultBone.parentBoneName = pTarget;
                            defaultBone.isParticle = 1;
                        }
                    }
                    else if (boneDefaultElemName == "collider")
                    {
                        float collisionCompliance = defaultBone.collisionCompliance;
                        boneDefaultElem->QueryFloatAttribute("margin", &defaultBone.collisionMargin);
                        boneDefaultElem->QueryFloatAttribute("friction", &defaultBone.collisionFriction);
                        boneDefaultElem->QueryFloatAttribute("rotationBias", &defaultBone.collisionRotationBias);
                        boneDefaultElem->QueryFloatAttribute("softness", &collisionCompliance);
                        defaultBone.collisionCompliance = collisionCompliance;
                    }
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
                    if (boneElemName == "physics")
                    {
                        boneElem->QueryFloatAttribute("mass", &newBone.mass);
                        boneElem->QueryFloatAttribute("damping", &newBone.damping);
                        boneElem->QueryFloatAttribute("inertiaScale", &newBone.inertiaScale);
                        boneElem->QueryFloatAttribute("restitution", &newBone.restitution);
                        boneElem->QueryFloatAttribute("rotationRatio", &newBone.rotationRatio);
                        boneElem->QueryFloatAttribute("gravity", &newBone.gravity);
                        boneElem->QueryFloatAttribute("linearRotTorque", &newBone.linearRotTorque);
                    }
                    else if (boneElemName == "offset")
                    {
                        boneElem->QueryFloatAttribute("posX", &newBone.offset.x);
                        boneElem->QueryFloatAttribute("posY", &newBone.offset.y);
                        boneElem->QueryFloatAttribute("posZ", &newBone.offset.z);
                        const char* pTarget = boneElem->Attribute("target");
                        if (pTarget)
                        {
                            newBone.parentBoneName = pTarget;
                            newBone.isParticle = 1;
                        }
                    }
                    else if (boneElemName == "collider")
                    {
                        float collisionCompliance = newBone.collisionCompliance;
                        boneElem->QueryFloatAttribute("margin", &newBone.collisionMargin);
                        boneElem->QueryFloatAttribute("friction", &newBone.collisionFriction);
                        boneElem->QueryFloatAttribute("rotationBias", &newBone.collisionRotationBias);
                        boneElem->QueryFloatAttribute("softness", &collisionCompliance);
                        newBone.collisionCompliance = collisionCompliance;
                    }
                    boneElem = boneElem->NextSiblingElement();
                }
                logger::debug("{} : bone physics {} - mass {} / damping {} / inertiaScale {} / restitution {} / rotationRatio {} / gravity {} / linearRotTorque {}", file, name, newBone.mass, newBone.damping, newBone.inertiaScale, newBone.restitution, newBone.rotationRatio, newBone.gravity, newBone.linearRotTorque);
                logger::debug("{} : bone offset {} - pos {}{}{}{}", file, name, newBone.offset, newBone.isParticle ? " /" : "", newBone.isParticle ? " target " : "", newBone.isParticle ? newBone.parentBoneName : "");
                logger::debug("{} : bone collider {} - margin {} / friction {} / rotationBias {} / softness {}", file, name, newBone.collisionMargin, newBone.collisionFriction, newBone.collisionRotationBias, newBone.collisionCompliance);
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
                    if (consElemName  == "compliance")
                    {
                        consElem->QueryFloatAttribute("squish", &defaultLinearConstraint.complianceSquish);
                        consElem->QueryFloatAttribute("stretch", &defaultLinearConstraint.complianceStretch);
                    }
                    else if (consElemName == "limit")
                    {
                        consElem->QueryFloatAttribute("squish", &defaultLinearConstraint.squishLimit);
                        consElem->QueryFloatAttribute("stretch", &defaultLinearConstraint.stretchLimit);
                        consElem->QueryFloatAttribute("angular", &defaultLinearConstraint.angularLimit);
                    }
                    else if (consElemName == "damping")
                    {
                        consElem->QueryFloatAttribute("squish", &defaultLinearConstraint.squishDamping);
                        consElem->QueryFloatAttribute("stretch", &defaultLinearConstraint.stretchDamping);
                    }
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
                    if (consElemName == "compliance")
                    {
                        consElem->QueryFloatAttribute("squish", &newCons.complianceSquish);
                        consElem->QueryFloatAttribute("stretch", &newCons.complianceStretch);
                    }
                    else if (consElemName == "limit")
                    {
                        consElem->QueryFloatAttribute("squish", &newCons.squishLimit);
                        consElem->QueryFloatAttribute("stretch", &newCons.stretchLimit);
                        consElem->QueryFloatAttribute("angular", &newCons.angularLimit);
                    }
                    else if (consElemName == "damping")
                    {
                        consElem->QueryFloatAttribute("squish", &newCons.squishDamping);
                        consElem->QueryFloatAttribute("stretch", &newCons.stretchDamping);
                    }
                    consElem = consElem->NextSiblingElement();
                }
                auto& cons = input.constraints[A];
                std::uint8_t anchorIdx = static_cast<std::uint8_t>(cons.anchorBoneNames.size());
                if (anchorIdx >= ANCHOR_MAX)
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
                logger::debug("{} : bone add anchor {}({}|{}) - complianceSquish {} / complianceStretch {} / squishLimit {} / stretchLimit {} / angularLimit {} / squishDamping {} / stretchDamping {}", file, A, B, anchorIdx, newCons.complianceSquish, newCons.complianceStretch, newCons.squishLimit, newCons.stretchLimit, newCons.angularLimit, newCons.squishDamping, newCons.stretchDamping);
            }
            else if (elemName == "angular-constraint-default")
            {
                tinyxml2::XMLElement* consElem = elem->FirstChildElement();
                while (consElem)
                {
                    std::string consElemName = consElem->Name();
                    std::transform(consElemName.begin(), consElemName.end(), consElemName.begin(), ::tolower);
                    if (consElemName == "compliance")
                    {
                        consElem->QueryFloatAttribute("value", &defaultAngularConstraint.compliance);
                    }
                    else if (consElemName == "limit")
                    {
                        consElem->QueryFloatAttribute("value", &defaultAngularConstraint.limit);
                    }
                    else if (consElemName == "damping")
                    {
                        consElem->QueryFloatAttribute("value", &defaultAngularConstraint.damping);
                    }
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
                    if (consElemName == "compliance")
                    {
                        consElem->QueryFloatAttribute("value", &newCons.compliance);
                    }
                    else if (consElemName == "limit")
                    {
                        consElem->QueryFloatAttribute("value", &newCons.limit);
                    }
                    else if (consElemName == "damping")
                    {
                        consElem->QueryFloatAttribute("value", &newCons.damping);
                    }
                    consElem = consElem->NextSiblingElement();
                }
                auto& cons = input.angularConstraints[A];
                std::uint8_t anchorIdx = static_cast<std::uint8_t>(cons.anchorBoneNames.size());
                if (anchorIdx >= ANCHOR_MAX)
                {
                    elem = elem->NextSiblingElement();
                    continue;
                }
                cons.anchorBoneNames.push_back(B);
                cons.compliance.push_back(newCons.compliance * COMPLIANCE_SCALE);
                cons.limit.push_back(DirectX::XMConvertToRadians(newCons.limit));
                cons.damping.push_back(newCons.damping);
                logger::debug("{} : bone add anchor {}({}|{}) - compliance {} / limit {} / damping {}", file, A, B, anchorIdx, newCons.compliance, newCons.limit, newCons.damping);
            }
            elem = elem->NextSiblingElement();
        }
        return true;
    }

    bool ConvertSMPConfig(const std::string& file, PhysicsInput& input)
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
    bool ConvertSMPConfig(tinyxml2::XMLElement* root, const std::string& file, PhysicsInput& input)
    {
        if (!root)
        {
            logger::error("{} : Unable to load root", file);
            return false;
        }
        input.infos.push_back({true, file});

        return true;
    }

    void FixBoneName(PhysicsInput& input, const RenameStringMap& map)
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
}