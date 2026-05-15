#include "MXPBD/PhysicsConfig.h"

namespace MXPBD
{
    const std::unordered_map<Mus::lString, std::uint32_t>& GetCollisionLayerEnum()
    {
        static bool isInit = false;
        static std::unordered_map<Mus::lString, std::uint32_t> LayerStrings;
        if (isInit)
            return LayerStrings;
        isInit = true;
        LayerStrings["Skeleton"] = CollisionLayer::kSkeleton;
        LayerStrings["Head"] = CollisionLayer::kHead;
        LayerStrings["RigidBody"] = CollisionLayer::kRigidBody;
        LayerStrings["SoftBody"] = CollisionLayer::kSoftBody;
        LayerStrings["Genitals"] = CollisionLayer::kGenitals;
        LayerStrings["Body"] = CollisionLayer::kBody;
        LayerStrings["Hair"] = CollisionLayer::kHair;
        LayerStrings["Cloth"] = CollisionLayer::kCloth;
        LayerStrings["Skirt"] = CollisionLayer::kSkirt;
        LayerStrings["Cape"] = CollisionLayer::kCape;
        LayerStrings["Outfit"] = CollisionLayer::kOutfit;
        LayerStrings["Wing"] = CollisionLayer::kWing;
        LayerStrings["Ears"] = CollisionLayer::kEars;
        LayerStrings["Tail"] = CollisionLayer::kTail;
        LayerStrings["Weapon"] = CollisionLayer::kWeapon;
        LayerStrings["Ground"] = CollisionLayer::kGround;
        LayerStrings["Static"] = CollisionLayer::kStatic;
        LayerStrings["Environment"] = CollisionLayer::kEnvironment;
        LayerStrings["AllLayer"] = CollisionLayer::kAllLayer;

        const std::uint32_t baseIdx = 16;
        for (std::uint32_t i = baseIdx + 1; i <= 31; ++i)
        {
            LayerStrings["Misc" + std::to_string(i- baseIdx)] = 1 << i;
        }
        return LayerStrings;
    }
    std::uint32_t GetStringAsBitMask(const Mus::lString& str)
    {
        const auto& layerStrings = GetCollisionLayerEnum();
        auto it = layerStrings.find(str);
        if (it != layerStrings.end())
        {
            return it->second;
        }
        return 0;
    }

    void PhysicsConfigReader::CreateParentPhysicsBone(RE::NiNode* rootNode, PhysicsInput& input) const
    {
        if (!rootNode)
            return;
        if (input.bones.empty())
            return;

        std::vector<std::pair<std::string, PhysicsInput::Bone>> newPhysicsBones;
        for (auto& bone : input.bones)
        {
            if (bone.second.mass <= Epsilon)
                continue;
            auto obj = rootNode->GetObjectByName(bone.first);
            if (!obj || !obj->parent)
                continue;
            auto parent = obj->parent;
            while (parent->name.empty() && parent->parent)
            {
                parent = parent->parent;
            }
            if (parent->name.empty())
                continue;
            logger::debug("Added {} parent node as physics bone", parent->name.c_str());
            newPhysicsBones.emplace_back(parent->name.c_str(), PhysicsInput::Bone());
        }
        for (const auto& newBone : newPhysicsBones)
        {
            if (input.bones.find(newBone.first) != input.bones.end())
                continue;
            input.bones[newBone.first] = newBone.second;
        }
    }

    void PhysicsConfigReader::CreateParentConstraint(RE::NiNode* rootNode, PhysicsInput& input) const
    {
        if (!rootNode)
            return;
        if (input.distanceConstraints.empty() && input.angularConstraints.empty() && input.deformConstraints.empty())
            return;

        constexpr std::string parConsName = "__PARENT__";
        std::vector<std::pair<std::string, PhysicsInput::Bone>> newPhysicsBones;
        for (auto& cons : input.distanceConstraints)
        {
            if (cons.second.anchors.empty())
                continue;
            auto found = std::find_if(cons.second.anchors.begin(), cons.second.anchors.end(), [&parConsName](const DistanceConstraintData& data) {
                return data.anchorBoneName == parConsName;
            });
            if (found == cons.second.anchors.end())
                continue;
            auto obj = rootNode->GetObjectByName(cons.first);
            if (!obj || !obj->parent)
                continue;
            auto parent = obj->parent;
            while (parent->name.empty() && parent->parent)
            {
                parent = parent->parent;
            }
            if (parent->name.empty())
                continue;
            std::string_view parentNodeName = parent->name.c_str();
            logger::debug("{} : parent distance constraint found {}", cons.first, parentNodeName);
            found->anchorBoneName = parentNodeName;
            PhysicsInput::Bone newParentBone;
            newParentBone.mass = 0.0f;
            newPhysicsBones.emplace_back(parentNodeName, newParentBone);
        }
        for (auto& cons : input.angularConstraints)
        {
            if (cons.second.anchors.empty())
                continue;
            auto found = std::find_if(cons.second.anchors.begin(), cons.second.anchors.end(), [&parConsName](const AngularConstraintData& data) {
                return data.anchorBoneName == parConsName;
            });
            if (found == cons.second.anchors.end())
                continue;
            auto obj = rootNode->GetObjectByName(cons.first);
            if (!obj || !obj->parent)
                continue;
            auto parent = obj->parent;
            while (parent->name.empty() && parent->parent)
            {
                parent = parent->parent;
            }
            if (parent->name.empty())
                continue;
            std::string_view parentNodeName = parent->name.c_str();
            logger::debug("{} : parent angular constraint found {}", cons.first, parentNodeName);
            found->anchorBoneName = parentNodeName;
            newPhysicsBones.emplace_back(parentNodeName, PhysicsInput::Bone());
        }
        for (auto& cons : input.deformConstraints)
        {
            if (cons.second.anchors.empty())
                continue;
            auto found = std::find_if(cons.second.anchors.begin(), cons.second.anchors.end(), [&parConsName](const DeformConstraintData& data) {
                return data.anchorBoneName == parConsName;
            });
            if (found == cons.second.anchors.end())
                continue;
            auto obj = rootNode->GetObjectByName(cons.first);
            if (!obj || !obj->parent)
                continue;
            auto parent = obj->parent;
            while (parent->name.empty() && parent->parent)
            {
                parent = parent->parent;
            }
            if (parent->name.empty())
                continue;
            std::string_view parentNodeName = parent->name.c_str();
            logger::debug("{} : parent deform constraint found {}", cons.first, parentNodeName);
            found->anchorBoneName = parentNodeName;
            newPhysicsBones.emplace_back(parentNodeName, PhysicsInput::Bone());
        }
        for (const auto& newBone : newPhysicsBones)
        {
            if (input.bones.find(newBone.first) != input.bones.end())
                continue;
            input.bones[newBone.first] = newBone.second;
        }
    }

    void PhysicsConfigReader::CreateOriginalConstraint(RE::NiNode* rootNode, PhysicsInput& input) const
    {
        if (!rootNode)
            return;
        if (input.distanceConstraints.empty() && input.angularConstraints.empty() && input.deformConstraints.empty())
            return;

        constexpr std::string orgConsName = "__ORIGINAL__";
        std::vector<std::pair<std::string, PhysicsInput::Bone>> newPhysicsBones;
        for (auto& cons : input.distanceConstraints)
        {
            if (cons.second.anchors.empty())
                continue;
            auto found = std::find_if(cons.second.anchors.begin(), cons.second.anchors.end(), [&orgConsName](const DistanceConstraintData& data) {
                return data.anchorBoneName == orgConsName;
            });
            if (found == cons.second.anchors.end())
                continue;
            auto obj = rootNode->GetObjectByName(cons.first);
            if (!obj || !obj->parent)
                continue;
            auto parent = obj->parent;
            while (parent->name.empty() && parent->parent)
            {
                parent = parent->parent;
            }
            if (parent->name.empty())
                continue;
            std::string_view parentNodeName = parent->name.c_str();
            logger::debug("{} : original distance constraint found {}", cons.first, parentNodeName);

            const RE::NiPoint3 worldDiff = obj->world.translate - parent->world.translate;
            const RE::NiPoint3 localOffset = (parent->world.rotate.Transpose() * worldDiff) * reciprocal(parent->world.scale);
            const std::string particleBoneName = cons.first + orgConsName;
            found->anchorBoneName = particleBoneName;
            PhysicsInput::Bone newParticleBone;
            newParticleBone.mass = 0.0f;
            newParticleBone.parentBoneName = parentNodeName;
            newParticleBone.isParticle = 1;
            newParticleBone.offset = localOffset;
            newPhysicsBones.emplace_back(parentNodeName, PhysicsInput::Bone());
            newPhysicsBones.emplace_back(particleBoneName, newParticleBone);
        }
        for (auto& cons : input.angularConstraints)
        {
            if (cons.second.anchors.empty())
                continue;
            auto found = std::find_if(cons.second.anchors.begin(), cons.second.anchors.end(), [&orgConsName](const AngularConstraintData& data) {
                return data.anchorBoneName == orgConsName;
            });
            if (found == cons.second.anchors.end())
                continue;
            auto obj = rootNode->GetObjectByName(cons.first);
            if (!obj || !obj->parent)
                continue;
            auto parent = obj->parent;
            while (parent->name.empty() && parent->parent)
            {
                parent = parent->parent;
            }
            if (parent->name.empty())
                continue;
            std::string_view parentNodeName = parent->name.c_str();
            logger::debug("{} : original angular constraint found {}", cons.first, parentNodeName);
            const RE::NiPoint3 worldDiff = obj->world.translate - parent->world.translate;
            const RE::NiPoint3 localOffset = (parent->world.rotate.Transpose() * worldDiff) * reciprocal(parent->world.scale);
            const std::string particleBoneName = cons.first + orgConsName;
            found->anchorBoneName = particleBoneName;
            PhysicsInput::Bone newParticleBone;
            newParticleBone.mass = 0.0f;
            newParticleBone.parentBoneName = parentNodeName;
            newParticleBone.isParticle = 1;
            newParticleBone.offset = localOffset;
            newPhysicsBones.emplace_back(parentNodeName, PhysicsInput::Bone());
            newPhysicsBones.emplace_back(particleBoneName, newParticleBone);
        }
        for (auto& cons : input.deformConstraints)
        {
            if (cons.second.anchors.empty())
                continue;
            auto found = std::find_if(cons.second.anchors.begin(), cons.second.anchors.end(), [&orgConsName](const DeformConstraintData& data) {
                return data.anchorBoneName == orgConsName;
            });
            if (found == cons.second.anchors.end())
                continue;
            auto obj = rootNode->GetObjectByName(cons.first);
            if (!obj || !obj->parent)
                continue;
            auto parent = obj->parent;
            while (parent->name.empty() && parent->parent)
            {
                parent = parent->parent;
            }
            if (parent->name.empty())
                continue;
            std::string_view parentNodeName = parent->name.c_str();
            logger::debug("{} : original deform constraint found {}", cons.first, parentNodeName);
            const RE::NiPoint3 worldDiff = obj->world.translate - parent->world.translate;
            const RE::NiPoint3 localOffset = (parent->world.rotate.Transpose() * worldDiff) * reciprocal(parent->world.scale);
            const std::string particleBoneName = cons.first + orgConsName;
            found->anchorBoneName = particleBoneName;
            PhysicsInput::Bone newParticleBone;
            newParticleBone.mass = 0.0f;
            newParticleBone.parentBoneName = parentNodeName;
            newParticleBone.isParticle = 1;
            newParticleBone.offset = localOffset;
            newPhysicsBones.emplace_back(parentNodeName, PhysicsInput::Bone());
            newPhysicsBones.emplace_back(particleBoneName, newParticleBone);
        }
        for (const auto& newBone : newPhysicsBones)
        {
            if (input.bones.find(newBone.first) != input.bones.end())
                continue;
            input.bones[newBone.first] = newBone.second;
        }
    }

    void PhysicsConfigReader::CreateVolumeConstraint(RE::NiNode* rootNode, PhysicsInput& input, const std::vector<RawCollider>& a_mergedConvexHulls) const
    {
        if (!rootNode)
            return;
        if (input.distanceConstraints.empty() && input.angularConstraints.empty() && input.deformConstraints.empty())
            return;

        constexpr std::string volumeName = "__VOLUME__";
        bool isVolume = false;
        for (auto& cons : input.distanceConstraints)
        {
            if (cons.second.anchors.empty())
                continue;
            auto found = std::find_if(cons.second.anchors.begin(), cons.second.anchors.end(), [&volumeName](const DistanceConstraintData& data) {
                return data.anchorBoneName == volumeName;
            });
            if (found == cons.second.anchors.end())
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
        for (auto& mergedHull : a_mergedConvexHulls)
        {
            ConvexHullDataBatch newConvexHullDataBatch;
            GenerateConvexHullBatch(mergedHull, newConvexHullDataBatch);
            convexHullDataBatches[mergedHull.boneName] = newConvexHullDataBatch;
        }

        std::vector<std::pair<std::string, PhysicsInput::Bone>> newPhysicsBones;
        for (auto& cons : input.distanceConstraints)
        {
            if (cons.second.anchors.empty())
                continue;
            for (std::uint8_t anchIdx = 0; anchIdx < ANCHOR_MAX; ++anchIdx)
            {
                if (cons.second.anchors.size() <= anchIdx)
                    break;
                if (cons.second.anchors[anchIdx].anchorBoneName != volumeName)
                    continue;

                auto chdbIt = convexHullDataBatches.find(cons.first);
                if (chdbIt == convexHullDataBatches.end())
                {
                    logger::error("{:x} : Unable to get mesh shape for {}. so set __ORIGINAL__", user->formID, cons.first);
                    cons.second.anchors[anchIdx].anchorBoneName = "__ORIGINAL__";
                    continue;
                }

                auto obj = rootNode->GetObjectByName(cons.first);
                if (!obj || !obj->parent)
                    continue;
                auto parent = obj->parent;
                while (parent->name.empty() && parent->parent)
                {
                    parent = parent->parent;
                }
                if (parent->name.empty())
                    continue;
                std::string_view parentNodeName = parent->name.c_str();
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

                cons.second.anchors.resize(ANCHOR_MAX);
                std::vector<std::string> anchorNames(ANCHOR_MAX);
                for (std::uint8_t i = 0; i < anchIdx; ++i)
                {
                    anchorNames[i] = cons.second.anchors[i].anchorBoneName;
                }
                for (std::uint8_t i = anchIdx; i < ANCHOR_MAX; ++i)
                {
                    const std::uint32_t vi = selectedIdx.empty() ? 0 : selectedIdx[std::min<std::size_t>(i - anchIdx, selectedIdx.size() - 1)];
                    const RE::NiPoint3 worldPos = obj->world.translate + (obj->world.rotate * RE::NiPoint3(chdbIt->second.vX[vi], chdbIt->second.vY[vi], chdbIt->second.vZ[vi]));
                    const RE::NiPoint3 worldDiff = worldPos - parent->world.translate;
                    const RE::NiPoint3 localOffset = (parent->world.rotate.Transpose() * worldDiff) * reciprocal(parent->world.scale);
                    std::string particleBoneName = cons.first + volumeName + std::to_string(i);

                    PhysicsInput::Bone particleBone;
                    particleBone.mass = 0.0f;
                    particleBone.isParticle = 1;
                    particleBone.parentBoneName = parentNodeName;
                    particleBone.offset = localOffset;

                    newPhysicsBones.emplace_back(particleBoneName, particleBone);
                    cons.second.anchors[i] = cons.second.anchors[anchIdx];
                    cons.second.anchors[i].anchorBoneName = particleBoneName;
                    logger::debug("{} : added particle for {} on {}", particleBoneName, cons.first, parentNodeName);
                }
                newPhysicsBones.emplace_back(parentNodeName, PhysicsInput::Bone());
                break;
            }
        }
        for (auto& cons : input.angularConstraints)
        {
            if (cons.second.anchors.empty())
                continue;
            for (std::uint8_t anchIdx = 0; anchIdx < ANCHOR_MAX; ++anchIdx)
            {
                if (cons.second.anchors.size() <= anchIdx)
                    break;
                if (cons.second.anchors[anchIdx].anchorBoneName != volumeName)
                    continue;

                auto chdbIt = convexHullDataBatches.find(cons.first);
                if (chdbIt == convexHullDataBatches.end())
                {
                    logger::error("{:x} : Unable to get mesh shape for {}. so set __ORIGINAL__", user->formID, cons.first);
                    cons.second.anchors[anchIdx].anchorBoneName = "__ORIGINAL__";
                    continue;
                }

                auto obj = rootNode->GetObjectByName(cons.first);
                if (!obj || !obj->parent)
                    continue;
                auto parent = obj->parent;
                while (parent->name.empty() && parent->parent)
                {
                    parent = parent->parent;
                }
                if (parent->name.empty())
                    continue;
                std::string_view parentNodeName = parent->name.c_str();
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

                cons.second.anchors.resize(ANCHOR_MAX);
                std::vector<std::string> anchorNames(ANCHOR_MAX);
                for (std::uint8_t i = 0; i < anchIdx; ++i)
                {
                    anchorNames[i] = cons.second.anchors[i].anchorBoneName;
                }
                for (std::uint8_t i = anchIdx; i < ANCHOR_MAX; ++i)
                {
                    const std::uint32_t vi = selectedIdx.empty() ? 0 : selectedIdx[std::min<std::size_t>(i - anchIdx, selectedIdx.size() - 1)];
                    const RE::NiPoint3 worldPos = obj->world.translate + (obj->world.rotate * RE::NiPoint3(chdbIt->second.vX[vi], chdbIt->second.vY[vi], chdbIt->second.vZ[vi]));
                    const RE::NiPoint3 worldDiff = worldPos - parent->world.translate;
                    const RE::NiPoint3 localOffset = (parent->world.rotate.Transpose() * worldDiff) * reciprocal(parent->world.scale);
                    std::string particleBoneName = cons.first + volumeName + std::to_string(i);

                    PhysicsInput::Bone particleBone;
                    particleBone.mass = 0.0f;
                    particleBone.isParticle = 1;
                    particleBone.parentBoneName = parentNodeName;
                    particleBone.offset = localOffset;

                    newPhysicsBones.emplace_back(particleBoneName, particleBone);
                    cons.second.anchors[i] = cons.second.anchors[anchIdx];
                    cons.second.anchors[i].anchorBoneName = particleBoneName;

                    logger::debug("{} : added particle for {} on {}", particleBoneName, cons.first, parentNodeName);
                }
                newPhysicsBones.emplace_back(parentNodeName, PhysicsInput::Bone());
                break;
            }
        }
        for (auto& cons : input.deformConstraints)
        {
            if (cons.second.anchors.empty())
                continue;
            for (std::uint8_t anchIdx = 0; anchIdx < ANCHOR_MAX; ++anchIdx)
            {
                if (cons.second.anchors.size() <= anchIdx)
                    break;
                if (cons.second.anchors[anchIdx].anchorBoneName != volumeName)
                    continue;

                auto chdbIt = convexHullDataBatches.find(cons.first);
                if (chdbIt == convexHullDataBatches.end())
                {
                    logger::error("{:x} : Unable to get mesh shape for {}. so set __ORIGINAL__", user->formID, cons.first);
                    cons.second.anchors[anchIdx].anchorBoneName = "__ORIGINAL__";
                    continue;
                }

                auto obj = rootNode->GetObjectByName(cons.first);
                if (!obj || !obj->parent)
                    continue;
                auto parent = obj->parent;
                while (parent->name.empty() && parent->parent)
                {
                    parent = parent->parent;
                }
                if (parent->name.empty())
                    continue;
                std::string_view parentNodeName = parent->name.c_str();
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

                cons.second.anchors.resize(ANCHOR_MAX);
                std::vector<std::string> anchorNames(ANCHOR_MAX);
                for (std::uint8_t i = 0; i < anchIdx; ++i)
                {
                    anchorNames[i] = cons.second.anchors[i].anchorBoneName;
                }
                for (std::uint8_t i = anchIdx; i < ANCHOR_MAX; ++i)
                {
                    const std::uint32_t vi = selectedIdx.empty() ? 0 : selectedIdx[std::min<std::size_t>(i - anchIdx, selectedIdx.size() - 1)];
                    const RE::NiPoint3 worldPos = obj->world.translate + (obj->world.rotate * RE::NiPoint3(chdbIt->second.vX[vi], chdbIt->second.vY[vi], chdbIt->second.vZ[vi]));
                    const RE::NiPoint3 worldDiff = worldPos - parent->world.translate;
                    const RE::NiPoint3 localOffset = (parent->world.rotate.Transpose() * worldDiff) * reciprocal(parent->world.scale);
                    std::string particleBoneName = cons.first + volumeName + std::to_string(i);

                    PhysicsInput::Bone particleBone;
                    particleBone.mass = 0.0f;
                    particleBone.isParticle = 1;
                    particleBone.parentBoneName = parentNodeName;
                    particleBone.offset = localOffset;

                    newPhysicsBones.emplace_back(particleBoneName, particleBone);
                    cons.second.anchors[i] = cons.second.anchors[anchIdx];
                    cons.second.anchors[i].anchorBoneName = particleBoneName;

                    logger::debug("{} : added particle for {} on {}", particleBoneName, cons.first, parentNodeName);
                }
                newPhysicsBones.emplace_back(parentNodeName, PhysicsInput::Bone());
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

    void PhysicsConfigReader::UpdateVolume(RE::NiNode* rootNode, PhysicsInput& input, const std::vector<RawCollider>& a_mergedConvexHulls) const
    {
        for (auto& bone : input.bones)
        {
            if (bone.second.mass <= Epsilon || !bone.second.enableDynamicVolume)
                continue;
            auto it = std::find_if(a_mergedConvexHulls.begin(), a_mergedConvexHulls.end(), [&](const RawCollider& convexHull) {
                return convexHull.boneName == bone.first;
            });
            if (it == a_mergedConvexHulls.end())
                continue;
            bone.second.currentVolume = GetVolume(*it) * SkyrimWorldScaleVolume * Mus::Config::GetSingleton().GetVolumeMultiplier();
        }
    }

    void PhysicsConfigReader::CreateProperties(RE::NiNode* rootNode, PhysicsInput& input, const std::vector<RawCollider>& a_mergedConvexHulls) const
    {
        // CreateParentPhysicsBone(rootNode, input);
        CreateParentConstraint(rootNode, input);
        CreateVolumeConstraint(rootNode, input, a_mergedConvexHulls);
        CreateOriginalConstraint(rootNode, input);
        UpdateVolume(rootNode, input, a_mergedConvexHulls);
    }
    void PhysicsConfigReader::CreateProperties(RE::NiNode* rootNode, PhysicsInput& input, const std::vector<RawColliderData>& a_rawConvexHullDatas) const
    {
        std::vector<RawCollider> mergedConvexHulls;
        {
            std::unordered_map<std::string, PointCloud> pointClouds;
            for (const auto& rawConvexHullData : a_rawConvexHullDatas)
            {
                for (const auto& rawConvexHull : rawConvexHullData.rawColliders)
                {
                    pointClouds[rawConvexHull.boneName].boneName = rawConvexHull.boneName;
                    pointClouds[rawConvexHull.boneName].vertices.append_range(rawConvexHull.vertices);
                }
            }
            for (auto& pc : pointClouds)
            {
                RawCollider mergedRawConvexHull;
                ConvexHullDataBatch newConvexHullDataBatch;
                GenerateRawConvexHull(pc.second, mergedRawConvexHull);
                mergedConvexHulls.push_back(std::move(mergedRawConvexHull));
            }
        }

        CreateProperties(rootNode, input, mergedConvexHulls);
    }

    void PhysicsConfigReader::AssignDefaultCollisionLayerGroup(const std::uint32_t collisionLayerGroup, PhysicsInput& input)
    {
        for (auto& bone : input.bones)
        {
            if (bone.second.collisionLayerGroup > 0)
                continue;
            bone.second.collisionLayerGroup |= collisionLayerGroup;
            bone.second.collisionCollideLayer &= ~collisionLayerGroup;
        }
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
        std::unordered_map<std::string, PhysicsInput::Bone> templateBone;
        DistanceConstraintData defaultDistanceConstraint;
        std::unordered_map<std::string, DistanceConstraintData> templateDistanceConstraint;
        AngularConstraintData defaultAngularConstraint;
        std::unordered_map<std::string, AngularConstraintData> templateAngularConstraint;
        DeformConstraintData defaultDeformConstraint;
        std::unordered_map<std::string, DeformConstraintData> templateDeformConstraint;

        tinyxml2::XMLElement* elem = root->FirstChildElement();
        while (elem)
        {
            const Mus::lString elemName = elem->Name();
            if (elemName == "bone-default")
            {
                PhysicsInput::Bone* currentBone = &defaultBone;
                if (const char* templateName = elem->Attribute("name"); templateName)
                {
                    templateBone.emplace(templateName, defaultBone);
                    currentBone = &templateBone[templateName];
                }
                tinyxml2::XMLElement* boneDefaultElem = elem->FirstChildElement();
                while (boneDefaultElem)
                {
                    GetBoneData(boneDefaultElem->Name(), boneDefaultElem, *currentBone);
                    boneDefaultElem = boneDefaultElem->NextSiblingElement();
                }
            }
            else if (elemName == "bone")
            {
                const char* boneName = elem->Attribute("name");
                if (IsEmptyChar(boneName))
                {
                    elem = elem->NextSiblingElement();
                    continue;
                }
                PhysicsInput::Bone defaultBoneCopy = defaultBone;
                if (const char* templateName = elem->Attribute("template"); templateName)
                {
                    if (auto tit = templateBone.find(templateName); tit != templateBone.end())
                    {
                        defaultBoneCopy = tit->second;
                    }
                }
                PhysicsInput::Bone* newBone = &defaultBoneCopy;
                tinyxml2::XMLElement* boneElem = elem->FirstChildElement();
                while (boneElem)
                {
                    GetBoneData(boneElem->Name(), boneElem, *newBone);
                    boneElem = boneElem->NextSiblingElement();
                }
                BoneLogging(file, boneName, *newBone);
                newBone->animDriveCompliance *= COMPLIANCE_SCALE;
                newBone->animDriveAngularCompliance *= COMPLIANCE_SCALE;
                newBone->collisionCompliance *= COMPLIANCE_SCALE;
                newBone->angularLimitPositive *= toRadian;
                newBone->angularLimitNegative *= toRadian;
                input.bones.emplace(boneName, *newBone);
            }
            else if (elemName == "nocollide")
            {
                const char* A = elem->Attribute("A");
                const char* B = elem->Attribute("B");
                if (IsEmptyChar(A) || IsEmptyChar(B))
                {
                    elem = elem->NextSiblingElement();
                    continue;
                }
                input.colliders.noCollideBones[A].insert(B);
                logger::debug("{} : bone no collide {} - {}", file, A, B);
            }
            else if (elemName == "distance-constraint-default")
            {
                DistanceConstraintData* currentCons = &defaultDistanceConstraint;
                if (const char* templateName = elem->Attribute("name"); templateName)
                {
                    templateDistanceConstraint.emplace(templateName, defaultDistanceConstraint);
                    currentCons = &templateDistanceConstraint[templateName];
                }
                tinyxml2::XMLElement* consElem = elem->FirstChildElement();
                while (consElem)
                {
                    GetDistanceConstraint(consElem->Name(), consElem, *currentCons);
                    consElem = consElem->NextSiblingElement();
                }
            }
            else if (elemName == "distance-constraint")
            {
                const char* A = elem->Attribute("A");
                const char* B = elem->Attribute("B");
                if (IsEmptyChar(A) || IsEmptyChar(B))
                {
                    elem = elem->NextSiblingElement();
                    continue;
                }
                DistanceConstraintData defaultConsCopy = defaultDistanceConstraint;
                if (const char* templateName = elem->Attribute("template"); templateName)
                {
                    if (auto tit = templateDistanceConstraint.find(templateName); tit != templateDistanceConstraint.end())
                    {
                        defaultConsCopy = tit->second;
                    }
                }
                DistanceConstraintData* newCons = &defaultConsCopy;
                tinyxml2::XMLElement* consElem = elem->FirstChildElement();
                while (consElem)
                {
                    GetDistanceConstraint(consElem->Name(), consElem, *newCons);
                    consElem = consElem->NextSiblingElement();
                }
                auto& cons = input.distanceConstraints[A];
                std::uint8_t anchIdx = static_cast<std::uint8_t>(cons.anchors.size());
                if (anchIdx >= ANCHOR_MAX)
                {
                    elem = elem->NextSiblingElement();
                    continue;
                }
                PhysicsInput::DistanceConstraint::AnchorData newData = {
                    .anchorBoneName = B,
                    .complianceSquish = newCons->complianceSquish * COMPLIANCE_SCALE,
                    .complianceStretch = newCons->complianceStretch * COMPLIANCE_SCALE,
                    .squishMargin = newCons->squishMargin,
                    .stretchMargin = newCons->stretchMargin,
                    .squishDamping = newCons->squishDamping,
                    .stretchDamping = newCons->stretchDamping,
                };
                cons.anchors.push_back(std::move(newData));
                DistanceConstraintLogging(file, A, B, anchIdx, *newCons);
            }
            else if (elemName == "angular-constraint-default")
            {
                AngularConstraintData* currentCons = &defaultAngularConstraint;
                if (const char* templateName = elem->Attribute("name"); templateName)
                {
                    templateAngularConstraint.emplace(templateName, defaultAngularConstraint);
                    currentCons = &templateAngularConstraint[templateName];
                }
                tinyxml2::XMLElement* consElem = elem->FirstChildElement();
                while (consElem)
                {
                    GetAngularConstraint(consElem->Name(), consElem, *currentCons);
                    consElem = consElem->NextSiblingElement();
                }
            }
            else if (elemName == "angular-constraint")
            {
                const char* A = elem->Attribute("A");
                const char* B = elem->Attribute("B");
                if (IsEmptyChar(A) || IsEmptyChar(B))
                {
                    elem = elem->NextSiblingElement();
                    continue;
                }
                AngularConstraintData defaultConsCopy = defaultAngularConstraint;
                if (const char* templateName = elem->Attribute("template"); templateName)
                {
                    if (auto tit = templateAngularConstraint.find(templateName); tit != templateAngularConstraint.end())
                    {
                        defaultConsCopy = tit->second;
                    }
                }
                AngularConstraintData* newCons = &defaultConsCopy;
                tinyxml2::XMLElement* consElem = elem->FirstChildElement();
                while (consElem)
                {
                    GetAngularConstraint(consElem->Name(), consElem, *newCons);
                    consElem = consElem->NextSiblingElement();
                }
                auto& cons = input.angularConstraints[A];
                std::uint8_t anchIdx = static_cast<std::uint8_t>(cons.anchors.size());
                if (anchIdx >= ANCHOR_MAX)
                {
                    elem = elem->NextSiblingElement();
                    continue;
                }
                PhysicsInput::AngularConstraint::AnchorData newData = {
                    .anchorBoneName = B,
                    .compliancePositive = newCons->compliancePositive * COMPLIANCE_SCALE,
                    .complianceNegative = newCons->complianceNegative * COMPLIANCE_SCALE,
                    .marginPositive = newCons->marginPositive * toRadian,
                    .marginNegative = newCons->marginNegative * toRadian,
                    .dampingPositive = newCons->dampingPositive,
                    .dampingNegative = newCons->dampingNegative,
                };
                cons.anchors.push_back(std::move(newData));
                AngularConstraintLogging(file, A, B, anchIdx, *newCons);
            }
            else if (elemName == "deform-constraint-default")
            {
                DeformConstraintData* currentCons = &defaultDeformConstraint;
                if (const char* templateName = elem->Attribute("name"); templateName)
                {
                    templateDeformConstraint.emplace(templateName, defaultDeformConstraint);
                    currentCons = &templateDeformConstraint[templateName];
                }
                tinyxml2::XMLElement* consElem = elem->FirstChildElement();
                while (consElem)
                {
                    GetDeformConstraint(consElem->Name(), consElem, *currentCons);
                    consElem = consElem->NextSiblingElement();
                }
            }
            else if (elemName == "deform-constraint")
            {
                const char* A = elem->Attribute("A");
                const char* B = elem->Attribute("B");
                if (IsEmptyChar(A) || IsEmptyChar(B))
                {
                    elem = elem->NextSiblingElement();
                    continue;
                }
                DeformConstraintData defaultConsCopy = defaultDeformConstraint;
                if (const char* templateName = elem->Attribute("template"); templateName)
                {
                    if (auto tit = templateDeformConstraint.find(templateName); tit != templateDeformConstraint.end())
                    {
                        defaultConsCopy = tit->second;
                    }
                }
                DeformConstraintData* newCons = &defaultConsCopy;
                tinyxml2::XMLElement* consElem = elem->FirstChildElement();
                while (consElem)
                {
                    GetDeformConstraint(consElem->Name(), consElem, *newCons);
                    consElem = consElem->NextSiblingElement();
                }
                auto& cons = input.deformConstraints[A];
                std::uint8_t anchIdx = static_cast<std::uint8_t>(cons.anchors.size());
                if (anchIdx >= ANCHOR_MAX)
                {
                    elem = elem->NextSiblingElement();
                    continue;
                }
                PhysicsInput::DeformConstraint::AnchorData newData = {
                    .anchorBoneName = B,
                    .squishWeight = newCons->squishWeight,
                };
                cons.anchors.push_back(std::move(newData));
                DeformConstraintLogging(file, A, B, anchIdx, *newCons);
            }
            else if (elemName == "generic-constraint-default")
            {
                DistanceConstraintData* currDistCons = &defaultDistanceConstraint;
                if (const char* templateName = elem->Attribute("name"); templateName)
                {
                    templateDistanceConstraint.emplace(templateName, defaultDistanceConstraint);
                    currDistCons = &templateDistanceConstraint[templateName];
                }
                AngularConstraintData* currAngCons = &defaultAngularConstraint;
                if (const char* templateName = elem->Attribute("name"); templateName)
                {
                    templateAngularConstraint.emplace(templateName, defaultAngularConstraint);
                    currAngCons = &templateAngularConstraint[templateName];
                }
                DeformConstraintData* currDeformCons = &defaultDeformConstraint;
                if (const char* templateName = elem->Attribute("name"); templateName)
                {
                    templateDeformConstraint.emplace(templateName, defaultDeformConstraint);
                    currDeformCons = &templateDeformConstraint[templateName];
                }
                GetGenericConstraint(elem, *currDistCons, *currAngCons, *currDeformCons);
            }
            else if (elemName == "generic-constraint")
            {
                const char* A = elem->Attribute("A");
                const char* B = elem->Attribute("B");
                if (IsEmptyChar(A) || IsEmptyChar(B))
                {
                    elem = elem->NextSiblingElement();
                    continue;
                }
                DistanceConstraintData defaultDistConsCopy = defaultDistanceConstraint;
                if (const char* templateName = elem->Attribute("template"); templateName)
                {
                    if (auto tit = templateDistanceConstraint.find(templateName); tit != templateDistanceConstraint.end())
                        defaultDistConsCopy = tit->second;
                }
                DistanceConstraintData* newDistCons = &defaultDistConsCopy;

                AngularConstraintData defaultAngConsCopy = defaultAngularConstraint;
                if (const char* templateName = elem->Attribute("template"); templateName)
                {
                    if (auto tit = templateAngularConstraint.find(templateName); tit != templateAngularConstraint.end())
                        defaultAngConsCopy = tit->second;
                }
                AngularConstraintData* newAngCons = &defaultAngConsCopy;

                DeformConstraintData defaultDeformConsCopy = defaultDeformConstraint;
                if (const char* templateName = elem->Attribute("template"); templateName)
                {
                    if (auto tit = templateDeformConstraint.find(templateName); tit != templateDeformConstraint.end())
                        defaultDeformConsCopy = tit->second;
                }
                DeformConstraintData* newDeformCons = &defaultDeformConsCopy;

                {
                    auto& cons = input.distanceConstraints[A];
                    std::uint8_t anchIdx = static_cast<std::uint8_t>(cons.anchors.size());
                    if (anchIdx >= ANCHOR_MAX)
                    {
                        elem = elem->NextSiblingElement();
                        continue;
                    }
                    PhysicsInput::DistanceConstraint::AnchorData newData = {
                        .anchorBoneName = B,
                        .complianceSquish = newDistCons->complianceSquish * COMPLIANCE_SCALE,
                        .complianceStretch = newDistCons->complianceStretch * COMPLIANCE_SCALE,
                        .squishMargin = newDistCons->squishMargin,
                        .stretchMargin = newDistCons->stretchMargin,
                        .squishDamping = newDistCons->squishDamping,
                        .stretchDamping = newDistCons->stretchDamping,
                    };
                    cons.anchors.push_back(std::move(newData));
                    DistanceConstraintLogging(file, A, B, anchIdx, *newDistCons);
                }
                {
                    auto& cons = input.angularConstraints[A];
                    std::uint8_t anchIdx = static_cast<std::uint8_t>(cons.anchors.size());
                    if (anchIdx >= ANCHOR_MAX)
                    {
                        elem = elem->NextSiblingElement();
                        continue;
                    }
                    PhysicsInput::AngularConstraint::AnchorData newData = {
                        .anchorBoneName = B,
                        .compliancePositive = newAngCons->compliancePositive * COMPLIANCE_SCALE,
                        .complianceNegative = newAngCons->complianceNegative * COMPLIANCE_SCALE,
                        .marginPositive = newAngCons->marginPositive * toRadian,
                        .marginNegative = newAngCons->marginNegative * toRadian,
                        .dampingPositive = newAngCons->dampingPositive,
                        .dampingNegative = newAngCons->dampingNegative,
                    };
                    cons.anchors.push_back(std::move(newData));
                    AngularConstraintLogging(file, A, B, anchIdx, *newAngCons);
                }
                {
                    auto& cons = input.deformConstraints[A];
                    std::uint8_t anchIdx = static_cast<std::uint8_t>(cons.anchors.size());
                    if (anchIdx >= ANCHOR_MAX)
                    {
                        elem = elem->NextSiblingElement();
                        continue;
                    }
                    PhysicsInput::DeformConstraint::AnchorData newData = {
                        .anchorBoneName = B,
                        .squishWeight = newDeformCons->squishWeight,
                    };
                    cons.anchors.push_back(std::move(newData));
                    DeformConstraintLogging(file, A, B, anchIdx, *newDeformCons);
                }
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

        smpMigration = {};

        auto ReadSMPMigration = [](const Mus::lString& rootElemName, tinyxml2::XMLElement* rootElem, SMPMigration& mig) {
            if (rootElemName.empty())
                return;
            if (rootElemName == "StiffnessChainCount")
                rootElem->QueryIntText(&mig.StiffnessChainCount);
            else if (rootElemName == "StiffnessStartCompliance")
                rootElem->QueryFloatText(&mig.StiffnessStartCompliance);
            else if (rootElemName == "StiffnessEndCompliance")
                rootElem->QueryFloatText(&mig.StiffnessEndCompliance);
        };

        tinyxml2::XMLElement* root = doc.RootElement();
        tinyxml2::XMLElement* elem = root->FirstChildElement();
        while (elem)
        {
            const Mus::lString elemName = elem->Name();
            if (elemName == "SMPMigration")
            {
                tinyxml2::XMLElement* migElem = elem->FirstChildElement();
                while (migElem)
                {
                    ReadSMPMigration(migElem->Name(), migElem, smpMigration);
                    migElem = migElem->NextSiblingElement();
                }
            }
            else if (elemName == "bone-default")
            {
                tinyxml2::XMLElement* boneElem = elem->FirstChildElement();
                while (boneElem)
                {
                    GetBoneData(boneElem->Name(), boneElem, smpMigration.defaultSMPBone);
                    boneElem = boneElem->NextSiblingElement();
                }
            }
            else if (elemName == "distance-constraint-default")
            {
                tinyxml2::XMLElement* consElem = elem->FirstChildElement();
                while (consElem)
                {
                    GetDistanceConstraint(consElem->Name(), consElem, smpMigration.defaultSMPDistanceCons);
                    consElem = consElem->NextSiblingElement();
                }
            }
            else if (elemName == "angular-constraint-default")
            {
                tinyxml2::XMLElement* consElem = elem->FirstChildElement();
                while (consElem)
                {
                    GetAngularConstraint(consElem->Name(), consElem, smpMigration.defaultSMPAngularCons);
                    consElem = consElem->NextSiblingElement();
                }
            }
            else if (elemName == "deform-constraint-default")
            {
                tinyxml2::XMLElement* consElem = elem->FirstChildElement();
                while (consElem)
                {
                    GetDeformConstraint(consElem->Name(), consElem, smpMigration.defaultSMPDeformCons);
                    consElem = consElem->NextSiblingElement();
                }
            }
            else if (elemName == "generic-constraint-default")
            {
                GetGenericConstraint(elem, smpMigration.defaultSMPDistanceCons, smpMigration.defaultSMPAngularCons, smpMigration.defaultSMPDeformCons);
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

        PhysicsInput::Bone defaultBone = smpMigration.defaultSMPBone;
        defaultBone.mass = 0.0f;
        /*defaultBone.dampingPositive = pZero;
        defaultBone.dampingNegative = pZero;
        defaultBone.angularDampingPositive = pZero;
        defaultBone.angularDampingNegative = pZero;
        defaultBone.limitPositive = pZero;
        defaultBone.limitNegative = pZero;
        defaultBone.angularLimitNegative = pZero;
        defaultBone.angularLimitNegative = pZero;*/
        defaultBone.gravity = 1.0f;
        defaultBone.collisionFriction = 0.0f;
        std::unordered_map<std::string, PhysicsInput::Bone> templateBoneData;
        DistanceConstraintData defaultDistanceConstraint = smpMigration.defaultSMPDistanceCons;
        AngularConstraintData defaultAngularConstraint = smpMigration.defaultSMPAngularCons;
        DeformConstraintData defaultDeformConstraint = smpMigration.defaultSMPDeformCons;
        PhysicsInput::Bone defaultConstraintBone;
        std::unordered_map<std::string, PhysicsInput::Bone> templateConsBone;

        auto BoneRead = [&](tinyxml2::XMLElement* rootElem, PhysicsInput::Bone& boneData) {

            tinyxml2::XMLElement* elem = rootElem->FirstChildElement();
            while (elem)
            {
                const Mus::lString elemName = elem->Name();
                if (elemName == "mass")
                {
                    elem->QueryFloatText(&boneData.mass);
                    boneData.mass *= smpMigration.defaultSMPBone.mass;
                }
                else if (elemName == "gravity-factor")
                {
                    elem->QueryFloatText(&boneData.gravity);
                    boneData.gravity *= smpMigration.defaultSMPBone.gravity;
                }
                else if (elemName == "friction")
                {
                    elem->QueryFloatText(&boneData.collisionFriction);
                    boneData.collisionFriction *= smpMigration.defaultSMPBone.collisionFriction;
                }
                else if (elemName == "restitution")
                {
                    elem->QueryFloatText(&boneData.collisionRestitution);
                    boneData.collisionRestitution *= smpMigration.defaultSMPBone.collisionRestitution;
                }
                elem = elem->NextSiblingElement();
            }
        };

        auto ConsRead = [&](tinyxml2::XMLElement* rootElem, PhysicsInput::Bone& boneData) {
            tinyxml2::XMLElement* elem = rootElem->FirstChildElement();
            while (elem)
            {
                const Mus::lString elemName = elem->Name();
                if (elemName == "linearLowerLimit")
                {
                    elem->QueryFloatAttribute("x", &boneData.limitNegative.x);
                    elem->QueryFloatAttribute("y", &boneData.limitNegative.y);
                    elem->QueryFloatAttribute("z", &boneData.limitNegative.z);
                    boneData.limitNegative = GetABSPoint3(boneData.limitNegative);
                    boneData.limitNegative.x *= smpMigration.defaultSMPBone.limitNegative.x;
                    boneData.limitNegative.y *= smpMigration.defaultSMPBone.limitNegative.y;
                    boneData.limitNegative.z *= smpMigration.defaultSMPBone.limitNegative.z;
                }
                else if (elemName == "linearUpperLimit")
                {
                    elem->QueryFloatAttribute("x", &boneData.limitPositive.x);
                    elem->QueryFloatAttribute("y", &boneData.limitPositive.y);
                    elem->QueryFloatAttribute("z", &boneData.limitPositive.z);
                    boneData.limitPositive = GetABSPoint3(boneData.limitPositive);
                    boneData.limitPositive.x *= smpMigration.defaultSMPBone.limitPositive.x;
                    boneData.limitPositive.y *= smpMigration.defaultSMPBone.limitPositive.y;
                    boneData.limitPositive.z *= smpMigration.defaultSMPBone.limitPositive.z;
                }
                else if (elemName == "angularLowerLimit")
                {
                    elem->QueryFloatAttribute("x", &boneData.angularLimitNegative.x);
                    elem->QueryFloatAttribute("y", &boneData.angularLimitNegative.y);
                    elem->QueryFloatAttribute("z", &boneData.angularLimitNegative.z);
                    boneData.angularLimitNegative = GetABSPoint3(boneData.angularLimitNegative) * toDegree;
                    boneData.angularLimitNegative.x *= smpMigration.defaultSMPBone.angularLimitNegative.x;
                    boneData.angularLimitNegative.y *= smpMigration.defaultSMPBone.angularLimitNegative.y;
                    boneData.angularLimitNegative.z *= smpMigration.defaultSMPBone.angularLimitNegative.z;
                }
                else if (elemName == "angularUpperLimit")
                {
                    elem->QueryFloatAttribute("x", &boneData.angularLimitPositive.x);
                    elem->QueryFloatAttribute("y", &boneData.angularLimitPositive.y);
                    elem->QueryFloatAttribute("z", &boneData.angularLimitPositive.z);
                    boneData.angularLimitPositive = GetABSPoint3(boneData.angularLimitPositive) * toDegree;
                    boneData.angularLimitPositive.x *= smpMigration.defaultSMPBone.angularLimitPositive.x;
                    boneData.angularLimitPositive.y *= smpMigration.defaultSMPBone.angularLimitPositive.y;
                    boneData.angularLimitPositive.z *= smpMigration.defaultSMPBone.angularLimitPositive.z;
                }
                else if (elemName == "linearDamping")
                {
                    elem->QueryFloatAttribute("x", &boneData.dampingPositive.x);
                    elem->QueryFloatAttribute("y", &boneData.dampingPositive.y);
                    elem->QueryFloatAttribute("z", &boneData.dampingPositive.z);
                    boneData.dampingPositive = GetABSPoint3(boneData.dampingPositive);
                    boneData.dampingPositive.x *= smpMigration.defaultSMPBone.dampingPositive.x;
                    boneData.dampingPositive.y *= smpMigration.defaultSMPBone.dampingPositive.y;
                    boneData.dampingPositive.z *= smpMigration.defaultSMPBone.dampingPositive.z;
                    boneData.dampingNegative = boneData.dampingPositive;
                }
                else if (elemName == "angularDamping")
                {
                    elem->QueryFloatAttribute("x", &boneData.angularDampingPositive.x);
                    elem->QueryFloatAttribute("y", &boneData.angularDampingPositive.y);
                    elem->QueryFloatAttribute("z", &boneData.angularDampingPositive.z);
                    boneData.angularDampingPositive = GetABSPoint3(boneData.angularDampingPositive);
                    boneData.angularDampingPositive.x *= smpMigration.defaultSMPBone.angularDampingPositive.x;
                    boneData.angularDampingPositive.y *= smpMigration.defaultSMPBone.angularDampingPositive.y;
                    boneData.angularDampingPositive.z *= smpMigration.defaultSMPBone.angularDampingPositive.z;
                    boneData.angularDampingNegative = boneData.angularDampingPositive;
                }
                elem = elem->NextSiblingElement();
            }
        };

        auto AssignBoneCons = [&](const char* A, const PhysicsInput::Bone& newConsBone) {
            auto it = input.bones.find(A);
            if (it == input.bones.end())
                return;
            it->second.dampingPositive = newConsBone.dampingPositive;
            it->second.dampingNegative = newConsBone.dampingNegative;
            it->second.angularDampingPositive = newConsBone.angularDampingPositive;
            it->second.angularDampingNegative = newConsBone.angularDampingNegative;
            it->second.limitPositive = newConsBone.limitPositive;
            it->second.limitNegative = newConsBone.limitNegative;
            it->second.angularLimitPositive = newConsBone.angularLimitPositive;
            it->second.angularLimitNegative = newConsBone.angularLimitNegative;
        };

        auto ConstraintAdd = [&](const char* A, const char* B) {
            if (Epsilon < defaultDistanceConstraint.complianceSquish + defaultDistanceConstraint.complianceStretch)
            {
                auto& cons = input.distanceConstraints[A];
                std::uint8_t anchIdx = static_cast<std::uint8_t>(cons.anchors.size());
                if (anchIdx < ANCHOR_MAX)
                {
                    DistanceConstraintData newCons = defaultDistanceConstraint;
                    newCons.anchorBoneName = B;
                    newCons.complianceSquish *= COMPLIANCE_SCALE;
                    newCons.complianceStretch *= COMPLIANCE_SCALE;
                    cons.anchors.push_back(newCons);
                    DistanceConstraintLogging(file, A, B, anchIdx, newCons);
                }
            }
            if (!IsAllZero(defaultAngularConstraint.compliancePositive) && !IsAllZero(defaultAngularConstraint.complianceNegative))
            {
                auto& cons = input.angularConstraints[A];
                std::uint8_t anchIdx = static_cast<std::uint8_t>(cons.anchors.size());
                if (anchIdx < ANCHOR_MAX)
                {
                    AngularConstraintData newCons = defaultAngularConstraint;
                    newCons.anchorBoneName = B;
                    newCons.compliancePositive *= COMPLIANCE_SCALE;
                    newCons.complianceNegative *= COMPLIANCE_SCALE;
                    newCons.marginPositive *= toRadian;
                    newCons.marginNegative *= toRadian;
                    cons.anchors.push_back(newCons);
                    AngularConstraintLogging(file, A, B, anchIdx, newCons);
                }
            }
            if (!IsAllZero(defaultDeformConstraint.bulgeWeight) && !IsAllZero(defaultDeformConstraint.squishWeight) && !IsAllZero(defaultDeformConstraint.stretchWeight))
            {
                auto& cons = input.deformConstraints[A];
                std::uint8_t anchIdx = static_cast<std::uint8_t>(cons.anchors.size());
                if (anchIdx < ANCHOR_MAX)
                {
                    DeformConstraintData newCons = defaultDeformConstraint;
                    newCons.anchorBoneName = B;
                    cons.anchors.push_back(newCons);
                    DeformConstraintLogging(file, A, B, anchIdx, newCons);
                }
            }
        };

        auto SMPConsBoneLogging = [](const std::string& file, std::string_view name, const PhysicsInput::Bone& bone) {
            logger::debug("{} : bone physics {} - linear damping neg{} pos{} / angular damping neg{} pos{} / linear limit neg{} pos{} / angular limit neg{} pos{}",
                          file, name, bone.dampingNegative, bone.dampingPositive, bone.angularDampingNegative, bone.angularDampingPositive, bone.limitNegative, bone.limitPositive, bone.angularLimitNegative, bone.angularLimitPositive);
        };

        tinyxml2::XMLElement* elem = root->FirstChildElement();
        while (elem)
        {
            const Mus::lString elemName = elem->Name();
            if (elemName == "bone-default")
            {
                PhysicsInput::Bone* currentDefaultBone = &defaultBone;
                if (const char* templateName = elem->Attribute("name"); templateName)
                {
                    templateBoneData.emplace(templateName, defaultBone);
                    currentDefaultBone = &templateBoneData[templateName];
                }
                BoneRead(elem, *currentDefaultBone);
            }
            else if (elemName == "bone")
            {
                const char* boneName = elem->Attribute("name");
                if (IsEmptyChar(boneName))
                {
                    elem = elem->NextSiblingElement();
                    continue;
                }
                PhysicsInput::Bone defaultBoneCopy = defaultBone;
                if (const char* templateName = elem->Attribute("template"); templateName)
                {
                    if (auto tit = templateBoneData.find(templateName); tit != templateBoneData.end())
                    {
                        defaultBoneCopy = tit->second;
                    }
                }
                PhysicsInput::Bone* newBone = &defaultBoneCopy;
                BoneRead(elem, *newBone);
                BoneLogging(file, boneName, *newBone);
                newBone->animDriveCompliance *= COMPLIANCE_SCALE;
                newBone->animDriveAngularCompliance *= COMPLIANCE_SCALE;
                newBone->collisionCompliance *= COMPLIANCE_SCALE;
                newBone->angularLimitPositive *= toRadian;
                newBone->angularLimitNegative *= toRadian;
                input.bones.emplace(boneName, *newBone);
            }
            else if (elemName == "generic-constraint-default")
            {
                PhysicsInput::Bone* currentConstraintBone = &defaultConstraintBone;
                if (const char* templateName = elem->Attribute("name"); templateName)
                {
                    templateConsBone.emplace(templateName, defaultConstraintBone);
                    currentConstraintBone = &templateConsBone[templateName];
                }
                //ConsRead(elem, *currentConstraintBone);
            }
            else if (elemName == "constraint-group")
            {
                tinyxml2::XMLElement* consElem = elem->FirstChildElement();
                while (consElem)
                {
                    const Mus::lString consElemName = consElem->Name();
                    if (consElemName == "generic-constraint")
                    {
                        const char* A = consElem->Attribute("bodyA");
                        const char* B = consElem->Attribute("bodyB");
                        if (IsEmptyChar(A) || IsEmptyChar(B))
                        {
                            consElem = consElem->NextSiblingElement();
                            continue;
                        }
                        PhysicsInput::Bone defaultConsCopy = defaultConstraintBone;
                        if (const char* templateName = elem->Attribute("template"); templateName)
                        {
                            if (auto tit = templateConsBone.find(templateName); tit != templateConsBone.end())
                            {
                                defaultConsCopy = tit->second;
                            }
                        }
                        PhysicsInput::Bone* newConsBone = &defaultConsCopy;
                        //ConsRead(elem, *newConsBone);
                        //AssignBoneCons(A, *newConsBone);
                        //SMPConsBoneLogging(file, A, *newConsBone);
                        ConstraintAdd(A, B);
                    }
                    consElem = consElem->NextSiblingElement();
                }
            }
            else if (elemName == "generic-constraint")
            {
                const char* A = elem->Attribute("bodyA");
                const char* B = elem->Attribute("bodyB");
                if (IsEmptyChar(A) || IsEmptyChar(B))
                {
                    elem = elem->NextSiblingElement();
                    continue;
                }
                PhysicsInput::Bone defaultConsCopy = defaultConstraintBone;
                if (const char* templateName = elem->Attribute("template"); templateName)
                {
                    if (auto tit = templateConsBone.find(templateName); tit != templateConsBone.end())
                    {
                        defaultConsCopy = tit->second;
                    }
                }
                PhysicsInput::Bone* newConsBone = &defaultConsCopy;
                //ConsRead(elem, *newConsBone);
                //AssignBoneCons(A, *newConsBone);
                //SMPConsBoneLogging(file, A, *newConsBone);
                ConstraintAdd(A, B);
            }
            elem = elem->NextSiblingElement();
        }

        std::unordered_set<std::string> visited;
        auto AddStiffness = [&](auto&& self, const std::string& boneName) -> std::int32_t {
            if (!visited.insert(boneName).second)
                return -1;

            auto it = input.distanceConstraints.find(boneName);
            if (it == input.distanceConstraints.end() || it->second.anchors.empty())
            {
                visited.erase(boneName);
                return 0;
            }

            auto& cons = it->second;
            std::int32_t parentDepth = -1;
            for (const auto& anchor : cons.anchors)
            {
                if (anchor.anchorBoneName != "__ORIGINAL__" && anchor.anchorBoneName != "__PARENT__" && anchor.anchorBoneName != "__VOLUME__")
                {
                    parentDepth = self(self, anchor.anchorBoneName);
                    break;
                }
            }

            std::int32_t currentDepth = (parentDepth == -1) ? -1 : (parentDepth + 1);
            for (std::uint32_t anchorIdx = 0; anchorIdx < cons.anchors.size(); ++anchorIdx)
            {
                if (cons.anchors[anchorIdx].anchorBoneName == "__ORIGINAL__")
                {
                    visited.erase(boneName);
                    return currentDepth;
                }
            }

            if (1 <= currentDepth && currentDepth <= smpMigration.StiffnessChainCount)
            {
                float t = 0.0f;
                if (smpMigration.StiffnessChainCount > 1)
                    t = static_cast<float>(currentDepth - 1) / static_cast<float>(smpMigration.StiffnessChainCount - 1);
                float currentCompliance = std::lerp(smpMigration.StiffnessStartCompliance, smpMigration.StiffnessEndCompliance, t);
                std::uint32_t anchIdx = static_cast<std::uint32_t>(cons.anchors.size());
                if (anchIdx < ANCHOR_MAX)
                {
                    DistanceConstraintData newConsData = {
                        .anchorBoneName = "__ORIGINAL__",
                        .complianceSquish = 0.001f,
                        .complianceStretch = currentCompliance * COMPLIANCE_SCALE,
                        .squishMargin = 0.0f,
                        .stretchMargin = 0.0f,
                        .squishDamping = 0.0f,
                        .stretchDamping = 0.0f,
                    };
                    cons.anchors.push_back(std::move(newConsData));
                    logger::debug("{} : bone add anchor {}({}|{}) - complianceStretch {}", file, boneName, "__ORIGINAL__", anchIdx, currentCompliance);
                }
            }

            visited.erase(boneName);
            return currentDepth;
        };

        if (smpMigration.StiffnessChainCount > 0)
        {
            for (auto& cons : input.distanceConstraints)
            {
                AddStiffness(AddStiffness, cons.first);
            }
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
        for (auto& cons : input.distanceConstraints)
        {
            for (auto& anch : cons.second.anchors)
            {
                auto it = map.find(anch.anchorBoneName);
                if (it != map.end())
                {
                    logger::debug("renamed anchor {} => {} for constraint {}", anch.anchorBoneName, it->second, cons.first);
                    anch.anchorBoneName = it->second;
                }
            }
            auto it = map.find(cons.first);
            if (it != map.end())
            {
                logger::debug("renamed distance constraint {} => {}", cons.first, it->second);
                fixedInput.distanceConstraints[it->second] = std::move(cons.second);
            }
            else
                fixedInput.distanceConstraints[cons.first] = std::move(cons.second);
        }
        for (auto& cons : input.angularConstraints)
        {
            for (auto& anch : cons.second.anchors)
            {
                auto it = map.find(anch.anchorBoneName);
                if (it != map.end())
                {
                    logger::debug("renamed anchor {} => {} for constraint {}", anch.anchorBoneName, it->second, cons.first);
                    anch.anchorBoneName = it->second;
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
        for (auto& cons : input.deformConstraints)
        {
            for (auto& anch : cons.second.anchors)
            {
                auto it = map.find(anch.anchorBoneName);
                if (it != map.end())
                {
                    logger::debug("renamed anchor {} => {} for constraint {}", anch.anchorBoneName, it->second, cons.first);
                    anch.anchorBoneName = it->second;
                }
            }
            auto it = map.find(cons.first);
            if (it != map.end())
            {
                logger::debug("renamed deform constraint {} => {}", cons.first, it->second);
                fixedInput.deformConstraints[it->second] = std::move(cons.second);
            }
            else
                fixedInput.deformConstraints[cons.first] = std::move(cons.second);
        }
        input = std::move(fixedInput);
    }

    
    void PhysicsConfigReader::GetBoneData(const Mus::lString& rootElemName, tinyxml2::XMLElement* rootElem, PhysicsInput::Bone& boneData) const
    {
        if (rootElemName.empty())
            return;
        if (rootElemName == "physics")
        {
            tinyxml2::XMLElement* elem = rootElem->FirstChildElement();
            while (elem)
            {
                const Mus::lString elemName = elem->Name();
                if (elemName == "mass")
                    elem->QueryFloatText(&boneData.mass);
                else if (elemName == "linearDampingPositive")
                {
                    elem->QueryFloatAttribute("x", &boneData.dampingPositive.x);
                    elem->QueryFloatAttribute("y", &boneData.dampingPositive.y);
                    elem->QueryFloatAttribute("z", &boneData.dampingPositive.z);
                    boneData.dampingPositive = GetABSPoint3(boneData.dampingPositive);
                }
                else if (elemName == "linearDampingNegative")
                {
                    elem->QueryFloatAttribute("x", &boneData.dampingNegative.x);
                    elem->QueryFloatAttribute("y", &boneData.dampingNegative.y);
                    elem->QueryFloatAttribute("z", &boneData.dampingNegative.z);
                    boneData.dampingNegative = GetABSPoint3(boneData.dampingNegative);
                }
                else if (elemName == "angularDampingPositive")
                {
                    elem->QueryFloatAttribute("x", &boneData.angularDampingPositive.x);
                    elem->QueryFloatAttribute("y", &boneData.angularDampingPositive.y);
                    elem->QueryFloatAttribute("z", &boneData.angularDampingPositive.z);
                    boneData.angularDampingPositive = GetABSPoint3(boneData.angularDampingPositive);
                }
                else if (elemName == "angularDampingNegative")
                {
                    elem->QueryFloatAttribute("x", &boneData.angularDampingNegative.x);
                    elem->QueryFloatAttribute("y", &boneData.angularDampingNegative.y);
                    elem->QueryFloatAttribute("z", &boneData.angularDampingNegative.z);
                    boneData.angularDampingNegative = GetABSPoint3(boneData.angularDampingNegative);
                }
                else if (elemName == "linearInertiaPositive")
                {
                    elem->QueryFloatAttribute("x", &boneData.inertiaPositive.x);
                    elem->QueryFloatAttribute("y", &boneData.inertiaPositive.y);
                    elem->QueryFloatAttribute("z", &boneData.inertiaPositive.z);
                    boneData.inertiaPositive = GetABSPoint3(boneData.inertiaPositive);
                }
                else if (elemName == "linearInertiaNegative")
                {
                    elem->QueryFloatAttribute("x", &boneData.inertiaNegative.x);
                    elem->QueryFloatAttribute("y", &boneData.inertiaNegative.y);
                    elem->QueryFloatAttribute("z", &boneData.inertiaNegative.z);
                    boneData.inertiaNegative = GetABSPoint3(boneData.inertiaNegative);
                }
                else if (elemName == "angularInertiaPositive")
                {
                    elem->QueryFloatAttribute("x", &boneData.angularInertiaPositive.x);
                    elem->QueryFloatAttribute("y", &boneData.angularInertiaPositive.y);
                    elem->QueryFloatAttribute("z", &boneData.angularInertiaPositive.z);
                    boneData.angularInertiaPositive = GetABSPoint3(boneData.angularInertiaPositive);
                }
                else if (elemName == "angularInertiaNegative")
                {
                    elem->QueryFloatAttribute("x", &boneData.angularInertiaNegative.x);
                    elem->QueryFloatAttribute("y", &boneData.angularInertiaNegative.y);
                    elem->QueryFloatAttribute("z", &boneData.angularInertiaNegative.z);
                    boneData.angularInertiaNegative = GetABSPoint3(boneData.angularInertiaNegative);
                }
                else if (elemName == "linearInertiaCorrectionPositive")
                {
                    elem->QueryFloatAttribute("x", &boneData.inertiaCorrectionPositive.x);
                    elem->QueryFloatAttribute("y", &boneData.inertiaCorrectionPositive.y);
                    elem->QueryFloatAttribute("z", &boneData.inertiaCorrectionPositive.z);
                    boneData.inertiaCorrectionPositive = GetABSPoint3(boneData.inertiaCorrectionPositive);
                }
                else if (elemName == "linearInertiaCorrectionNegative")
                {
                    elem->QueryFloatAttribute("x", &boneData.inertiaCorrectionNegative.x);
                    elem->QueryFloatAttribute("y", &boneData.inertiaCorrectionNegative.y);
                    elem->QueryFloatAttribute("z", &boneData.inertiaCorrectionNegative.z);
                    boneData.inertiaCorrectionNegative = GetABSPoint3(boneData.inertiaCorrectionNegative);
                }
                else if (elemName == "linearlimitPositive")
                {
                    elem->QueryFloatAttribute("x", &boneData.limitPositive.x);
                    elem->QueryFloatAttribute("y", &boneData.limitPositive.y);
                    elem->QueryFloatAttribute("z", &boneData.limitPositive.z);
                    boneData.limitPositive = GetABSPoint3(boneData.limitPositive);
                }
                else if (elemName == "linearlimitNegative")
                {
                    elem->QueryFloatAttribute("x", &boneData.limitNegative.x);
                    elem->QueryFloatAttribute("y", &boneData.limitNegative.y);
                    elem->QueryFloatAttribute("z", &boneData.limitNegative.z);
                    boneData.limitNegative = GetABSPoint3(boneData.limitNegative);
                }
                else if (elemName == "angularLimitPositive")
                {
                    elem->QueryFloatAttribute("x", &boneData.angularLimitPositive.x);
                    elem->QueryFloatAttribute("y", &boneData.angularLimitPositive.y);
                    elem->QueryFloatAttribute("z", &boneData.angularLimitPositive.z);
                    boneData.angularLimitPositive = GetABSPoint3(boneData.angularLimitPositive);
                }
                else if (elemName == "angularLimitNegative")
                {
                    elem->QueryFloatAttribute("x", &boneData.angularLimitNegative.x);
                    elem->QueryFloatAttribute("y", &boneData.angularLimitNegative.y);
                    elem->QueryFloatAttribute("z", &boneData.angularLimitNegative.z);
                    boneData.angularLimitNegative = GetABSPoint3(boneData.angularLimitNegative);
                }
                else if (elemName == "gravity")
                    elem->QueryFloatText(&boneData.gravity);
                else if (elemName == "angularBlendFactor")
                    elem->QueryFloatText(&boneData.angularBlendFactor);
                else if (elemName == "windFactor")
                {
                    elem->QueryFloatAttribute("x", &boneData.windFactor.x);
                    elem->QueryFloatAttribute("y", &boneData.windFactor.y);
                    elem->QueryFloatAttribute("z", &boneData.windFactor.z);
                }
                else if (elemName == "linearXRotTorque")
                {
                    elem->QueryFloatAttribute("x", &boneData.linearRotTorque[0].x);
                    elem->QueryFloatAttribute("y", &boneData.linearRotTorque[0].y);
                    elem->QueryFloatAttribute("z", &boneData.linearRotTorque[0].z);
                }
                else if (elemName == "linearYRotTorque")
                {
                    elem->QueryFloatAttribute("x", &boneData.linearRotTorque[1].x);
                    elem->QueryFloatAttribute("y", &boneData.linearRotTorque[1].y);
                    elem->QueryFloatAttribute("z", &boneData.linearRotTorque[1].z);
                }
                else if (elemName == "linearZRotTorque")
                {
                    elem->QueryFloatAttribute("x", &boneData.linearRotTorque[2].x);
                    elem->QueryFloatAttribute("y", &boneData.linearRotTorque[2].y);
                    elem->QueryFloatAttribute("z", &boneData.linearRotTorque[2].z);
                }
                elem = elem->NextSiblingElement();
            }
        }
        else if (rootElemName == "animDrive")
        {
            tinyxml2::XMLElement* elem = rootElem->FirstChildElement();
            while (elem)
            {
                const Mus::lString elemName = elem->Name();
                if (elemName == "linearCompliance")
                {
                    elem->QueryFloatAttribute("x", &boneData.animDriveCompliance.x);
                    elem->QueryFloatAttribute("y", &boneData.animDriveCompliance.y);
                    elem->QueryFloatAttribute("z", &boneData.animDriveCompliance.z);
                }
                else if (elemName == "angularCompliance")
                {
                    elem->QueryFloatAttribute("x", &boneData.animDriveAngularCompliance.x);
                    elem->QueryFloatAttribute("y", &boneData.animDriveAngularCompliance.y);
                    elem->QueryFloatAttribute("z", &boneData.animDriveAngularCompliance.z);
                }
                elem = elem->NextSiblingElement();
            }
        }
        else if (rootElemName == "offset")
        {
            rootElem->QueryFloatAttribute("x", &boneData.offset.x);
            rootElem->QueryFloatAttribute("y", &boneData.offset.y);
            rootElem->QueryFloatAttribute("z", &boneData.offset.z);
            const char* pTarget = rootElem->Attribute("target");
            if (!IsEmptyChar(pTarget))
            {
                boneData.parentBoneName = pTarget;
                boneData.isParticle = 1;
            }
        }
        else if (rootElemName == "collider")
        {
            tinyxml2::XMLElement* elem = rootElem->FirstChildElement();
            while (elem)
            {
                const Mus::lString elemName = elem->Name();
                if (elemName == "margin")
                    elem->QueryFloatText(&boneData.collisionMargin);
                else if (elemName == "friction")
                    elem->QueryFloatText(&boneData.collisionFriction);
                else if (elemName == "softness")
                    elem->QueryFloatText(&boneData.collisionCompliance);
                else if (elemName == "restitution")
                    elem->QueryFloatText(&boneData.collisionRestitution);
                else if (elemName == "layerGroup")
                    boneData.collisionLayerGroup |= GetStringAsBitMask(elem->GetText());
                else if (elemName == "ignoreLayer")
                    boneData.collisionCollideLayer &= ~GetStringAsBitMask(elem->GetText());
                else if (elemName == "colliderType")
                    boneData.colliderType = GetColliderType(elem->GetText());
                elem = elem->NextSiblingElement();
            }
        }
        else if (rootElemName == "deformation")
        {
            tinyxml2::XMLElement* elem = rootElem->FirstChildElement();
            while (elem)
            {
                const Mus::lString elemName = elem->Name();
                if (elemName == "limitMin")
                {
                    elem->QueryFloatAttribute("x", &boneData.deformMin.x);
                    elem->QueryFloatAttribute("y", &boneData.deformMin.y);
                    elem->QueryFloatAttribute("z", &boneData.deformMin.z);
                }
                else if (elemName == "limitMax")
                {
                    elem->QueryFloatAttribute("x", &boneData.deformMax.x);
                    elem->QueryFloatAttribute("y", &boneData.deformMax.y);
                    elem->QueryFloatAttribute("z", &boneData.deformMax.z);
                }
                else if (elemName == "volumePreservation")
                {
                    elem->QueryFloatAttribute("x", &boneData.deformVolumePreservation.x);
                    elem->QueryFloatAttribute("y", &boneData.deformVolumePreservation.y);
                    elem->QueryFloatAttribute("z", &boneData.deformVolumePreservation.z);
                }
                else if (elemName == "squishSensitivity")
                {
                    elem->QueryFloatAttribute("x", &boneData.deformSquishSensitivity.x);
                    elem->QueryFloatAttribute("y", &boneData.deformSquishSensitivity.y);
                    elem->QueryFloatAttribute("z", &boneData.deformSquishSensitivity.z);
                }
                else if (elemName == "stretchSensitivity")
                {
                    elem->QueryFloatAttribute("x", &boneData.deformStretchSensitivity.x);
                    elem->QueryFloatAttribute("y", &boneData.deformStretchSensitivity.y);
                    elem->QueryFloatAttribute("z", &boneData.deformStretchSensitivity.z);
                }
                else if (elemName == "bulgeSensitivity")
                {
                    elem->QueryFloatAttribute("x", &boneData.deformBulgeSensitivity.x);
                    elem->QueryFloatAttribute("y", &boneData.deformBulgeSensitivity.y);
                    elem->QueryFloatAttribute("z", &boneData.deformBulgeSensitivity.z);
                }
                else if (elemName == "squishStiffness")
                {
                    elem->QueryFloatAttribute("x", &boneData.deformSquishStiffness.x);
                    elem->QueryFloatAttribute("y", &boneData.deformSquishStiffness.y);
                    elem->QueryFloatAttribute("z", &boneData.deformSquishStiffness.z);
                }
                else if (elemName == "stretchStiffness")
                {
                    elem->QueryFloatAttribute("x", &boneData.deformStretchStiffness.x);
                    elem->QueryFloatAttribute("y", &boneData.deformStretchStiffness.y);
                    elem->QueryFloatAttribute("z", &boneData.deformStretchStiffness.z);
                }
                else if (elemName == "squishDamping")
                {
                    elem->QueryFloatAttribute("x", &boneData.deformSquishDamping.x);
                    elem->QueryFloatAttribute("y", &boneData.deformSquishDamping.y);
                    elem->QueryFloatAttribute("z", &boneData.deformSquishDamping.z);
                }
                else if (elemName == "stretchDamping")
                {
                    elem->QueryFloatAttribute("x", &boneData.deformStretchDamping.x);
                    elem->QueryFloatAttribute("y", &boneData.deformStretchDamping.y);
                    elem->QueryFloatAttribute("z", &boneData.deformStretchDamping.z);
                }
                elem = elem->NextSiblingElement();
            }
        }
        else if (rootElemName == "dynamicVolume")
        {
            tinyxml2::XMLElement* elem = rootElem->FirstChildElement();
            while (elem)
            {
                const Mus::lString elemName = elem->Name();
                if (elemName == "enable")
                {
                    elem->QueryBoolText(&boneData.enableDynamicVolume);
                }
                else if (elemName == "volume")
                {
                    elem->QueryFloatAttribute("min", &boneData.volumeMin);
                    elem->QueryFloatAttribute("max", &boneData.volumeMax);
                }
                else if (elemName == "physicsScale")
                {
                    elem->QueryFloatAttribute("min", &boneData.physicsScaleMin);
                    elem->QueryFloatAttribute("max", &boneData.physicsScaleMax);
                }
                else if (elemName == "clampPhysicsScale")
                {
                    elem->QueryBoolText(&boneData.clampPhysicsScale);
                }
                elem = elem->NextSiblingElement();
            }
        }
    };

    void PhysicsConfigReader::GetDistanceConstraint(const Mus::lString& rootElemName, tinyxml2::XMLElement* elem, DistanceConstraintData& consData) const
    {
        if (rootElemName.empty())
            return;
        if (rootElemName == "compliance")
        {
            elem->QueryFloatAttribute("squish", &consData.complianceSquish);
            elem->QueryFloatAttribute("stretch", &consData.complianceStretch);
        }
        else if (rootElemName == "margin")
        {
            elem->QueryFloatAttribute("squish", &consData.squishMargin);
            elem->QueryFloatAttribute("stretch", &consData.stretchMargin);
        }
        else if (rootElemName == "damping")
        {
            elem->QueryFloatAttribute("squish", &consData.squishDamping);
            elem->QueryFloatAttribute("stretch", &consData.stretchDamping);
        }
    };

    void PhysicsConfigReader::GetAngularConstraint(const Mus::lString& rootElemName, tinyxml2::XMLElement* elem, AngularConstraintData& consData) const
    {
        if (rootElemName.empty())
            return;
        if (rootElemName == "compliancePositive")
        {
            elem->QueryFloatAttribute("x", &consData.compliancePositive.x);
            elem->QueryFloatAttribute("y", &consData.compliancePositive.y);
            elem->QueryFloatAttribute("z", &consData.compliancePositive.z);
            consData.compliancePositive = GetABSPoint3(consData.compliancePositive);
        }
        else if (rootElemName == "complianceNegative")
        {
            elem->QueryFloatAttribute("x", &consData.complianceNegative.x);
            elem->QueryFloatAttribute("y", &consData.complianceNegative.y);
            elem->QueryFloatAttribute("z", &consData.complianceNegative.z);
            consData.complianceNegative = GetABSPoint3(consData.complianceNegative);
        }
        else if (rootElemName == "marginPositive")
        {
            elem->QueryFloatAttribute("x", &consData.marginPositive.x);
            elem->QueryFloatAttribute("y", &consData.marginPositive.y);
            elem->QueryFloatAttribute("z", &consData.marginPositive.z);
            consData.marginPositive = GetABSPoint3(consData.marginPositive);
        }
        else if (rootElemName == "marginNegative")
        {
            elem->QueryFloatAttribute("x", &consData.marginNegative.x);
            elem->QueryFloatAttribute("y", &consData.marginNegative.y);
            elem->QueryFloatAttribute("z", &consData.marginNegative.z);
            consData.marginNegative = GetABSPoint3(consData.marginNegative);
        }
        else if (rootElemName == "dampingPositive")
        {
            elem->QueryFloatAttribute("x", &consData.dampingPositive.x);
            elem->QueryFloatAttribute("y", &consData.dampingPositive.y);
            elem->QueryFloatAttribute("z", &consData.dampingPositive.z);
            consData.dampingPositive = GetABSPoint3(consData.dampingPositive);
        }
        else if (rootElemName == "dampingNegative")
        {
            elem->QueryFloatAttribute("x", &consData.dampingNegative.x);
            elem->QueryFloatAttribute("y", &consData.dampingNegative.y);
            elem->QueryFloatAttribute("z", &consData.dampingNegative.z);
            consData.dampingNegative = GetABSPoint3(consData.dampingNegative);
        }
    };

    void PhysicsConfigReader::GetDeformConstraint(const Mus::lString& rootElemName, tinyxml2::XMLElement* elem, DeformConstraintData& consData) const
    {
        if (rootElemName.empty())
            return;
        if (rootElemName == "squishWeight")
        {
            elem->QueryFloatAttribute("x", &consData.squishWeight.x);
            elem->QueryFloatAttribute("y", &consData.squishWeight.y);
            elem->QueryFloatAttribute("z", &consData.squishWeight.z);
        }
        else if (rootElemName == "stretchWeight")
        {
            elem->QueryFloatAttribute("x", &consData.stretchWeight.x);
            elem->QueryFloatAttribute("y", &consData.stretchWeight.y);
            elem->QueryFloatAttribute("z", &consData.stretchWeight.z);
        }
        else if (rootElemName == "bulgeWeight")
        {
            elem->QueryFloatAttribute("x", &consData.bulgeWeight.x);
            elem->QueryFloatAttribute("y", &consData.bulgeWeight.y);
            elem->QueryFloatAttribute("z", &consData.bulgeWeight.z);
        }
    };

    void PhysicsConfigReader::GetGenericConstraint(tinyxml2::XMLElement* rootElem, DistanceConstraintData& distConsData, AngularConstraintData& angConsData, DeformConstraintData& deformConsData) const
    {
        tinyxml2::XMLElement* elem = rootElem->FirstChildElement();
        while (elem)
        {
            const Mus::lString elemName = elem->Name();
            if (elemName == "distance")
            {
                tinyxml2::XMLElement* consElem = elem->FirstChildElement();
                while (consElem)
                {
                    const Mus::lString consElemName = consElem->Name();
                    GetDistanceConstraint(consElemName, consElem, distConsData);
                    consElem = consElem->NextSiblingElement();
                }
            }
            else if (elemName == "angular")
            {
                tinyxml2::XMLElement* consElem = elem->FirstChildElement();
                while (consElem)
                {
                    const Mus::lString consElemName = consElem->Name();
                    GetAngularConstraint(consElemName, consElem, angConsData);
                    consElem = consElem->NextSiblingElement();
                }
            }
            else if (elemName == "deform")
            {
                tinyxml2::XMLElement* consElem = elem->FirstChildElement();
                while (consElem)
                {
                    const Mus::lString consElemName = consElem->Name();
                    GetDeformConstraint(consElemName, consElem, deformConsData);
                    consElem = consElem->NextSiblingElement();
                }
            }
            elem = elem->NextSiblingElement();
        }
    }

    void PhysicsConfigReader::BoneLogging(const std::string& file, std::string_view name, const PhysicsInput::Bone& bone) const
    {
        logger::debug("{} : bone physics {} - mass {} / inertia neg{} pos{} / angularInertia neg{} pos{} / inertiaCorrection neg{} pos{} / angularBlendFactor {} / gravity {} / windFactor {} / linearRotTorque x{} y{}, z{}", 
                      file, name, bone.mass, bone.inertiaNegative, bone.inertiaPositive, bone.angularInertiaNegative, bone.angularInertiaPositive, bone.inertiaCorrectionNegative, bone.inertiaCorrectionPositive, bone.angularBlendFactor, bone.gravity, bone.windFactor, bone.linearRotTorque[0], bone.linearRotTorque[1], bone.linearRotTorque[2]);
        logger::debug("{} : bone physics {} - linear damping neg{} pos{} / angular damping neg{} pos{} / linear limit neg{} pos{} / angular limit neg{} pos{}", 
                      file, name, bone.dampingNegative, bone.dampingPositive, bone.angularDampingNegative, bone.angularDampingPositive, bone.limitNegative, bone.limitPositive, bone.angularLimitNegative, bone.angularLimitPositive);
        logger::debug("{} : bone offset {} - pos {}{}{}{}", 
                      file, name, bone.offset, bone.isParticle ? " /" : "", bone.isParticle ? " target " : "", bone.isParticle ? bone.parentBoneName : "");
        logger::debug("{} : bone animDrive {} - linear compliance {} / angular compliance {}", 
                      file, name, bone.animDriveCompliance, bone.animDriveAngularCompliance);
        logger::debug("{} : bone collider {} - margin {} / friction {} / softness {} / restitution {} / layerGroup {:x} / collideLayer {:x} / colliderType {}", 
                      file, name, bone.collisionMargin, bone.collisionFriction, bone.collisionCompliance, bone.collisionRestitution, bone.collisionLayerGroup, bone.collisionCollideLayer, GetColliderTypeStr(bone.colliderType));  
        logger::debug("{} : bone deform {} - limit min{} max{} / volume preservation {} / sensitivity squish{} stretch{} bulge{} / stiffness squish{} stretch{} / damping squish{} stretch{}", 
                      file, name, bone.deformMin, bone.deformMax, bone.deformVolumePreservation, bone.deformSquishSensitivity, bone.deformStretchSensitivity, bone.deformBulgeSensitivity, bone.deformSquishStiffness, bone.deformStretchStiffness, bone.deformSquishDamping, bone.deformStretchDamping);   
        if (bone.enableDynamicVolume)
            logger::debug("{} : bone dynamic volume {} - volume min({}) max({}) / physicsScale min({}) max({}) / clampPhysicsScale {}", 
                          file, name, bone.volumeMin, bone.volumeMax, bone.physicsScaleMin, bone.physicsScaleMax, bone.clampPhysicsScale);
    }

    void PhysicsConfigReader::DistanceConstraintLogging(const std::string& file, const std::string_view A, const std::string_view B, std::uint32_t anchIdx, const DistanceConstraintData& cons) const
    {
        logger::debug("{} : distance constraint {}({}|{}) - compliance squish({}) stretch({}) / limit squish({}) stretch({}) / damping squish({}) stretch({})", 
                      file, A, B, anchIdx, cons.complianceSquish, cons.complianceStretch, cons.squishMargin, cons.stretchMargin, cons.squishDamping, cons.stretchDamping);         
    }

    void PhysicsConfigReader::AngularConstraintLogging(const std::string& file, const std::string_view A, const std::string_view B, std::uint32_t anchIdx, const AngularConstraintData& cons) const
    {
        logger::debug("{} : angular constraint {}({}|{}) - compliance neg{} pos{} / limit neg{} pos{} / damping neg{} pos{}", 
                      file, A, B, anchIdx, cons.complianceNegative, cons.compliancePositive, cons.marginNegative, cons.marginPositive, cons.dampingNegative, cons.dampingPositive);     
    }

    void PhysicsConfigReader::DeformConstraintLogging(const std::string& file, const std::string_view A, const std::string_view B, std::uint32_t anchIdx, const DeformConstraintData& cons) const
    {
        logger::debug("{} : deform constraint {}({}|{}) - weight squish{} stretch{} bulge{}", 
                      file, A, B, anchIdx, cons.squishWeight, cons.stretchWeight, cons.bulgeWeight);     
    }
}