#include "MXPBD/PhysicsConfig.h"

namespace MXPBD
{
    void PhysicsConfigReader::CreateParentPhysicsBone(RE::NiNode* rootNode, PhysicsInput& input) const
    {
        if (!rootNode)
            return;
        if (input.bones.empty())
            return;

        logger::debug("Creating parent physics bone...");
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
        if (input.constraintEmpty())
            return;
        
        logger::debug("Creating parent constraint...");
        constexpr std::string parConsName = "__PARENT__";
        std::vector<std::pair<std::string, PhysicsInput::Bone>> newPhysicsBones;

        auto AddParentCons = [&](auto& cons, std::string_view consName) {
            const bool isParentA = cons.boneNameA == parConsName;
            const bool isParentB = cons.boneNameB == parConsName;
            if (isParentA == isParentB)
                return;
            std::string_view childBoneName = isParentA ? cons.boneNameB : cons.boneNameA;
            auto obj = rootNode->GetObjectByName(childBoneName);
            if (!obj || !obj->parent)
                return;
            auto parent = obj->parent;
            while (parent->name.empty() && parent->parent)
            {
                parent = parent->parent;
            }
            if (parent->name.empty())
                return;
            std::string_view parentNodeName = parent->name.c_str();
            logger::debug("{} : parent {} constraint found {}", childBoneName, consName, parentNodeName);
            if (isParentA)
                cons.boneNameA = parentNodeName;
            else
                cons.boneNameB = parentNodeName;
            PhysicsInput::Bone newParentBone;
            newParentBone.mass = 0.0f;
            newPhysicsBones.emplace_back(parentNodeName, newParentBone);
        };

        for (auto& cons : input.distanceConstraints)
        {
            AddParentCons(cons, "distance");
        }
        for (auto& cons : input.angularConstraints)
        {
            AddParentCons(cons, "angular");
        }
        for (auto& cons : input.coneConstraints)
        {
            AddParentCons(cons, "cone");
        }
        for (auto& cons : input.deformConstraints)
        {
            AddParentCons(cons, "deform");
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
        if (input.constraintEmpty())
            return;

        logger::debug("Creating original constraint...");
        constexpr std::string orgConsName = "__ORIGINAL__";
        std::vector<std::pair<std::string, PhysicsInput::Bone>> newPhysicsBones;

        auto AddOriginalCons = [&](auto& cons, std::string_view consName) {
            const bool isOrgA = cons.boneNameA == orgConsName;
            const bool isOrgB = cons.boneNameB == orgConsName;
            if (isOrgA == isOrgB)
                return;
            std::string_view baseBoneName = isOrgA ? cons.boneNameB : cons.boneNameA;
            auto obj = rootNode->GetObjectByName(baseBoneName);
            if (!obj || !obj->parent)
                return;
            auto parent = obj->parent;
            while (parent->name.empty() && parent->parent)
            {
                parent = parent->parent;
            }
            if (parent->name.empty())
                return;
            std::string_view parentNodeName = parent->name.c_str();
            logger::debug("{} : original {} constraint found {}", baseBoneName, consName, parentNodeName);

            const RE::NiPoint3 worldDiff = obj->world.translate - parent->world.translate;
            const RE::NiPoint3 localOffset = (parent->world.rotate.Transpose() * worldDiff) * reciprocal(parent->world.scale);
            const std::string particleBoneName = std::string(baseBoneName) + orgConsName;
            if (isOrgA)
                cons.boneNameA = particleBoneName;
            else
                cons.boneNameB = particleBoneName;
            PhysicsInput::Bone newParticleBone;
            newParticleBone.mass = 0.0f;
            newParticleBone.parentBoneName = parentNodeName;
            newParticleBone.isParticle = 1;
            newParticleBone.offset = localOffset;
            newPhysicsBones.emplace_back(parentNodeName, PhysicsInput::Bone());
            newPhysicsBones.emplace_back(particleBoneName, newParticleBone);
        };

        for (auto& cons : input.distanceConstraints)
        {
            AddOriginalCons(cons, "distance");
        }
        for (auto& cons : input.angularConstraints)
        {
            AddOriginalCons(cons, "angular");
        }
        for (auto& cons : input.coneConstraints)
        {
            AddOriginalCons(cons, "cone");
        }
        for (auto& cons : input.deformConstraints)
        {
            AddOriginalCons(cons, "deform");
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
        if (input.constraintEmpty())
            return;

        logger::debug("Creating volume constraint...");
        constexpr std::string volumeName = "__VOLUME__";
        std::vector<PhysicsInput::DistanceConstraint> newDistanceConstraints;
        std::vector<PhysicsInput::AngularConstraint> newAngularConstraints;
        std::vector<PhysicsInput::ConeConstraint> newConeConstraints;
        std::vector<PhysicsInput::DeformConstraint> newDeformConstraints;
        bool isVolume = std::ranges::any_of(input.distanceConstraints, [&](auto& cons) {
            return (cons.boneNameA == volumeName) != (cons.boneNameB == volumeName);
        });
        isVolume = isVolume ? isVolume : std::ranges::any_of(input.angularConstraints, [&](auto& cons) {
            return (cons.boneNameA == volumeName) != (cons.boneNameB == volumeName);
        });
        isVolume = isVolume ? isVolume : std::ranges::any_of(input.deformConstraints, [&](auto& cons) {
            return (cons.boneNameA == volumeName) != (cons.boneNameB == volumeName);
        });
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

        auto AddVolumeCons = [&](auto& cons, auto& newConstraints, std::string_view consName) {
            const bool isVolumeA = cons.boneNameA == volumeName;
            const bool isVolumeB = cons.boneNameB == volumeName;
            if (isVolumeA == isVolumeB)
                return;
            const std::string targetBoneName = isVolumeA ? cons.boneNameB : cons.boneNameA;
            auto chdbIt = convexHullDataBatches.find(targetBoneName);
            if (chdbIt == convexHullDataBatches.end())
            {
                logger::error("{:x} : Unable to get mesh shape for {}. so set __ORIGINAL__", user->formID, targetBoneName);
                if (isVolumeA)
                    cons.boneNameA = "__ORIGINAL__";
                else
                    cons.boneNameB = "__ORIGINAL__";
                return;
            }

            auto obj = rootNode->GetObjectByName(targetBoneName);
            if (!obj || !obj->parent)
                return;
            auto parent = obj->parent;
            while (parent->name.empty() && parent->parent)
            {
                parent = parent->parent;
            }
            if (parent->name.empty())
                return;
            std::string_view parentNodeName = parent->name.c_str();
            std::vector<std::uint32_t> selectedIdx;
            AABB aabb = AABB();
            for (std::uint8_t v = 0; v < COL_VERTEX_MAX; ++v)
            {
                const Vector p = DirectX::XMVectorSet(chdbIt->second.vX[v], chdbIt->second.vY[v], chdbIt->second.vZ[v], 0.0f);
                const AABB vAABB(p, p);
                aabb = aabb.Merge(vAABB);
            }
            const Vector center = DirectX::XMVectorMultiply(DirectX::XMVectorAdd(aabb.min, aabb.max), vHalf);

            std::uint32_t bestV = 0;
            Vector maxDistSqV = vNegOne;
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

            for (std::uint8_t k = 1; k < COL_VERTEX_MAX; ++k)
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

            if (selectedIdx.empty())
            {
                logger::error("{:x} : Unable to get volume for {}. so set __ORIGINAL__", user->formID, targetBoneName);
                if (isVolumeA)
                    cons.boneNameA = "__ORIGINAL__";
                else
                    cons.boneNameB = "__ORIGINAL__";
                return;
            }

            for (std::uint32_t i = 0; i < selectedIdx.size(); ++i)
            {
                const std::uint32_t vi = selectedIdx[i];
                const RE::NiPoint3 worldPos = obj->world.translate + (obj->world.rotate * RE::NiPoint3(chdbIt->second.vX[vi], chdbIt->second.vY[vi], chdbIt->second.vZ[vi]));
                const RE::NiPoint3 worldDiff = worldPos - parent->world.translate;
                const RE::NiPoint3 localOffset = (parent->world.rotate.Transpose() * worldDiff) * reciprocal(parent->world.scale);
                std::string particleBoneName = targetBoneName + volumeName + std::to_string(vi);

                PhysicsInput::Bone particleBone;
                particleBone.mass = 0.0f;
                particleBone.isParticle = 1;
                particleBone.parentBoneName = parentNodeName;
                particleBone.offset = localOffset;

                if (i == 0)
                {
                    if (isVolumeA)
                        cons.boneNameA = particleBoneName;
                    else
                        cons.boneNameB = particleBoneName;
                }
                else
                {
                    auto newCons = cons;
                    if (isVolumeA)
                        newCons.boneNameA = particleBoneName;
                    else
                        newCons.boneNameB = particleBoneName;
                    newConstraints.push_back(newCons);
                }
                newPhysicsBones.emplace_back(particleBoneName, particleBone);
                logger::debug("{} : added particle for {} on {}", particleBoneName, targetBoneName, parentNodeName);
            }
            newPhysicsBones.emplace_back(parentNodeName, PhysicsInput::Bone());
        };

        for (auto& cons : input.distanceConstraints)
        {
            AddVolumeCons(cons, newDistanceConstraints, "distance");
        }
        input.distanceConstraints.append_range(newDistanceConstraints);

        for (auto& cons : input.angularConstraints)
        {
            AddVolumeCons(cons, newAngularConstraints, "angular");
        }
        input.angularConstraints.append_range(newAngularConstraints);

        for (auto& cons : input.coneConstraints)
        {
            AddVolumeCons(cons, newConeConstraints, "cone");
        }
        input.coneConstraints.append_range(newConeConstraints);

        for (auto& cons : input.deformConstraints)
        {
            AddVolumeCons(cons, newDeformConstraints, "deform");
        }
        input.deformConstraints.append_range(newDeformConstraints);

        for (const auto& newBone : newPhysicsBones)
        {
            if (input.bones.find(newBone.first) != input.bones.end())
                continue;
            input.bones[newBone.first] = newBone.second;
        }
    }

    void PhysicsConfigReader::SetCenterPhysicsBone(RE::NiNode* rootNode, PhysicsInput& input, const std::vector<RawCollider>& a_mergedConvexHulls) const
    {
        logger::debug("Setting center of mass...");
        for (const auto& convexHull : a_mergedConvexHulls)
        {
            auto it = input.bones.find(convexHull.boneName);
            if (it == input.bones.end())
                continue;
            RE::NiPoint3& center = it->second.centerOfMass;
            center = pZero;
            for (const auto& v :convexHull.vertices)
            {
                center += v;
            }
            center *= reciprocal(convexHull.vertices.size());
        }
    }

    void PhysicsConfigReader::UpdateVolume(RE::NiNode* rootNode, PhysicsInput& input, const std::vector<RawCollider>& a_mergedConvexHulls) const
    {
        logger::debug("Updating volumes...");
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

    void PhysicsConfigReader::CreateClothCluster(RE::NiNode* rootNode, PhysicsInput& input, const std::vector<RawCollider>& a_mergedConvexHulls) const
    {
        if (!rootNode || input.clothClusters.empty())
            return;

        logger::debug("Creating cloth clusters...");
        std::unordered_map<std::string, const std::unordered_set<std::string>*> nearBonesLookup;
        for (const auto& hull : a_mergedConvexHulls)
        {
            if (!hull.nearBones.empty())
            {
                nearBonesLookup[hull.boneName] = &hull.nearBones;
            }
        }

        for (const auto& cluster : input.clothClusters)
        {
            auto AddAutoDistanceConstraint = [this, &input, &cluster](const std::string& nameA, const std::string& nameB, float t_curve, bool isDiagonal, bool isHorizontal) {
                if (cluster.root.distanceConstraint.complianceStretch > 0.0f || cluster.tip.distanceConstraint.complianceStretch > 0.0f)
                {
                    float multiplier = 1.0f;
                    if (isHorizontal)
                        multiplier = cluster.distanceHorizontalMultiplier;
                    else if (isDiagonal)
                        multiplier = cluster.distanceDiagonalMultiplier;
                    if (multiplier <= Epsilon)
                        return;
                    DistanceConstraintData distCons = cluster.tip.distanceConstraint;
                    distCons.boneNameA = nameA;
                    distCons.boneNameB = nameB;
                    distCons.complianceSquish = std::lerp(cluster.root.distanceConstraint.complianceSquish, cluster.tip.distanceConstraint.complianceSquish, t_curve) * COMPLIANCE_SCALE * multiplier;
                    distCons.complianceStretch = std::lerp(cluster.root.distanceConstraint.complianceStretch, cluster.tip.distanceConstraint.complianceStretch, t_curve) * COMPLIANCE_SCALE * multiplier;
                    if (distCons.complianceSquish + distCons.complianceStretch <= Epsilon)
                        return;
                    distCons.squishMargin = std::lerp(cluster.root.distanceConstraint.squishMargin, cluster.tip.distanceConstraint.squishMargin, t_curve);
                    distCons.stretchMargin = std::lerp(cluster.root.distanceConstraint.stretchMargin, cluster.tip.distanceConstraint.stretchMargin, t_curve);
                    distCons.squishLimit = std::lerp(cluster.root.distanceConstraint.squishLimit, cluster.tip.distanceConstraint.squishLimit, t_curve);
                    distCons.stretchLimit = std::lerp(cluster.root.distanceConstraint.stretchLimit, cluster.tip.distanceConstraint.stretchLimit, t_curve);
                    distCons.squishDamping = std::lerp(cluster.root.distanceConstraint.squishDamping, cluster.tip.distanceConstraint.squishDamping, t_curve);
                    distCons.stretchDamping = std::lerp(cluster.root.distanceConstraint.stretchDamping, cluster.tip.distanceConstraint.stretchDamping, t_curve);
                    LoggingDistanceConstraint(input.infos.front().inputPath, nameA, nameB, distCons);
                    input.distanceConstraints.push_back(distCons);
                }
            };
            auto AddAutoAngularConstraint = [this, &input, &cluster](const std::string& nameA, const std::string& nameB, float t_curve, bool isHorizontal) {
                if (cluster.root.angularConstraint.compliancePositive.Length() > 0.0f || cluster.tip.angularConstraint.compliancePositive.Length() > 0.0f)
                {
                    float multiplier = isHorizontal ? cluster.angularHorizontalMultiplier : 1.0f;
                    if (multiplier <= Epsilon)
                        return;
                    AngularConstraintData angCons = cluster.tip.angularConstraint;
                    angCons.boneNameA = nameA;
                    angCons.boneNameB = nameB;
                    angCons.compliancePositive = lerp(cluster.root.angularConstraint.compliancePositive, cluster.tip.angularConstraint.compliancePositive, t_curve) * COMPLIANCE_SCALE * multiplier;
                    angCons.complianceNegative = lerp(cluster.root.angularConstraint.complianceNegative, cluster.tip.angularConstraint.complianceNegative, t_curve) * COMPLIANCE_SCALE * multiplier;
                    if (angCons.compliancePositive.SqrLength() + angCons.complianceNegative.SqrLength() <= EpsilonSq)
                        return;
                    angCons.marginPositive = lerp(cluster.root.angularConstraint.marginPositive, cluster.tip.angularConstraint.marginPositive, t_curve) * toRadian;
                    angCons.marginNegative = lerp(cluster.root.angularConstraint.marginNegative, cluster.tip.angularConstraint.marginNegative, t_curve) * toRadian;
                    angCons.limitPositive = lerp(cluster.root.angularConstraint.limitPositive, cluster.tip.angularConstraint.limitPositive, t_curve) * toRadian;
                    angCons.limitNegative = lerp(cluster.root.angularConstraint.limitNegative, cluster.tip.angularConstraint.limitNegative, t_curve) * toRadian;
                    angCons.dampingPositive = lerp(cluster.root.angularConstraint.dampingPositive, cluster.tip.angularConstraint.dampingPositive, t_curve);
                    angCons.dampingNegative = lerp(cluster.root.angularConstraint.dampingNegative, cluster.tip.angularConstraint.dampingNegative, t_curve);
                    LoggingAngularConstraint(input.infos.front().inputPath, nameA, nameB, angCons);
                    input.angularConstraints.push_back(angCons);
                }
            };
            auto AddAutoConeConstraint = [this, &input, &cluster](const std::string& nameA, const std::string& nameB, float t_curve) {
                if (cluster.root.coneConstraint.compliancePositive.Length() > 0.0f || cluster.tip.coneConstraint.compliancePositive.Length() > 0.0f)
                {
                    ConeConstraintData coneCons = cluster.tip.coneConstraint;
                    coneCons.boneNameA = nameA;
                    coneCons.boneNameB = nameB;
                    coneCons.compliancePositive = lerp(cluster.root.coneConstraint.compliancePositive, cluster.tip.coneConstraint.compliancePositive, t_curve) * COMPLIANCE_SCALE;
                    coneCons.complianceNegative = lerp(cluster.root.coneConstraint.complianceNegative, cluster.tip.coneConstraint.complianceNegative, t_curve) * COMPLIANCE_SCALE;
                    if (coneCons.compliancePositive.SqrLength() + coneCons.complianceNegative.SqrLength() <= EpsilonSq)
                        return;
                    coneCons.marginPositive = lerp(cluster.root.coneConstraint.marginPositive, cluster.tip.coneConstraint.marginPositive, t_curve) * toRadian;
                    coneCons.marginNegative = lerp(cluster.root.coneConstraint.marginNegative, cluster.tip.coneConstraint.marginNegative, t_curve) * toRadian;
                    coneCons.limitPositive = lerp(cluster.root.coneConstraint.limitPositive, cluster.tip.coneConstraint.limitPositive, t_curve) * toRadian;
                    coneCons.limitNegative = lerp(cluster.root.coneConstraint.limitNegative, cluster.tip.coneConstraint.limitNegative, t_curve) * toRadian;
                    coneCons.dampingPositive = lerp(cluster.root.coneConstraint.dampingPositive, cluster.tip.coneConstraint.dampingPositive, t_curve);
                    coneCons.dampingNegative = lerp(cluster.root.coneConstraint.dampingNegative, cluster.tip.coneConstraint.dampingNegative, t_curve);
                    LoggingConeConstraint(input.infos.front().inputPath, nameA, nameB, coneCons);
                    input.coneConstraints.push_back(coneCons);
                }
            };

            if (!cluster.rootList.empty())
            {
                std::unordered_map<std::string, RE::NiNode*> boneMap;
                std::unordered_map<std::string, std::uint32_t> nodeDepths;
                std::vector<std::string> allBoneNames;
                std::uint32_t maxDepth = 0;
                std::queue<RE::NiNode*> q;
                for (const auto& rootName : cluster.rootList)
                {
                    auto objNode = rootNode->GetObjectByName(rootName);
                    if (!objNode || objNode->name.empty())
                        continue;
                    auto node = objNode->AsNode();
                    if (!node)
                        continue;
                    const std::string name = node->name.c_str();
                    if (nodeDepths.find(name) == nodeDepths.end())
                    {
                        q.push(node);
                        nodeDepths[name] = 0;
                        boneMap[name] = node;
                    }
                }

                while (!q.empty())
                {
                    RE::NiNode* curr = q.front();
                    const std::string currName = curr->name.c_str();
                    q.pop();

                    const std::uint32_t depth = nodeDepths[currName];
                    maxDepth = std::max(maxDepth, depth);
                    allBoneNames.push_back(currName);

                    for (auto& child : curr->GetChildren())
                    {
                        RE::NiNode* childNode = child ? child->AsNode() : nullptr;
                        if (!childNode || childNode->name.empty())
                            continue;
                        const std::string childName = childNode->name.c_str();
                        if (nodeDepths.find(childName) == nodeDepths.end())
                        {
                            nodeDepths[childName] = depth + 1;
                            boneMap[childName] = childNode;
                            q.push(childNode);
                        }
                    }
                }

                for (const auto& rootName : cluster.rootList)
                {
                    auto objNode = rootNode->GetObjectByName(rootName);
                    if (objNode && objNode->parent && !objNode->parent->name.empty())
                    {
                        const std::string parentName = objNode->parent->name.c_str();
                        PhysicsInput::Bone parentBone = cluster.root.bone;
                        parentBone.mass = 0.0f;
                        parentBone.gravity = 0.0f;
                        parentBone.dampingPositive = pZero;
                        parentBone.dampingNegative = pZero;
                        parentBone.angularDampingPositive = pZero;
                        parentBone.angularDampingNegative = pZero;
                        LoggingBone(input.infos.front().inputPath, parentName, parentBone);
                        input.bones[parentName] = parentBone;
                    }
                }

                const float depthRange = std::max(1.0f, static_cast<float>(maxDepth));

                for (const std::string& boneName : allBoneNames)
                {
                    RE::NiNode* node = boneMap[boneName];
                    const std::uint32_t depth = nodeDepths[boneName];

                    const float t = std::clamp(static_cast<float>(depth - 1) * reciprocal(depthRange), 0.0f, 1.0f);
                    const float t_curve = std::pow(t, cluster.gradientCurve);

                    PhysicsInput::Bone newBone = cluster.tip.bone;
                    newBone.mass = std::lerp(cluster.root.bone.mass, cluster.tip.bone.mass, t_curve);
                    newBone.gravity = std::lerp(cluster.root.bone.gravity, cluster.tip.bone.gravity, t_curve);
                    newBone.dampingPositive = lerp(cluster.root.bone.dampingPositive, cluster.tip.bone.dampingPositive, t_curve);
                    newBone.dampingNegative = lerp(cluster.root.bone.dampingNegative, cluster.tip.bone.dampingNegative, t_curve);
                    newBone.angularDampingPositive = lerp(cluster.root.bone.angularDampingPositive, cluster.tip.bone.angularDampingPositive, t_curve);
                    newBone.angularDampingNegative = lerp(cluster.root.bone.angularDampingNegative, cluster.tip.bone.angularDampingNegative, t_curve);
                    newBone.inertiaPositive = lerp(cluster.root.bone.inertiaPositive, cluster.tip.bone.inertiaPositive, t_curve);
                    newBone.inertiaNegative = lerp(cluster.root.bone.inertiaNegative, cluster.tip.bone.inertiaNegative, t_curve);
                    newBone.angularInertiaPositive = lerp(cluster.root.bone.angularInertiaPositive, cluster.tip.bone.angularInertiaPositive, t_curve);
                    newBone.angularInertiaNegative = lerp(cluster.root.bone.angularInertiaNegative, cluster.tip.bone.angularInertiaNegative, t_curve);
                    newBone.angularBlendFactor = std::lerp(cluster.root.bone.angularBlendFactor, cluster.tip.bone.angularBlendFactor, t_curve);
                    newBone.windFactor = lerp(cluster.root.bone.windFactor, cluster.tip.bone.windFactor, t_curve);
                    newBone.deformMax = lerp(cluster.root.bone.deformMax, cluster.tip.bone.deformMax, t_curve);
                    newBone.deformMin = lerp(cluster.root.bone.deformMin, cluster.tip.bone.deformMin, t_curve);
                    newBone.deformVolumePreservation = lerp(cluster.root.bone.deformVolumePreservation, cluster.tip.bone.deformVolumePreservation, t_curve);
                    newBone.deformSquishSensitivity = lerp(cluster.root.bone.deformSquishSensitivity, cluster.tip.bone.deformSquishSensitivity, t_curve);
                    newBone.deformStretchSensitivity = lerp(cluster.root.bone.deformStretchSensitivity, cluster.tip.bone.deformStretchSensitivity, t_curve);
                    newBone.deformBulgeSensitivity = lerp(cluster.root.bone.deformBulgeSensitivity, cluster.tip.bone.deformBulgeSensitivity, t_curve);
                    newBone.deformSquishStiffness = lerp(cluster.root.bone.deformSquishStiffness, cluster.tip.bone.deformSquishStiffness, t_curve);
                    newBone.deformStretchStiffness = lerp(cluster.root.bone.deformStretchStiffness, cluster.tip.bone.deformStretchStiffness, t_curve);
                    newBone.deformSquishDamping = lerp(cluster.root.bone.deformSquishDamping, cluster.tip.bone.deformSquishDamping, t_curve);
                    newBone.deformStretchDamping = lerp(cluster.root.bone.deformStretchDamping, cluster.tip.bone.deformStretchDamping, t_curve);
                    newBone.animDriveCompliance = lerp(cluster.root.bone.animDriveCompliance, cluster.tip.bone.animDriveCompliance, t_curve);
                    newBone.animDriveAngularCompliance = lerp(cluster.root.bone.animDriveAngularCompliance, cluster.tip.bone.animDriveAngularCompliance, t_curve);
                    newBone.linearRotTorque[0] = lerp(cluster.root.bone.linearRotTorque[0], cluster.tip.bone.linearRotTorque[0], t_curve);
                    newBone.linearRotTorque[1] = lerp(cluster.root.bone.linearRotTorque[1], cluster.tip.bone.linearRotTorque[1], t_curve);
                    newBone.linearRotTorque[2] = lerp(cluster.root.bone.linearRotTorque[2], cluster.tip.bone.linearRotTorque[2], t_curve);
                    newBone.centerOfMass = lerp(cluster.root.bone.centerOfMass, cluster.tip.bone.centerOfMass, t_curve);
                    newBone.offset = lerp(cluster.root.bone.offset, cluster.tip.bone.offset, t_curve);
                    newBone.collisionMargin = std::lerp(cluster.root.bone.collisionMargin, cluster.tip.bone.collisionMargin, t_curve);
                    newBone.collisionFriction = std::lerp(cluster.root.bone.collisionFriction, cluster.tip.bone.collisionFriction, t_curve);
                    newBone.collisionCompliance = std::lerp(cluster.root.bone.collisionCompliance, cluster.tip.bone.collisionCompliance, t_curve) * COMPLIANCE_SCALE;
                    newBone.collisionRestitution = std::lerp(cluster.root.bone.collisionRestitution, cluster.tip.bone.collisionRestitution, t_curve) * COMPLIANCE_SCALE;

                    LoggingBone(input.infos.front().inputPath, boneName, newBone);
                    input.bones[boneName] = newBone;
                    if (node->parent && !node->parent->name.empty())
                    {
                        const std::string parentName = node->parent->name.c_str();
                        AddAutoDistanceConstraint(boneName, parentName, t_curve, false, false);
                        AddAutoAngularConstraint(boneName, parentName, t_curve, false);
                        AddAutoConeConstraint(boneName, parentName, t_curve);
                    }
                }

                for (const std::string& nameA : allBoneNames)
                {
                    auto itNear = nearBonesLookup.find(nameA);
                    if (itNear == nearBonesLookup.end())
                        continue;

                    const std::unordered_set<std::string>& nearSet = *(itNear->second);
                    const std::uint32_t depthA = nodeDepths[nameA];
                    RE::NiNode* nodeA = boneMap[nameA];
                    const RE::NiPoint3 posA = nodeA->world.translate;

                    struct NearBoneInfo
                    {
                        std::string name;
                        std::uint32_t depth;
                        float distSq;
                        bool isHorizontal;
                    };

                    std::vector<NearBoneInfo> horizontalCandidates;
                    std::vector<NearBoneInfo> diagonalCandidates;

                    for (const std::string& nameB : nearSet)
                    {
                        if (nameA >= nameB)
                            continue;
                        if (nodeDepths.find(nameB) == nodeDepths.end())
                            continue;

                        const std::uint32_t depthB = nodeDepths[nameB];
                        RE::NiNode* nodeB = boneMap[nameB];
                        if (nodeA->parent == nodeB || nodeB->parent == nodeA)
                            continue;

                        const RE::NiPoint3 posB = nodeB->world.translate;
                        const RE::NiPoint3 diff = posA - posB;
                        const float distSq = diff.SqrLength();
                        if (depthA == depthB)
                            horizontalCandidates.push_back({nameB, depthB, distSq, true});
                        else if (std::abs(static_cast<std::int32_t>(depthA) - static_cast<std::int32_t>(depthB)) == 1)
                            diagonalCandidates.push_back({nameB, depthB, distSq, false});
                    }

                    auto sortFunc = [](const NearBoneInfo& a, const NearBoneInfo& b) {
                        return a.distSq < b.distSq;
                    };
                    std::sort(horizontalCandidates.begin(), horizontalCandidates.end(), sortFunc);
                    std::sort(diagonalCandidates.begin(), diagonalCandidates.end(), sortFunc);

                    const std::uint32_t MAX_HORIZONTAL_LINKS = 2;
                    const std::uint32_t MAX_DIAGONAL_LINKS = 4;
                    const float ABSOLUTE_MAX_DIST_SQ = 100.0f * 100.0f;
                    const std::uint32_t horizontalCount = static_cast<std::uint32_t>(horizontalCandidates.size());
                    const std::uint32_t diagonalCount = static_cast<std::uint32_t>(diagonalCandidates.size());
                    for (std::uint32_t i = 0; i < std::min(horizontalCount, MAX_HORIZONTAL_LINKS); ++i)
                    {
                        if (horizontalCandidates[i].distSq > ABSOLUTE_MAX_DIST_SQ)
                            continue;
                        const float t = std::clamp(static_cast<float>(horizontalCandidates[i].depth - 1) * reciprocal(depthRange), 0.0f, 1.0f);
                        const float t_curve = std::pow(t, cluster.gradientCurve);
                        AddAutoDistanceConstraint(nameA, horizontalCandidates[i].name, t_curve, false, true);
                        AddAutoAngularConstraint(nameA, horizontalCandidates[i].name, t_curve, true);
                    }

                    for (std::uint32_t i = 0; i < std::min(diagonalCount, MAX_DIAGONAL_LINKS); ++i)
                    {
                        if (diagonalCandidates[i].distSq > ABSOLUTE_MAX_DIST_SQ)
                            continue;

                        const std::uint32_t minDepth = std::min(depthA, diagonalCandidates[i].depth);
                        const float t = std::clamp(static_cast<float>(minDepth - 1) * reciprocal(depthRange), 0.0f, 1.0f);
                        const float t_curve = std::pow(t, cluster.gradientCurve);
                        AddAutoDistanceConstraint(nameA, diagonalCandidates[i].name, t_curve, true, false);
                    }
                }
            }
            if (!cluster.chainList.empty())
            {
                for (const auto& chain : cluster.chainList)
                {
                    const float depthRange = std::max(1.0f, static_cast<float>(chain.size() - 1));
                    float depth = 0.0f;
                    std::string boneA, boneB;
                    for (const std::string& boneName : chain)
                    {
                        boneA = boneB;
                        boneB = boneName;
                        ++depth;
                        if (boneA.empty())
                            continue;
                        const float t = std::clamp(static_cast<float>(depth - 1) * reciprocal(depthRange), 0.0f, 1.0f);
                        const float t_curve = std::pow(t, cluster.gradientCurve);
                        AddAutoDistanceConstraint(boneA, boneB, t_curve, false, false);
                        AddAutoAngularConstraint(boneA, boneB, t_curve, false);
                        AddAutoConeConstraint(boneA, boneB, t_curve);
                    }
                }
            }

            struct ConstraintLookupInfo
            {
                std::string boneNameB = "";
                bool hasOriginal = false;
                bool hasAnyConstraint = false;
            };
            std::unordered_map<std::string, ConstraintLookupInfo> targetLookup;
            targetLookup.reserve(input.distanceConstraints.size());
            for (const auto& cons : input.distanceConstraints)
            {
                auto& info = targetLookup[cons.boneNameA];
                info.hasAnyConstraint = true;
                if (cons.boneNameB == "__ORIGINAL__")
                {
                    info.hasOriginal = true;
                }
                else if (info.boneNameB.empty() &&
                         cons.boneNameB != "__PARENT__" &&
                         cons.boneNameB != "__VOLUME__")
                {
                    info.boneNameB = cons.boneNameB;
                }
            }
            std::unordered_set<std::string> visited;
            auto AddStiffness = [&](auto&& self, const std::string& boneName) -> std::int32_t {
                if (!visited.insert(boneName).second)
                    return -1;

                auto it = targetLookup.find(boneName);
                if (it == targetLookup.end() || !it->second.hasAnyConstraint)
                {
                    visited.erase(boneName);
                    return 0;
                }

                const auto& info = it->second;
                std::int32_t parentDepth = -1;
                if (!info.boneNameB.empty())
                {
                    parentDepth = self(self, info.boneNameB);
                }

                std::int32_t currentDepth = (parentDepth == -1) ? -1 : (parentDepth + 1);
                if (info.hasOriginal)
                {
                    visited.erase(boneName);
                    return currentDepth;
                }

                if (1 <= currentDepth && currentDepth <= cluster.StiffnessChainCount)
                {
                    float t = 0.0f;
                    if (cluster.StiffnessChainCount > 1)
                        t = static_cast<float>(currentDepth - 1) / static_cast<float>(cluster.StiffnessChainCount - 1);
                    const float currentCompliance = std::lerp(cluster.StiffnessStartCompliance, cluster.StiffnessEndCompliance, t);
                    PhysicsInput::DistanceConstraint newDistCons = {
                        .boneNameA = boneName,
                        .boneNameB = "__ORIGINAL__",
                        .complianceSquish = 0.001f,
                        .complianceStretch = currentCompliance * COMPLIANCE_SCALE,
                        .squishMargin = 0.0f,
                        .stretchMargin = 0.0f,
                        .squishDamping = 0.0f,
                        .stretchDamping = 0.0f};
                    input.distanceConstraints.push_back(std::move(newDistCons));
                    logger::debug("add distance constraint {}({}) - complianceStretch {} for stiffness", boneName, "__ORIGINAL__", currentCompliance);
                }

                visited.erase(boneName);
                return currentDepth;
            };
            if (cluster.StiffnessChainCount > 0)
            {
                for (auto& cons : input.distanceConstraints)
                    AddStiffness(AddStiffness, cons.boneNameA);
            }
        }
    }

    void PhysicsConfigReader::CreateProperties(RE::NiNode* rootNode, PhysicsInput& input, const std::vector<RawCollider>& a_mergedConvexHulls) const
    {
        CreateClothCluster(rootNode, input, a_mergedConvexHulls);
        // CreateParentPhysicsBone(rootNode, input);
        CreateParentConstraint(rootNode, input);
        CreateVolumeConstraint(rootNode, input, a_mergedConvexHulls);
        CreateOriginalConstraint(rootNode, input);
        UpdateVolume(rootNode, input, a_mergedConvexHulls);
        SetCenterPhysicsBone(rootNode, input, a_mergedConvexHulls);
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
        //input.defaultCollisionLayerGroup |= collisionLayerGroup;
        //input.defaultCollisionCollideLayer &= ~collisionLayerGroup;
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
        ConeConstraintData defaultConeConstraint;
        std::unordered_map<std::string, ConeConstraintData> templateConeConstraint;
        DeformConstraintData defaultDeformConstraint;
        std::unordered_map<std::string, DeformConstraintData> templateDeformConstraint;

        auto AddDistanceConstraint = [&](std::string_view A, std::string_view B, const DistanceConstraintData& consBase) {
            PhysicsInput::DistanceConstraint newCons;
            newCons.boneNameA = A;
            newCons.boneNameB = B;
            newCons.complianceSquish = consBase.complianceSquish * COMPLIANCE_SCALE,
            newCons.complianceStretch = consBase.complianceStretch * COMPLIANCE_SCALE,
            newCons.squishMargin = consBase.squishMargin,
            newCons.stretchMargin = consBase.stretchMargin,
            newCons.squishLimit = consBase.squishLimit,
            newCons.stretchLimit = consBase.stretchLimit,
            newCons.squishDamping = consBase.squishDamping,
            newCons.stretchDamping = consBase.stretchDamping,
            input.distanceConstraints.push_back(std::move(newCons));
            LoggingDistanceConstraint(file, A, B, consBase);
        };

        auto AddAngularConstraint = [&](std::string_view A, std::string_view B, const AngularConstraintData& consBase) {
            PhysicsInput::AngularConstraint newCons;
            newCons.boneNameA = A;
            newCons.boneNameB = B;
            newCons.compliancePositive = consBase.compliancePositive * COMPLIANCE_SCALE,
            newCons.complianceNegative = consBase.complianceNegative * COMPLIANCE_SCALE,
            newCons.marginPositive = consBase.marginPositive * toRadian,
            newCons.marginNegative = consBase.marginNegative * toRadian,
            newCons.limitPositive = consBase.limitPositive * toRadian,
            newCons.limitNegative = consBase.limitNegative * toRadian,
            newCons.dampingPositive = consBase.dampingPositive,
            newCons.dampingNegative = consBase.dampingNegative,
            input.angularConstraints.push_back(std::move(newCons));
            LoggingAngularConstraint(file, A, B, consBase);
        };

        auto AddConeConstraint = [&](std::string_view A, std::string_view B, const ConeConstraintData& consBase) {
            PhysicsInput::ConeConstraint newCons;
            newCons.boneNameA = A;
            newCons.boneNameB = B;
            newCons.compliancePositive = consBase.compliancePositive * COMPLIANCE_SCALE,
            newCons.complianceNegative = consBase.complianceNegative * COMPLIANCE_SCALE,
            newCons.marginPositive = consBase.marginPositive * toRadian,
            newCons.marginNegative = consBase.marginNegative * toRadian,
            newCons.limitPositive = consBase.limitPositive * toRadian,
            newCons.limitNegative = consBase.limitNegative * toRadian,
            newCons.dampingPositive = consBase.dampingPositive,
            newCons.dampingNegative = consBase.dampingNegative,
            input.coneConstraints.push_back(std::move(newCons));
            LoggingConeConstraint(file, A, B, consBase);
        };

        auto AddDeformConstraint = [&](std::string_view A, std::string_view B, const DeformConstraintData& consBase) {
            PhysicsInput::DeformConstraint newCons;
            newCons.boneNameA = A;
            newCons.boneNameB = B;
            newCons.squishWeight = consBase.squishWeight;
            newCons.stretchWeight = consBase.stretchWeight;
            newCons.bulgeWeight = consBase.bulgeWeight;
            input.deformConstraints.push_back(std::move(newCons));
            LoggingDeformConstraint(file, A, B, consBase);
        };

        auto AddShapeMatchingConstraint = [&](const ShapeMatchingConstraintData& consBase) {
            PhysicsInput::ShapeMatchingConstraint newCons;
            newCons.compliancePositive = consBase.compliancePositive * COMPLIANCE_SCALE;
            newCons.complianceNegative = consBase.complianceNegative * COMPLIANCE_SCALE;
            newCons.inertiaScale = consBase.inertiaScale;
            newCons.bones = consBase.bones;
            input.shapeMatchingConstraints.push_back(std::move(newCons));
            LoggingShapeMatchingConstraint(file, consBase);
        };

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
                LoggingBone(file, boneName, *newBone);
                newBone->animDriveCompliance *= COMPLIANCE_SCALE;
                newBone->animDriveAngularCompliance *= COMPLIANCE_SCALE;
                newBone->collisionCompliance *= COMPLIANCE_SCALE;
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
                DistanceConstraintData newCons = defaultDistanceConstraint;
                if (const char* templateName = elem->Attribute("template"); templateName)
                {
                    if (auto tit = templateDistanceConstraint.find(templateName); tit != templateDistanceConstraint.end())
                    {
                        newCons = tit->second;
                    }
                }
                tinyxml2::XMLElement* consElem = elem->FirstChildElement();
                while (consElem)
                {
                    GetDistanceConstraint(consElem->Name(), consElem, newCons);
                    consElem = consElem->NextSiblingElement();
                }
                AddDistanceConstraint(A, B, newCons);
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
                AngularConstraintData newCons = defaultAngularConstraint;
                if (const char* templateName = elem->Attribute("template"); templateName)
                {
                    if (auto tit = templateAngularConstraint.find(templateName); tit != templateAngularConstraint.end())
                    {
                        newCons = tit->second;
                    }
                }
                tinyxml2::XMLElement* consElem = elem->FirstChildElement();
                while (consElem)
                {
                    GetAngularConstraint(consElem->Name(), consElem, newCons);
                    consElem = consElem->NextSiblingElement();
                }
                AddAngularConstraint(A, B, newCons);
            }
            else if (elemName == "cone-constraint-default")
            {
                ConeConstraintData* currentCons = &defaultConeConstraint;
                if (const char* templateName = elem->Attribute("name"); templateName)
                {
                    templateConeConstraint.emplace(templateName, defaultConeConstraint);
                    currentCons = &templateConeConstraint[templateName];
                }
                tinyxml2::XMLElement* consElem = elem->FirstChildElement();
                while (consElem)
                {
                    GetConeConstraint(consElem->Name(), consElem, *currentCons);
                    consElem = consElem->NextSiblingElement();
                }
            }
            else if (elemName == "cone-constraint")
            {
                const char* A = elem->Attribute("A");
                const char* B = elem->Attribute("B");
                if (IsEmptyChar(A) || IsEmptyChar(B))
                {
                    elem = elem->NextSiblingElement();
                    continue;
                }
                ConeConstraintData newCons = defaultConeConstraint;
                if (const char* templateName = elem->Attribute("template"); templateName)
                {
                    if (auto tit = templateConeConstraint.find(templateName); tit != templateConeConstraint.end())
                    {
                        newCons = tit->second;
                    }
                }
                tinyxml2::XMLElement* consElem = elem->FirstChildElement();
                while (consElem)
                {
                    GetConeConstraint(consElem->Name(), consElem, newCons);
                    consElem = consElem->NextSiblingElement();
                }
                AddConeConstraint(A, B, newCons);
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
                DeformConstraintData newCons = defaultDeformConstraint;
                if (const char* templateName = elem->Attribute("template"); templateName)
                {
                    if (auto tit = templateDeformConstraint.find(templateName); tit != templateDeformConstraint.end())
                        newCons = tit->second;
                }
                tinyxml2::XMLElement* consElem = elem->FirstChildElement();
                while (consElem)
                {
                    GetDeformConstraint(consElem->Name(), consElem, newCons);
                    consElem = consElem->NextSiblingElement();
                }
                AddDeformConstraint(A, B, newCons);
            }
            else if (elemName == "shape-matching-constraint")
            {
                ShapeMatchingConstraintData newCons;
                tinyxml2::XMLElement* consElem = elem->FirstChildElement();
                while (consElem)
                {
                    GetShapeMatchingConstraint(consElem->Name(), consElem, newCons);
                    consElem = consElem->NextSiblingElement();
                }
                AddShapeMatchingConstraint(newCons);
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
                ConeConstraintData* currConeCons = &defaultConeConstraint;
                if (const char* templateName = elem->Attribute("name"); templateName)
                {
                    templateConeConstraint.emplace(templateName, defaultConeConstraint);
                    currConeCons = &templateConeConstraint[templateName];
                }
                DeformConstraintData* currDeformCons = &defaultDeformConstraint;
                if (const char* templateName = elem->Attribute("name"); templateName)
                {
                    templateDeformConstraint.emplace(templateName, defaultDeformConstraint);
                    currDeformCons = &templateDeformConstraint[templateName];
                }
                GetGenericConstraint(elem, currDistCons, currAngCons, currConeCons, currDeformCons);
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
                {
                    DistanceConstraintData consBase = defaultDistanceConstraint;
                    if (const char* templateName = elem->Attribute("template"); templateName)
                    {
                        if (auto tit = templateDistanceConstraint.find(templateName); tit != templateDistanceConstraint.end())
                            consBase = tit->second;
                    }
                    AddDistanceConstraint(A, B, consBase);
                }
                {
                    AngularConstraintData consBase = defaultAngularConstraint;
                    if (const char* templateName = elem->Attribute("template"); templateName)
                    {
                        if (auto tit = templateAngularConstraint.find(templateName); tit != templateAngularConstraint.end())
                            consBase = tit->second;
                    }
                    AddAngularConstraint(A, B, consBase);
                }
                {
                    DeformConstraintData consBase = defaultDeformConstraint;
                    if (const char* templateName = elem->Attribute("template"); templateName)
                    {
                        if (auto tit = templateDeformConstraint.find(templateName); tit != templateDeformConstraint.end())
                            consBase = tit->second;
                    }
                    AddDeformConstraint(A, B, consBase);
                }
            }
            else if (elemName == "cloth-cluster")
            {
                PhysicsInput::ClothCluster newCluster;
                GetClothCluster(elem, newCluster);
                LoggingClothCluster(file, newCluster);
                input.clothClusters.push_back(std::move(newCluster));
            }
            elem = elem->NextSiblingElement();
        }
        return true;
    }

    bool PhysicsConfigReader::GetDriverInput(const std::string& file, DriverInput& input) const
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
        return GetDriverInput(root, file, input);
    }
    bool PhysicsConfigReader::GetDriverInput(tinyxml2::XMLElement* root, const std::string& file, DriverInput& input) const
    {
        if (!root)
        {
            logger::error("{} : Unable to load root", file);
            return false;
        }

        tinyxml2::XMLElement* elem = root->FirstChildElement();
        while (elem)
        {
            const Mus::lString elemName = elem->Name();
            if (elemName == "DirectionDriver")
            {
                DriverInput::DirectionOffset newOffset;
                tinyxml2::XMLElement* offsetElem = elem->FirstChildElement();
                while (offsetElem)
                {
                    const Mus::lString offsetElemName = offsetElem->Name();
                    if (offsetElemName == "SourceBoneName")
                    {
                        if (const char* sourceBone = offsetElem->GetText(); !IsEmptyChar(sourceBone))
                            newOffset.srcBoneName = sourceBone;
                    }
                    else if (offsetElemName == "TargetBoneName")
                    {
                        if (const char* targetBone = offsetElem->GetText(); !IsEmptyChar(targetBone))
                            newOffset.dstBoneName = targetBone;
                    }
                    else if (offsetElemName == "IsWorldOffset")
                    {
                        offsetElem->BoolText(&newOffset.isWorldOffset);
                    }
                    else if (offsetElemName == "BaseDirection")
                    {
                        offsetElem->QueryFloatAttribute("x", &newOffset.baseDirection.x);
                        offsetElem->QueryFloatAttribute("y", &newOffset.baseDirection.y);
                        offsetElem->QueryFloatAttribute("z", &newOffset.baseDirection.z);
                    }
                    else if (offsetElemName == "TargetDirection")
                    {
                        offsetElem->QueryFloatAttribute("x", &newOffset.targetDirection.x);
                        offsetElem->QueryFloatAttribute("y", &newOffset.targetDirection.y);
                        offsetElem->QueryFloatAttribute("z", &newOffset.targetDirection.z);
                    }
                    else if (offsetElemName == "MinOffset")
                    {
                        offsetElem->QueryFloatAttribute("x", &newOffset.minOffset.x);
                        offsetElem->QueryFloatAttribute("y", &newOffset.minOffset.y);
                        offsetElem->QueryFloatAttribute("z", &newOffset.minOffset.z);
                    }
                    else if (offsetElemName == "MaxOffset")
                    {
                        offsetElem->QueryFloatAttribute("x", &newOffset.maxOffset.x);
                        offsetElem->QueryFloatAttribute("y", &newOffset.maxOffset.y);
                        offsetElem->QueryFloatAttribute("z", &newOffset.maxOffset.z);
                    }
                    offsetElem = offsetElem->NextSiblingElement();
                }
                logger::debug("{} : sourceBone {} / targetBone {} - isWorld {} / BaseDirection {} / TargetDirection {} / offset min{} max{}", file,
                             newOffset.srcBoneName, newOffset.dstBoneName, newOffset.isWorldOffset, newOffset.baseDirection, newOffset.targetDirection, newOffset.minOffset, newOffset.maxOffset);
                input.directionPosOffsets[file].push_back(std::move(newOffset));
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
        smpMigration.defaultSMPBone.mass = 1.0f;
        smpMigration.defaultClothCluster.gradientCurve = 0.0f;

        tinyxml2::XMLElement* root = doc.RootElement();
        tinyxml2::XMLElement* elem = root->FirstChildElement();
        while (elem)
        {
            const Mus::lString elemName = elem->Name();
            if (elemName == "bone-default")
            {
                tinyxml2::XMLElement* boneElem = elem->FirstChildElement();
                while (boneElem)
                {
                    GetBoneData(boneElem->Name(), boneElem, smpMigration.defaultSMPBone);
                    boneElem = boneElem->NextSiblingElement();
                }
                LoggingBone(file, "defaultSMPBone", smpMigration.defaultSMPBone);
            }
            else if (elemName == "distance-constraint-default")
            {
                tinyxml2::XMLElement* consElem = elem->FirstChildElement();
                while (consElem)
                {
                    GetDistanceConstraint(consElem->Name(), consElem, smpMigration.defaultSMPDistanceCons);
                    consElem = consElem->NextSiblingElement();
                }
                LoggingDistanceConstraint(file, "defaultSMPDistanceCons", "", smpMigration.defaultSMPDistanceCons);
            }
            else if (elemName == "angular-constraint-default")
            {
                tinyxml2::XMLElement* consElem = elem->FirstChildElement();
                while (consElem)
                {
                    GetAngularConstraint(consElem->Name(), consElem, smpMigration.defaultSMPAngularCons);
                    consElem = consElem->NextSiblingElement();
                }
                LoggingAngularConstraint(file, "defaultSMPAngularCons", "", smpMigration.defaultSMPAngularCons);
            }
            else if (elemName == "cone-constraint-default")
            {
                tinyxml2::XMLElement* consElem = elem->FirstChildElement();
                while (consElem)
                {
                    GetConeConstraint(consElem->Name(), consElem, smpMigration.defaultSMPConeCons);
                    consElem = consElem->NextSiblingElement();
                }
                LoggingConeConstraint(file, "defaultSMPConeCons", "", smpMigration.defaultSMPConeCons);
            }
            else if (elemName == "deform-constraint-default")
            {
                tinyxml2::XMLElement* consElem = elem->FirstChildElement();
                while (consElem)
                {
                    GetDeformConstraint(consElem->Name(), consElem, smpMigration.defaultSMPDeformCons);
                    consElem = consElem->NextSiblingElement();
                }
                LoggingDeformConstraint(file, "defaultSMPDeformCons", "", smpMigration.defaultSMPDeformCons);
            }
            else if (elemName == "generic-constraint-default")
            {
                GetGenericConstraint(elem, &smpMigration.defaultSMPDistanceCons, &smpMigration.defaultSMPAngularCons,&smpMigration.defaultSMPConeCons, &smpMigration.defaultSMPDeformCons);
            }
            else if (elemName == "cloth-cluster")
            {
                GetClothCluster(elem, smpMigration.defaultClothCluster);
                LoggingClothCluster(file, smpMigration.defaultClothCluster);
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
        std::vector<PhysicsInput::Bone> bones;
        std::unordered_map<std::string, PhysicsInput::Bone> templateBoneData;
        DistanceConstraintData defaultDistanceConstraint = smpMigration.defaultSMPDistanceCons;
        AngularConstraintData defaultAngularConstraint = smpMigration.defaultSMPAngularCons;
        ConeConstraintData defaultConeConstraint = smpMigration.defaultSMPConeCons;
        DeformConstraintData defaultDeformConstraint = smpMigration.defaultSMPDeformCons;
        PhysicsInput::ClothCluster defaultClothCluster = smpMigration.defaultClothCluster;
        PhysicsInput::Bone defaultConstraintBone;
        std::unordered_map<std::string, PhysicsInput::Bone> templateConsBone;
        struct ConsPair
        {
            std::string boneNameA;
            std::string boneNameB;
        };
        std::vector<ConsPair> constraintPairs;

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

        auto AssignBoneCons = [&](const char* A, const PhysicsInput::Bone& newConsBone) {
            auto it = input.bones.find(A);
            if (it == input.bones.end())
                return;
            it->second.dampingPositive = newConsBone.dampingPositive;
            it->second.dampingNegative = newConsBone.dampingNegative;
            it->second.angularDampingPositive = newConsBone.angularDampingPositive;
            it->second.angularDampingNegative = newConsBone.angularDampingNegative;
        };

        auto ConstraintAdd = [&](std::string_view A, std::string_view B) {
            if (Epsilon < defaultDistanceConstraint.complianceSquish + defaultDistanceConstraint.complianceStretch)
            {
                DistanceConstraintData newCons = defaultDistanceConstraint;
                newCons.boneNameA = A;
                newCons.boneNameB = B;
                newCons.complianceSquish *= COMPLIANCE_SCALE;
                newCons.complianceStretch *= COMPLIANCE_SCALE;
                input.distanceConstraints.push_back(newCons);
                LoggingDistanceConstraint(file, A, B, newCons);
            }
            if (!IsAllZero(defaultAngularConstraint.compliancePositive) && !IsAllZero(defaultAngularConstraint.complianceNegative))
            {
                AngularConstraintData newCons = defaultAngularConstraint;
                newCons.boneNameA = A;
                newCons.boneNameB = B;
                newCons.compliancePositive *= COMPLIANCE_SCALE;
                newCons.complianceNegative *= COMPLIANCE_SCALE;
                newCons.marginPositive *= toRadian;
                newCons.marginNegative *= toRadian;
                newCons.limitPositive *= toRadian;
                newCons.limitNegative *= toRadian;
                input.angularConstraints.push_back(newCons);
                LoggingAngularConstraint(file, A, B, newCons);
            }
            if (!IsAllZero(defaultConeConstraint.compliancePositive) && !IsAllZero(defaultConeConstraint.complianceNegative))
            {
                ConeConstraintData newCons = defaultConeConstraint;
                newCons.boneNameA = A;
                newCons.boneNameB = B;
                newCons.compliancePositive *= COMPLIANCE_SCALE;
                newCons.complianceNegative *= COMPLIANCE_SCALE;
                newCons.marginPositive *= toRadian;
                newCons.marginNegative *= toRadian;
                newCons.limitPositive *= toRadian;
                newCons.limitNegative *= toRadian;
                input.coneConstraints.push_back(newCons);
                LoggingConeConstraint(file, A, B, newCons);
            }
            if (!IsAllZero(defaultDeformConstraint.bulgeWeight) && !IsAllZero(defaultDeformConstraint.squishWeight) && !IsAllZero(defaultDeformConstraint.stretchWeight))
            {
                DeformConstraintData newCons = defaultDeformConstraint;
                newCons.boneNameA = A;
                newCons.boneNameB = B;
                input.deformConstraints.push_back(newCons);
                LoggingDeformConstraint(file, A, B, newCons);
            }
        };

        auto SMPConsBoneLogging = [](const std::string& file, std::string_view name, const PhysicsInput::Bone& bone) {
            logger::debug("{} : bone physics {} - linear damping neg{} pos{} / angular damping neg{} pos{}",
                          file, name, bone.dampingNegative, bone.dampingPositive, bone.angularDampingNegative, bone.angularDampingPositive);
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
                LoggingBone(file, boneName, *newBone);
                newBone->animDriveCompliance *= COMPLIANCE_SCALE;
                newBone->animDriveAngularCompliance *= COMPLIANCE_SCALE;
                newBone->collisionCompliance *= COMPLIANCE_SCALE;
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
                // ConsRead(elem, *currentConstraintBone);
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
                        constraintPairs.emplace_back(A, B);
                        PhysicsInput::Bone defaultConsCopy = defaultConstraintBone;
                        if (const char* templateName = elem->Attribute("template"); templateName)
                        {
                            if (auto tit = templateConsBone.find(templateName); tit != templateConsBone.end())
                            {
                                defaultConsCopy = tit->second;
                            }
                        }
                        // PhysicsInput::Bone* newConsBone = &defaultConsCopy;
                        // ConsRead(elem, *newConsBone);
                        // AssignBoneCons(A, *newConsBone);
                        // SMPConsBoneLogging(file, A, *newConsBone);
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
                constraintPairs.emplace_back(A, B);
                PhysicsInput::Bone defaultConsCopy = defaultConstraintBone;
                if (const char* templateName = elem->Attribute("template"); templateName)
                {
                    if (auto tit = templateConsBone.find(templateName); tit != templateConsBone.end())
                    {
                        defaultConsCopy = tit->second;
                    }
                }
                // PhysicsInput::Bone* newConsBone = &defaultConsCopy;
                // ConsRead(elem, *newConsBone);
                // AssignBoneCons(A, *newConsBone);
                // SMPConsBoneLogging(file, A, *newConsBone);
                ConstraintAdd(A, B);
            }
            elem = elem->NextSiblingElement();
        }

        if (!constraintPairs.empty() && defaultClothCluster.gradientCurve > Epsilon)
        {
            std::vector<std::string> clusterRoots;
            std::unordered_set<std::string> addedRoots;
            for (const auto& [boneA, boneB] : constraintPairs)
            {
                auto itA = input.bones.find(boneA);
                auto itB = input.bones.find(boneB);
                float massA = (itA != input.bones.end()) ? itA->second.mass : 0.0f;
                float massB = (itB != input.bones.end()) ? itB->second.mass : 0.0f;
                if (massA == 0.0f && massB > 0.0f)
                {
                    if (addedRoots.insert(boneB).second)
                    {
                        clusterRoots.push_back(boneB);
                        logger::debug("{} : added cloth cluster root {}", file, boneB);
                    }
                }
                if (massB == 0.0f && massA > 0.0f)
                {
                    if (addedRoots.insert(boneA).second)
                    {
                        clusterRoots.push_back(boneA);
                        logger::debug("{} : added cloth cluster root {}", file, boneA);
                    }
                }
            }
            if (!clusterRoots.empty())
            {
                PhysicsInput::ClothCluster newCluster = defaultClothCluster;
                newCluster.rootList = std::move(clusterRoots);
                LoggingClothCluster(file, newCluster);
                input.clear();
                input.clothClusters.push_back(std::move(newCluster));
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
            auto ait = map.find(cons.boneNameA);
            auto bit = map.find(cons.boneNameB);
            if (ait != map.end() || bit != map.end())
            {
                const std::string newA = ait != map.end() ? ait->second : cons.boneNameA;
                const std::string newB = bit != map.end() ? bit->second : cons.boneNameB;
                logger::debug("renamed anchor {} => {} | {} => {} for distance constraint", cons.boneNameA, newA, cons.boneNameB, newB);
                cons.boneNameA = newA;
                cons.boneNameB = newB;
            }
            fixedInput.distanceConstraints.push_back(std::move(cons));
        }
        for (auto& cons : input.angularConstraints)
        {
            auto ait = map.find(cons.boneNameA);
            auto bit = map.find(cons.boneNameB);
            if (ait != map.end() || bit != map.end())
            {
                const std::string newA = ait != map.end() ? ait->second : cons.boneNameA;
                const std::string newB = bit != map.end() ? bit->second : cons.boneNameB;
                logger::debug("renamed anchor {} => {} | {} => {} for angular constraint", cons.boneNameA, newA, cons.boneNameB, newB);
                cons.boneNameA = newA;
                cons.boneNameB = newB;
            }
            fixedInput.angularConstraints.push_back(std::move(cons));
        }
        for (auto& cons : input.deformConstraints)
        {
            auto ait = map.find(cons.boneNameA);
            auto bit = map.find(cons.boneNameB);
            if (ait != map.end() || bit != map.end())
            {
                const std::string newA = ait != map.end() ? ait->second : cons.boneNameA;
                const std::string newB = bit != map.end() ? bit->second : cons.boneNameB;
                logger::debug("renamed anchor {} => {} | {} => {} for deform constraint", cons.boneNameA, newA, cons.boneNameB, newB);
                cons.boneNameA = newA;
                cons.boneNameB = newB;
            }
            fixedInput.deformConstraints.push_back(std::move(cons));
        }
        for (auto& cluster : input.clothClusters)
        {
            for (auto& root : cluster.rootList)
            {
                auto it = map.find(root);
                if (it != map.end())
                {
                    logger::debug("renamed root {} => {} for cloth cluster", root, it->second);
                    root = it->second;
                }
            }
            for (auto& chain : cluster.chainList)
            {
                for (auto& bone : chain)
                {
                    auto it = map.find(bone);
                    if (it != map.end())
                    {
                        logger::debug("renamed chain bone {} => {} for cloth cluster", bone, it->second);
                        bone = it->second;
                    }
                }
            }
            fixedInput.clothClusters.push_back(std::move(cluster));
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
                    boneData.dampingPositive = abs(boneData.dampingPositive);
                }
                else if (elemName == "linearDampingNegative")
                {
                    elem->QueryFloatAttribute("x", &boneData.dampingNegative.x);
                    elem->QueryFloatAttribute("y", &boneData.dampingNegative.y);
                    elem->QueryFloatAttribute("z", &boneData.dampingNegative.z);
                    boneData.dampingNegative = abs(boneData.dampingNegative);
                }
                else if (elemName == "angularDampingPositive")
                {
                    elem->QueryFloatAttribute("x", &boneData.angularDampingPositive.x);
                    elem->QueryFloatAttribute("y", &boneData.angularDampingPositive.y);
                    elem->QueryFloatAttribute("z", &boneData.angularDampingPositive.z);
                    boneData.angularDampingPositive = abs(boneData.angularDampingPositive);
                }
                else if (elemName == "angularDampingNegative")
                {
                    elem->QueryFloatAttribute("x", &boneData.angularDampingNegative.x);
                    elem->QueryFloatAttribute("y", &boneData.angularDampingNegative.y);
                    elem->QueryFloatAttribute("z", &boneData.angularDampingNegative.z);
                    boneData.angularDampingNegative = abs(boneData.angularDampingNegative);
                }
                else if (elemName == "linearInertiaPositive")
                {
                    elem->QueryFloatAttribute("x", &boneData.inertiaPositive.x);
                    elem->QueryFloatAttribute("y", &boneData.inertiaPositive.y);
                    elem->QueryFloatAttribute("z", &boneData.inertiaPositive.z);
                    boneData.inertiaPositive = abs(boneData.inertiaPositive);
                }
                else if (elemName == "linearInertiaNegative")
                {
                    elem->QueryFloatAttribute("x", &boneData.inertiaNegative.x);
                    elem->QueryFloatAttribute("y", &boneData.inertiaNegative.y);
                    elem->QueryFloatAttribute("z", &boneData.inertiaNegative.z);
                    boneData.inertiaNegative = abs(boneData.inertiaNegative);
                }
                else if (elemName == "angularInertiaPositive")
                {
                    elem->QueryFloatAttribute("x", &boneData.angularInertiaPositive.x);
                    elem->QueryFloatAttribute("y", &boneData.angularInertiaPositive.y);
                    elem->QueryFloatAttribute("z", &boneData.angularInertiaPositive.z);
                    boneData.angularInertiaPositive = abs(boneData.angularInertiaPositive);
                }
                else if (elemName == "angularInertiaNegative")
                {
                    elem->QueryFloatAttribute("x", &boneData.angularInertiaNegative.x);
                    elem->QueryFloatAttribute("y", &boneData.angularInertiaNegative.y);
                    elem->QueryFloatAttribute("z", &boneData.angularInertiaNegative.z);
                    boneData.angularInertiaNegative = abs(boneData.angularInertiaNegative);
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
        else if (rootElemName == "limit")
        {
            elem->QueryFloatAttribute("squish", &consData.squishLimit);
            elem->QueryFloatAttribute("stretch", &consData.stretchLimit);
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
            consData.compliancePositive = abs(consData.compliancePositive);
        }
        else if (rootElemName == "complianceNegative")
        {
            elem->QueryFloatAttribute("x", &consData.complianceNegative.x);
            elem->QueryFloatAttribute("y", &consData.complianceNegative.y);
            elem->QueryFloatAttribute("z", &consData.complianceNegative.z);
            consData.complianceNegative = abs(consData.complianceNegative);
        }
        else if (rootElemName == "marginPositive")
        {
            elem->QueryFloatAttribute("x", &consData.marginPositive.x);
            elem->QueryFloatAttribute("y", &consData.marginPositive.y);
            elem->QueryFloatAttribute("z", &consData.marginPositive.z);
            consData.marginPositive = abs(consData.marginPositive);
        }
        else if (rootElemName == "marginNegative")
        {
            elem->QueryFloatAttribute("x", &consData.marginNegative.x);
            elem->QueryFloatAttribute("y", &consData.marginNegative.y);
            elem->QueryFloatAttribute("z", &consData.marginNegative.z);
            consData.marginNegative = -abs(consData.marginNegative);
        }
        else if (rootElemName == "limitPositive")
        {
            elem->QueryFloatAttribute("x", &consData.limitPositive.x);
            elem->QueryFloatAttribute("y", &consData.limitPositive.y);
            elem->QueryFloatAttribute("z", &consData.limitPositive.z);
            consData.limitPositive = abs(consData.limitPositive);
        }
        else if (rootElemName == "limitNegative")
        {
            elem->QueryFloatAttribute("x", &consData.limitNegative.x);
            elem->QueryFloatAttribute("y", &consData.limitNegative.y);
            elem->QueryFloatAttribute("z", &consData.limitNegative.z);
            consData.limitNegative = -abs(consData.limitNegative);
        }
        else if (rootElemName == "dampingPositive")
        {
            elem->QueryFloatAttribute("x", &consData.dampingPositive.x);
            elem->QueryFloatAttribute("y", &consData.dampingPositive.y);
            elem->QueryFloatAttribute("z", &consData.dampingPositive.z);
            consData.dampingPositive = abs(consData.dampingPositive);
        }
        else if (rootElemName == "dampingNegative")
        {
            elem->QueryFloatAttribute("x", &consData.dampingNegative.x);
            elem->QueryFloatAttribute("y", &consData.dampingNegative.y);
            elem->QueryFloatAttribute("z", &consData.dampingNegative.z);
            consData.dampingNegative = abs(consData.dampingNegative);
        }
    };

    void PhysicsConfigReader::GetConeConstraint(const Mus::lString& rootElemName, tinyxml2::XMLElement* elem, ConeConstraintData& consData) const
    {
        if (rootElemName.empty())
            return;
        if (rootElemName == "compliancePositive")
        {
            elem->QueryFloatAttribute("x", &consData.compliancePositive.x);
            //elem->QueryFloatAttribute("y", &consData.compliancePositive.y);
            elem->QueryFloatAttribute("z", &consData.compliancePositive.z);
            consData.compliancePositive = abs(consData.compliancePositive);
        }
        else if (rootElemName == "complianceNegative")
        {
            elem->QueryFloatAttribute("x", &consData.complianceNegative.x);
            //elem->QueryFloatAttribute("y", &consData.complianceNegative.y);
            elem->QueryFloatAttribute("z", &consData.complianceNegative.z);
            consData.complianceNegative = abs(consData.complianceNegative);
        }
        else if (rootElemName == "marginPositive")
        {
            elem->QueryFloatAttribute("x", &consData.marginPositive.x);
            //elem->QueryFloatAttribute("y", &consData.marginPositive.y);
            elem->QueryFloatAttribute("z", &consData.marginPositive.z);
            consData.marginPositive = abs(consData.marginPositive);
        }
        else if (rootElemName == "marginNegative")
        {
            elem->QueryFloatAttribute("x", &consData.marginNegative.x);
            //elem->QueryFloatAttribute("y", &consData.marginNegative.y);
            elem->QueryFloatAttribute("z", &consData.marginNegative.z);
            consData.marginNegative = abs(consData.marginNegative);
        }
        else if (rootElemName == "limitPositive")
        {
            elem->QueryFloatAttribute("x", &consData.limitPositive.x);
            //elem->QueryFloatAttribute("y", &consData.limitPositive.y);
            elem->QueryFloatAttribute("z", &consData.limitPositive.z);
            consData.limitPositive = abs(consData.limitPositive);
        }
        else if (rootElemName == "limitNegative")
        {
            elem->QueryFloatAttribute("x", &consData.limitNegative.x);
            //elem->QueryFloatAttribute("y", &consData.limitNegative.y);
            elem->QueryFloatAttribute("z", &consData.limitNegative.z);
            consData.limitNegative = abs(consData.limitNegative);
        }
        else if (rootElemName == "dampingPositive")
        {
            elem->QueryFloatAttribute("x", &consData.dampingPositive.x);
            //elem->QueryFloatAttribute("y", &consData.dampingPositive.y);
            elem->QueryFloatAttribute("z", &consData.dampingPositive.z);
            consData.dampingPositive = abs(consData.dampingPositive);
        }
        else if (rootElemName == "dampingNegative")
        {
            elem->QueryFloatAttribute("x", &consData.dampingNegative.x);
            //elem->QueryFloatAttribute("y", &consData.dampingNegative.y);
            elem->QueryFloatAttribute("z", &consData.dampingNegative.z);
            consData.dampingNegative = abs(consData.dampingNegative);
        }
    }

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

    void PhysicsConfigReader::GetShapeMatchingConstraint(const Mus::lString& rootElemName, tinyxml2::XMLElement* rootElem, ShapeMatchingConstraintData& consData) const
    {
        if (rootElemName.empty())
            return;
        if (rootElemName == "compliancePositive")
        {
            rootElem->QueryFloatAttribute("x", &consData.compliancePositive.x);
            rootElem->QueryFloatAttribute("y", &consData.compliancePositive.y);
            rootElem->QueryFloatAttribute("z", &consData.compliancePositive.z);
            consData.compliancePositive = abs(consData.compliancePositive);
        }
        else if (rootElemName == "complianceNegative")
        {
            rootElem->QueryFloatAttribute("x", &consData.complianceNegative.x);
            rootElem->QueryFloatAttribute("y", &consData.complianceNegative.y);
            rootElem->QueryFloatAttribute("z", &consData.complianceNegative.z);
            consData.complianceNegative = abs(consData.complianceNegative);
        }
        else if (rootElemName == "inertiaScale")
        {
            rootElem->QueryFloatAttribute("x", &consData.inertiaScale.x);
            rootElem->QueryFloatAttribute("y", &consData.inertiaScale.y);
            rootElem->QueryFloatAttribute("z", &consData.inertiaScale.z);
        }
        else if (rootElemName == "bones")
        {
            tinyxml2::XMLElement* elem = rootElem->FirstChildElement();
            while (elem)
            {
                const Mus::lString elemName = elem->Name();
                if (elemName == "bone")
                {
                    const char* boneName = elem->GetText();
                    if (!IsEmptyChar(boneName))
                        consData.bones.push_back(boneName);
                }
                elem = elem->NextSiblingElement();
            }
        }
    }

    void PhysicsConfigReader::GetGenericConstraint(tinyxml2::XMLElement* rootElem, DistanceConstraintData* distConsData, AngularConstraintData* angConsData, ConeConstraintData* coneConsData, DeformConstraintData* deformConsData) const
    {
        tinyxml2::XMLElement* elem = rootElem->FirstChildElement();
        while (elem)
        {
            const Mus::lString elemName = elem->Name();
            if (elemName == "distance" && distConsData)
            {
                tinyxml2::XMLElement* consElem = elem->FirstChildElement();
                while (consElem)
                {
                    const Mus::lString consElemName = consElem->Name();
                    GetDistanceConstraint(consElemName, consElem, *distConsData);
                    consElem = consElem->NextSiblingElement();
                }
            }
            else if (elemName == "angular" && angConsData)
            {
                tinyxml2::XMLElement* consElem = elem->FirstChildElement();
                while (consElem)
                {
                    const Mus::lString consElemName = consElem->Name();
                    GetAngularConstraint(consElemName, consElem, *angConsData);
                    consElem = consElem->NextSiblingElement();
                }
            }
            else if (elemName == "cone" && angConsData)
            {
                tinyxml2::XMLElement* consElem = elem->FirstChildElement();
                while (consElem)
                {
                    const Mus::lString consElemName = consElem->Name();
                    GetConeConstraint(consElemName, consElem, *coneConsData);
                    consElem = consElem->NextSiblingElement();
                }
            }
            else if (elemName == "deform" && deformConsData)
            {
                tinyxml2::XMLElement* consElem = elem->FirstChildElement();
                while (consElem)
                {
                    const Mus::lString consElemName = consElem->Name();
                    GetDeformConstraint(consElemName, consElem, *deformConsData);
                    consElem = consElem->NextSiblingElement();
                }
            }
            elem = elem->NextSiblingElement();
        }
    }

    void PhysicsConfigReader::GetClothCluster(tinyxml2::XMLElement* rootElem, PhysicsInput::ClothCluster& clothCluster) const
    {
        tinyxml2::XMLElement* elem = rootElem->FirstChildElement();
        while (elem)
        {
            const Mus::lString elemName = elem->Name();
            if (elemName == "RootList")
            {
                tinyxml2::XMLElement* rootListElem = elem->FirstChildElement();
                while (rootListElem)
                {
                    const Mus::lString rootListElemName = rootListElem->Name();
                    if (rootListElemName == "Bone")
                    {
                        const char* boneName = rootListElem->GetText();
                        if (!IsEmptyChar(boneName))
                        {
                            clothCluster.rootList.push_back(boneName);
                        }
                    }
                    rootListElem = rootListElem->NextSiblingElement();
                }
            }
            else if (elemName == "ChainList")
            {
                tinyxml2::XMLElement* chainListElem = elem->FirstChildElement();
                std::vector<std::string> chain;
                while (chainListElem)
                {
                    const Mus::lString chainListElemName = chainListElem->Name();
                    if (chainListElemName == "Bone")
                    {
                        const char* boneName = chainListElem->GetText();
                        if (!IsEmptyChar(boneName))
                        {
                            chain.push_back(boneName);
                        }
                    }
                    chainListElem = chainListElem->NextSiblingElement();
                }
                if (!chain.empty())
                    clothCluster.chainList.push_back(std::move(chain));
            }
            else if (elemName == "Setting")
            {
                auto GetSetting = [this](tinyxml2::XMLElement* rootElem, PhysicsInput::Bone& bone, 
                                     DistanceConstraintData& distCons, 
                                     AngularConstraintData& angCons,
                                     ConeConstraintData& coneCons,
                                     DeformConstraintData& deformCons) {
                    tinyxml2::XMLElement* elem = rootElem->FirstChildElement();
                    while (elem)
                    {
                        const Mus::lString elemName = elem->Name();
                        if (elemName == "Bone")
                        {
                            tinyxml2::XMLElement* boneElem = elem->FirstChildElement();
                            while (boneElem)
                            {
                                const Mus::lString boneElemName = boneElem->Name();
                                GetBoneData(boneElemName, boneElem, bone);
                                boneElem = boneElem->NextSiblingElement();
                            }
                        }
                        else if (elemName == "distance-constraint")
                        {
                            tinyxml2::XMLElement* consElem = elem->FirstChildElement();
                            while (consElem)
                            {
                                const Mus::lString consElemName = consElem->Name();
                                GetDistanceConstraint(consElemName, consElem, distCons);
                                consElem = consElem->NextSiblingElement();
                            }
                        }
                        else if (elemName == "angular-constraint")
                        {
                            tinyxml2::XMLElement* consElem = elem->FirstChildElement();
                            while (consElem)
                            {
                                const Mus::lString consElemName = consElem->Name();
                                GetAngularConstraint(consElemName, consElem, angCons);
                                consElem = consElem->NextSiblingElement();
                            }
                        }
                        else if (elemName == "cone-constraint")
                        {
                            tinyxml2::XMLElement* consElem = elem->FirstChildElement();
                            while (consElem)
                            {
                                const Mus::lString consElemName = consElem->Name();
                                GetConeConstraint(consElemName, consElem, coneCons);
                                consElem = consElem->NextSiblingElement();
                            }
                        }
                        else if (elemName == "deform-constraint")
                        {
                            tinyxml2::XMLElement* consElem = elem->FirstChildElement();
                            while (consElem)
                            {
                                const Mus::lString consElemName = consElem->Name();
                                GetDeformConstraint(consElemName, consElem, deformCons);
                                consElem = consElem->NextSiblingElement();
                            }
                        }
                        elem = elem->NextSiblingElement();
                    }
                    return 0.0f;
                };
                tinyxml2::XMLElement* settingElem = elem->FirstChildElement();
                while (settingElem)
                {
                    const Mus::lString settingElemName = settingElem->Name();
                    if (settingElemName == "GradientCurve")
                        settingElem->QueryFloatText(&clothCluster.gradientCurve);
                    else if (settingElemName == "distanceHorizontal")
                        settingElem->QueryFloatText(&clothCluster.distanceHorizontalMultiplier);
                    else if (settingElemName == "distanceDiagonal")
                        settingElem->QueryFloatText(&clothCluster.distanceDiagonalMultiplier);
                    else if (settingElemName == "angularHorizontal")
                        settingElem->QueryFloatText(&clothCluster.angularHorizontalMultiplier);
                    else if (settingElemName == "StiffnessChainCount")
                        settingElem->QueryIntText(&clothCluster.StiffnessChainCount);
                    else if (settingElemName == "StiffnessStartCompliance")
                        settingElem->QueryFloatText(&clothCluster.StiffnessStartCompliance);
                    else if (settingElemName == "StiffnessEndCompliance")
                        settingElem->QueryFloatText(&clothCluster.StiffnessEndCompliance);
                    else if (settingElemName == "Root")
                        GetSetting(settingElem, clothCluster.root.bone, clothCluster.root.distanceConstraint, clothCluster.root.angularConstraint, clothCluster.root.coneConstraint, clothCluster.root.deformConstraint);
                    else if (settingElemName == "Tip")
                        GetSetting(settingElem, clothCluster.tip.bone, clothCluster.tip.distanceConstraint, clothCluster.tip.angularConstraint, clothCluster.tip.coneConstraint, clothCluster.tip.deformConstraint);
                    settingElem = settingElem->NextSiblingElement();
                }
            }
            elem = elem->NextSiblingElement();
        }
    }

    void PhysicsConfigReader::LoggingBone(const std::string& file, std::string_view name, const PhysicsInput::Bone& bone) const
    {
        logger::debug("{} : bone physics {} - mass {} / inertia neg{} pos{} / angularInertia neg{} pos{} / angularBlendFactor {} / gravity {} / windFactor {} / linearRotTorque x{} y{}, z{}", 
                      file, name, bone.mass, bone.inertiaNegative, bone.inertiaPositive, bone.angularInertiaNegative, bone.angularInertiaPositive, bone.angularBlendFactor, bone.gravity, bone.windFactor, bone.linearRotTorque[0], bone.linearRotTorque[1], bone.linearRotTorque[2]);
        logger::debug("{} : bone offset {} - pos {}{}{}{}", 
                      file, name, bone.offset, bone.isParticle ? " /" : "", bone.isParticle ? " target " : "", bone.isParticle ? bone.parentBoneName : "");
        logger::debug("{} : bone animDrive {} - linear compliance {} / angular compliance {} / linear damping neg{} pos{} / angular damping neg{} pos{}", 
                      file, name, bone.animDriveCompliance, bone.animDriveAngularCompliance, bone.dampingNegative, bone.dampingPositive, bone.angularDampingNegative, bone.angularDampingPositive);
        logger::debug("{} : bone collider {} - margin {} / friction {} / softness {} / restitution {} / layerGroup {:x} / collideLayer {:x} / colliderType {}", 
                      file, name, bone.collisionMargin, bone.collisionFriction, bone.collisionCompliance, bone.collisionRestitution, bone.collisionLayerGroup, bone.collisionCollideLayer, GetColliderTypeStr(bone.colliderType));  
        logger::debug("{} : bone deform {} - limit min{} max{} / volume preservation {} / sensitivity squish{} stretch{} bulge{} / stiffness squish{} stretch{} / damping squish{} stretch{}", 
                      file, name, bone.deformMin, bone.deformMax, bone.deformVolumePreservation, bone.deformSquishSensitivity, bone.deformStretchSensitivity, bone.deformBulgeSensitivity, bone.deformSquishStiffness, bone.deformStretchStiffness, bone.deformSquishDamping, bone.deformStretchDamping);   
        if (bone.enableDynamicVolume)
            logger::debug("{} : bone dynamic volume {} - volume min({}) max({}) / physicsScale min({}) max({}) / clampPhysicsScale {}", 
                          file, name, bone.volumeMin, bone.volumeMax, bone.physicsScaleMin, bone.physicsScaleMax, bone.clampPhysicsScale);
    }

    void PhysicsConfigReader::LoggingDistanceConstraint(const std::string& file, const std::string_view A, const std::string_view B, const DistanceConstraintData& cons) const
    {
        logger::debug("{} : distance constraint {}|{} - compliance squish({}) stretch({}) / margin squish({}) stretch({}) / limit squish({}) stretch({}) / damping squish({}) stretch({})", 
                      file, A, B, cons.complianceSquish, cons.complianceStretch, cons.squishMargin, cons.stretchMargin, cons.squishLimit, cons.stretchLimit, cons.squishDamping, cons.stretchDamping);         
    }

    void PhysicsConfigReader::LoggingAngularConstraint(const std::string& file, const std::string_view A, const std::string_view B, const AngularConstraintData& cons) const
    {
        logger::debug("{} : angular constraint {}|{} - compliance neg{} pos{} / margin neg{} pos{} / limit neg{} pos{} / damping neg{} pos{}", 
                      file, A, B, cons.complianceNegative, cons.compliancePositive, cons.marginNegative, cons.marginPositive, cons.limitNegative, cons.limitPositive, cons.dampingNegative, cons.dampingPositive);     
    }

    void PhysicsConfigReader::LoggingConeConstraint(const std::string& file, const std::string_view A, const std::string_view B, const ConeConstraintData& cons) const
    {
        logger::debug("{} : cone constraint {}|{} - compliance neg{} pos{} / margin neg{} pos{} / limit neg{} pos{} / damping neg{} pos{}", 
                      file, A, B, cons.complianceNegative, cons.compliancePositive, cons.marginNegative, cons.marginPositive, cons.limitNegative, cons.limitPositive, cons.dampingNegative, cons.dampingPositive);     
    }

    void PhysicsConfigReader::LoggingDeformConstraint(const std::string& file, const std::string_view A, const std::string_view B, const DeformConstraintData& cons) const
    {
        logger::debug("{} : deform constraint {}|{} - weight squish{} stretch{} bulge{}", 
                      file, A, B, cons.squishWeight, cons.stretchWeight, cons.bulgeWeight);     
    }

    void PhysicsConfigReader::LoggingShapeMatchingConstraint(const std::string& file, const ShapeMatchingConstraintData& cons) const
    {
        std::string boneListStr = "";
        for (const auto& bone : cons.bones)
        {
            if (!boneListStr.empty())
                boneListStr += ", ";
            boneListStr += bone;
        }
        logger::debug("{} : shape matching constraint - compliance neg{} pos{} / inertiaScale {} / bones {}", 
                      file, cons.complianceNegative, cons.compliancePositive, cons.inertiaScale, boneListStr);
    }

    void PhysicsConfigReader::LoggingClothCluster(const std::string& file, PhysicsInput::ClothCluster& clothCluster) const
    {
        logger::debug("{} : cloth cluster - gradientCurve {}", file, clothCluster.gradientCurve);
        LoggingBone(file, "root", clothCluster.root.bone);
        LoggingDistanceConstraint(file, "root", "", clothCluster.root.distanceConstraint);
        LoggingAngularConstraint(file, "root", "", clothCluster.root.angularConstraint);
        LoggingDeformConstraint(file, "root", "", clothCluster.root.deformConstraint);
        LoggingBone(file, "tip", clothCluster.tip.bone);
        LoggingDistanceConstraint(file, "tip", "", clothCluster.tip.distanceConstraint);
        LoggingAngularConstraint(file, "tip", "", clothCluster.tip.angularConstraint);
        LoggingDeformConstraint(file, "tip", "", clothCluster.tip.deformConstraint);
        std::string rootListStr = "";
        for (const auto& root : clothCluster.rootList)
        {
            if (!rootListStr.empty())
                rootListStr += ", ";
            rootListStr += root;
        }
        if (!rootListStr.empty())
            logger::debug("{} : cloth cluster root bone - {}", file, rootListStr);
        for (const auto& chain : clothCluster.chainList)
        {
            std::string chainListStr = "";
            for (const auto& bone : chain)
            {
                if (!chainListStr.empty())
                    chainListStr += ", ";
                chainListStr += bone;
            }
            if (!chainListStr.empty())
                logger::debug("{} : cloth cluster chain bone - {}", file, chainListStr);
        }
    }
}