#include "MXPBD/PhysicsWorld.h"

namespace MXPBD
{
    XPBDWorld::XPBDWorld()
    {
        driver = std::make_unique<XPBDDriver>();
    }

    void XPBDWorld::SetThreads(std::int32_t subtractThreadCount)
    {
        WaitForPhysicsWorldAsync();
        std::uint32_t pCoreCount = 0;
        std::uint64_t pCoreMask = 0;
        GetPCore(pCoreCount, pCoreMask);
        std::uint32_t totalCoreCount = std::clamp(pCoreCount - subtractThreadCount, 1u, pCoreCount);
        if (!threadPool)
            threadPool = std::make_unique<TBB_ThreadPool>(pCoreCount, pCoreMask);
        threadPoolAsync = std::make_unique<TBB_ThreadPool>(totalCoreCount, pCoreMask);
    }

    void XPBDWorld::AddPhysics(RE::TESObjectREFR* object, RE::NiNode* rootNode, const Internal::RootType rootType, const PhysicsInput& input)
    {
        if (!object || !rootNode)
            return;
        if (input.empty())
            return;
        if (rootType == XPBDWorld::RootType::kNone || rootType == Internal::RootType::kCollider)
            return;

        WaitForPhysicsWorldAsync();

        const Internal::ObjectDatas::Root newRoot = {.type = rootType, .bipedSlot = input.bipedSlot};

        std::lock_guard lg(lock);
        CheckUpdate();

        std::uint32_t objIdx = FindObject(object);
        if (objIdx == UINT32_MAX)
            objIdx = AllocateObject(object);
        std::uint32_t rootIdx = FindRoot(objIdx, newRoot);
        if (rootIdx != UINT32_MAX)
        {
            logger::info("{:x} : physics already exists. so skipping adding physics", object->formID);
            return;
        }
        else
            rootIdx = AllocateRoot(objIdx, newRoot);

        auto& nameMap = boneNameToIdx[objIdx];

        std::unordered_map<std::string, std::vector<std::string>> particleParentToName;
        for (const auto& bone : input.bones)
        {
            if (!bone.second.isParticle)
                continue;
            if (nameMap.find(bone.first) != nameMap.end())
                continue;
            particleParentToName[bone.second.parentBoneName].push_back(bone.first);
        }

        ReserveBone(input.bones.size());
        ReserveDistanceConstraint(input.distanceConstraints.size());
        ReserveAngularConstraint(input.angularConstraints.size());
        ReserveConeConstraint(input.coneConstraints.size());
        ReserveDeformConstraint(input.deformConstraints.size());
        ReserveShapeMatchingConstraint(input.deformConstraints.size());

        UpdateChildTreeData(rootNode, RE::NiUpdateData::Flag::kDirty);

        // add bone data
        Mus::nif::VisitObjects(rootNode, [&](RE::NiAVObject* node, std::uint32_t depth) {
            if (!node || node->name.empty())
                return true;
            std::string_view nodeName(node->name.c_str());
            if (nodeName == "NPC")
            {
                RE::NiNode* npcNode = node->AsNode();
                if (npcNode)
                    objectDatas.npcNode[objIdx] = RE::NiPointer(npcNode);
                return true;
            }

            std::uint32_t bi = 0;
            if (auto boneIdxIt = nameMap.find(node->name.c_str()); boneIdxIt != nameMap.end())
                bi = boneIdxIt->second;
            else
            {
                auto found = input.bones.find(node->name.c_str());
                if (found == input.bones.end())
                    return true;

                const float physicsScale = found->second.GetPhysicsScaleByVolume();
                const Vector vPhysicsScale = DirectX::XMVectorReplicate(physicsScale);
                logger::info("{:x} : add physics bone {}{}{}", object->formID, found->first,
                             found->second.enableDynamicVolume ? "(volume : " + std::to_string(found->second.currentVolume) + "L / " : "",
                             found->second.enableDynamicVolume ? "PhysicsScale : " + std::to_string(physicsScale) + ")" : "");
                bi = AllocateBone();
                const Vector offset = DirectX::XMVector3Rotate(DirectX::XMVectorScale(ToVector(found->second.offset), node->world.scale), ToQuaternion(node->world.rotate));
                physicsBones.pos[bi] = DirectX::XMVectorAdd(ToVector(node->world.translate), offset);
                physicsBones.predPos[bi] = physicsBones.pos[bi];
                physicsBones.posVel[bi] = vZero;

                physicsBones.rot[bi] = ToQuaternion(node->world.rotate);
                physicsBones.predRot[bi] = physicsBones.rot[bi];
                physicsBones.angVel[bi] = vZero;

                physicsBones.offset[bi] = ToVector(found->second.offset);

                physicsBones.deformScale[bi] = vmIdentity;
                physicsBones.deformScaleCache[bi] = vmZeroAll;
                physicsBones.deformVelocityScale[bi] = vmZeroAll;
                physicsBones.deformCount[bi] = 0;

                physicsBones.node[bi] = RE::NiPointer(node);
                physicsBones.isParticle[bi] = 0;
                physicsBones.parentBoneIdx[bi] = UINT32_MAX;
                if (auto parent = node->parent; parent && found->second.mass > Epsilon)
                {
                    auto pit = nameMap.find(parent->name.c_str());
                    while (pit == nameMap.end())
                    {
                        parent = parent->parent;
                        if (!parent->name.empty())
                            pit = nameMap.find(parent->name.c_str());
                    }
                    if (pit != nameMap.end())
                        physicsBones.parentBoneIdx[bi] = pit->second;
                }
                physicsBones.objIdx[bi] = objIdx;
                physicsBones.rootIdx[bi] = rootIdx;
                physicsBones.depth[bi] = depth;

                physicsBones.prevNodeWorldPos[bi] = physicsBones.pos[bi];
                physicsBones.targetNodeWorldPos[bi] = physicsBones.pos[bi];
                physicsBones.prevNodeWorldRot[bi] = physicsBones.rot[bi];
                physicsBones.targetNodeWorldRot[bi] = physicsBones.rot[bi];

                const Vector localPos = ToVector(node->local.translate);
                const Vector boneAxis = DirectX::XMVector3Normalize(localPos);
                const Vector cross = DirectX::XMVector3Cross(vYone, boneAxis);
                const float dot = DirectX::XMVectorGetX(DirectX::XMVector3Dot(vYone, boneAxis));
                Quaternion qAlign;
                if (dot < -0.9999f)
                    qAlign = DirectX::XMQuaternionRotationAxis(vXone, DirectX::XM_PI);
                else
                {
                    qAlign = DirectX::XMVectorSetW(cross, 1.0f + dot);
                    qAlign = DirectX::XMQuaternionNormalize(qAlign);
                }
                physicsBones.alignRot[bi] = qAlign;
                physicsBones.invAlignRot[bi] = DirectX::XMQuaternionConjugate(qAlign);

                physicsBones.orgWorldScale[bi] = node->world.scale;
                physicsBones.orgLocalPos[bi] = localPos;
                physicsBones.orgLocalRot[bi] = ToQuaternion(node->local.rotate);

                SetBone(bi, found->second);

                nameMap[found->first] = bi;
            }

            if (auto pptn = particleParentToName.find(node->name.c_str()); pptn != particleParentToName.end())
            {
                auto func = [this, &object, &input, &nameMap, objIdx, rootIdx, depth, &particleParentToName](auto&& func, const std::string& parentName, const std::vector<std::string>& pptnList, const std::uint32_t parentIdx, std::uint32_t particleDepth) -> void {
                    for (const auto& particleName : pptnList)
                    {
                        const auto pit = input.bones.find(particleName);
                        if (pit == input.bones.end())
                            continue;
                        if (nameMap.find(particleName) != nameMap.end())
                            continue;

                        const float physicsScale = physicsBones.physicsScale[parentIdx];
                        const Vector vPhysicsScale = DirectX::XMVectorReplicate(physicsScale);
                        logger::info("{:x} : add physics particle({}) bone {}{} for {}", object->formID, particleDepth, particleName,
                                     pit->second.enableDynamicVolume ? "(PhysicsScale : " + std::to_string(physicsScale) + ")" : "", parentName);
                        const std::uint32_t particleBi = AllocateBone();
                        const Vector offset = DirectX::XMVector3Rotate(DirectX::XMVectorScale(ToVector(pit->second.offset), physicsBones.orgWorldScale[parentIdx]), physicsBones.rot[parentIdx]);
                        physicsBones.pos[particleBi] = DirectX::XMVectorAdd(physicsBones.pos[parentIdx], offset);
                        physicsBones.predPos[particleBi] = physicsBones.pos[particleBi];
                        physicsBones.posVel[particleBi] = vZero;

                        physicsBones.rot[particleBi] = physicsBones.rot[parentIdx];
                        physicsBones.predRot[particleBi] = physicsBones.rot[particleBi];
                        physicsBones.angVel[particleBi] = vZero;

                        physicsBones.offset[particleBi] = ToVector(pit->second.offset);

                        physicsBones.deformScale[particleBi] = vmIdentity;
                        physicsBones.deformScaleCache[particleBi] = vmZeroAll;
                        physicsBones.deformVelocityScale[particleBi] = vmZeroAll;
                        physicsBones.deformCount[particleBi] = 0;

                        physicsBones.node[particleBi] = nullptr;
                        physicsBones.particleName[particleBi] = pit->first;
                        physicsBones.isParticle[particleBi] = 1;
                        physicsBones.particleDepth[particleBi] = particleDepth;
                        physicsBones.parentBoneIdx[particleBi] = parentIdx;
                        physicsBones.objIdx[particleBi] = objIdx;
                        physicsBones.rootIdx[particleBi] = rootIdx;
                        physicsBones.depth[particleBi] = depth;

                        physicsBones.prevNodeWorldPos[particleBi] = physicsBones.pos[parentIdx];
                        physicsBones.targetNodeWorldPos[particleBi] = physicsBones.pos[parentIdx];
                        physicsBones.prevNodeWorldRot[particleBi] = physicsBones.rot[parentIdx];
                        physicsBones.targetNodeWorldRot[particleBi] = physicsBones.rot[parentIdx];

                        physicsBones.orgWorldScale[particleBi] = physicsBones.orgWorldScale[parentIdx];
                        physicsBones.orgLocalPos[particleBi] = vZero;
                        physicsBones.orgLocalRot[particleBi] = qZero;

                        SetBone(particleBi, pit->second);

                        nameMap[pit->first] = particleBi;

                        if (const auto pptn = particleParentToName.find(pit->first); pptn != particleParentToName.end())
                        {
                            func(func, pptn->first, pptn->second, particleBi, particleDepth + 1);
                        }
                    }
                };
                func(func, pptn->first, pptn->second, bi, 0);
            }
            return true;
        });

        // add distance constraint
        for (const auto& distanceConstraint : input.distanceConstraints)
        {
            auto bitA = nameMap.find(distanceConstraint.boneNameA);
            if (bitA == nameMap.end())
            {
                logger::error("{:x} : Unable to get physics node {} for distance constrant", object->formID, distanceConstraint.boneNameA);
                continue;
            }
            auto bitB = nameMap.find(distanceConstraint.boneNameB);
            if (bitB == nameMap.end())
            {
                logger::error("{:x} : Unable to get physics node {} for distance constrant", object->formID, distanceConstraint.boneNameB);
                continue;
            }

            const std::uint32_t ci = AllocateDistanceConstraint();
            distanceConstraints.boneIdx[ci] = bitA->second;
            distanceConstraints.anchIdx[ci] = bitB->second;
            distanceConstraints.objIdx[ci] = objIdx;
            distanceConstraints.rootIdx[ci] = rootIdx;

            const float physicsScale = (physicsBones.physicsScale[bitA->second] + physicsBones.physicsScale[bitB->second]) * 0.5f;
            const Vector tPos = physicsBones.pos[bitA->second];
            const Vector aPos = physicsBones.pos[bitB->second];
            distanceConstraints.restLen[ci] = DirectX::XMVectorGetX(DirectX::XMVector3Length(DirectX::XMVectorSubtract(tPos, aPos)));
            logger::info("{:x} : add distance constraint {}-{}", object->formID, distanceConstraint.boneNameA, distanceConstraint.boneNameB);
            SetDistanceConstraint(ci, distanceConstraint, physicsScale);
        }

        // add angular constraint
        for (const auto& angularConstraint : input.angularConstraints)
        {
            auto bitA = nameMap.find(angularConstraint.boneNameA);
            if (bitA == nameMap.end())
            {
                logger::error("{:x} : Unable to get physics node {} for angular constrant", object->formID, angularConstraint.boneNameA);
                continue;
            }
            auto bitB = nameMap.find(angularConstraint.boneNameB);
            if (bitB == nameMap.end())
            {
                logger::error("{:x} : Unable to get physics node {} for angular constrant", object->formID, angularConstraint.boneNameB);
                continue;
            }

            const std::uint32_t ci = AllocateAngularConstraint();
            angularConstraints.boneIdx[ci] = bitA->second;
            angularConstraints.anchIdx[ci] = bitB->second;
            angularConstraints.objIdx[ci] = objIdx;
            angularConstraints.rootIdx[ci] = rootIdx;

            physicsBones.advancedRotation[bitA->second] = 1;
            physicsBones.advancedRotation[bitB->second] = 1;

            const float physicsScale = (physicsBones.physicsScale[bitA->second] + physicsBones.physicsScale[bitB->second]) * 0.5f;
            const Quaternion childRot = physicsBones.rot[bitA->second];
            const Quaternion anchorRot = physicsBones.rot[bitB->second];
            const Quaternion anchorRotInv = DirectX::XMQuaternionInverse(anchorRot);
            angularConstraints.restRot[ci] = DirectX::XMQuaternionNormalize(DirectX::XMQuaternionMultiply(childRot, anchorRotInv));
            logger::info("{:x} : add angular constraint {}-{}", object->formID, angularConstraint.boneNameA, angularConstraint.boneNameB);
            SetAngularConstraint(ci, angularConstraint, physicsScale);
        }

        // add cone constraint
        for (const auto& coneConstraint : input.coneConstraints)
        {
            auto bitA = nameMap.find(coneConstraint.boneNameA);
            if (bitA == nameMap.end())
            {
                logger::error("{:x} : Unable to get physics node {} for cone constrant", object->formID, coneConstraint.boneNameA);
                continue;
            }
            auto bitB = nameMap.find(coneConstraint.boneNameB);
            if (bitB == nameMap.end())
            {
                logger::error("{:x} : Unable to get physics node {} for cone constrant", object->formID, coneConstraint.boneNameB);
                continue;
            }

            const std::uint32_t ci = AllocateConeConstraint();
            coneConstraints.boneIdx[ci] = bitA->second;
            coneConstraints.anchIdx[ci] = bitB->second;
            coneConstraints.objIdx[ci] = objIdx;
            coneConstraints.rootIdx[ci] = rootIdx;

            const float physicsScale = (physicsBones.physicsScale[bitA->second] + physicsBones.physicsScale[bitB->second]) * 0.5f;
            const Vector tPos = physicsBones.pos[bitA->second];
            const Vector aPos = physicsBones.pos[bitB->second];
            Vector worldDir = DirectX::XMVectorSubtract(aPos, tPos);
            if (DirectX::XMVector3Less(DirectX::XMVector3LengthSq(worldDir), vEpsilon))
                worldDir = DirectX::XMVector3Rotate(physicsBones.orgLocalPos[bitB->second], physicsBones.rot[bitB->second]);
            worldDir = DirectX::XMVector3Normalize(worldDir);
            const Vector localDir = DirectX::XMVector3Rotate(worldDir, DirectX::XMQuaternionConjugate(physicsBones.rot[bitB->second]));
            const Vector boneAxis = DirectX::XMVector3Normalize(localDir);
            const Vector cross = DirectX::XMVector3Cross(vYone, boneAxis);
            const float dot = DirectX::XMVectorGetX(DirectX::XMVector3Dot(vYone, boneAxis));
            Quaternion qAlign;
            if (dot < -0.9999f)
                qAlign = DirectX::XMQuaternionRotationAxis(vXone, DirectX::XM_PI);
            else
            {
                qAlign = DirectX::XMVectorSetW(cross, 1.0f + dot);
                qAlign = DirectX::XMQuaternionNormalize(qAlign);
            }
            coneConstraints.alignRot[ci] = qAlign;
            coneConstraints.invAlignRot[ci] = DirectX::XMQuaternionConjugate(qAlign);
            //coneConstraints.restDir[ci] = localDir;
            logger::info("{:x} : add cone constraint {}-{}", object->formID, coneConstraint.boneNameA, coneConstraint.boneNameB);
            SetConeConstraint(ci, coneConstraint, physicsScale);
        }

        // add deform constraint
        for (const auto& deformConstraint : input.deformConstraints)
        {
            auto bitA = nameMap.find(deformConstraint.boneNameA);
            if (bitA == nameMap.end())
            {
                logger::error("{:x} : Unable to get physics node {} for deform constrant", object->formID, deformConstraint.boneNameA);
                continue;
            }
            auto bitB = nameMap.find(deformConstraint.boneNameB);
            if (bitB == nameMap.end())
            {
                logger::error("{:x} : Unable to get physics node {} for deform constrant", object->formID, deformConstraint.boneNameB);
                continue;
            }

            const std::uint32_t dci = AllocateDeformConstraint();
            deformConstraints.boneIdx[dci] = bitA->second;
            deformConstraints.anchIdx[dci] = bitB->second;
            deformConstraints.objIdx[dci] = objIdx;
            deformConstraints.rootIdx[dci] = rootIdx;

            const float physicsScale = (physicsBones.physicsScale[bitA->second] + physicsBones.physicsScale[bitB->second]) * 0.5f;

            const Vector tPos = physicsBones.pos[bitA->second];
            const Vector aPos = physicsBones.pos[bitB->second];
            deformConstraints.restLen[dci] = DirectX::XMVectorGetX(DirectX::XMVector3Length(DirectX::XMVectorSubtract(tPos, aPos)));
            const Quaternion childRot = physicsBones.rot[bitA->second];
            const Quaternion anchorRot = physicsBones.rot[bitB->second];
            const Quaternion anchorRotInv = DirectX::XMQuaternionInverse(anchorRot);
            deformConstraints.restRot[dci] = DirectX::XMQuaternionNormalize(DirectX::XMQuaternionMultiply(childRot, anchorRotInv));
            logger::info("{:x} : add deform constraint {}-{}", object->formID, deformConstraint.boneNameA, deformConstraint.boneNameB);
            SetDeformConstraint(dci, deformConstraint, physicsScale);
        }

        // add shape matching constraint
        for (const auto& shapeMatchingConstraint : input.shapeMatchingConstraints)
        {
            if (shapeMatchingConstraint.bones.size() < 3)
                continue;

            std::vector<std::uint32_t> tempIndices;
            std::vector<float> tempMasses;
            std::vector<Vector> tempPositions;

            float totalMass = 0.0f;
            Vector currentCoM = vZero;

            for (const std::string& boneName : shapeMatchingConstraint.bones)
            {
                auto it = boneNameToIdx[objIdx].find(boneName);
                if (it == boneNameToIdx[objIdx].end())
                    continue;

                const std::uint32_t bi = it->second;
                float mass = 1.0f;
                auto inputBoneIt = input.bones.find(boneName);
                if (inputBoneIt != input.bones.end() && inputBoneIt->second.mass > Epsilon)
                    mass = inputBoneIt->second.mass;
                else if (physicsBones.invMass[bi] > Epsilon)
                    mass = reciprocal(physicsBones.invMass[bi]);

                const Vector& pos = physicsBones.pos[bi];
                tempIndices.push_back(bi);
                tempMasses.push_back(mass);
                tempPositions.push_back(pos);

                currentCoM = DirectX::XMVectorAdd(currentCoM, DirectX::XMVectorScale(pos, mass));
                totalMass += mass;
            }

            if (tempIndices.size() < 3 || totalMass <= Epsilon)
                continue;

            currentCoM = DirectX::XMVectorMultiply(currentCoM, DirectX::XMVectorReciprocal(DirectX::XMVectorReplicate(totalMass)));

            const std::uint32_t ci = AllocateShapeMatchingConstraint();
            shapeMatchingConstraints.objIdx[ci] = objIdx;
            shapeMatchingConstraints.rootIdx[ci] = rootIdx;
            shapeMatchingConstraints.cluster[ci].compliancePositive = ToVector(shapeMatchingConstraint.compliancePositive);
            shapeMatchingConstraints.cluster[ci].complianceNegative = ToVector(shapeMatchingConstraint.complianceNegative);
            shapeMatchingConstraints.cluster[ci].inertiaScale = ToVector(shapeMatchingConstraint.inertiaScale);

            ReserveShapeMatchingConstraintCluster(tempIndices.size());
            for (std::uint32_t i = 0; i < tempIndices.size(); ++i)
            {
                const std::uint32_t cci = AllocateShapeMatchingConstraintCluster(ci);
                shapeMatchingConstraints.boneIdx[cci] = tempIndices[i];
                shapeMatchingConstraints.boneMass[cci] = tempMasses[i];
                shapeMatchingConstraints.restRelativePos[cci] = DirectX::XMVectorSubtract(tempPositions[i], currentCoM);
            }
        }
        ReorderMaps();
    }

    void XPBDWorld::AddCollider(RE::TESObjectREFR* object, const PhysicsInput& input)
    {
        if (!object)
            return;
        if (input.colliders.datas.empty())
            return;

        WaitForPhysicsWorldAsync();

        const Internal::ObjectDatas::Root newRoot = {.type = Internal::RootType::kCollider, .bipedSlot = input.bipedSlot};

        std::lock_guard lg(lock);
        CheckUpdate();

        std::uint32_t objIdx = AllocateObject(object);
        std::uint32_t rootIdx = AllocateRoot(objIdx, newRoot);

        auto& nameMap = boneNameToIdx[objIdx];

        ReserveCollider(input.colliders.datas.size());
        RemoveCollider(object, newRoot);

        if (colliders.numColliders == 0)
            convexHullCache.reserve(input.colliders.datas.size() * 8);

        std::uint32_t addedColCount = 0;
        for (auto& collider : input.colliders.datas)
        {
            const auto& boneName = collider.boneName;
            auto bit = nameMap.find(boneName);
            if (bit == nameMap.end())
                continue;

            const std::uint32_t coi = AllocateCollider();
            colliders.boneIdx[coi] = bit->second;
            colliders.objIdx[coi] = objIdx;
            colliders.rootIdx[coi] = rootIdx;
            colliders.colliderType[coi] = collider.colliderType;
            if (collider.colliderType == ColliderType::kSphere)
            {
                logger::info("{:x} => add sphere collider {} / colGroup {:x} / colLayer {:x}", object->formID, boneName, physicsBones.layerGroup[bit->second], physicsBones.collideLayer[bit->second]);
                colliders.sphereData[coi] = collider.sphereData;

                addedColCount++;

                AABB aabb = AABB();
                for (std::uint32_t s = 0; s < COL_SPHERE_MAX; ++s)
                {
                    const Vector p = DirectX::XMVectorSet(colliders.sphereData[coi].cX[s], colliders.sphereData[coi].cY[s], colliders.sphereData[coi].cZ[s], 0.0f);
                    AABB vAABB(p, p);
                    vAABB.Fatten(colliders.sphereData[coi].radius[s]);
                    aabb = aabb.Merge(vAABB);
                }
                colliders.boundingAABB[coi] = aabb;

                const Vector center = aabb.GetCenter();
                colliders.boundingSphereCenter[coi] = center;

                float maxBoundingRadius = 0.0f;
                for (std::uint32_t s = 0; s < COL_SPHERE_MAX; ++s)
                {
                    const Vector p = DirectX::XMVectorSet(colliders.sphereData[coi].cX[s], colliders.sphereData[coi].cY[s], colliders.sphereData[coi].cZ[s], 0.0f);
                    const float r = colliders.sphereData[coi].radius[s];
                    const Vector distVec = DirectX::XMVectorSubtract(p, center);
                    const float distToCenter = DirectX::XMVectorGetX(DirectX::XMVector3Length(distVec));
                    const float requiredRadius = distToCenter + r;
                    if (requiredRadius > maxBoundingRadius)
                        maxBoundingRadius = requiredRadius;
                }
                colliders.boundingSphere[coi] = maxBoundingRadius;
            }
            else if (collider.colliderType == ColliderType::kConvexHull)
            {
                logger::info("{:x} => add convexHull collider {} / colGroup {:x} / colLayer {:x}", object->formID, boneName, physicsBones.layerGroup[bit->second], physicsBones.collideLayer[bit->second]);
                colliders.convexHullData[coi] = collider.convexHullData;

                addedColCount++;

                AABB aabb = AABB();
                for (std::uint32_t v = 0; v < COL_VERTEX_MAX; ++v)
                {
                    const Vector p = DirectX::XMVectorSet(colliders.convexHullData[coi].vX[v], colliders.convexHullData[coi].vY[v], colliders.convexHullData[coi].vZ[v], 0.0f);
                    const AABB vAABB(p, p);
                    aabb = aabb.Merge(vAABB);
                }
                colliders.boundingAABB[coi] = aabb;

                const Vector center = aabb.GetCenter();
                colliders.boundingSphereCenter[coi] = center;

                Vector maxRadiusSq = vZero;
                for (std::uint32_t v = 0; v < COL_VERTEX_MAX; ++v)
                {
                    const Vector p = DirectX::XMVectorSet(colliders.convexHullData[coi].vX[v], colliders.convexHullData[coi].vY[v], colliders.convexHullData[coi].vZ[v], 0.0f);
                    const Vector dist = DirectX::XMVectorSubtract(p, center);
                    const Vector distSq = DirectX::XMVector3LengthSq(dist);
                    if (DirectX::XMVector3Less(maxRadiusSq, distSq))
                        maxRadiusSq = distSq;
                }
                colliders.boundingSphere[coi] = DirectX::XMVectorGetX(DirectX::XMVectorSqrt(maxRadiusSq));

                if (const float colShrink = physicsBones.collisionShrink[bit->second]; colShrink > Epsilon)
                {
                    const float maxShrink = colliders.boundingSphere[coi] * 0.9f;
                    const float finalShrink = std::min(physicsBones.collisionShrink[bit->second], maxShrink);
                    const float shrinkRatio = 1.0f - (finalShrink * reciprocal(colliders.boundingSphere[coi]));

                    AABB shrinkedAABB = AABB();
                    DirectX::XMFLOAT3 center_f3;
                    DirectX::XMStoreFloat3(&center_f3, colliders.boundingSphereCenter[coi]);
                    for (std::uint32_t v = 0; v < COL_VERTEX_MAX; ++v)
                    {
                        colliders.convexHullData[coi].vX[v] = center_f3.x + (colliders.convexHullData[coi].vX[v] - center_f3.x) * shrinkRatio;
                        colliders.convexHullData[coi].vY[v] = center_f3.y + (colliders.convexHullData[coi].vY[v] - center_f3.y) * shrinkRatio;
                        colliders.convexHullData[coi].vZ[v] = center_f3.z + (colliders.convexHullData[coi].vZ[v] - center_f3.z) * shrinkRatio;
                        const Vector p = DirectX::XMVectorSet(colliders.convexHullData[coi].vX[v], colliders.convexHullData[coi].vY[v], colliders.convexHullData[coi].vZ[v], 0.0f);
                        shrinkedAABB = shrinkedAABB.Merge(AABB(p, p));
                    }
                    colliders.boundingSphere[coi] *= shrinkRatio;
                    colliders.boundingAABB[coi] = shrinkedAABB;
                }
            }

            std::uint32_t ncCount = 0;
            {
                auto& node = physicsBones.node[bit->second];
                if (node && node->parent && !node->parent->name.empty())
                {
                    const std::string ncName = node->parent->name.c_str();
                    auto ncBit = nameMap.find(ncName);
                    if (ncBit != nameMap.end())
                    {
                        colliders.noCollideBoneIdx[static_cast<std::uint32_t>(coi) * NOCOLLIDE_MAX + ncCount] = ncBit->second;
                        logger::debug("{:x} : {} => add no collide {}", object->formID, boneName, ncName);
                        ncCount++;
                    }
                }
            }

            auto ncIt = input.colliders.noCollideBones.find(boneName);
            if (ncIt != input.colliders.noCollideBones.end())
            {
                for (const auto& ncName : ncIt->second)
                {
                    if (boneName == ncName)
                        continue;
                    auto ncBit = nameMap.find(ncName);
                    if (ncBit == nameMap.end())
                        continue;
                    const std::uint32_t ncbi = ncBit->second;
                    auto begin = colliders.noCollideBoneIdx.begin() + static_cast<std::uint32_t>(coi) * NOCOLLIDE_MAX;
                    auto end = begin + ncCount;
                    auto it = std::find(begin, end, ncbi);
                    if (it != end)
                        continue;
                    colliders.noCollideBoneIdx[static_cast<std::uint32_t>(coi) * NOCOLLIDE_MAX + ncCount] = ncbi;
                    logger::debug("{:x} : {} => add no collide {}", object->formID, boneName, ncName);
                    ncCount++;
                }
            }
            colliders.noCollideCount[coi] = ncCount;
        }
        logger::debug("{:x} : add colliders {}", object->formID, addedColCount);
        ReorderMaps();
    }

    void XPBDWorld::AddDriver(RE::TESObjectREFR* object, const RootType rootType, const DriverInput& input)
    {
        std::lock_guard lg(lock);
        CheckUpdate();
        {
            const std::uint32_t oi = FindObject(object);
            const std::uint32_t ri = FindRoot(oi, Internal::ObjectDatas::Root(rootType, input.bipedSlot));
            Internal::RemoveDataList removeList;
            removeList.emplace(oi, ri);
            driver->RemoveDriver(GetContext(), removeList);
        }
        driver->AddDriver(object, GetContext(), input);
    }

    void XPBDWorld::UpdatePhysicsSetting(RE::TESObjectREFR* object, const PhysicsInput& input, bool reset)
    {
        if (!object)
            return;

        WaitForPhysicsWorldAsync();

        // logger::info("{:x} : Replacing physics setting...", object->formID);
        std::lock_guard lg(lock);
        CheckUpdate();

        // find object
        std::uint32_t currentObjIdx = static_cast<std::uint32_t>(objectDatas.objectID.size());
        bool isObjectFound = false;
        for (std::uint32_t oi = 0; oi < objectDatas.objectID.size(); ++oi)
        {
            if (objectDatas.objectID[oi] != object->formID)
                continue;
            currentObjIdx = oi;
            isObjectFound = true;
            logger::debug("{:x} : Found objIdx", object->formID);
            break;
        }
        if (!isObjectFound)
            return;

        if (reset)
            Reset(currentObjIdx);

        if (physicsBonesGroup.empty())
            return;

        {
            const std::uint32_t groups = physicsBonesGroup.size() - 1ull;
            for (std::uint32_t g = 0; g < groups; ++g)
            {
                const std::uint32_t begin = physicsBonesGroup[g];
                const std::uint32_t end = physicsBonesGroup[g + 1];
                if (begin >= end)
                    continue;
                if (physicsBones.objIdx[begin] != currentObjIdx)
                    continue;
                logger::debug("{:x} : Found bones group", object->formID);
                for (std::uint32_t bi = begin; bi < end; ++bi)
                {
                    if (auto& node = physicsBones.node[bi]; node && !node->name.empty())
                    {
                        const auto found = input.bones.find(node->name.c_str());
                        if (found == input.bones.end())
                            continue;

                        logger::info("{:x} : Set bone ({}{}{}) for physics", object->formID, node->name.c_str(),
                                     (found->second.enableDynamicVolume ? "(volume : " + std::to_string(found->second.currentVolume) + "L / " : ""),
                                     (found->second.enableDynamicVolume ? "PhysicsScale : " + std::to_string(found->second.GetPhysicsScaleByVolume()) + ")" : ""));
                        physicsBones.orgWorldScale[bi] = node->world.scale;
                        SetBone(bi, found->second);
                    }
                    else if (physicsBones.isParticle[bi])
                    {
                        const std::uint32_t pbi = physicsBones.parentBoneIdx[bi];
                        if (pbi == UINT32_MAX)
                            continue;

                        const auto found = input.bones.find(physicsBones.particleName[bi]);
                        if (found == input.bones.end())
                            continue;

                        logger::debug("{:x} : Found particle {}{} for physics", object->formID, physicsBones.particleName[bi], ("(PhysicsScale : " + std::to_string(physicsBones.physicsScale[pbi]) + ")"));
                        physicsBones.orgWorldScale[bi] = physicsBones.orgWorldScale[pbi];
                        SetBone(bi, found->second);
                    }
                }
            }
        }

        if (!distanceConstraintsGroup.empty())
        {
            const std::uint32_t groups = distanceConstraintsGroup.size() - 1ull;
            for (std::uint32_t g = 0; g < groups; ++g)
            {
                const std::uint32_t begin = distanceConstraintsGroup[g];
                const std::uint32_t end = distanceConstraintsGroup[g + 1];
                if (begin >= end)
                    continue;
                if (distanceConstraints.objIdx[begin] != currentObjIdx)
                    continue;
                logger::debug("{:x} : Found distance constraints group", object->formID);
                for (std::uint32_t ci = begin; ci < end; ++ci)
                {
                    const std::uint32_t bi = distanceConstraints.boneIdx[ci];
                    const std::uint32_t abi = distanceConstraints.anchIdx[ci];
                    if (bi == UINT32_MAX || abi == UINT32_MAX)
                        continue;
                    std::string boneName;
                    if (auto& node = physicsBones.node[bi]; node && !node->name.empty())
                        boneName = node->name.c_str();
                    else if (!physicsBones.particleName[bi].empty())
                        boneName = physicsBones.particleName[bi];
                    else
                        continue;
                    std::string anchName;
                    if (auto& node = physicsBones.node[abi]; node && !node->name.empty())
                        anchName = node->name.c_str();
                    else if (!physicsBones.particleName[abi].empty())
                        anchName = physicsBones.particleName[abi];
                    else
                        continue;
                    auto found = std::find_if(input.distanceConstraints.begin(), input.distanceConstraints.end(), [&](const auto& cons) {
                        return cons.boneNameA == boneName && cons.boneNameB == anchName;
                    });
                    logger::debug("{:x} : Found bone {}-{} for distance constraints", object->formID, boneName, anchName);
                    const float physicsScale = (physicsBones.physicsScale[bi] + physicsBones.physicsScale[abi]) * 0.5f;
                    SetDistanceConstraint(ci, *found, physicsScale);
                }
            }
        }
        if (!angularConstraintsGroup.empty())
        {
            const std::uint32_t groups = angularConstraintsGroup.size() - 1ull;
            for (std::uint32_t g = 0; g < groups; ++g)
            {
                const std::uint32_t begin = angularConstraintsGroup[g];
                const std::uint32_t end = angularConstraintsGroup[g + 1];
                if (begin >= end)
                    continue;
                if (angularConstraints.objIdx[begin] != currentObjIdx)
                    continue;
                logger::debug("{:x} : Found angular constraints group", object->formID);
                for (std::uint32_t ci = begin; ci < end; ++ci)
                {
                    const std::uint32_t bi = angularConstraints.boneIdx[ci];
                    const std::uint32_t abi = angularConstraints.anchIdx[ci];
                    if (bi == UINT32_MAX || abi == UINT32_MAX)
                        continue;
                    std::string boneName;
                    if (auto& node = physicsBones.node[bi]; node && !node->name.empty())
                        boneName = node->name.c_str();
                    else if (!physicsBones.particleName[bi].empty())
                        boneName = physicsBones.particleName[bi];
                    else
                        continue;
                    std::string anchName;
                    if (auto& node = physicsBones.node[abi]; node && !node->name.empty())
                        anchName = node->name.c_str();
                    else if (!physicsBones.particleName[abi].empty())
                        anchName = physicsBones.particleName[abi];
                    else
                        continue;
                    auto found = std::find_if(input.angularConstraints.begin(), input.angularConstraints.end(), [&](const auto& cons) {
                        return cons.boneNameA == boneName && cons.boneNameB == anchName;
                    });
                    logger::debug("{:x} : Found bone {}-{} for angular constraints", object->formID, boneName, anchName);
                    const float physicsScale = (physicsBones.physicsScale[bi] + physicsBones.physicsScale[abi]) * 0.5f;
                    SetAngularConstraint(ci, *found, physicsScale);
                }
            }
        }
        if (!coneConstraintsGroup.empty())
        {
            const std::uint32_t groups = coneConstraintsGroup.size() - 1ull;
            for (std::uint32_t g = 0; g < groups; ++g)
            {
                const std::uint32_t begin = coneConstraintsGroup[g];
                const std::uint32_t end = coneConstraintsGroup[g + 1];
                if (begin >= end)
                    continue;
                if (coneConstraints.objIdx[begin] != currentObjIdx)
                    continue;
                logger::debug("{:x} : Found cone constraints group", object->formID);
                for (std::uint32_t ci = begin; ci < end; ++ci)
                {
                    const std::uint32_t bi = coneConstraints.boneIdx[ci];
                    const std::uint32_t abi = coneConstraints   .anchIdx[ci];
                    if (bi == UINT32_MAX || abi == UINT32_MAX)
                        continue;
                    std::string boneName;
                    if (auto& node = physicsBones.node[bi]; node && !node->name.empty())
                        boneName = node->name.c_str();
                    else if (!physicsBones.particleName[bi].empty())
                        boneName = physicsBones.particleName[bi];
                    else
                        continue;
                    std::string anchName;
                    if (auto& node = physicsBones.node[abi]; node && !node->name.empty())
                        anchName = node->name.c_str();
                    else if (!physicsBones.particleName[abi].empty())
                        anchName = physicsBones.particleName[abi];
                    else
                        continue;
                    auto found = std::find_if(input.coneConstraints.begin(), input.coneConstraints.end(), [&](const auto& cons) {
                        return cons.boneNameA == boneName && cons.boneNameB == anchName;
                    });
                    logger::debug("{:x} : Found bone {}-{} for cone constraints", object->formID, boneName, anchName);
                    const float physicsScale = (physicsBones.physicsScale[bi] + physicsBones.physicsScale[abi]) * 0.5f;
                    SetConeConstraint(ci, *found, physicsScale);
                }
            }
        }
        if (!deformConstraintsGroup.empty())
        {
            const std::uint32_t groups = deformConstraintsGroup.size() - 1ull;
            for (std::uint32_t g = 0; g < groups; ++g)
            {
                const std::uint32_t begin = deformConstraintsGroup[g];
                const std::uint32_t end = deformConstraintsGroup[g + 1];
                if (begin >= end)
                    continue;
                if (deformConstraints.objIdx[begin] != currentObjIdx)
                    continue;
                logger::debug("{:x} : Found deform constraints group", object->formID);
                for (std::uint32_t ci = begin; ci < end; ++ci)
                {
                    const std::uint32_t bi = deformConstraints.boneIdx[ci];
                    const std::uint32_t abi = deformConstraints.anchIdx[ci];
                    if (bi == UINT32_MAX || abi == UINT32_MAX)
                        continue;
                    std::string boneName;
                    if (auto& node = physicsBones.node[bi]; node && !node->name.empty())
                        boneName = node->name.c_str();
                    else if (!physicsBones.particleName[bi].empty())
                        boneName = physicsBones.particleName[bi];
                    else
                        continue;
                    std::string anchName;
                    if (auto& node = physicsBones.node[abi]; node && !node->name.empty())
                        anchName = node->name.c_str();
                    else if (!physicsBones.particleName[abi].empty())
                        anchName = physicsBones.particleName[abi];
                    else
                        continue;
                    auto found = std::find_if(input.deformConstraints.begin(), input.deformConstraints.end(), [&](const auto& cons) {
                        return cons.boneNameA == boneName && cons.boneNameB == anchName;
                    });
                    logger::debug("{:x} : Found bone {}-{} for deform constraints", object->formID, boneName, anchName);
                    const float physicsScale = (physicsBones.physicsScale[bi] + physicsBones.physicsScale[abi]) * 0.5f;
                    SetDeformConstraint(ci, *found, physicsScale);
                }
            }
        }
    }

    void XPBDWorld::ResetAll()
    {
        WaitForPhysicsWorldAsync();

        std::lock_guard lg(lock);
        CheckUpdate();

        for (std::uint32_t oi = 0; oi < objectDatas.objectID.size(); ++oi)
        {
            if (RE::TESObjectREFR* object = GetREFR(objectDatas.objectID[oi]); object)
            {
                auto npcNode = objectDatas.npcNode[oi] ? objectDatas.npcNode[oi].get() : GetNPCNode(object);
                if (npcNode)
                    objectDatas.prevWorldPos[oi] = ToVector(npcNode->world.translate);
                else
                    objectDatas.prevWorldPos[oi] = ToVector(object->GetPosition());
            }
            objectDatas.acceleration[oi] = vZero;
        }

        // reset bone data
        if (!physicsBonesGroup.empty())
        {
            const std::uint32_t groups = physicsBonesGroup.size() - 1ull;
            threadPool->Execute([&] {
                tbb::parallel_for(
                    tbb::blocked_range<std::uint32_t>(0, groups),
                    [&](const tbb::blocked_range<std::uint32_t>& r) {
                        for (std::uint32_t g = r.begin(); g != r.end(); ++g)
                        {
                            const std::uint32_t begin = physicsBonesGroup[g];
                            const std::uint32_t end = physicsBonesGroup[g + 1];
                            if (begin >= end)
                                continue;
                            const std::uint32_t oi = physicsBones.objIdx[begin];
                            if (oi == UINT32_MAX)
                                continue;
                            for (std::uint32_t bi = begin; bi < end; ++bi)
                            {
                                ResetBone(bi);
                            }
                            auto npcNode = objectDatas.npcNode[oi] ? objectDatas.npcNode[oi].get() : GetNPCNodeByObjectIndex(oi);
                            UpdateChildTreeData(npcNode, RE::NiUpdateData::Flag::kDirty);
                            for (std::uint32_t bi = begin; bi < end; ++bi)
                            {
                                ResetParticleBone(bi);
                            }
                        }
                    },
                    tbb::auto_partitioner()
                );
            });
        }
    }

    void XPBDWorld::Reset(RE::TESObjectREFR* object)
    {
        if (!object)
            return;
        WaitForPhysicsWorldAsync();
        std::lock_guard lg(lock);
        CheckUpdate();

        // find object
        std::uint32_t objIdx = UINT32_MAX;
        for (std::uint32_t oi = 0; oi < objectDatas.objectID.size(); ++oi)
        {
            if (objectDatas.objectID[oi] == object->formID)
            {
                objIdx = oi;
                break;
            }
        }
        if (objIdx == UINT32_MAX)
            return;
        Reset(objIdx);
    }

    void XPBDWorld::Reset(const std::uint32_t oi)
    {
        if (oi == UINT32_MAX)
            return;
        // reset object data
        RE::TESObjectREFR* object = GetREFR(objectDatas.objectID[oi]);
        if (!object || !object->loadedData || !object->loadedData->data3D)
            return;
        auto npcNode = objectDatas.npcNode[oi] ? objectDatas.npcNode[oi].get() : GetNPCNode(object);
        if (npcNode)
            objectDatas.prevWorldPos[oi] = ToVector(npcNode->world.translate);
        else
            objectDatas.prevWorldPos[oi] = ToVector(object->GetPosition());
        /*objectDatas.deltaWorldPos[oi] = vZero;
        objectDatas.velocity[oi] = vZero;
        objectDatas.acceleration[oi] = vZero;*/

        // reset bone data
        if (!physicsBonesGroup.empty())
        {
            const std::uint32_t groups = physicsBonesGroup.size() - 1ull;
            for (std::uint32_t g = 0; g < groups; ++g)
            {
                const std::uint32_t begin = physicsBonesGroup[g];
                const std::uint32_t end = physicsBonesGroup[g + 1];
                if (begin >= end)
                    continue;
                if (physicsBones.objIdx[begin] != oi)
                    continue;
                for (std::uint32_t bi = begin; bi < end; ++bi)
                {
                    ResetBone(bi);
                }
                /*const std::uint32_t oi = physicsBones.objIdx[begin];
                auto npcNode = objectDatas.npcNode[oi] ? objectDatas.npcNode[oi].get() : GetNPCNodeByObjectIndex(oi);
                UpdateChildTreeData(npcNode, RE::NiUpdateData::Flag::kDirty);*/
                for (std::uint32_t bi = begin; bi < end; ++bi)
                {
                    ResetParticleBone(bi);
                }
                break;
            }
        }
    }

    void XPBDWorld::ResetBone(const std::uint32_t bi)
    {
        if (auto& node = physicsBones.node[bi]; node)
        {
            const RE::NiPoint3 localPos = ToNiPoint(physicsBones.orgLocalPos[bi]);
            const RE::NiMatrix3 localRot = ToNiMatrix(physicsBones.orgLocalRot[bi]);
            memcpy(&node->local.translate, &localPos, sizeof(localPos));
            memcpy(&node->local.rotate, &localRot, sizeof(localRot));

            if (node->parent)
            {
                const Vector parentWorldPos = ToVector(node->parent->world.translate);
                const Quaternion parentWorldRot = ToQuaternion(node->parent->world.rotate);
                const RE::NiPoint3 worldPos = ToNiPoint(DirectX::XMVectorAdd(DirectX::XMVector3Rotate(DirectX::XMVectorScale(physicsBones.orgLocalPos[bi], node->parent->world.scale), parentWorldRot), parentWorldPos));
                const RE::NiMatrix3 worldRot = ToNiMatrix(DirectX::XMQuaternionNormalize(DirectX::XMQuaternionMultiply(physicsBones.orgLocalRot[bi], parentWorldRot)));
                memcpy(&node->world.translate, &worldPos, sizeof(worldPos));
                memcpy(&node->world.rotate, &worldRot, sizeof(worldRot));
            }

            const Vector offset = DirectX::XMVector3Rotate(DirectX::XMVectorScale(physicsBones.offset[bi], physicsBones.orgWorldScale[bi]), ToQuaternion(node->world.rotate));
            physicsBones.pos[bi] = DirectX::XMVectorAdd(ToVector(node->world.translate), offset);
            physicsBones.rot[bi] = ToQuaternion(node->world.rotate);

            physicsBones.prevPos[bi] = physicsBones.pos[bi];
            physicsBones.predPos[bi] = physicsBones.pos[bi];
            physicsBones.backupPos[bi] = physicsBones.pos[bi];
            if (const std::uint32_t oi = physicsBones.objIdx[bi]; oi != UINT32_MAX)
                physicsBones.posVel[bi] = objectDatas.velocity[oi];
            else
                physicsBones.posVel[bi] = vZero;

            physicsBones.prevRot[bi] = physicsBones.rot[bi];
            physicsBones.predRot[bi] = physicsBones.rot[bi];
            physicsBones.backupRot[bi] = physicsBones.rot[bi];
            physicsBones.angVel[bi] = vZero;

            physicsBones.deformScale[bi] = vmIdentity;
            physicsBones.deformScaleCache[bi] = vmZeroAll;
            physicsBones.deformVelocityScale[bi] = vmZeroAll;

            physicsBones.groundCache[bi] = {};
        }
    }

    void XPBDWorld::ResetParticleBone(const std::uint32_t bi)
    {
        if (physicsBones.isParticle[bi] && physicsBones.parentBoneIdx[bi] != UINT32_MAX)
        {
            const std::uint32_t pbi = physicsBones.parentBoneIdx[bi];
            const Vector offset = DirectX::XMVector3Rotate(DirectX::XMVectorScale(physicsBones.offset[bi], physicsBones.orgWorldScale[pbi]), physicsBones.rot[pbi]);
            physicsBones.pos[bi] = DirectX::XMVectorAdd(physicsBones.pos[pbi], offset);
            physicsBones.prevPos[bi] = physicsBones.pos[bi];
            physicsBones.predPos[bi] = physicsBones.pos[bi];
            physicsBones.backupPos[bi] = physicsBones.pos[bi];
            if (const std::uint32_t oi = physicsBones.objIdx[bi]; oi != UINT32_MAX)
                physicsBones.posVel[bi] = objectDatas.velocity[oi];
            else
                physicsBones.posVel[bi] = vZero;

            physicsBones.rot[bi] = physicsBones.rot[pbi];
            physicsBones.prevRot[bi] = physicsBones.rot[bi];
            physicsBones.predRot[bi] = physicsBones.rot[bi];
            physicsBones.backupRot[bi] = physicsBones.rot[bi];
            physicsBones.angVel[bi] = vZero;

            physicsBones.deformScale[bi] = vmIdentity;
            physicsBones.deformScaleCache[bi] = vmZeroAll;
            physicsBones.deformVelocityScale[bi] = vmZeroAll;
        }
    }

    void XPBDWorld::RemovePhysics(const RE::FormID objectID)
    {
        WaitForPhysicsWorldAsync();
        std::lock_guard lg(lock);
        Internal::RemoveDataList removeList;
        for (std::uint32_t oi = 0; oi < objectDatas.objectID.size(); ++oi)
        {
            if (objectDatas.objectID[oi] != objectID)
                continue;
            Reset(oi);
            objectDatas.objectID[oi] = 0;
            objectDatas.isDisable[oi] = true;
            objectDatas.isDisableByToggle[oi] = true;
            for (std::uint32_t ri = 0; ri < objectDatas.roots[oi].size(); ++ri)
            {
                removeList.insert(Internal::RemoveData(oi, ri));
            }
            objectDatas.roots[oi].clear();
            RemovePhysics(removeList);
            break;
        }
    }

    void XPBDWorld::RemovePhysics(const RE::FormID objectID, const RootType rootType, const std::uint32_t bipedSlot)
    {
        const Internal::ObjectDatas::Root targetRoot = {.type = rootType, .bipedSlot = bipedSlot};
        WaitForPhysicsWorldAsync();
        std::lock_guard lg(lock);
        Internal::RemoveDataList removeList;
        for (std::uint32_t i = 0; i < objectDatas.objectID.size(); ++i)
        {
            if (objectDatas.objectID[i] != objectID)
                continue;
            auto& root = objectDatas.roots[i];
            for (std::uint32_t ri = 0; ri < root.size(); ++ri)
            {
                if (root[ri] != targetRoot)
                    continue;
                removeList.insert(Internal::RemoveData(i, ri));
                root[ri].type = RootType::kNone;
                break;
            }
            RemovePhysics(removeList);
            break;
        }
    }

    void XPBDWorld::RemoveDriver(const RE::FormID objectID)
    {
        WaitForPhysicsWorldAsync();
        std::lock_guard lg(lock);
        Internal::RemoveDataList removeList;
        for (std::uint32_t oi = 0; oi < objectDatas.objectID.size(); ++oi)
        {
            if (objectDatas.objectID[oi] != objectID)
                continue;
            Reset(oi);
            for (std::uint32_t ri = 0; ri < objectDatas.roots[oi].size(); ++ri)
            {
                removeList.insert(Internal::RemoveData(oi, ri));
            }
            driver->RemoveDriver(GetContext(), removeList);
            break;
        }
    }

    void XPBDWorld::RemoveDriver(const RE::FormID objectID, const RootType rootType, const std::uint32_t bipedSlot)
    {
        const Internal::ObjectDatas::Root targetRoot = {.type = rootType, .bipedSlot = bipedSlot};
        WaitForPhysicsWorldAsync();
        std::lock_guard lg(lock);
        Internal::RemoveDataList removeList;
        for (std::uint32_t i = 0; i < objectDatas.objectID.size(); ++i)
        {
            if (objectDatas.objectID[i] != objectID)
                continue;
            auto& root = objectDatas.roots[i];
            for (std::uint32_t ri = 0; ri < root.size(); ++ri)
            {
                if (root[ri] != targetRoot)
                    continue;
                removeList.insert(Internal::RemoveData(i, ri));
                break;
            }
            driver->RemoveDriver(GetContext(), removeList);
            break;
        }
    }

    void XPBDWorld::TogglePhysics(const RE::FormID objectID, bool disable)
    {
        WaitForPhysicsWorldAsync();
        std::lock_guard lg(lock);
        for (std::uint32_t i = 0; i < objectDatas.objectID.size(); ++i)
        {
            if (objectDatas.objectID[i] != objectID)
                continue;
            if (objectDatas.isDisable[i] != disable)
            {
                objectDatas.isDisableByToggle[i] = disable;
            }
            break;
        }
    }

    void XPBDWorld::RemovePhysics(const Internal::RemoveDataList& removeList)
    {
        if (removeList.empty())
            return;

        threadPool->Execute([&] {
            // remove nodes
            for (std::uint32_t bi = 0; bi != physicsBones.numBones; ++bi)
            {
                if (removeList.count(Internal::RemoveData(physicsBones.objIdx[bi], physicsBones.rootIdx[bi])) > 0)
                {
                    physicsBones.node[bi] = nullptr;
                    physicsBones.objIdx[bi] = UINT32_MAX;
                    physicsBones.rootIdx[bi] = UINT32_MAX;
                    physicsBones.parentBoneIdx[bi] = UINT32_MAX;
                    physicsBones.particleName[bi].clear();
                    physicsBones.particleDepth[bi] = UINT32_MAX;
                }
                else
                {
                    const std::uint32_t pbi = physicsBones.parentBoneIdx[bi];
                    if (pbi == UINT32_MAX)
                        continue;
                    if (physicsBones.objIdx[pbi] == UINT32_MAX || physicsBones.rootIdx[pbi] == UINT32_MAX || removeList.count(Internal::RemoveData(physicsBones.objIdx[pbi], physicsBones.rootIdx[pbi])) > 0)
                    {
                        physicsBones.parentBoneIdx[bi] = UINT32_MAX;
                        if (physicsBones.isParticle[bi])
                        {
                            physicsBones.node[bi] = nullptr;
                            physicsBones.objIdx[bi] = UINT32_MAX;
                            physicsBones.rootIdx[bi] = UINT32_MAX;
                            physicsBones.parentBoneIdx[bi] = UINT32_MAX;
                            physicsBones.particleName[bi].clear();
                            physicsBones.particleDepth[bi] = UINT32_MAX;
                        }
                    }
                }
            }
            tbb::parallel_invoke([&] {
                // remove distance constraints
                for (std::uint32_t i = 0; i < distanceConstraints.numConstraints; ++i)
                {
                    const bool invalidA = distanceConstraints.boneIdx[i] != UINT32_MAX && (!physicsBones.node[distanceConstraints.boneIdx[i]] && physicsBones.particleName[distanceConstraints.boneIdx[i]].empty());
                    const bool invalidB = distanceConstraints.anchIdx[i] != UINT32_MAX && (!physicsBones.node[distanceConstraints.anchIdx[i]] && physicsBones.particleName[distanceConstraints.anchIdx[i]].empty());
                    if (invalidA || invalidB)
                    {
                        distanceConstraints.boneIdx[i] = UINT32_MAX;
                        distanceConstraints.anchIdx[i] = UINT32_MAX;
                        distanceConstraints.objIdx[i] = UINT32_MAX;
                        distanceConstraints.rootIdx[i] = UINT32_MAX;
                    }
                }
            }, [&] {
                // remove angular constraints
                for (std::uint32_t i = 0; i < angularConstraints.numConstraints; ++i)
                {
                    const bool invalidA = angularConstraints.boneIdx[i] != UINT32_MAX && (!physicsBones.node[angularConstraints.boneIdx[i]] && physicsBones.particleName[angularConstraints.boneIdx[i]].empty());
                    const bool invalidB = angularConstraints.anchIdx[i] != UINT32_MAX && (!physicsBones.node[angularConstraints.anchIdx[i]] && physicsBones.particleName[angularConstraints.anchIdx[i]].empty());
                    if (invalidA || invalidB)
                    {
                        angularConstraints.boneIdx[i] = UINT32_MAX;
                        angularConstraints.anchIdx[i] = UINT32_MAX;
                        angularConstraints.objIdx[i] = UINT32_MAX;
                        angularConstraints.rootIdx[i] = UINT32_MAX;
                    }
                }
            }, [&] {
                // remove cone constraints
                for (std::uint32_t i = 0; i < coneConstraints.numConstraints; ++i)
                {
                    const bool invalidA = coneConstraints.boneIdx[i] != UINT32_MAX && (!physicsBones.node[coneConstraints.boneIdx[i]] && physicsBones.particleName[coneConstraints.boneIdx[i]].empty());
                    const bool invalidB = coneConstraints.anchIdx[i] != UINT32_MAX && (!physicsBones.node[coneConstraints.anchIdx[i]] && physicsBones.particleName[coneConstraints.anchIdx[i]].empty());
                    if (invalidA || invalidB)
                    {
                        coneConstraints.boneIdx[i] = UINT32_MAX;
                        coneConstraints.anchIdx[i] = UINT32_MAX;
                        coneConstraints.objIdx[i] = UINT32_MAX;
                        coneConstraints.rootIdx[i] = UINT32_MAX;
                    }
                }
            }, [&] {
                // remove deform constraints
                for (std::uint32_t i = 0; i < deformConstraints.numConstraints; ++i)
                {
                    const bool invalidA = deformConstraints.boneIdx[i] != UINT32_MAX && (!physicsBones.node[deformConstraints.boneIdx[i]] && physicsBones.particleName[deformConstraints.boneIdx[i]].empty());
                    const bool invalidB = deformConstraints.anchIdx[i] != UINT32_MAX && (!physicsBones.node[deformConstraints.anchIdx[i]] && physicsBones.particleName[deformConstraints.anchIdx[i]].empty());
                    if (invalidA || invalidB)
                    {
                        deformConstraints.boneIdx[i] = UINT32_MAX;
                        deformConstraints.anchIdx[i] = UINT32_MAX;
                        deformConstraints.objIdx[i] = UINT32_MAX;
                        deformConstraints.rootIdx[i] = UINT32_MAX;
                    }
                }
            }, [&] {
                // remove shape matching constraints
                for (std::uint32_t i = 0; i < shapeMatchingConstraints.numConstraints; ++i)
                {
                    if (!physicsBones.node[shapeMatchingConstraints.boneIdx[i]] && physicsBones.particleName[shapeMatchingConstraints.boneIdx[i]].empty())
                    {
                        shapeMatchingConstraints.objIdx[i] = UINT32_MAX;
                        shapeMatchingConstraints.rootIdx[i] = UINT32_MAX;
                    }
                }
            }, [&] {
                // remove colliders
                for (std::uint32_t i = 0; i < colliders.numColliders; ++i)
                {
                    if ((colliders.boneIdx[i] != UINT32_MAX && !physicsBones.node[colliders.boneIdx[i]]))
                    {
                        colliders.boneIdx[i] = UINT32_MAX;
                        colliders.objIdx[i] = UINT32_MAX;
                        colliders.rootIdx[i] = UINT32_MAX;
                    }
                }
            }, [&] {
                driver->RemoveDriver(GetContext(), removeList);
            });
        });
        orderDirty = true;
    }

    void XPBDWorld::RemoveCollider(RE::TESObjectREFR* object, const Internal::ObjectDatas::Root& targetRoot)
    {
        if (!object)
            return;

        std::uint32_t currentObjIdx = UINT32_MAX;
        std::uint32_t currentRootIdx = UINT32_MAX;
        for (std::uint32_t i = 0; i < objectDatas.objectID.size(); ++i)
        {
            if (objectDatas.objectID[i] != object->formID)
                continue;
            currentObjIdx = i;
            auto& root = objectDatas.roots[i];
            for (std::uint32_t ri = 0; ri < root.size(); ++ri)
            {
                if (root[ri] != targetRoot)
                    continue;
                currentRootIdx = ri;
                break;
            }
            break;
        }
        if (currentObjIdx == UINT32_MAX || currentRootIdx == UINT32_MAX)
            return;

        if (!collidersGroup.empty())
        {
            const std::uint32_t groups = collidersGroup.size() - 1ull;
            for (std::uint32_t g = 0; g < groups; ++g)
            {
                const std::uint32_t begin = collidersGroup[g];
                const std::uint32_t end = collidersGroup[g + 1];
                if (colliders.objIdx[begin] == currentObjIdx)
                {
                    for (std::uint32_t ci = begin; ci < end; ++ci)
                    {
                        if (colliders.rootIdx[ci] == currentRootIdx)
                        {
                            colliders.boneIdx[ci] = UINT32_MAX;
                            colliders.objIdx[ci] = UINT32_MAX;
                            colliders.rootIdx[ci] = UINT32_MAX;
                        }
                    }
                    orderDirty = true;
                }
            }
        }
    }

    void XPBDWorld::RunPhysicsWorld(const float deltaTime)
    {
        std::lock_guard lg(lock);
        CheckUpdate();

        static Internal::TimeProfiler timeProfiler(__func__);
        if (DEBUG_MODE)
            timeProfiler.Start();

        threadPool->Execute([&] {
            if (deltaTime > Epsilon)
            {
                objectAccelerationTime += deltaTime;
                const float prevTimeAccumulator = timeAccumulator;
                const float fixedDeltaTime = std::min(DeltaTime60 * 6, deltaTime); // low 10 fps
                timeAccumulator += fixedDeltaTime;
                if (timeAccumulator >= DeltaTime60)
                {
                    UpdateObjectData(objectAccelerationTime);
                    objectAccelerationTime = 0.0f;
                    preCalcStepCount = 0;
                    {
                        float calcTimeAccumulator = timeAccumulator;
                        while (calcTimeAccumulator >= DeltaTime60)
                        {
                            calcTimeAccumulator -= DeltaTime60;
                            preCalcStepCount++;
                        }
                    }
                    ClampObjectRotation(DeltaTime60 * preCalcStepCount, preCalcStepCount);
                    PrefetchBoneDatas();
                    ObjectCulling();
                }
                std::uint32_t stepCount = 0;
                while (timeAccumulator >= DeltaTime60)
                {
                    const std::uint32_t minExpectedCollisionCount = colliders.numColliders * 4;
                    if (manifoldCacheCount > collideMaxObserved)
                        collideMaxObserved = manifoldCacheCount;
                    else
                        collideMaxObserved = static_cast<std::uint32_t>(collideMaxObserved * 0.98f);
                    convexHullCache.reserve(static_cast<std::uint32_t>(collideMaxObserved * 1.3f));
                    expectedCollisionCount = std::max(static_cast<std::uint32_t>(collideMaxObserved * 1.3f), minExpectedCollisionCount);
                    manifoldCache.resize(expectedCollisionCount);
                    manifoldCacheCount = 0;
                    for (std::uint32_t s = 0; s < subDeltaTimeSteps; ++s)
                    {
                        const float subStepTimeEnd = (stepCount * DeltaTime60) + ((s + 1) * subDeltaTime);
                        const float alpha = std::clamp((subStepTimeEnd - prevTimeAccumulator) * reciprocal(fixedDeltaTime), 0.0f, 1.0f);
                        const float nextAlpha = std::clamp((subStepTimeEnd + subDeltaTime - prevTimeAccumulator) * reciprocal(fixedDeltaTime), 0.0f, 1.0f);
                        InterpolateBoneDatas(alpha, nextAlpha);
                        PredictBones(subDeltaTime);
                        RunDriver(subDeltaTime);
                        if (s == 0)
                        {
                            UpdateGlobalAABBTree();
                            UpdateWindStrength();
                            tbb::parallel_invoke(
                                [&] {
                                    CreateLocalSpatialHash();
                                    GenerateCollisionManifolds();
                                },
                                [&] {
                                    GenerateGroundCache(1.0f);
                                }
                            );
                        }
                        for (std::uint32_t i = 0; i < ITERATION_MAX; ++i)
                        {
                            SolveCachedCollisions(subDeltaTime);
                            SolveCachedGroundCollisions(subDeltaTime);
                            SolveConstraints(subDeltaTime, i == 0u);
                            SolveAnimDrive(subDeltaTime, i == 0u);
                            UpdateProceduralRotations();
                        }
                        SolveDeformConstraint(subDeltaTime);
                        UpdateBoneVelocity(subDeltaTime);
                    }
                    timeAccumulator -= DeltaTime60;
                    stepCount++;
                    currentFrame++;
                }
            }
            ApplyToSkyrim(false);
        });

        if (DEBUG_MODE)
        {
            timeProfiler.End([this](const std::string& name, const double ms) {
                this->LoggingTimeProfiler(name, ms);
            });
        }
    }

    void XPBDWorld::RunPhysicsWorldAsync(const float deltaTime)
    {
        if (deltaTime <= Epsilon)
            return;
        std::lock_guard lg(lock);
        CheckUpdate();

        static Internal::TimeProfiler timeProfiler(__func__);
        if (DEBUG_MODE)
            timeProfiler.Start();

        threadPoolAsync->Execute([&] {
            if (deltaTime <= Epsilon)
                ApplyToSkyrim(false);
            else
            {
                objectAccelerationTime += deltaTime;
                const float prevTimeAccumulator = timeAccumulator;
                const float fixedDeltaTime = std::min(DeltaTime60 * 6, deltaTime); // low 10 fps
                timeAccumulator += fixedDeltaTime;                                 // low 10 fps
                if (timeAccumulator >= DeltaTime60)
                {
                    UpdateObjectData(objectAccelerationTime);
                    objectAccelerationTime = 0.0f;
                    PrefetchBoneDatas();
                    preCalcStepCount = 0;
                    {
                        float calcTimeAccumulator = timeAccumulator;
                        while (calcTimeAccumulator >= DeltaTime60)
                        {
                            calcTimeAccumulator -= DeltaTime60;
                            preCalcStepCount++;
                        }
                    }
                    ObjectCulling();
                    GenerateGroundCache(preCalcStepCount);
                    UpdateWindStrength();
                }
                isTaskLoading = true;
                backGroundTask.run([this, prevTimeAccumulator, fixedDeltaTime] {
                    std::uint32_t stepCount = 0;
                    while (timeAccumulator >= DeltaTime60)
                    {
                        const std::uint32_t minExpectedCollisionCount = colliders.numColliders * 4;
                        if (manifoldCacheCount > collideMaxObserved)
                            collideMaxObserved = manifoldCacheCount;
                        else
                            collideMaxObserved = static_cast<std::uint32_t>(collideMaxObserved * 0.98f);
                        convexHullCache.reserve(static_cast<std::uint32_t>(collideMaxObserved * 1.3f));
                        expectedCollisionCount = std::max(static_cast<std::uint32_t>(collideMaxObserved * 1.3f), minExpectedCollisionCount);
                        manifoldCache.resize(expectedCollisionCount);
                        manifoldCacheCount = 0;
                        for (std::uint32_t s = 0; s < subDeltaTimeSteps; ++s)
                        {
                            const float subStepTimeEnd = (stepCount * DeltaTime60) + ((s + 1) * subDeltaTime);
                            const float alpha = std::clamp((subStepTimeEnd - prevTimeAccumulator) * reciprocal(fixedDeltaTime), 0.0f, 1.0f);
                            const float nextAlpha = std::clamp((subStepTimeEnd + subDeltaTime - prevTimeAccumulator) * reciprocal(fixedDeltaTime), 0.0f, 1.0f);
                            InterpolateBoneDatas(alpha, nextAlpha);
                            PredictBones(subDeltaTime);
                            RunDriver(subDeltaTime);
                            if (s == 0)
                            {
                                UpdateGlobalAABBTree();
                                CreateLocalSpatialHash();
                                GenerateCollisionManifolds();
                            }
                            for (std::uint32_t i = 0; i < ITERATION_MAX; ++i)
                            {
                                SolveCachedCollisions(subDeltaTime);
                                SolveCachedGroundCollisions(subDeltaTime);
                                SolveConstraints(subDeltaTime, i == 0);
                                SolveAnimDrive(subDeltaTime, i == 0);
                                UpdateProceduralRotations();
                            }
                            SolveDeformConstraint(subDeltaTime);
                            UpdateBoneVelocity(subDeltaTime);
                        }
                        timeAccumulator -= DeltaTime60;
                        stepCount++;
                        currentFrame++;
                    }
                });
            }
        });

        if(DEBUG_MODE)
        {
            timeProfiler.End([this](const std::string& name, const double ms) {
                this->LoggingTimeProfiler(name, ms);
            });
        }
    }

    void XPBDWorld::WaitForPhysicsWorldAsync()
    {
        if (!isTaskLoading)
            return;

        static Internal::TimeProfiler timeProfiler(__func__);
        if (DEBUG_MODE)
            timeProfiler.Start();

        std::lock_guard lg(lock);
        threadPoolAsync->Execute([&] { backGroundTask.wait(); });
        threadPool->Execute([&] {
            ClampObjectRotation(DeltaTime60 * preCalcStepCount, preCalcStepCount);
            ApplyToSkyrim(true);
        });
        isTaskLoading = false;

        if (DEBUG_MODE)
        {
            timeProfiler.End([this](const std::string& name, const double ms) {
                this->LoggingTimeProfiler(name, ms);
            });
        }
    }

    void XPBDWorld::UpdateObjectData(const float deltaTime)
    {
        if (physicsBonesGroup.empty())
            return;

        static Internal::TimeProfiler timeProfiler(__func__);
        if (DEBUG_MODE)
            timeProfiler.Start();

        const Vector invDt = DirectX::XMVectorReciprocal(DirectX::XMVectorReplicate(deltaTime));
        const std::uint32_t groups = physicsBonesGroup.size() - 1u;
        tbb::parallel_for(
            tbb::blocked_range<std::uint32_t>(0, groups),
            [&](const tbb::blocked_range<std::uint32_t>& r) {
                for (std::uint32_t g = r.begin(); g != r.end(); ++g)
                {
                    const std::uint32_t begin = physicsBonesGroup[g];
                    const std::uint32_t end = physicsBonesGroup[g + 1u];
                    if (begin >= end)
                        continue;
                    const std::uint32_t oi = physicsBones.objIdx[begin];
                    if (oi == UINT32_MAX || objectDatas.objectID[oi] == 0)
                        continue;
                    RE::TESObjectREFR* object = GetREFR(objectDatas.objectID[oi]);
                    if (!object || !object->loadedData || !object->loadedData->data3D)
                        continue;
                    const Vector currentWorldPos = ToVector(object->loadedData->data3D->world.translate);
                    objectDatas.deltaWorldPos[oi] = DirectX::XMVectorSubtract(currentWorldPos, objectDatas.prevWorldPos[oi]);
                    const Vector currentVel = DirectX::XMVectorMultiply(objectDatas.deltaWorldPos[oi], invDt);
                    objectDatas.acceleration[oi] = DirectX::XMVectorMultiply(DirectX::XMVectorSubtract(currentVel, objectDatas.velocity[oi]), invDt);
                    objectDatas.velocity[oi] = currentVel;
                    objectDatas.prevWorldPos[oi] = currentWorldPos;
                    if (RE::TESObjectCELL* cell = object->GetParentCell(); cell)
                        objectDatas.bhkWorld[oi] = cell->GetbhkWorld();
                    else
                        objectDatas.bhkWorld[oi] = nullptr;
                }
            },
            tbb::static_partitioner()
        );

        if (DEBUG_MODE)
        {
            timeProfiler.End([this](const std::string& name, const double ms) {
                this->LoggingTimeProfiler(name, ms);
            });
        }
    }

    void XPBDWorld::ClampObjectRotation(const float deltaTime, const float stepCount)
    {
        if (physicsBonesGroup.empty() || ROTATION_CLAMP <= Epsilon)
            return;

        static Internal::TimeProfiler timeProfiler(__func__);
        if (DEBUG_MODE)
            timeProfiler.Start();

        auto isNeedClamp = [&](const std::uint32_t oi) {
            if (objectDatas.objectID[oi] != 0x14)
                return false;
            RE::Actor* object = GetActor(objectDatas.objectID[oi]);
            if (!object || !object->loadedData || !object->loadedData->data3D)
                return false;
            RE::ActorState* state = object->AsActorState();
            if (state && state->IsWeaponDrawn())
                return false;
            RE::PlayerCamera* playerCamera = RE::PlayerCamera::GetSingleton();
            return playerCamera && !playerCamera->IsInFirstPerson() && !playerCamera->IsInFreeCameraMode();
        };

        const float invDt = reciprocal(deltaTime);
        const float clamp = ROTATION_CLAMP * stepCount;
        const std::uint32_t groups = physicsBonesGroup.size() - 1;
        tbb::parallel_for(
            tbb::blocked_range<std::uint32_t>(0, groups),
            [&](const tbb::blocked_range<std::uint32_t>& r) {
                for (std::uint32_t g = r.begin(); g != r.end(); ++g)
                {
                    const std::uint32_t begin = physicsBonesGroup[g];
                    const std::uint32_t oi = physicsBones.objIdx[begin];
                    auto& npcNode = objectDatas.npcNode[oi];
                    if (!npcNode || !npcNode->parent)
                        continue;

                    const Quaternion q_prev = objectDatas.prevNPCWorldRot[oi];
                    const Quaternion q_inv = DirectX::XMQuaternionConjugate(q_prev);
                    if (isNeedClamp(oi))
                    {
                        const Quaternion q_curr = ToQuaternion(npcNode->world.rotate);
                        const Quaternion q_diff = DirectX::XMQuaternionMultiply(q_inv, q_curr);
                        const Quaternion q_target = DirectX::XMQuaternionNormalize(DirectX::XMQuaternionMultiply(q_diff, objectDatas.targetNPCWorldRot[oi]));
                        objectDatas.targetNPCWorldRot[oi] = q_target;

                        Quaternion q_delta = DirectX::XMQuaternionMultiply(q_inv, q_target);
                        if (DirectX::XMVectorGetW(q_delta) < 0.0f)
                            q_delta = DirectX::XMVectorNegate(q_delta);

                        const float cosHalfAngle = std::clamp(DirectX::XMVectorGetW(q_delta), -1.0f, 1.0f);
                        const float angle = 2.0f * DirectX::XMScalarACos(cosHalfAngle);
                        const float t = (angle > clamp) ? (clamp * reciprocal(angle)) : 1.0f;
                        const Quaternion q_final = DirectX::XMQuaternionNormalize(DirectX::XMQuaternionSlerp(q_prev, q_target, t));
                        const RE::NiMatrix3 npc_world = ToNiMatrix(q_final);
                        const RE::NiMatrix3 npc_local = npcNode->parent->world.rotate.Transpose() * npc_world;
                        objectDatas.prevNPCWorldRot[oi] = q_final;
                        objectDatas.deltaWorldRot[oi] = DirectX::XMQuaternionSlerp(qZero, DirectX::XMQuaternionMultiply(q_inv, q_final), reciprocal(stepCount));

                        std::memcpy(&npcNode->local.rotate, &npc_local, sizeof(npc_local));
                        std::memcpy(&npcNode->world.rotate, &npc_world, sizeof(npc_world));
                        RE::NiUpdateData ctx = {0.0f, RE::NiUpdateData ::Flag::kDirty};
                        npcNode->UpdateWorldData(&ctx);
                        UpdateChildTreeData(npcNode.get(), RE::NiUpdateData::Flag::kDirty);
                    }
                    else
                    {
                        objectDatas.prevNPCWorldRot[oi] = objectDatas.targetNPCWorldRot[oi];
                        const Quaternion q_final = ToQuaternion(npcNode->world.rotate);
                        objectDatas.targetNPCWorldRot[oi] = q_final;
                        objectDatas.deltaWorldRot[oi] = DirectX::XMQuaternionSlerp(qZero, DirectX::XMQuaternionMultiply(q_inv, q_final), reciprocal(stepCount));
                    }
                    Vector axis;
                    float angle;
                    DirectX::XMQuaternionToAxisAngle(&axis, &angle, objectDatas.deltaWorldRot[oi]);
                    objectDatas.omegaWorldRot[oi] = DirectX::XMVectorScale(axis, angle * invDt);
                }
            }
        );

        if (DEBUG_MODE)
        {
            timeProfiler.End([this](const std::string& name, const double ms) {
                this->LoggingTimeProfiler(name, ms);
            });
        }
    }

    void XPBDWorld::PrefetchBoneDatas()
    {
        static Internal::TimeProfiler timeProfiler(__func__);
        if (DEBUG_MODE)
            timeProfiler.Start();

        tbb::parallel_for(
            tbb::blocked_range<std::uint32_t>(0, physicsBones.numBones, 32),
            [&](const tbb::blocked_range<std::uint32_t>& r) {
                for (std::uint32_t bi = r.begin(); bi != r.end(); ++bi)
                {
                    physicsBones.groundCache[bi].isCast = false;

                    if (physicsBones.isParticle[bi])
                        continue;
                    auto& node = physicsBones.node[bi];
                    if (!node)
                        continue;
                    const std::uint32_t oi = physicsBones.objIdx[bi];
                    if (oi == UINT32_MAX)
                        continue;
                    if (Epsilon < physicsBones.invMass[bi])
                    {
                        physicsBones.pos[bi] = DirectX::XMVectorAdd(physicsBones.pos[bi], objectDatas.deltaWorldPos[oi]);
                        physicsBones.prevPos[bi] = DirectX::XMVectorAdd(physicsBones.prevPos[bi], objectDatas.deltaWorldPos[oi]);
                        physicsBones.predPos[bi] = DirectX::XMVectorAdd(physicsBones.predPos[bi], objectDatas.deltaWorldPos[oi]);
                    }
                    physicsBones.prevNodeWorldPos[bi] = physicsBones.targetNodeWorldPos[bi];
                    physicsBones.prevNodeWorldRot[bi] = physicsBones.targetNodeWorldRot[bi];
                    physicsBones.targetNodeWorldPos[bi] = ToVector(node->world.translate);
                    physicsBones.targetNodeWorldRot[bi] = ToQuaternion(node->world.rotate);
                    physicsBones.orgWorldScale[bi] = node->world.scale;
                }
            },
            tbb::static_partitioner()
        );

        if (DEBUG_MODE)
        {
            timeProfiler.End([this](const std::string& name, const double ms) {
                this->LoggingTimeProfiler(name, ms);
            });
        }
    }

    void XPBDWorld::InterpolateBoneDatas(const float alpha, const float nextAlpha)
    {
        static Internal::TimeProfiler timeProfiler(__func__);
        if (DEBUG_MODE)
            timeProfiler.Start();

        tbb::parallel_for(
            tbb::blocked_range<std::uint32_t>(0, physicsBones.numBones, 32),
            [&](const tbb::blocked_range<std::uint32_t>& r) {
                for (std::uint32_t bi = r.begin(); bi != r.end(); ++bi)
                {
                    if (!physicsBones.isParticle[bi])
                    {
                        const Vector interPos = DirectX::XMVectorLerp(physicsBones.prevNodeWorldPos[bi], physicsBones.targetNodeWorldPos[bi], alpha);
                        const Quaternion interRot = DirectX::XMQuaternionSlerp(physicsBones.prevNodeWorldRot[bi], physicsBones.targetNodeWorldRot[bi], alpha);

                        if (physicsBones.invMass[bi] <= Epsilon)
                        {
                            const Vector offset = DirectX::XMVector3Rotate(DirectX::XMVectorScale(physicsBones.offset[bi], physicsBones.orgWorldScale[bi]), interRot);
                            physicsBones.backupPos[bi] = physicsBones.pos[bi];
                            physicsBones.pos[bi] = DirectX::XMVectorAdd(interPos, offset);
                            physicsBones.prevPos[bi] = physicsBones.pos[bi];
                            const Vector nextInterPos = DirectX::XMVectorLerp(physicsBones.prevNodeWorldPos[bi], physicsBones.targetNodeWorldPos[bi], nextAlpha);
                            const Quaternion nextInterRot = DirectX::XMQuaternionSlerp(physicsBones.prevNodeWorldRot[bi], physicsBones.targetNodeWorldRot[bi], nextAlpha);
                            const Vector nextOffset = DirectX::XMVector3Rotate(DirectX::XMVectorScale(physicsBones.offset[bi], physicsBones.orgWorldScale[bi]), nextInterRot);
                            physicsBones.predPos[bi] = DirectX::XMVectorAdd(nextInterPos, nextOffset);

                            physicsBones.backupRot[bi] = physicsBones.rot[bi];
                            physicsBones.rot[bi] = interRot;
                            physicsBones.prevRot[bi] = physicsBones.rot[bi];
                            physicsBones.predRot[bi] = nextInterRot;
                        }
                    }
                    else
                    {
                        if (physicsBones.parentBoneIdx[bi] != UINT32_MAX)
                        {
                            const std::uint32_t pbi = physicsBones.parentBoneIdx[bi];
                            if (physicsBones.invMass[bi] <= Epsilon)
                            {
                                const Vector offset = DirectX::XMVector3Rotate(DirectX::XMVectorScale(physicsBones.offset[bi], physicsBones.orgWorldScale[pbi]), physicsBones.rot[pbi]);
                                physicsBones.backupPos[bi] = physicsBones.pos[bi];
                                physicsBones.pos[bi] = DirectX::XMVectorAdd(physicsBones.pos[pbi], offset);
                                physicsBones.prevPos[bi] = physicsBones.pos[bi];
                                const Vector nextOffset = DirectX::XMVector3Rotate(DirectX::XMVectorScale(physicsBones.offset[bi], physicsBones.orgWorldScale[pbi]), physicsBones.predRot[pbi]);
                                physicsBones.predPos[bi] = DirectX::XMVectorAdd(physicsBones.predPos[pbi], nextOffset);

                                physicsBones.backupRot[bi] = physicsBones.rot[bi];
                                physicsBones.rot[bi] = physicsBones.rot[pbi];
                                physicsBones.prevRot[bi] = physicsBones.rot[bi];
                                physicsBones.predRot[bi] = physicsBones.predRot[pbi];
                            }
                            physicsBones.rot[bi] = physicsBones.rot[pbi];
                            physicsBones.orgWorldScale[bi] = physicsBones.orgWorldScale[pbi];
                        }
                    }
                }
            },
            tbb::static_partitioner()
        );

        if (DEBUG_MODE)
        {
            timeProfiler.End([this](const std::string& name, const double ms) {
                this->LoggingTimeProfiler(name, ms);
            });
        }
    }

    void XPBDWorld::UpdateGlobalAABBTree()
    {
        static Internal::TimeProfiler timeProfiler(__func__);
        if (DEBUG_MODE)
            timeProfiler.Start();

        if (objIdxToTreeNodeIdx.size() < objectDatas.objectID.size())
            objIdxToTreeNodeIdx.resize(objectDatas.objectID.size(), UINT32_MAX);

        for (std::uint32_t oi = 0; oi < objectDatas.objectID.size(); ++oi)
        {
            if (objectDatas.objectID[oi] == 0)
            {
                if (objIdxToTreeNodeIdx[oi] != UINT32_MAX)
                {
                    globalAABBTree.RemoveLeaf(objIdxToTreeNodeIdx[oi]);
                    objIdxToTreeNodeIdx[oi] = UINT32_MAX;
                }
                objectDatas.boundingAABB[oi] = AABB();
                continue;
            }
            objectDatas.boundingAABB[oi] = GetObjectAABB(oi);
            if (objectDatas.boundingAABB[oi].IsInvalid())
                continue;
            if (objIdxToTreeNodeIdx[oi] == UINT32_MAX)
            {
                AABB fatAABB = objectDatas.boundingAABB[oi];
                fatAABB.Fatten();
                objIdxToTreeNodeIdx[oi] = globalAABBTree.InsertLeaf(oi, fatAABB);
            }
            else
                objIdxToTreeNodeIdx[oi] = globalAABBTree.UpdateLeaf(objIdxToTreeNodeIdx[oi], objectDatas.boundingAABB[oi]);
        }

        if (DEBUG_MODE)
        {
            timeProfiler.End([this](const std::string& name, const double ms) {
                this->LoggingTimeProfiler(name, ms);
            });
        }
    }

    void XPBDWorld::ObjectCulling()
    {
        RE::NiCamera* niCam = RE::Main::WorldRootCamera();
        if (!niCam)
            return;

        static Internal::TimeProfiler timeProfiler(__func__);
        if (DEBUG_MODE)
            timeProfiler.Start();

        const Vector camPos = ToVector(niCam->world.translate);

        const auto m = niCam->GetRuntimeData().worldToCam;
        const Vector r0 = DirectX::XMVectorSet(m[0][0], m[0][1], m[0][2], m[0][3]);
        const Vector r1 = DirectX::XMVectorSet(m[1][0], m[1][1], m[1][2], m[1][3]);
        const Vector r2 = DirectX::XMVectorSet(m[2][0], m[2][1], m[2][2], m[2][3]);
        const Vector r3 = DirectX::XMVectorSet(m[3][0], m[3][1], m[3][2], m[3][3]);

        Vector planes[6];
        planes[0] = DirectX::XMVectorAdd(r3, r0);      // left
        planes[1] = DirectX::XMVectorSubtract(r3, r0); // right
        planes[2] = DirectX::XMVectorAdd(r3, r1);      // bottom
        planes[3] = DirectX::XMVectorSubtract(r3, r1); // top
        planes[4] = r2;                                // near
        planes[5] = DirectX::XMVectorSubtract(r3, r2); // far
        Vector absPlanes[6];
        for (int i = 0; i < 6; ++i)
        {
            absPlanes[i] = DirectX::XMVectorAbs(planes[i]);
        }

        const std::uint32_t objects = objectDatas.objectID.size();
        tbb::parallel_for(
            tbb::blocked_range<std::uint32_t>(0, objects),
            [&](const tbb::blocked_range<std::uint32_t>& r) {
                for (std::uint32_t oi = r.begin(); oi != r.end(); ++oi)
                {
                    if (objectDatas.objectID[oi] == 0)
                        continue;
                    if (objectDatas.isDisableByToggle[oi])
                    {
                        objectDatas.isDisable[oi] = objectDatas.isDisableByToggle[oi];
                        continue;
                    }
                    if (objectDatas.objectID[oi] == 0x14)
                        continue;
                    const AABB& boundingAABB = objectDatas.boundingAABB[oi];
                    if (boundingAABB.IsInvalid())
                        continue;

                    // culling by distance
                    const Vector closePt = DirectX::XMVectorClamp(camPos, boundingAABB.min, boundingAABB.max);
                    const Vector vDistSq = DirectX::XMVector3LengthSq(DirectX::XMVectorSubtract(camPos, closePt));
                    const float distSq = DirectX::XMVectorGetX(vDistSq);
                    if (CULLING_DISTANCE_SQ < distSq)
                    {
                        objectDatas.isDisable[oi] = 1;
                        continue;
                    }

                    // culling by camera
                    const Vector center = boundingAABB.GetCenter();
                    const Vector extents = boundingAABB.GetExtents();
                    bool isOutside = false;
                    for (std::int32_t p = 0; p < 6; ++p)
                    {
                        const Vector dotC = DirectX::XMVector3Dot(planes[p], center);
                        const Vector d = DirectX::XMVectorAdd(dotC, DirectX::XMVectorSplatW(planes[p]));
                        const Vector r_proj = DirectX::XMVector3Dot(absPlanes[p], extents);
                        if (DirectX::XMVector3Less(DirectX::XMVectorAdd(d, r_proj), vZero))
                        {
                            isOutside = true;
                            break;
                        }
                    }
                    const bool isChanged = objectDatas.isDisable[oi] != isOutside;
                    objectDatas.isDisable[oi] = isOutside;

                    if (!isOutside)
                    {
                        // collision culling
                        const float normalizedDistSq = distSq * INV_CULLING_DISTANCE_SQ;
                        const std::uint32_t lodPoints = static_cast<std::uint32_t>(4.0f - (3.0f * normalizedDistSq));
                        objectDatas.maxManifoldPoints[oi] = std::clamp(lodPoints, 1u, 4u);
                    }
                }
            },
            tbb::static_partitioner()
        );
        if (DEBUG_MODE)
        {
            timeProfiler.End([this](const std::string& name, const double ms) {
                this->LoggingTimeProfiler(name, ms);
            });
        }
    }

    void XPBDWorld::UpdateWindStrength()
    {
        if (physicsBonesGroup.empty() || windSpeed <= Epsilon)
            return;

        static Internal::TimeProfiler timeProfiler(__func__);
        if (DEBUG_MODE)
            timeProfiler.Start();

        const std::uint32_t groups = physicsBonesGroup.size() - 1u;
        const std::uint32_t quality = std::max(1u, std::min(WIND_DETECT_QUALITY, groups));
        const std::uint32_t chunkSize = (groups + quality - 1u) * reciprocal(quality);
        const std::uint32_t gBegin = (currentFrame % quality) * chunkSize;
        const std::uint32_t gEnd = std::min(gBegin + chunkSize, groups);
        if (gBegin >= groups)
            return;
        float windAngleSin, windAngleCos;
        DirectX::XMScalarSinCos(&windAngleSin, &windAngleCos, windAngle);
        const Vector windVector = DirectX::XMVector3Normalize(DirectX::XMVectorSet(windAngleSin, windAngleCos, 0.0f, 0.0f));

        class WindHitCollector : public RE::hkpRayHitCollector
        {
        public:
            void AddRayHit(const RE::hkpCdBody& a_body, const RE::hkpShapeRayCastCollectorOutput& a_hitInfo) override
            {
                if (rayHit.hitFraction <= a_hitInfo.hitFraction)
                    return;
                const RE::hkpCdBody* hkpCdBody = &a_body;
                while (hkpCdBody->parent != nullptr)
                {
                    hkpCdBody = hkpCdBody->parent;
                }
                const RE::hkpCollidable* collidable = static_cast<const RE::hkpCollidable*>(hkpCdBody);
                const RE::COL_LAYER layer = collidable->GetCollisionLayer();

                if (layer != RE::COL_LAYER::kStatic &&
                    layer != RE::COL_LAYER::kGround &&
                    layer != RE::COL_LAYER::kTrees &&
                    layer != RE::COL_LAYER::kAnimStatic &&
                    layer != RE::COL_LAYER::kTerrain)
                {
                    return;
                }
                rayHit.rootCollidable = collidable;
                rayHit.hitFraction = a_hitInfo.hitFraction;
                rayHit.normal = a_hitInfo.normal;
                earlyOutHitFraction = a_hitInfo.hitFraction;
            }
            RE::hkpWorldRayCastOutput rayHit;
        };

        for (std::uint32_t g = gBegin; g < gEnd; ++g)
        {
            const std::uint32_t begin = physicsBonesGroup[g];
            const std::uint32_t end = physicsBonesGroup[g + 1u];
            if (begin >= end)
                continue;
            const std::uint32_t oi = physicsBones.objIdx[begin];
            if (IsDisable(oi))
                continue;
            RE::bhkWorld* bhkWorld = objectDatas.bhkWorld[oi];
            if (!bhkWorld)
                continue;

            const AABB& worldAABB = objectDatas.boundingAABB[oi];
            const Vector worldCenter = worldAABB.GetCenter();
            const Vector extents = worldAABB.GetExtents(0.75f);

            const float rx = rand_PCG32_Float(objectDatas.randState[oi]);
            const float ry = rand_PCG32_Float(objectDatas.randState[oi]);
            const float rz = rand_PCG32_Float(objectDatas.randState[oi]);
            const Vector randomOffset = DirectX::XMVectorMultiply(DirectX::XMVectorSet(rx, ry, rz, 0.0f), extents);

            const Vector from = DirectX::XMVectorAdd(worldCenter, randomOffset);
            const Vector havokFrom = DirectX::XMVectorMultiply(from, vSkyrimWorldScale);
            const Vector windRayTo = DirectX::XMVectorMultiply(windVector, WIND_DETECT_RANGE);
            const Vector to = DirectX::XMVectorAdd(from, windRayTo);
            const Vector havokTo = DirectX::XMVectorMultiply(to, vSkyrimWorldScale);

            RE::bhkPickData pickData;
            pickData.rayInput.from = havokFrom;
            pickData.rayInput.to = havokTo;

            WindHitCollector hitCollector;
            hitCollector.Reset();
            pickData.rayHitCollectorA8 = reinterpret_cast<RE::hkpClosestRayHitCollector*>(&hitCollector);
            bhkWorld->PickObject(pickData);

            float windMultiplier = 1.0f;
            if (hitCollector.rayHit.HasHit())
            {
                const float distanceFactor = hitCollector.rayHit.hitFraction * hitCollector.rayHit.hitFraction;
                const Vector hitNormal = DirectX::XMVectorSetW(hitCollector.rayHit.normal.quad, 0.0f);
                const float normalDot = DirectX::XMVectorGetX(DirectX::XMVector3Dot(windVector, hitNormal));
                float angleAttenuation = 0.0f;
                if (Epsilon < normalDot)
                {
                    angleAttenuation = std::abs(normalDot);
                    angleAttenuation = angleAttenuation * rsqrt(angleAttenuation);
                }
                windMultiplier = 1.0f - ((1.0f - distanceFactor) * angleAttenuation);
            }
            const float currentMultiplier = DirectX::XMVectorGetX(objectDatas.windMultiplier[oi]);
            float responseSpeed = 0.02f + (windSpeed * 0.5f);
            if (windMultiplier > currentMultiplier)
                responseSpeed *= 2.5f;
            else
                responseSpeed *= 1.2f;
            responseSpeed = std::clamp(responseSpeed, 0.01f, 1.0f);
            objectDatas.windMultiplier[oi] = DirectX::XMVectorReplicate(currentMultiplier + (windMultiplier - currentMultiplier) * responseSpeed);
        }

        if (DEBUG_MODE)
        {
            timeProfiler.End([this](const std::string& name, const double ms) {
                this->LoggingTimeProfiler(name, ms);
            });
        }
    }

    void XPBDWorld::PredictBones(const float deltaTime)
    {
        static Internal::TimeProfiler timeProfiler(__func__);
        if (DEBUG_MODE)
            timeProfiler.Start();

        const Vector dt = DirectX::XMVectorReplicate(deltaTime);
        const Vector invDt = DirectX::XMVectorReciprocal(dt);
        const Vector invDtDb = DirectX::XMVectorScale(invDt, 2.0f);
        float windAngleSin, windAngleCos;
        DirectX::XMScalarSinCos(&windAngleSin, &windAngleCos, windAngle);
        const float windBaseX = windAngleSin * windSpeed * WIND_MULTIPLIER;
        const float windBaseY = windAngleCos * windSpeed * WIND_MULTIPLIER;
        const float windBaseZ = windSpeed * WIND_MULTIPLIER * 0.1f;
        const Vector baseWind = DirectX::XMVectorSet(windBaseX, windBaseY, windBaseZ, 0.0f);
        const float timeFreqSlow = currentFrame * 0.02f;
        const float timeFreqFast = currentFrame * 0.1f;
        tbb::parallel_for(
            tbb::blocked_range<std::uint32_t>(0, physicsBones.numBones, 128),
            [&](const tbb::blocked_range<std::uint32_t>& r) {
                for (std::uint32_t bi = r.begin(); bi != r.end(); ++bi)
                {
                    physicsBones.dynamicLinearOffset[bi] = vZero;
                    physicsBones.dynamicAngularOffset[bi] = qZero;

                    const std::uint32_t oi = physicsBones.objIdx[bi];
                    if (IsDisable(oi))
                        continue;
                    if (physicsBones.invMass[bi] <= Epsilon)
                        continue;
                    const std::uint32_t pbi = physicsBones.parentBoneIdx[bi];
                    if (pbi == UINT32_MAX)
                        continue;
                    Vector linearDeltaVel = vZero;
                    const Quaternion invRot = DirectX::XMQuaternionConjugate(physicsBones.rot[pbi]);

                    std::uint32_t rootPbi = pbi;
                    while (rootPbi != UINT32_MAX && Epsilon < physicsBones.invMass[rootPbi])
                    {
                        rootPbi = physicsBones.parentBoneIdx[rootPbi];
                    }

                    // apply linear inertia
                    {
                        const Vector& worldAcc = objectDatas.acceleration[oi];
                        const Vector localAcc = DirectX::XMVector3Rotate(worldAcc, invRot);
                        const Vector isPositive = DirectX::XMVectorGreaterOrEqual(localAcc, vZero);
                        const Vector currentInertia = DirectX::XMVectorSelect(physicsBones.inertiaNegative[bi], physicsBones.inertiaPositive[bi], isPositive);
                        const Vector scaledLocalAcc = DirectX::XMVectorMultiply(localAcc, currentInertia);
                        const Vector scaledWorldAcc = DirectX::XMVector3Rotate(scaledLocalAcc, physicsBones.rot[pbi]);
                        const Vector fictAcc = DirectX::XMVectorNegate(scaledWorldAcc);
                        const Vector totalAcc = DirectX::XMVectorMultiply(DirectX::XMVectorAdd(physicsBones.gravity[bi], fictAcc), dt);
                        linearDeltaVel = DirectX::XMVectorAdd(linearDeltaVel, totalAcc);
                        physicsBones.posVel[bi] = DirectX::XMVectorAdd(physicsBones.posVel[bi], totalAcc);
                    }

                    // apply wind
                    if (DirectX::XMVector3Greater(objectDatas.windMultiplier[oi], vEpsilon))
                    {
                        const Vector& pos = physicsBones.pos[bi];
                        const float phase = DirectX::XMVectorGetX(pos) * 0.03f + DirectX::XMVectorGetY(pos) * 0.02f + DirectX::XMVectorGetZ(pos) * 0.05f;
                        const float gustNoise1 = DirectX::XMScalarSin(timeFreqSlow + phase);
                        const float gustNoise2 = DirectX::XMScalarCos(timeFreqFast - phase * 1.5f);
                        const float gustScale = windSpeed * WIND_MULTIPLIER * 0.4f;
                        const Vector gust = DirectX::XMVectorSet(gustNoise1 * gustScale, gustNoise2 * gustScale, (gustNoise1 * gustNoise2) * gustScale * 1.5f, 0.0f);
                        const Vector worldWind = DirectX::XMVectorAdd(baseWind, gust);
                        const Vector relVel = DirectX::XMVectorSubtract(worldWind, physicsBones.posVel[bi]);
                        const Vector localWindScale = DirectX::XMVectorMultiply(physicsBones.windFactor[bi], objectDatas.windMultiplier[oi]);
                        const Vector worldWindForce = DirectX::XMVectorMultiply(relVel, DirectX::XMVectorMultiply(localWindScale, dt));
                        linearDeltaVel = DirectX::XMVectorAdd(linearDeltaVel, worldWindForce);
                        physicsBones.posVel[bi] = DirectX::XMVectorAdd(physicsBones.posVel[bi], worldWindForce);
                    }

                    // apply centrifugal force
                    /*{
                        const Quaternion q_delta = objectDatas.deltaWorldRot[oi];
                        const Vector axisRaw = DirectX::XMVectorSetW(q_delta, 0.0f);
                        const Vector axisLenSq = DirectX::XMVector3LengthSq(axisRaw);
                        if (DirectX::XMVector3Greater(axisLenSq, vAngularEpsilonSq))
                        {
                            const float w = std::clamp(DirectX::XMVectorGetW(q_delta), -1.0f, 1.0f);
                            const float angle = 2.0f * DirectX::XMScalarACos(w);
                            const Vector axis = DirectX::XMVectorMultiply(axisRaw, DirectX::XMVectorReciprocalSqrt(axisLenSq));
                            const Vector omega = DirectX::XMVectorMultiply(axis, DirectX::XMQuaternionMultiply(DirectX::XMVectorReplicate(angle), invDt));
                            const Vector rootPos = (rootPbi != UINT32_MAX) ? physicsBones.pos[rootPbi] : objectDatas.prevWorldPos[oi];
                            const Vector r = DirectX::XMVectorSubtract(physicsBones.pos[bi], rootPos);
                            const Vector tangential = DirectX::XMVector3Cross(omega, r);
                            const Vector centrifugalAccel = DirectX::XMVector3Cross(tangential, omega);
                            const Vector localCentrifugal = DirectX::XMVector3Rotate(centrifugalAccel, invRot);
                            const Vector isPositive = DirectX::XMVectorGreaterOrEqual(localCentrifugal, vZero);
                            const Vector currentInertia = DirectX::XMVectorSelect(physicsBones.inertiaNegative[bi], physicsBones.inertiaPositive[bi], isPositive);
                            const Vector scaledLocalCentrifugal = DirectX::XMVectorMultiply(localCentrifugal, currentInertia);
                            const Vector finalCentrifugalAccel = DirectX::XMVector3Rotate(scaledLocalCentrifugal, physicsBones.rot[pbi]);
                            const Vector centrifugalDeltaVel = DirectX::XMVectorMultiply(finalCentrifugalAccel, dt);
                            linearDeltaVel = DirectX::XMVectorAdd(linearDeltaVel, centrifugalDeltaVel);
                            physicsBones.posVel[bi] = DirectX::XMVectorAdd(physicsBones.posVel[bi], centrifugalDeltaVel);
                        }
                    }*/

                    // predic linear
                    {
                        const Vector relVel = DirectX::XMVectorSubtract(physicsBones.posVel[bi], objectDatas.velocity[oi]);
                        physicsBones.predPos[bi] = DirectX::XMVectorMultiplyAdd(relVel, dt, physicsBones.pos[bi]);
                    }

                    // apply angular velocity
                    if (physicsBones.advancedRotation[bi])
                    {
                        const Quaternion invRotBi = DirectX::XMQuaternionConjugate(physicsBones.rot[bi]);

                        // linear rot torque
                        {
                            const Vector localDeltaVel = DirectX::XMVector3Rotate(linearDeltaVel, invRotBi);
                            const Vector localFakeTorque = DirectX::XMVector3TransformNormal(localDeltaVel, physicsBones.linearRotTorque[bi]);
                            if (DirectX::XMVector3Greater(DirectX::XMVector3LengthSq(localFakeTorque), vAngularEpsilonSq))
                            {
                                const Vector worldFakeTorque = DirectX::XMVector3Rotate(localFakeTorque, physicsBones.rot[bi]);
                                physicsBones.angVel[bi] = DirectX::XMVectorAdd(physicsBones.angVel[bi], worldFakeTorque);
                            }
                        }

                        // add gravity rotate
                        /*{
                            const Vector localCom = physicsBones.centerOfMass[bi];
                            if (DirectX::XMVector3Greater(DirectX::XMVector3LengthSq(localCom), vEpsilonSq))
                            {
                                const Vector localGravityAccel = DirectX::XMVector3Rotate(SkyrimGravity, invRotBi);
                                const Vector localCross = DirectX::XMVector3Cross(localCom, localGravityAccel);
                                const Vector isPositiveTorque = DirectX::XMVectorGreaterOrEqual(localCross, vZero);
                                const Vector localInvInertia = DirectX::XMVectorSelect(physicsBones.invAngularInertiaNegative[bi], physicsBones.invAngularInertiaPositive[bi], isPositiveTorque);
                                const Vector localDeltaGravityAngVel = DirectX::XMVectorMultiply(DirectX::XMVectorMultiply(localCross, localInvInertia), dt);
                                const Vector worldDeltaGravityAngVel = DirectX::XMVector3Rotate(localDeltaGravityAngVel, physicsBones.rot[bi]);
                                physicsBones.angVel[bi] = DirectX::XMVectorAdd(physicsBones.angVel[bi], worldDeltaGravityAngVel);
                            }
                        }*/
                    }

                    // predic rotation
                    {
                        const Quaternion& q = physicsBones.rot[bi];
                        const Quaternion w = DirectX::XMVectorMultiply(physicsBones.angVel[bi], DirectX::XMVectorMultiply(dt, vHalf));
                        physicsBones.predRot[bi] = DirectX::XMQuaternionNormalize(DirectX::XMVectorAdd(q, DirectX::XMQuaternionMultiply(q, w)));
                    }

                    if (IsSkippedFrame(bi) ||
                        IsInvalid(physicsBones.predPos[bi]) ||
                        IsInvalid(physicsBones.predRot[bi]) ||
                        IsInvalid(physicsBones.posVel[bi]) ||
                        IsInvalid(physicsBones.angVel[bi]))
                    {
                        physicsBones.predPos[bi] = physicsBones.pos[bi];
                        physicsBones.predRot[bi] = physicsBones.rot[bi];
                        physicsBones.posVel[bi] = vZero;
                        physicsBones.angVel[bi] = vZero;
                    }

                    physicsBones.lastFrame[bi] = currentFrame;
                }
            },
            tbb::static_partitioner()
        );

        if (DEBUG_MODE)
        {
            timeProfiler.End([this](const std::string& name, const double ms) {
                this->LoggingTimeProfiler(name, ms);
            });
        }
    }

    void XPBDWorld::RunDriver(const float deltaTime)
    {
        auto ctx = GetContext();
        driver->RunDriver(ctx, deltaTime);    
    }

    void XPBDWorld::CreateLocalSpatialHash()
    {
        if (collidersGroup.empty())
            return;
        if (objectHashesSmall.size() < objectDatas.objectID.size())
            objectHashesSmall.resize(objectDatas.objectID.size());
        if (objectHashesLarge.size() < objectDatas.objectID.size())
            objectHashesLarge.resize(objectDatas.objectID.size());

        static Internal::TimeProfiler timeProfiler(__func__);
        if (DEBUG_MODE)
            timeProfiler.Start();

        const std::uint32_t groups = collidersGroup.size() - 1ull;
        tbb::parallel_for(
            tbb::blocked_range<std::uint32_t>(0, groups),
            [&](const tbb::blocked_range<std::uint32_t>& r) {
                for (std::uint32_t g = r.begin(); g != r.end(); ++g)
                {
                    const std::uint32_t begin = collidersGroup[g];
                    const std::uint32_t end = collidersGroup[g + 1];
                    if (begin >= end)
                        continue;
                    const std::uint32_t oi = colliders.objIdx[begin];
                    if (IsDisable(oi))
                        continue;
                    auto& localHashSmall = objectHashesSmall[oi];
                    auto& localHashLarge = objectHashesLarge[oi];
                    localHashSmall.Init(end - begin, SMALL_GRID_SIZE, COL_HASH_TABLE_SIZE);
                    localHashLarge.Init(end - begin, LARGE_GRID_SIZE, COL_HASH_TABLE_SIZE);
                    struct EntryCache {
                        float radius = 0.0f;
                        Vector worldCenter = vZero;
                        std::uint32_t hashHigh = 0;
                        std::uint32_t hashLow = 0;
                    };
                    std::vector<EntryCache> entryCache(end - begin);
                    for (std::uint32_t ci = begin; ci < end; ++ci)
                    {
                        auto& entry = entryCache[ci - begin];
                        const std::uint32_t bi = colliders.boneIdx[ci];
                        entry.radius = colliders.boundingSphere[ci] * physicsBones.orgWorldScale[bi] + physicsBones.collisionMargin[bi];
                        const Quaternion worldRot = DirectX::XMQuaternionMultiply(physicsBones.dynamicAngularOffset[bi], physicsBones.predRot[bi]);
                        const Vector rotateCenter = DirectX::XMVector3Rotate(DirectX::XMVectorScale(colliders.boundingSphereCenter[ci], physicsBones.orgWorldScale[bi]), worldRot);
                        entry.worldCenter = DirectX::XMVectorAdd(DirectX::XMVectorAdd(physicsBones.predPos[bi], rotateCenter), physicsBones.dynamicLinearOffset[bi]);
                        if (entry.radius <= SMALL_GRID_SIZE * 0.5f)
                        {
                            entry.hashHigh = localHashSmall.HashWorldCoordsHigh(entry.worldCenter);
                            localHashSmall.cellCount[entry.hashHigh]++;
                            entry.hashLow = localHashSmall.HashWorldCoordsLow(entry.worldCenter);
                            localHashSmall.cellCount[entry.hashLow]++;
                        }
                        else
                        {
                            entry.hashHigh = localHashLarge.HashWorldCoordsHigh(entry.worldCenter);
                            localHashLarge.cellCount[entry.hashHigh]++;
                            entry.hashLow = localHashLarge.HashWorldCoordsLow(entry.worldCenter);
                            localHashLarge.cellCount[entry.hashLow]++;
                        }
                    }
                    localHashSmall.cell[0] = 0;
                    localHashLarge.cell[0] = 0;
                    for (std::uint32_t i = 0; i < COL_HASH_TABLE_SIZE; ++i)
                    {
                        localHashSmall.cell[i + 1] = localHashSmall.cell[i] + localHashSmall.cellCount[i];
                        localHashSmall.cellCount[i] = 0;
                        localHashLarge.cell[i + 1] = localHashLarge.cell[i] + localHashLarge.cellCount[i];
                        localHashLarge.cellCount[i] = 0;
                    }
                    for (std::uint32_t ci = begin; ci < end; ++ci)
                    {
                        const auto& entry = entryCache[ci - begin]; 
                        if (entry.radius <= SMALL_GRID_SIZE * 0.5f)
                        {
                            const std::uint32_t offsetHigh = localHashSmall.cell[entry.hashHigh] + localHashSmall.cellCount[entry.hashHigh]++;
                            localHashSmall.entries[offsetHigh] = ci;
                            const std::uint32_t offsetLow = localHashSmall.cell[entry.hashLow] + localHashSmall.cellCount[entry.hashLow]++;
                            localHashSmall.entries[offsetLow] = ci;
                        }
                        else
                        {
                            const std::uint32_t offsetHigh = localHashLarge.cell[entry.hashHigh] + localHashLarge.cellCount[entry.hashHigh]++;
                            localHashLarge.entries[offsetHigh] = ci;
                            const std::uint32_t offsetLow = localHashLarge.cell[entry.hashLow] + localHashLarge.cellCount[entry.hashLow]++;
                            localHashLarge.entries[offsetLow] = ci;
                        }
                    }
                }
            },
            tbb::auto_partitioner()
        );

        if (DEBUG_MODE)
        {
            timeProfiler.End([this](const std::string& name, const double ms) {
                this->LoggingTimeProfiler(name, ms);
            });
        }
    }

    void XPBDWorld::GenerateCollisionManifolds()
    {
        if (collidersGroup.empty())
            return;

        static Internal::TimeProfiler timeProfiler(__func__);
        if (DEBUG_MODE)
            timeProfiler.Start();

        auto AddManifold = [&](const std::uint32_t coiA, const std::uint32_t coiB) {
            Internal::ContactManifold manifold;
            bool isCollide = false;
            bool isSwapped = false;
            if (colliders.colliderType[coiA] == ColliderType::kSphere)
            {
                if (colliders.colliderType[coiB] == ColliderType::kSphere)
                    isCollide = SpherevsSphere(coiA, coiB, manifold);
                else if (colliders.colliderType[coiB] == ColliderType::kConvexHull)
                {
                    isCollide = ConvexHullvsSphere(coiB, coiA, manifold);
                    isSwapped = true;
                }
            }
            else if (colliders.colliderType[coiA] == ColliderType::kConvexHull)
            {
                if (colliders.colliderType[coiB] == ColliderType::kSphere)
                    isCollide = ConvexHullvsSphere(coiA, coiB, manifold);
                else if (colliders.colliderType[coiB] == ColliderType::kConvexHull)
                    isCollide = ConvexHullvsConvexHull(coiA, coiB, manifold);
            }
            if (!isCollide)
                return false;
            const std::uint32_t idx = std::atomic_ref<std::uint32_t>(manifoldCacheCount).fetch_add(1, std::memory_order_relaxed);
            if (idx < expectedCollisionCount)
            {
                manifoldCache[idx] = {coiA, coiB, manifold};
                if (isSwapped)
                    std::swap(manifoldCache[idx].coiA, manifoldCache[idx].coiB);
            }
            /*const std::uint32_t biA = colliders.boneIdx[coiA];
            const std::uint32_t biB = colliders.boneIdx[coiB];
            logger::debug("collide {} <-> {}", physicsBones.node[biA]->name.c_str(), physicsBones.node[biB]->name.c_str());*/
            return true;
        };

        tbb::enumerable_thread_specific<std::vector<std::uint32_t>> tls_checkedB(
            []() {
                std::vector<std::uint32_t> v;
                v.reserve(128);
                return v;
            });

        const std::uint32_t groups = collidersGroup.size() - 1;
        tbb::parallel_invoke([&] {
            tbb::parallel_for(
                tbb::blocked_range<std::uint32_t>(0u, colliders.numColliders),
                [&](const tbb::blocked_range<std::uint32_t>& cr) {
                    auto& checkedB = tls_checkedB.local();
                    for (std::uint32_t coiA = cr.begin(); coiA != cr.end(); ++coiA)
                    {
                        const std::uint32_t oi = colliders.objIdx[coiA];
                        if (IsDisable(oi))
                            continue;
                        const auto& ownHashSmall = objectHashesSmall[oi];
                        const auto& ownHashLarge = objectHashesLarge[oi];
                        if (ownHashSmall.cell.empty() || ownHashLarge.cell.empty())
                            continue;
                        const std::uint32_t biA = colliders.boneIdx[coiA];
                        const Quaternion worldRotA = DirectX::XMQuaternionMultiply(physicsBones.dynamicAngularOffset[biA], physicsBones.predRot[biA]);
                        const Vector rotateCenterA = DirectX::XMVector3Rotate(DirectX::XMVectorScale(colliders.boundingSphereCenter[coiA], physicsBones.orgWorldScale[biA]), worldRotA);
                        const Vector worldCenterA = DirectX::XMVectorAdd(DirectX::XMVectorAdd(physicsBones.predPos[biA], rotateCenterA), physicsBones.dynamicLinearOffset[biA]);
                        checkedB.clear();
                        auto CheckCell = [&](const std::uint32_t hash, const Internal::LocalSpatialHash& ownHash) {
                            const std::uint32_t beginHash = ownHash.cell[hash];
                            const std::uint32_t endHash = ownHash.cell[hash + 1];
                            for (std::uint32_t ei = beginHash; ei < endHash; ++ei)
                            {
                                const std::uint32_t coiB = ownHash.entries[ei];
                                if (coiA >= coiB)
                                    continue;
                                const std::uint32_t biB = colliders.boneIdx[coiB];
                                if (biA == biB)
                                    continue;
                                if (!IsCollide(biB, biA) && !IsCollide(biA, biB))
                                    continue;
                                if (physicsBones.invMass[biA] + physicsBones.invMass[biB] <= Epsilon)
                                    continue;
                                if (std::find(checkedB.begin(), checkedB.end(), coiB) != checkedB.end())
                                    continue;
                                checkedB.push_back(coiB);

                                const std::uint32_t noColCountA = colliders.noCollideCount[coiA];
                                auto beginA = colliders.noCollideBoneIdx.begin() + static_cast<std::uint32_t>(coiA) * NOCOLLIDE_MAX;
                                auto endA = beginA + noColCountA;
                                const std::uint32_t noColCountB = colliders.noCollideCount[coiB];
                                auto beginB = colliders.noCollideBoneIdx.begin() + static_cast<std::uint32_t>(coiB) * NOCOLLIDE_MAX;
                                auto endB = beginB + noColCountB;
                                if (std::find(beginA, endA, biB) != endA || std::find(beginB, endB, biA) != endB)
                                    continue;

                                AddManifold(coiA, coiB);
                            }
                        };

                        {
                            const std::uint32_t hashHighA = ownHashSmall.HashWorldCoordsHigh(worldCenterA);
                            const std::uint32_t hashLowA = ownHashSmall.HashWorldCoordsLow(worldCenterA);
                            CheckCell(hashHighA, ownHashSmall);
                            CheckCell(hashLowA, ownHashSmall);
                        }
                        {
                            const std::uint32_t hashHighA = ownHashLarge.HashWorldCoordsHigh(worldCenterA);
                            const std::uint32_t hashLowA = ownHashLarge.HashWorldCoordsLow(worldCenterA);
                            CheckCell(hashHighA, ownHashLarge);
                            CheckCell(hashLowA, ownHashLarge);
                        }
                    }
                },
                tbb::auto_partitioner()
            );
        }, [&] {
            std::vector<AABBPair> pairs;
            pairs.reserve(groups * 4u);
            std::vector<AABBPair> tempPairs;
            tempPairs.reserve(32);
            for (std::uint32_t g = 0; g != groups; ++g)
            {
                const std::uint32_t beginA = collidersGroup[g];
                const std::uint32_t endA = collidersGroup[g + 1];
                if (beginA >= endA)
                    continue;
                const std::uint32_t oi = colliders.objIdx[beginA];
                if (IsDisable(oi))
                    continue;
                tempPairs.clear();
                const AABB& objAABB = objectDatas.boundingAABB[oi];
                globalAABBTree.QueryPairs(oi, objAABB, tempPairs);
                if (tempPairs.empty())
                    continue;
                for (auto& p : tempPairs)
                {
                    if (IsDisable(oi))
                        continue;
                    p.beginA = beginA;
                    p.endA = endA;
                    pairs.push_back(std::move(p));
                }
            }
            const std::uint32_t pairsSize = pairs.size();
            tbb::parallel_for(
                tbb::blocked_range<std::uint32_t>(0, pairsSize),
                [&](const tbb::blocked_range<std::uint32_t>& pr) {
                    for (std::uint32_t p = pr.begin(); p != pr.end(); ++p)
                    {
                        const auto& pair = pairs[p];
                        const auto& anotherHashSmall = objectHashesSmall[pair.objIdxB];
                        const auto& anotherHashLarge = objectHashesLarge[pair.objIdxB];
                        if (anotherHashSmall.cell.empty() || anotherHashLarge.cell.empty())
                            continue;
                        const std::uint32_t beginA = pair.beginA;
                        const std::uint32_t endA = pair.endA;
                        if (beginA >= endA)
                            continue;

                        tbb::parallel_for(
                            tbb::blocked_range<std::uint32_t>(beginA, endA),
                            [&](const tbb::blocked_range<std::uint32_t>& cr) {
                                auto& checkedB = tls_checkedB.local();
                                for (std::uint32_t coiA = cr.begin(); coiA != cr.end(); ++coiA)
                                {
                                    const std::uint32_t biA = colliders.boneIdx[coiA];
                                    const Quaternion worldRotA = DirectX::XMQuaternionMultiply(physicsBones.dynamicAngularOffset[biA], physicsBones.predRot[biA]);
                                    const Vector rotateCenterA = DirectX::XMVector3Rotate(DirectX::XMVectorScale(colliders.boundingSphereCenter[coiA], physicsBones.orgWorldScale[biA]), worldRotA);
                                    const Vector worldCenterA = DirectX::XMVectorAdd(DirectX::XMVectorAdd(physicsBones.predPos[biA], rotateCenterA), physicsBones.dynamicLinearOffset[biA]);
                                    checkedB.clear();
                                    auto CheckCell = [&](const std::uint32_t hash, const Internal::LocalSpatialHash& anotherHash) {
                                        if (anotherHash.cell.empty())
                                            return;
                                        const std::uint32_t beginHash = anotherHash.cell[hash];
                                        const std::uint32_t endHash = anotherHash.cell[hash + 1];
                                        for (std::uint32_t ei = beginHash; ei < endHash; ++ei)
                                        {
                                            const std::uint32_t coiB = anotherHash.entries[ei];
                                            if (coiA >= coiB)
                                                continue;
                                            const std::uint32_t biB = colliders.boneIdx[coiB];
                                            if (!IsCollide(biB, biA) && !IsCollide(biA, biB))
                                                continue;
                                            if (physicsBones.invMass[biA] <= Epsilon && physicsBones.invMass[biB] <= Epsilon)
                                                continue;
                                            if (std::find(checkedB.begin(), checkedB.end(), coiB) != checkedB.end())
                                                continue;
                                            checkedB.push_back(coiB);
                                            AddManifold(coiA, coiB);
                                        }
                                    };

                                    {
                                        const std::uint32_t hashHighA = anotherHashSmall.HashWorldCoordsHigh(worldCenterA);
                                        const std::uint32_t hashLowA = anotherHashSmall.HashWorldCoordsLow(worldCenterA);
                                        CheckCell(hashHighA, anotherHashSmall);
                                        CheckCell(hashLowA, anotherHashSmall);
                                    }
                                    {
                                        const std::uint32_t hashHighA = anotherHashLarge.HashWorldCoordsHigh(worldCenterA);
                                        const std::uint32_t hashLowA = anotherHashLarge.HashWorldCoordsLow(worldCenterA);
                                        CheckCell(hashHighA, anotherHashLarge);
                                        CheckCell(hashLowA, anotherHashLarge);
                                    }
                                }
                            },
                            tbb::auto_partitioner()
                        );
                    }
                },
                tbb::auto_partitioner()
            );
        });

        if (DEBUG_MODE)
        {
            timeProfiler.End([this](const std::string& name, const double ms) {
                this->LoggingTimeProfiler(name, ms);
            });

            {
                colCandidatesStackCount++;
                if (colCandidatesStackCount >= 1000)
                {
                    logger::info("total collide candidate count {}", static_cast<std::uint32_t>(std::floor(totalColCandidates * 0.001f)));
                    totalColCandidates = 0;
                    colCandidatesStackCount = 0;
                }

                static double totalValidCollisions = 0;
                static std::uint32_t colSolveCount = 0;
                totalValidCollisions += std::min(manifoldCacheCount, expectedCollisionCount);
                colSolveCount++;
                if (colSolveCount >= 1000)
                {
                    logger::info("total actual collide count {}", static_cast<std::uint32_t>(std::floor(totalValidCollisions * 0.001f)));
                    totalValidCollisions = 0;
                    colSolveCount = 0;
                }
            }
        }
    }

    void XPBDWorld::GenerateGroundCache(const float stepCount)
    {
        if (collidersLeafs.empty() || GROUND_DETECT_RANGE <= Epsilon)
            return;

        static Internal::TimeProfiler timeProfiler(__func__);
        if (DEBUG_MODE)
            timeProfiler.Start();

        class GroundHitCollector : public RE::hkpRayHitCollector
        {
        public:
            void AddRayHit(const RE::hkpCdBody& a_body, const RE::hkpShapeRayCastCollectorOutput& a_hitInfo) override
            {
                if (rayHit.hitFraction <= a_hitInfo.hitFraction)
                    return;
                const RE::hkpCdBody* hkpCdBody = &a_body;
                while (hkpCdBody->parent != nullptr)
                {
                    hkpCdBody = hkpCdBody->parent;
                }
                const RE::hkpCollidable* collidable = static_cast<const RE::hkpCollidable*>(hkpCdBody);
                const RE::COL_LAYER layer = collidable->GetCollisionLayer();

                if (layer != RE::COL_LAYER::kStatic &&
                    layer != RE::COL_LAYER::kGround &&
                    layer != RE::COL_LAYER::kTerrain)
                {
                    return;
                }
                rayHit.rootCollidable = collidable;
                rayHit.hitFraction = a_hitInfo.hitFraction;
                rayHit.normal = a_hitInfo.normal;
                earlyOutHitFraction = a_hitInfo.hitFraction;
            }
            RE::hkpWorldRayCastOutput rayHit;
        };

        std::uint32_t quality = static_cast<std::uint32_t>(std::round(GROUND_DETECT_QUALITY * reciprocal(stepCount)));
        const std::uint32_t totalLeafs = collidersLeafs.size();
        quality = std::max(1u, std::min(quality, totalLeafs));
        const std::uint32_t chunkSize = (totalLeafs + quality - 1u) * reciprocal(quality);
        const std::uint32_t begin = (currentFrame % quality) * chunkSize;
        const std::uint32_t end = std::min(begin + chunkSize, totalLeafs);
        if (begin >= totalLeafs)
            return;
        for (std::uint32_t i = begin; i < end; ++i)
        {
            const std::uint32_t ci = collidersLeafs[i];
            const std::uint32_t oi = colliders.objIdx[ci];
            if (IsDisable(oi))
                continue;
            RE::bhkWorld* bhkWorld = objectDatas.bhkWorld[oi];
            if (!bhkWorld)
                continue;

            const std::uint32_t bi = colliders.boneIdx[ci];
            if (bi == UINT32_MAX)
                continue;
            if (physicsBones.groundCache[bi].isCast)
                continue;

            const AABB& localAABB = colliders.boundingAABB[ci];
            const Quaternion worldRot = DirectX::XMQuaternionMultiply(physicsBones.dynamicAngularOffset[bi], physicsBones.predRot[bi]);
            const Vector worldPos = DirectX::XMVectorAdd(physicsBones.predPos[bi], physicsBones.dynamicLinearOffset[bi]);
            AABB worldAABB = colliders.boundingAABB[ci].GetWorldAABB(worldPos, worldRot, physicsBones.orgWorldScale[bi]);
            worldAABB.Fatten(physicsBones.collisionMargin[bi]);
            const Vector worldBottomCenter = DirectX::XMVectorSetZ(worldAABB.GetCenter(), DirectX::XMVectorGetZ(worldAABB.min));

            const Vector from = DirectX::XMVectorAdd(worldBottomCenter, groundRayFrom);
            const Vector havokFrom = DirectX::XMVectorMultiply(from, vSkyrimWorldScale);
            const Vector to = DirectX::XMVectorAdd(worldBottomCenter, groundRayTo);
            const Vector havokTo = DirectX::XMVectorMultiply(to, vSkyrimWorldScale);

            RE::bhkPickData pickData;
            pickData.rayInput.from = havokFrom;
            pickData.rayInput.to = havokTo;

            GroundHitCollector hitCollector;
            hitCollector.Reset();
            pickData.rayHitCollectorA8 = reinterpret_cast<RE::hkpClosestRayHitCollector*>(&hitCollector);
            bhkWorld->PickObject(pickData);
            physicsBones.groundCache[bi].isCast = true;
            if (hitCollector.rayHit.HasHit())
            {
                physicsBones.groundCache[bi].hasHit = true;
                const float fromZ = DirectX::XMVectorGetZ(from);
                const float toZ = DirectX::XMVectorGetZ(to);
                physicsBones.groundCache[bi].height = fromZ + (toZ - fromZ) * hitCollector.rayHit.hitFraction;
                physicsBones.groundCache[bi].normal = DirectX::XMVectorSetW(hitCollector.rayHit.normal.quad, 0.0f);
            }
            std::uint32_t pbi = physicsBones.parentBoneIdx[bi];
            while (pbi != UINT32_MAX)
            {
                tbb::spin_mutex::scoped_lock sl(physicsBonesLock[pbi]);
                if (physicsBones.groundCache[pbi].isCast)
                {
                    if (physicsBones.groundCache[pbi].hasHit && physicsBones.groundCache[pbi].height >= physicsBones.groundCache[bi].height)
                        break;
                }
                physicsBones.groundCache[pbi] = physicsBones.groundCache[bi];
                pbi = physicsBones.parentBoneIdx[pbi];
            }
        }

        if (DEBUG_MODE)
        {
            timeProfiler.End([this](const std::string& name, const double ms) {
                this->LoggingTimeProfiler(name, ms);
            });
        }
    }

    void XPBDWorld::SolveCachedCollisions(const float deltaTime)
    {
        const std::uint32_t validCollisions = std::min(manifoldCacheCount, expectedCollisionCount);
        if (validCollisions == 0)
            return;

        static Internal::TimeProfiler timeProfiler(__func__);
        if (DEBUG_MODE)
            timeProfiler.Start();

        const float InvDtSq = reciprocal(deltaTime * deltaTime);
        auto SolveManifold = [&](const std::uint32_t coiA, const std::uint32_t coiB, const Internal::ContactManifold& manifold) {
            if (manifold.pointCount == 0)
                return;
            const std::uint32_t biA = colliders.boneIdx[coiA];
            const std::uint32_t biB = colliders.boneIdx[coiB];
            const bool isACollideWithB = IsCollide(biB, biA);
            const bool isBCollideWithA = IsCollide(biA, biB);
            if (!isACollideWithB && !isBCollideWithA)
                return;

            const float biasA = isACollideWithB ? 1.0f : 0.0f;
            const float biasB = isBCollideWithA ? 1.0f : 0.0f;
            const float wA = physicsBones.invMass[biA] * biasA;
            const float wB = physicsBones.invMass[biB] * biasB;
            const Vector sumRotInertiaA = DirectX::XMVectorAdd(physicsBones.invAngularInertiaPositive[biA], physicsBones.invAngularInertiaNegative[biA]);
            const float proxyInvInertiaA = DirectX::XMVectorGetX(DirectX::XMVector3LengthSq(sumRotInertiaA)) * biasA;
            const Vector sumRotInertiaB = DirectX::XMVectorAdd(physicsBones.invAngularInertiaPositive[biB], physicsBones.invAngularInertiaNegative[biB]);
            const float proxyInvInertiaB = DirectX::XMVectorGetX(DirectX::XMVector3LengthSq(sumRotInertiaB)) * biasB;
            if (wA + wB <= Epsilon && proxyInvInertiaA + proxyInvInertiaB <= AngularEpsilonSq)
                return;

            tbb::spin_mutex::scoped_lock lock1, lock2;
            if (biA < biB)
            {
                if (Epsilon < wA || AngularEpsilonSq < proxyInvInertiaA)
                    lock1.acquire(physicsBonesLock[biA]);
                if (Epsilon < wB || AngularEpsilonSq < proxyInvInertiaB)
                    lock2.acquire(physicsBonesLock[biB]);
            }
            else
            {
                if (Epsilon < wB || AngularEpsilonSq < proxyInvInertiaB)
                    lock1.acquire(physicsBonesLock[biB]);
                if (Epsilon < wA || AngularEpsilonSq < proxyInvInertiaA)
                    lock2.acquire(physicsBonesLock[biA]);
            }

            float rotConfidence = 1.0f;
            if (manifold.pointCount == 1u)
                rotConfidence = 0.1f;
            else if (manifold.pointCount >= 4u)
                rotConfidence = 1.0f;
            else
            {
                const float ratio = static_cast<float>(manifold.pointCount) * 0.25f;
                rotConfidence = ratio * ratio;
            }

            const Vector& normal = manifold.normal;
            const float compliance = std::max(physicsBones.collisionCompliance[biA], physicsBones.collisionCompliance[biB]);
            const float alphaProxy = compliance * InvDtSq;

            for (std::uint32_t i = 0; i < manifold.pointCount; ++i)
            {
                const auto& cp = manifold.points[i];
                const Vector pA = DirectX::XMVectorAdd(physicsBones.predPos[biA], physicsBones.dynamicLinearOffset[biA]);
                const Quaternion qA = DirectX::XMQuaternionMultiply(physicsBones.dynamicAngularOffset[biA], physicsBones.predRot[biA]);
                const Vector rA = DirectX::XMVector3Rotate(DirectX::XMVectorScale(cp.localPointA, physicsBones.orgWorldScale[biA]), qA);
                const Vector wPtA = DirectX::XMVectorAdd(pA, rA);

                const Vector pB = DirectX::XMVectorAdd(physicsBones.predPos[biB], physicsBones.dynamicLinearOffset[biB]);
                const Quaternion qB = DirectX::XMQuaternionMultiply(physicsBones.dynamicAngularOffset[biB], physicsBones.predRot[biB]);
                const Vector rB = DirectX::XMVector3Rotate(DirectX::XMVectorScale(cp.localPointB, physicsBones.orgWorldScale[biB]), qB);
                const Vector wPtB = DirectX::XMVectorAdd(pB, rB);

                const Vector penetration = DirectX::XMVectorSubtract(wPtA, wPtB);
                const float currentDepth = -DirectX::XMVectorGetX(DirectX::XMVector3Dot(penetration, normal));
                if (currentDepth <= Epsilon)
                    continue;

                const Vector rAxN = DirectX::XMVector3Cross(rA, normal);
                const Vector rBxN = DirectX::XMVector3Cross(rB, normal);
                float wRotA = 0.0f;
                Vector worldInvTauA = vZero;
                if (AngularEpsilonSq < proxyInvInertiaA)
                {
                    const Quaternion invRotA = DirectX::XMQuaternionConjugate(qA);
                    const Vector localRAxN = DirectX::XMVector3Rotate(rAxN, invRotA);
                    const Vector isPositiveA = DirectX::XMVectorGreaterOrEqual(localRAxN, vZero);
                    const Vector invInertiaA = DirectX::XMVectorSelect(physicsBones.invAngularInertiaNegative[biA], physicsBones.invAngularInertiaPositive[biA], isPositiveA);
                    const Vector localInertiaA = DirectX::XMVectorMultiply(localRAxN, invInertiaA);
                    worldInvTauA = DirectX::XMVector3Rotate(localInertiaA, qA);
                    wRotA = DirectX::XMVectorGetX(DirectX::XMVector3Dot(rAxN, worldInvTauA)) * biasA;

                    const float maxRotWeightA = wA * COL_ROT_WEIGHT_MULT;
                    if (wRotA > maxRotWeightA && wA > Epsilon)
                    {
                        const float scale = maxRotWeightA * reciprocal(wRotA);
                        wRotA = maxRotWeightA;
                        worldInvTauA = DirectX::XMVectorScale(worldInvTauA, scale);
                    }
                }

                float wRotB = 0.0f;
                Vector worldInvTauB = vZero;
                if (AngularEpsilonSq < proxyInvInertiaB)
                {
                    const Quaternion invRotB = DirectX::XMQuaternionConjugate(qB);
                    const Vector localRBxN = DirectX::XMVector3Rotate(rBxN, invRotB);
                    const Vector isPositiveB = DirectX::XMVectorGreaterOrEqual(localRBxN, vZero);
                    const Vector invInertiaB = DirectX::XMVectorSelect( physicsBones.invAngularInertiaNegative[biB], physicsBones.invAngularInertiaPositive[biB], isPositiveB);
                    const Vector localInertiaB = DirectX::XMVectorMultiply(localRBxN, invInertiaB);
                    worldInvTauB = DirectX::XMVector3Rotate(localInertiaB, qB);
                    wRotB = DirectX::XMVectorGetX(DirectX::XMVector3Dot(rBxN, worldInvTauB)) * biasB;

                    const float maxRotWeightB = wB * COL_ROT_WEIGHT_MULT;
                    if (wRotB > maxRotWeightB && wB > Epsilon)
                    {
                        const float scale = maxRotWeightB * reciprocal(wRotB);
                        wRotB = maxRotWeightB;
                        worldInvTauB = DirectX::XMVectorScale(worldInvTauB, scale);
                    }
                }

                const float wNormSum = wA + wB + wRotA + wRotB;
                if (wNormSum <= Epsilon)
                    continue;

                const float lambdaN = currentDepth * reciprocal(wNormSum + alphaProxy);
                const float relaxedLambda = lambdaN * COL_CONVERGENCE;
                const Vector pCorrN = DirectX::XMVectorScale(normal, relaxedLambda);

                if (Epsilon < wA)
                    physicsBones.predPos[biA] = DirectX::XMVectorAdd(physicsBones.predPos[biA], DirectX::XMVectorScale(pCorrN, wA));
                if (AngularEpsilonSq < proxyInvInertiaA)
                {
                    const Vector dThetaA = DirectX::XMVectorScale(worldInvTauA, relaxedLambda * rotConfidence * biasA);
                    const Vector thetaSqA = DirectX::XMVector3LengthSq(dThetaA);
                    if (DirectX::XMVector3Greater(thetaSqA, vAngularEpsilonSq))
                    {
                        const Vector angleA = DirectX::XMVectorSqrt(thetaSqA);
                        const Vector axisA = DirectX::XMVectorMultiply(dThetaA, DirectX::XMVectorReciprocal(angleA));
                        const Quaternion qCorrA = DirectX::XMQuaternionRotationNormal(axisA, DirectX::XMVectorGetX(angleA));
                        Quaternion newRotA = DirectX::XMQuaternionMultiply(physicsBones.predRot[biA], qCorrA);
                        if (DirectX::XMVector3Less(DirectX::XMQuaternionDot(physicsBones.predRot[biA], newRotA), vZero))
                            newRotA = DirectX::XMVectorNegate(newRotA);
                        physicsBones.predRot[biA] = DirectX::XMQuaternionNormalize(newRotA);
                    }
                }

                if (Epsilon < wB)
                    physicsBones.predPos[biB] = DirectX::XMVectorSubtract(physicsBones.predPos[biB], DirectX::XMVectorScale(pCorrN, wB));
                if (AngularEpsilonSq < proxyInvInertiaB)
                {
                    const Vector dThetaB = DirectX::XMVectorScale(worldInvTauB, -relaxedLambda * rotConfidence * biasB);
                    const Vector thetaSqB = DirectX::XMVector3LengthSq(dThetaB);
                    if (DirectX::XMVector3Greater(thetaSqB, vAngularEpsilonSq))
                    {
                        const Vector angleB = DirectX::XMVectorSqrt(thetaSqB);
                        const Vector axisB = DirectX::XMVectorMultiply(dThetaB, DirectX::XMVectorReciprocal(angleB));
                        const Quaternion qCorrB = DirectX::XMQuaternionRotationNormal(axisB, DirectX::XMVectorGetX(angleB));
                        Quaternion newRotB = DirectX::XMQuaternionMultiply(physicsBones.predRot[biB], qCorrB);
                        if (DirectX::XMVector3Less(DirectX::XMQuaternionDot(physicsBones.predRot[biB], newRotB), vZero))
                            newRotB = DirectX::XMVectorNegate(newRotB);
                        physicsBones.predRot[biB] = DirectX::XMQuaternionNormalize(newRotB);
                    }
                }

                const float actualPush = relaxedLambda * wNormSum;
                const float squishDepth = currentDepth - actualPush;
                if (Epsilon < wA || AngularEpsilonSq < proxyInvInertiaA)
                {
                    physicsBones.frictionCache[biA].normal = DirectX::XMVectorAdd(physicsBones.frictionCache[biA].normal, normal);
                    physicsBones.frictionCache[biA].depth = std::max(physicsBones.frictionCache[biA].depth, currentDepth);

                    if (squishDepth > physicsBones.deformCache[biA].depth)
                    {
                        physicsBones.deformCache[biA].normal = normal;
                        physicsBones.deformCache[biA].depth = squishDepth;
                    }
                }
                if (Epsilon < wB || AngularEpsilonSq < proxyInvInertiaB)
                {
                    physicsBones.frictionCache[biB].normal = DirectX::XMVectorAdd(physicsBones.frictionCache[biB].normal, DirectX::XMVectorNegate(normal));
                    physicsBones.frictionCache[biB].depth = std::max(physicsBones.frictionCache[biB].depth, currentDepth);

                    if (squishDepth > physicsBones.deformCache[biB].depth)
                    {
                        physicsBones.deformCache[biB].normal = DirectX::XMVectorNegate(normal);
                        physicsBones.deformCache[biB].depth = squishDepth;
                    }
                }
            }
        };

        tbb::parallel_for(
            tbb::blocked_range<std::uint32_t>(0, validCollisions),
            [&](const tbb::blocked_range<std::uint32_t>& r) {
                for (std::uint32_t i = r.begin(); i != r.end(); ++i)
                {
                    const auto& cached = manifoldCache[i];
                    SolveManifold(cached.coiA, cached.coiB, cached.manifold);
                }
            },
            tbb::auto_partitioner()
        );

        if (DEBUG_MODE)
        {
            timeProfiler.End([this](const std::string& name, const double ms) {
                this->LoggingTimeProfiler(name, ms);
            });
        }
    }

    void XPBDWorld::SolveCachedGroundCollisions(const float deltaTime)
    {
        if (collidersLeafs.empty() || GROUND_DETECT_RANGE <= Epsilon)
            return;

        static Internal::TimeProfiler timeProfiler(__func__);
        if (DEBUG_MODE)
            timeProfiler.Start();

        const float invDtSq = reciprocal(deltaTime * deltaTime);
        tbb::parallel_for(
            tbb::blocked_range<std::uint32_t>(0, colliders.numColliders, 32),
            [&](const tbb::blocked_range<std::uint32_t>& r) {
                for (std::uint32_t ci = r.begin(); ci != r.end(); ++ci)
                {
                    const std::uint32_t oi = colliders.objIdx[ci];
                    if (IsDisable(oi))
                        continue;
                    const std::uint32_t bi = colliders.boneIdx[ci];
                    if (bi == UINT32_MAX || !IsCollideGround(bi))
                        continue;
                    if (!physicsBones.groundCache[bi].hasHit)
                        continue;
                    const float w = physicsBones.invMass[bi];
                    if (w <= Epsilon)
                        continue;

                    const Vector pos = DirectX::XMVectorAdd(physicsBones.predPos[bi], physicsBones.dynamicLinearOffset[bi]);

                    const float groundHeight = physicsBones.groundCache[bi].height;
                    const Vector& normal = physicsBones.groundCache[bi].normal;
                    const float scale = physicsBones.orgWorldScale[bi];
                    const Quaternion rot = DirectX::XMQuaternionMultiply(physicsBones.dynamicAngularOffset[bi], physicsBones.predRot[bi]);
                    const float margin = physicsBones.collisionMargin[bi];

                    const AABB& localAABB = colliders.boundingAABB[ci];
                    const Vector localExtents = localAABB.GetExtents(scale);
                    
                    AABB worldAABB = localAABB.GetWorldAABB(pos, rot, scale);
                    worldAABB.Fatten(margin);
                    const Vector worldCenter = worldAABB.GetCenter();

                    const Quaternion invRot = DirectX::XMQuaternionConjugate(rot);
                    const Vector localNormal = DirectX::XMVector3Rotate(normal, invRot);
                    const Vector absLocalNormal = DirectX::XMVectorAbs(localNormal);
                    const float projRadius = DirectX::XMVectorGetX(DirectX::XMVector3Dot(localExtents, absLocalNormal)) + margin;

                    const Vector pointOnPlane = DirectX::XMVectorSetZ(pos, groundHeight);
                    const float distanceToPlane = DirectX::XMVectorGetX(DirectX::XMVector3Dot(normal, DirectX::XMVectorSubtract(worldCenter, pointOnPlane)));
                    const float currentDepth = projRadius - distanceToPlane;
                    if (currentDepth <= Epsilon)
                        continue;

                    const Vector contactPoint = DirectX::XMVectorSubtract(worldCenter, DirectX::XMVectorScale(normal, projRadius));
                    const Vector rVector = DirectX::XMVectorSubtract(contactPoint, pos);

                    const Vector rXN = DirectX::XMVector3Cross(rVector, normal);
                    const Vector localRxN = DirectX::XMVector3Rotate(rXN, invRot);
                    const Vector isPositive = DirectX::XMVectorGreaterOrEqual(localRxN, vZero);
                    const Vector invInertia = DirectX::XMVectorSelect(physicsBones.invAngularInertiaNegative[bi], physicsBones.invAngularInertiaPositive[bi], isPositive);
                    const Vector localInertia = DirectX::XMVectorMultiply(localRxN, invInertia);
                    Vector worldInvTau = DirectX::XMVector3Rotate(localInertia, rot);
                    float wRot = DirectX::XMVectorGetX(DirectX::XMVector3Dot(rXN, worldInvTau));
                    const float maxRotWeight = w * COL_ROT_WEIGHT_MULT;
                    if (wRot > maxRotWeight && w > Epsilon)
                    {
                        const float scale = maxRotWeight * reciprocal(wRot);
                        wRot = maxRotWeight;
                        worldInvTau = DirectX::XMVectorScale(worldInvTau, scale);
                    }
                    const float wSum = w + wRot;

                    const float alphaProxy = physicsBones.collisionCompliance[bi] * invDtSq;
                    const float lambda = currentDepth * reciprocal(wSum + alphaProxy);
                    const float relaxedLambda = lambda * COL_CONVERGENCE;
                    const Vector correction = DirectX::XMVectorScale(normal, relaxedLambda * w);
                    physicsBones.predPos[bi] = DirectX::XMVectorAdd(physicsBones.predPos[bi], correction);

                    if (AngularEpsilon < wRot)
                    {
                        const Vector dTheta = DirectX::XMVectorScale(worldInvTau, relaxedLambda);
                        const Vector thetaSq = DirectX::XMVector3LengthSq(dTheta);
                        if (DirectX::XMVector3Greater(thetaSq, vAngularEpsilonSq))
                        {
                            const Vector angle = DirectX::XMVectorSqrt(thetaSq);
                            const Vector axis = DirectX::XMVectorMultiply(dTheta, DirectX::XMVectorReciprocal(angle));
                            const Quaternion qCorr = DirectX::XMQuaternionRotationNormal(axis, DirectX::XMVectorGetX(angle));
                            Quaternion newRot = DirectX::XMQuaternionMultiply(physicsBones.predRot[bi], qCorr);
                            if (DirectX::XMVector3Less(DirectX::XMQuaternionDot(physicsBones.predRot[bi], newRot), vZero))
                                newRot = DirectX::XMVectorNegate(newRot);
                            physicsBones.predRot[bi] = DirectX::XMQuaternionNormalize(newRot);
                        }
                    }

                    physicsBones.frictionCache[bi].normal = DirectX::XMVectorAdd(physicsBones.frictionCache[bi].normal, DirectX::XMVectorNegate(normal));
                    physicsBones.frictionCache[bi].depth = std::max(physicsBones.frictionCache[bi].depth, currentDepth);
                }
            },
            tbb::auto_partitioner()
        );

        if (DEBUG_MODE)
        {
            timeProfiler.End([this](const std::string& name, const double ms) {
                this->LoggingTimeProfiler(name, ms);
            });
        }
    }

    void XPBDWorld::SolveDistanceConstraints(const float deltaTime, const float invDeltaTimeSq, const bool initLambda)
    {
        if (distanceConstraintsGroup.empty() || distanceConstraintsColorGroup.empty())
            return;

        static Internal::TimeProfiler timeProfiler(__func__);
        if (DEBUG_MODE)
            timeProfiler.Start();

        const float invDeltaTime = reciprocal(deltaTime);
        const std::uint32_t groups = distanceConstraintsGroup.size() - 1ull;
        tbb::parallel_for(
            tbb::blocked_range<std::uint32_t>(0, groups),
            [&](const tbb::blocked_range<std::uint32_t>& r) {
                for (std::uint32_t g = r.begin(); g != r.end(); ++g)
                {
                    const std::uint32_t begin = distanceConstraintsGroup[g];
                    const std::uint32_t end = distanceConstraintsGroup[g + 1];
                    if (begin >= end)
                        continue;
                    const std::uint32_t oi = distanceConstraints.objIdx[begin];
                    if (IsDisable(oi))
                        continue;

                    auto ccgIt = std::lower_bound(distanceConstraintsColorGroup.begin(), distanceConstraintsColorGroup.end(), begin);
                    std::uint32_t ccgi = std::distance(distanceConstraintsColorGroup.begin(), ccgIt);
                    const std::uint32_t colorGroups = distanceConstraintsColorGroup.size() - 1ull;
                    for (; ccgi < colorGroups && distanceConstraintsColorGroup[ccgi] < end; ++ccgi)
                    {
                        std::uint32_t c_begin = distanceConstraintsColorGroup[ccgi];
                        std::uint32_t c_end = std::min(static_cast<std::uint32_t>(distanceConstraintsColorGroup[ccgi + 1]), end);

                        for (std::uint32_t ci = c_begin; ci < c_end; ++ci)
                        {
                            const std::uint32_t bi = distanceConstraints.boneIdx[ci];
                            const std::uint32_t abi = distanceConstraints.anchIdx[ci];
                            if (bi == UINT32_MAX || abi == UINT32_MAX)
                                continue;
                            const float& invMass = physicsBones.invMass[bi];
                            const float& invMassA = physicsBones.invMass[abi];
                            if (invMass <= Epsilon && invMassA <= Epsilon)
                                continue;
                            const float wSum = invMass + invMassA;
                            if (wSum <= Epsilon)
                                continue;

                            const Vector dir = DirectX::XMVectorSubtract(physicsBones.predPos[bi], physicsBones.predPos[abi]);
                            const float distSq = DirectX::XMVectorGetX(DirectX::XMVector3LengthSq(dir));
                            if (distSq < EpsilonSq)
                                continue;

                            const float invDist = rsqrt(distSq);
                            const Vector normal = DirectX::XMVectorScale(dir, invDist);
                            const float dist = distSq * invDist;

                            const float C = dist - distanceConstraints.restLen[ci];
                            const float C_abs = std::abs(C);
                            const bool isLessThanZero = C < 0.0f;
                            const float currentComp = isLessThanZero ? distanceConstraints.complianceSquish[ci] : distanceConstraints.complianceStretch[ci];
                            const float currentMargin = isLessThanZero ? distanceConstraints.squishMargin[ci] : distanceConstraints.stretchMargin[ci];
                            float currentLimit = isLessThanZero ? distanceConstraints.squishLimit[ci] : distanceConstraints.stretchLimit[ci];
                            const float sign = isLessThanZero ? -1.0f : 1.0f;
                            currentLimit = std::max(currentLimit, currentMargin);
                            if (C_abs <= currentMargin)
                                continue;

                            float currentFactor = 1.0f;
                            if (Epsilon < currentMargin)
                            {
                                const float ratio = std::min(C_abs * reciprocal(currentMargin), 1.0f);
                                const float ratioCubic = ratio * ratio * ratio;
                                currentFactor = std::max(1.0f - ratioCubic, Epsilon);
                            }

                            const Vector deltaBi = DirectX::XMVectorSubtract(physicsBones.predPos[bi], physicsBones.pos[bi]);
                            const Vector deltaAbi = DirectX::XMVectorSubtract(physicsBones.predPos[abi], physicsBones.pos[abi]);
                            const Vector relDelta = DirectX::XMVectorSubtract(deltaBi, deltaAbi);
                            const float projDelta = DirectX::XMVectorGetX(DirectX::XMVector3Dot(relDelta, normal));
                            const float currentDamping = isLessThanZero ? distanceConstraints.squishDamping[ci] : distanceConstraints.stretchDamping[ci];

                            const float alphaProxy = (currentComp * currentFactor) * invDeltaTimeSq;
                            const float betaProxy = currentDamping;
                            const float denomSoft = wSum + alphaProxy + betaProxy;
                            const float denomHard = wSum;
                            float deltaLambda = 0.0f;
                            if (initLambda)
                                distanceConstraints.lambda[ci] = 0.0f;

                            if (C_abs > currentLimit)
                            {
                                const float C_hard = C - (sign * currentLimit);
                                const float C_soft_max = (sign * currentLimit) - (sign * currentMargin);
                                const float dLambda_hard = -C_hard * reciprocal(denomHard);
                                const float dLambda_soft = (-C_soft_max - (alphaProxy * distanceConstraints.lambda[ci]) - (betaProxy * projDelta)) * reciprocal(denomSoft);

                                deltaLambda = dLambda_hard + dLambda_soft;
                                distanceConstraints.lambda[ci] += dLambda_soft;
                            }
                            else
                            {
                                const float C_soft = C - (sign * currentMargin);
                                const float dLambda_soft = (-C_soft - (alphaProxy * distanceConstraints.lambda[ci]) - (betaProxy * projDelta)) * reciprocal(denomSoft);

                                deltaLambda = dLambda_soft;
                                distanceConstraints.lambda[ci] += dLambda_soft;
                            }

                            if (Epsilon < invMass)
                            {
                                const float correctionMagBi = deltaLambda * invMass;
                                physicsBones.predPos[bi] = DirectX::XMVectorAdd(physicsBones.predPos[bi], DirectX::XMVectorScale(normal, correctionMagBi));
                            }
                            if (Epsilon < invMassA)
                            {
                                const float correctionMagAbi = deltaLambda * invMassA;
                                physicsBones.predPos[abi] = DirectX::XMVectorSubtract(physicsBones.predPos[abi], DirectX::XMVectorScale(normal, correctionMagAbi));
                            }
                        }
                    }
                }
            },
            tbb::auto_partitioner()
        );

        if (DEBUG_MODE)
        {
            timeProfiler.End([this](const std::string& name, const double ms) {
                this->LoggingTimeProfiler(name, ms);
            });
        }
    }

    void XPBDWorld::SolveAngularConstraints(const float deltaTime, const float invDeltaTimeSq, const bool initLambda)
    {
        if (angularConstraintsGroup.empty() || angularConstraintsColorGroup.empty())
            return;

        static Internal::TimeProfiler timeProfiler(__func__);
        if (DEBUG_MODE)
            timeProfiler.Start();

        const std::uint32_t groups = angularConstraintsGroup.size() - 1ull;
        tbb::parallel_for(
            tbb::blocked_range<std::uint32_t>(0, groups),
            [&](const tbb::blocked_range<std::uint32_t>& r) {
                for (std::uint32_t g = r.begin(); g != r.end(); ++g)
                {
                    const std::uint32_t begin = angularConstraintsGroup[g];
                    const std::uint32_t end = angularConstraintsGroup[g + 1];
                    if (begin >= end)
                        continue;
                    const std::uint32_t oi = angularConstraints.objIdx[begin];
                    if (IsDisable(oi))
                        continue;

                    auto cgIt = std::lower_bound(angularConstraintsColorGroup.begin(), angularConstraintsColorGroup.end(), begin);
                    std::uint32_t ccgi = std::distance(angularConstraintsColorGroup.begin(), cgIt);
                    const std::uint32_t colorGroups = angularConstraintsColorGroup.size() - 1ull;
                    for (; ccgi < colorGroups && angularConstraintsColorGroup[ccgi] < end; ++ccgi)
                    {
                        std::uint32_t c_begin = angularConstraintsColorGroup[ccgi];
                        std::uint32_t c_end = std::min(static_cast<std::uint32_t>(angularConstraintsColorGroup[ccgi + 1]), end);

                        for (std::uint32_t ci = c_begin; ci < c_end; ++ci)
                        {
                            const std::uint32_t bi = angularConstraints.boneIdx[ci];
                            const std::uint32_t abi = angularConstraints.anchIdx[ci];
                            if (bi == UINT32_MAX || abi == UINT32_MAX)
                                continue;
                            const float invMass = physicsBones.invMass[bi];
                            const float invMassA = physicsBones.invMass[abi];
                            if (invMass <= Epsilon && invMassA <= Epsilon)
                                continue;
                            const Vector sumRotInertia = DirectX::XMVectorAdd(physicsBones.invAngularInertiaPositive[bi], physicsBones.invAngularInertiaNegative[bi]);
                            const Vector sumRotInertiaA = DirectX::XMVectorAdd(physicsBones.invAngularInertiaPositive[abi], physicsBones.invAngularInertiaNegative[abi]);
                            const float proxyInvInertia = DirectX::XMVectorGetX(DirectX::XMVector3LengthSq(sumRotInertia));
                            const float proxyInvInertiaA = DirectX::XMVectorGetX(DirectX::XMVector3LengthSq(sumRotInertiaA));
                            if (proxyInvInertia <= AngularEpsilonSq && proxyInvInertiaA <= AngularEpsilonSq)
                                continue;

                            const Quaternion target = DirectX::XMQuaternionMultiply(angularConstraints.restRot[ci], physicsBones.predRot[abi]);
                            const Quaternion targetInv = DirectX::XMQuaternionConjugate(target);
                            const Quaternion diff = DirectX::XMQuaternionMultiply(physicsBones.predRot[bi], targetInv);
                            const Quaternion& invAlign = physicsBones.invAlignRot[bi];
                            const Quaternion& align = physicsBones.alignRot[bi];
                            Quaternion diffAligned = DirectX::XMQuaternionMultiply(DirectX::XMQuaternionMultiply(align, diff), invAlign);
                            if (DirectX::XMVector3Less(DirectX::XMVectorSplatW(diffAligned), vZero))
                                diffAligned = DirectX::XMVectorNegate(diffAligned);
                            
                            const Vector qVecAligned = DirectX::XMVectorSetW(diffAligned, 0.0f);
                            const Vector currentAngleSq = DirectX::XMVector3LengthSq(qVecAligned);
                            if (DirectX::XMVector3LessOrEqual(currentAngleSq, vEpsilonSq))
                                continue;

                            const Vector vC = DirectX::XMVectorSqrt(currentAngleSq);
                            const float C = DirectX::XMVectorGetX(vC);

                            const Vector localDir = DirectX::XMVectorMultiply(qVecAligned, DirectX::XMVectorReciprocal(vC));
                            const Vector localDirAbs = DirectX::XMVectorAbs(localDir);
                            const Vector isPositive = DirectX::XMVectorGreaterOrEqual(qVecAligned, vZero);

                            const Vector marginPosRad = DirectX::XMVectorMultiply(angularConstraints.marginPositive[ci], vHalf);
                            const Vector marginNegRad = DirectX::XMVectorMultiply(angularConstraints.marginNegative[ci], vHalf);
                            const Vector marginQ = DirectX::XMVectorSin(DirectX::XMVectorSelect(marginNegRad, marginPosRad, isPositive));

                            const Vector limitPosRad = DirectX::XMVectorMultiply(angularConstraints.limitPositive[ci], vHalf);
                            const Vector limitNegRad = DirectX::XMVectorMultiply(angularConstraints.limitNegative[ci], vHalf);
                            const Vector limitQ = DirectX::XMVectorSin(DirectX::XMVectorSelect(limitNegRad, limitPosRad, isPositive));
                            
                            const Vector dirSq = DirectX::XMVectorMultiply(localDirAbs, localDirAbs);
                            const Vector limitMagQ = DirectX::XMVectorAbs(limitQ);
                            const Vector marginMagQ = DirectX::XMVectorAbs(marginQ);
                            const Vector marginRatioSq = DirectX::XMVectorMultiply(dirSq, DirectX::XMVectorReciprocal(DirectX::XMVectorMultiply(marginMagQ, marginMagQ)));
                            const float currentMargin = DirectX::XMVectorGetX(DirectX::XMVectorReciprocalSqrt(DirectX::XMVector3Dot(marginRatioSq, vOne)));

                            const Vector limitRatioSq = DirectX::XMVectorMultiply(dirSq, DirectX::XMVectorReciprocal(DirectX::XMVectorMultiply(limitMagQ, limitMagQ)));
                            float currentLimit = DirectX::XMVectorGetX(DirectX::XMVectorReciprocalSqrt(DirectX::XMVector3Dot(limitRatioSq, vOne)));
                            currentLimit = std::max(currentMargin, currentLimit);
                            if (C <= currentMargin)
                                continue;

                            const Vector worldDir = DirectX::XMVector3Rotate(DirectX::XMVector3Rotate(localDir, align), target);
                            float wRotA = 0.0f;
                            Vector worldInvTauA = vZero;
                            if (AngularEpsilonSq < proxyInvInertiaA)
                            {
                                const Quaternion invRotA = DirectX::XMQuaternionConjugate(physicsBones.predRot[abi]);
                                const Vector localTauA = DirectX::XMVector3Rotate(worldDir, invRotA);
                                const Vector isPosA = DirectX::XMVectorGreaterOrEqual(localTauA, vZero);
                                const Vector invInertiaA = DirectX::XMVectorSelect(physicsBones.invAngularInertiaNegative[abi], physicsBones.invAngularInertiaPositive[abi], isPosA);

                                const Vector localInvTauA = DirectX::XMVectorMultiply(localTauA, invInertiaA);
                                worldInvTauA = DirectX::XMVector3Rotate(localInvTauA, physicsBones.predRot[abi]);
                                wRotA = DirectX::XMVectorGetX(DirectX::XMVector3Dot(worldDir, worldInvTauA));
                            }

                            float wRot = 0.0f;
                            Vector worldInvTau = vZero;
                            if (AngularEpsilonSq < proxyInvInertia)
                            {
                                const Quaternion invRot = DirectX::XMQuaternionConjugate(physicsBones.predRot[bi]);
                                const Vector localTau = DirectX::XMVector3Rotate(worldDir, invRot);
                                const Vector isPos = DirectX::XMVectorGreaterOrEqual(localTau, vZero);
                                const Vector invInertia = DirectX::XMVectorSelect(physicsBones.invAngularInertiaNegative[bi], physicsBones.invAngularInertiaPositive[bi], isPos);

                                const Vector localInvTau = DirectX::XMVectorMultiply(localTau, invInertia);
                                worldInvTau = DirectX::XMVector3Rotate(localInvTau, physicsBones.predRot[bi]);
                                wRot = DirectX::XMVectorGetX(DirectX::XMVector3Dot(worldDir, worldInvTau));
                            }

                            const float wSum = wRotA + wRot;
                            if (wSum <= AngularEpsilon)
                                continue;

                            const Vector compVec = DirectX::XMVectorSelect(angularConstraints.complianceNegative[ci], angularConstraints.compliancePositive[ci], isPositive);
                            const float currentComp = DirectX::XMVectorGetX(DirectX::XMVector3Dot(compVec, localDirAbs));
                            float currentFactor = 1.0f;
                            if (Epsilon < currentMargin)
                            {
                                const float ratio = std::min(C * reciprocal(currentMargin), 1.0f);
                                const float ratioCubic = ratio * ratio * ratio;
                                currentFactor = std::max(1.0f - ratioCubic, Epsilon);
                            }

                            Quaternion dq = DirectX::XMQuaternionMultiply(DirectX::XMQuaternionConjugate(physicsBones.rot[bi]), physicsBones.predRot[bi]);
                            if (DirectX::XMVector3Less(DirectX::XMVectorSplatW(dq), vZero))
                                dq = DirectX::XMVectorNegate(dq);
                            const Vector angVel = DirectX::XMVectorSetW(DirectX::XMVectorMultiply(dq, vTwo), 0.0f);
                            Quaternion dqA = DirectX::XMQuaternionMultiply(DirectX::XMQuaternionConjugate(physicsBones.rot[abi]), physicsBones.predRot[abi]);
                            if (DirectX::XMVector3Less(DirectX::XMVectorSplatW(dqA), vZero))
                                dqA = DirectX::XMVectorNegate(dqA);
                            const Vector angVelA = DirectX::XMVectorSetW(DirectX::XMVectorMultiply(dqA, vTwo), 0.0f);
                            const Vector relAngVel = DirectX::XMVectorSubtract(angVel, angVelA);
                            const Vector dampVec = DirectX::XMVectorSelect(angularConstraints.dampingNegative[ci], angularConstraints.dampingPositive[ci], isPositive);
                            const float currentDamping = DirectX::XMVectorGetX(DirectX::XMVector3Dot(dampVec, localDirAbs));

                            const float relAngVelDot = DirectX::XMVectorGetX(DirectX::XMVector3Dot(relAngVel, worldDir));
                            const float alphaProxy = (currentComp * currentFactor) * invDeltaTimeSq;
                            const float betaProxy = currentDamping;
                            const float denomSoft = wSum + alphaProxy + betaProxy;
                            const float denomHard = wSum;
                            float deltaLambda = 0.0f;
                            if (initLambda)
                                angularConstraints.lambda[ci] = 0.0f;

                            if (C > currentLimit)
                            {
                                const float C_hard = C - currentLimit;
                                const float C_soft_max = currentLimit - currentMargin;
                                const float dLambda_hard = -C_hard * reciprocal(denomHard);
                                const float dLambda_soft = (-C_soft_max - (alphaProxy * angularConstraints.lambda[ci]) - (betaProxy * relAngVelDot)) * reciprocal(denomSoft);
                                deltaLambda = dLambda_hard + dLambda_soft;
                                angularConstraints.lambda[ci] += dLambda_soft;
                            }
                            else
                            {
                                const float C_soft = C - currentMargin;
                                const float dLambda_soft = (-C_soft - (alphaProxy * angularConstraints.lambda[ci]) - (betaProxy * relAngVelDot)) * reciprocal(denomSoft);
                                deltaLambda = dLambda_soft;
                                angularConstraints.lambda[ci] += dLambda_soft;
                            }

                            if (AngularEpsilonSq < proxyInvInertiaA)
                            {
                                const Vector dThetaA = DirectX::XMVectorScale(worldInvTauA, -deltaLambda);
                                const Vector thetaSqA = DirectX::XMVector3LengthSq(dThetaA);
                                if (DirectX::XMVector3Greater(thetaSqA, vAngularEpsilonSq))
                                {
                                    const Vector angleA = DirectX::XMVectorSqrt(thetaSqA);
                                    const Vector axisA = DirectX::XMVectorMultiply(dThetaA, DirectX::XMVectorReciprocal(angleA));
                                    const Quaternion qCorrA = DirectX::XMQuaternionRotationNormal(axisA, DirectX::XMVectorGetX(angleA));
                                    Quaternion newRotA = DirectX::XMQuaternionMultiply(physicsBones.predRot[abi], qCorrA);
                                    if (DirectX::XMVector3Less(DirectX::XMQuaternionDot(physicsBones.predRot[abi], newRotA), vZero))
                                        newRotA = DirectX::XMVectorNegate(newRotA);
                                    physicsBones.predRot[abi] = DirectX::XMQuaternionNormalize(newRotA);
                                }
                            }
                            if (AngularEpsilonSq < proxyInvInertia)
                            {
                                const Vector dTheta = DirectX::XMVectorScale(worldInvTau, deltaLambda);
                                const Vector thetaSq = DirectX::XMVector3LengthSq(dTheta);
                                if (DirectX::XMVector3Greater(thetaSq, vAngularEpsilonSq))
                                {
                                    const Vector angle = DirectX::XMVectorSqrt(thetaSq);
                                    const Vector axis = DirectX::XMVectorMultiply(dTheta, DirectX::XMVectorReciprocal(angle));
                                    const Quaternion qCorr = DirectX::XMQuaternionRotationNormal(axis, DirectX::XMVectorGetX(angle));
                                    Quaternion newRot = DirectX::XMQuaternionMultiply(physicsBones.predRot[bi], qCorr);
                                    if (DirectX::XMVector3Less(DirectX::XMQuaternionDot(physicsBones.predRot[bi], newRot), vZero))
                                        newRot = DirectX::XMVectorNegate(newRot);
                                    physicsBones.predRot[bi] = DirectX::XMQuaternionNormalize(newRot);
                                }
                            }
                        }
                    }
                }
            },
            tbb::auto_partitioner()
        );

        if (DEBUG_MODE)
        {
            timeProfiler.End([this](const std::string& name, const double ms) {
                this->LoggingTimeProfiler(name, ms);
            });
        }
    }

    void XPBDWorld::SolveConeConstraints(const float deltaTime, const float invDeltaTimeSq, const bool initLambda)
    {
        if (coneConstraintsGroup.empty() || coneConstraintsColorGroup.empty())
            return;

        static Internal::TimeProfiler timeProfiler(__func__);
        if (DEBUG_MODE)
            timeProfiler.Start();

        const std::uint32_t groups = coneConstraintsGroup.size() - 1ull;
        tbb::parallel_for(
            tbb::blocked_range<std::uint32_t>(0, groups),
            [&](const tbb::blocked_range<std::uint32_t>& r) {
                for (std::uint32_t g = r.begin(); g != r.end(); ++g)
                {
                    const std::uint32_t begin = coneConstraintsGroup[g];
                    const std::uint32_t end = coneConstraintsGroup[g + 1];
                    if (begin >= end)
                        continue;

                    const std::uint32_t oi = coneConstraints.objIdx[begin];
                    if (IsDisable(oi))
                        continue;

                    auto cgIt = std::lower_bound(coneConstraintsColorGroup.begin(), coneConstraintsColorGroup.end(), begin);
                    std::uint32_t ccgi = std::distance(coneConstraintsColorGroup.begin(), cgIt);
                    const std::uint32_t colorGroups = coneConstraintsColorGroup.size() - 1ull;
                    for (; ccgi < colorGroups && coneConstraintsColorGroup[ccgi] < end; ++ccgi)
                    {
                        std::uint32_t c_begin = coneConstraintsColorGroup[ccgi];
                        std::uint32_t c_end = std::min(static_cast<std::uint32_t>(coneConstraintsColorGroup[ccgi + 1]), end);

                        for (std::uint32_t ci = c_begin; ci < c_end; ++ci)
                        {
                            const std::uint32_t bi = coneConstraints.boneIdx[ci];
                            const std::uint32_t abi = coneConstraints.anchIdx[ci];
                            if (bi == UINT32_MAX || abi == UINT32_MAX)
                                continue;

                            const float wBi = physicsBones.invMass[bi];
                            const float wAbi = physicsBones.invMass[abi];
                            const float wSum = wBi + wAbi;
                            if (wSum <= Epsilon)
                                continue;

                            const Vector& pBi = physicsBones.predPos[bi];
                            const Vector& pAbi = physicsBones.predPos[abi];
                            const Vector d = DirectX::XMVectorSubtract(pBi, pAbi);

                            const Vector lSqVec = DirectX::XMVector3LengthSq(d);
                            if (DirectX::XMVector3Less(lSqVec, vEpsilonSq))
                                continue;

                            const float lSq = DirectX::XMVectorGetX(lSqVec);

                            const Vector invLVec = DirectX::XMVectorReciprocalSqrt(lSqVec);
                            const Vector negInvLVec = DirectX::XMVectorNegate(invLVec);
                            const Vector worldDir = DirectX::XMVectorMultiply(d, negInvLVec);

                            const Vector& rotAbi = physicsBones.predRot[abi];
                            const Vector localDir = DirectX::XMVector3Rotate(worldDir, DirectX::XMQuaternionConjugate(rotAbi));
                            const Vector& invAlign = coneConstraints.invAlignRot[ci];
                            const Vector coneDir = DirectX::XMVector3Rotate(localDir, invAlign);

                            if (initLambda)
                                coneConstraints.lambda[ci] = 0.0f;

                            const Vector coneDirXZ = DirectX::XMVectorAndInt(coneDir, maskXZ);
                            const Vector rSqVec = DirectX::XMVector3LengthSq(coneDirXZ);
                            if (DirectX::XMVector3Less(rSqVec, vEpsilonSq))
                                continue;

                            const Vector maskNeg = DirectX::XMVectorLess(coneDir, vZero);
                            Vector limit = DirectX::XMVectorSelect(coneConstraints.limitPositive[ci], coneConstraints.limitNegative[ci], maskNeg);
                            limit = DirectX::XMVectorMax(limit, vEpsilon);
                            Vector margin = DirectX::XMVectorSelect(coneConstraints.marginPositive[ci], coneConstraints.marginNegative[ci], maskNeg);
                            margin = DirectX::XMVectorMax(margin, vEpsilon);

                            const Vector rVec = DirectX::XMVectorSqrt(rSqVec);

                            const Vector divLimit = DirectX::XMVectorMultiply(coneDirXZ, DirectX::XMVectorReciprocal(limit));
                            const Vector denomLimitSq = DirectX::XMVector3LengthSq(divLimit);
                            const float currentLimitBase = DirectX::XMVectorGetX(DirectX::XMVectorMultiply(rVec, DirectX::XMVectorReciprocalSqrt(denomLimitSq)));

                            const Vector divMargin = DirectX::XMVectorMultiply(coneDirXZ, DirectX::XMVectorReciprocal(margin));
                            const Vector denomMarginSq = DirectX::XMVector3LengthSq(divMargin);
                            const float currentMargin = DirectX::XMVectorGetX(DirectX::XMVectorMultiply(rVec, DirectX::XMVectorReciprocalSqrt(denomMarginSq)));
                            const float currentLimit = std::max(currentMargin, currentLimitBase);

                            const Vector y = DirectX::XMVectorClamp(DirectX::XMVectorSplatY(coneDir), vNegOne, vOne);
                            const float theta = DirectX::XMVectorGetX(DirectX::XMVectorACos(y));
                            if (theta <= currentMargin)
                                continue;

                            Vector cone_axis = DirectX::XMVectorSwizzle<DirectX::XM_SWIZZLE_Z, DirectX::XM_SWIZZLE_Y, DirectX::XM_SWIZZLE_X, DirectX::XM_SWIZZLE_W>(coneDirXZ);
                            cone_axis = DirectX::XMVectorMultiply(cone_axis, vConeAxisMult);
                            cone_axis = DirectX::XMVectorMultiply(cone_axis, DirectX::XMVectorReciprocal(rVec));

                            const Vector& align = coneConstraints.alignRot[ci];
                            const Vector local_axis = DirectX::XMVector3Rotate(cone_axis, align);
                            const Vector world_axis = DirectX::XMVector3Rotate(local_axis, rotAbi);
                            const Vector cross_axis_d = DirectX::XMVector3Cross(world_axis, d);

                            float currentFactor = 1.0f;
                            if (Epsilon < currentMargin)
                            {
                                const float ratio = std::min(theta * reciprocal(currentMargin), 1.0f);
                                const float ratioCubic = ratio * ratio * ratio;
                                currentFactor = std::max(1.0f - ratioCubic, Epsilon);
                            }

                            const Vector coneDirXZSq = DirectX::XMVectorMultiply(coneDirXZ, coneDirXZ);
                            const Vector dampVec = DirectX::XMVectorSelect(coneConstraints.dampingPositive[ci], coneConstraints.dampingNegative[ci], maskNeg);
                            const Vector dampWeighted = DirectX::XMVectorMultiply(coneDirXZSq, dampVec);
                            const float currentDamping = DirectX::XMVectorGetX(DirectX::XMVector3Dot(dampWeighted, vOne)) * DirectX::XMVectorGetX(DirectX::XMVectorReciprocal(rSqVec));

                            const Vector compVec = DirectX::XMVectorSelect(coneConstraints.compliancePositive[ci], coneConstraints.complianceNegative[ci], maskNeg);
                            const Vector compWeighted = DirectX::XMVectorMultiply(coneDirXZSq, compVec);
                            const float currentComp = DirectX::XMVectorGetX(DirectX::XMVector3Dot(compWeighted, vOne)) * DirectX::XMVectorGetX(DirectX::XMVectorReciprocal(rSqVec));

                            const Vector deltaBi = DirectX::XMVectorSubtract(physicsBones.predPos[bi], physicsBones.pos[bi]);
                            const Vector deltaAbi = DirectX::XMVectorSubtract(physicsBones.predPos[abi], physicsBones.pos[abi]);
                            const Vector relDelta = DirectX::XMVectorSubtract(deltaBi, deltaAbi);

                            const float projDelta = DirectX::XMVectorGetX(DirectX::XMVector3Dot(relDelta, cross_axis_d)) * DirectX::XMVectorGetX(DirectX::XMVectorReciprocal(lSqVec));
                            const float alphaProxy = (currentComp * currentFactor) * invDeltaTimeSq;
                            const float betaProxy = currentDamping;
                            const float W = wSum * DirectX::XMVectorGetX(DirectX::XMVectorReciprocal(lSqVec));
                            const float denomSoft = W + alphaProxy + betaProxy;
                            const float denomHard = W;
                            float deltaLambda = 0.0f;

                            if (theta > currentLimit)
                            {
                                const float C_hard = theta - currentLimit;
                                const float C_soft_max = currentLimit - currentMargin;
                                const float dLambda_hard = -C_hard * reciprocal(denomHard);
                                const float dLambda_soft = (-C_soft_max - (alphaProxy * coneConstraints.lambda[ci]) - (betaProxy * projDelta)) * reciprocal(denomSoft);
                                deltaLambda = dLambda_hard + dLambda_soft;
                                coneConstraints.lambda[ci] += dLambda_soft;
                            }
                            else
                            {
                                const float C_soft = theta - currentMargin;
                                const float dLambda_soft = (-C_soft - (alphaProxy * coneConstraints.lambda[ci]) - (betaProxy * projDelta)) * reciprocal(denomSoft);
                                deltaLambda = dLambda_soft;
                                coneConstraints.lambda[ci] += dLambda_soft;
                            }
                            if (deltaLambda == 0.0f)
                                continue;
                            const float gamma = deltaLambda * W;

                            Vector sinG, cosG;
                            DirectX::XMVectorSinCos(&sinG, &cosG, DirectX::XMVectorReplicate(gamma));
                            const Vector d_cos = DirectX::XMVectorMultiply(d, cosG);
                            const Vector d_corr = DirectX::XMVectorMultiplyAdd(cross_axis_d, sinG, d_cos);
                            const Vector delta_pos = DirectX::XMVectorSubtract(d_corr, d);

                            if (Epsilon < wBi)
                            {
                                const Vector wBiRatio = DirectX::XMVectorReplicate(wBi * reciprocal(wSum));
                                physicsBones.predPos[bi] = DirectX::XMVectorMultiplyAdd(delta_pos, wBiRatio, pBi);
                            }
                            if (Epsilon < wAbi)
                            {
                                const Vector negWAbiRatio = DirectX::XMVectorReplicate(-wAbi * reciprocal(wSum));
                                physicsBones.predPos[abi] = DirectX::XMVectorMultiplyAdd(delta_pos, negWAbiRatio, pAbi);
                            }
                        }
                    }
                }
            },
            tbb::auto_partitioner()
        );

        if (DEBUG_MODE)
        {
            timeProfiler.End([this](const std::string& name, const double ms) {
                this->LoggingTimeProfiler(name, ms);
            });
        }
    }

    void XPBDWorld::SolveConstraints(const float deltaTime, const bool initLambda)
    {
        SolveShapeMatchingConstraint(deltaTime);
        const float invDeltaTimeSq = reciprocal(deltaTime * deltaTime);
        tbb::parallel_invoke([&] {
                SolveDistanceConstraints(deltaTime, invDeltaTimeSq, initLambda);
            }, [&] {
                SolveAngularConstraints(deltaTime, invDeltaTimeSq, initLambda);
            }
        );
        SolveConeConstraints(deltaTime, invDeltaTimeSq, initLambda);
    }

    void XPBDWorld::UpdateProceduralRotations()
    { 
        if (physicsBonesGroup.empty())
            return;

        static Internal::TimeProfiler timeProfiler(__func__);
        if (DEBUG_MODE)
            timeProfiler.Start();

        const std::uint32_t groups = physicsBonesGroup.size() - 1ull;
        tbb::parallel_for(
            tbb::blocked_range<std::uint32_t>(0, groups),
            [&](const tbb::blocked_range<std::uint32_t>& r) {
                for (std::uint32_t g = r.begin(); g != r.end(); ++g)
                {
                    const std::uint32_t begin = physicsBonesGroup[g];
                    const std::uint32_t end = physicsBonesGroup[g + 1];
                    if (begin >= end)
                        continue;
                    const std::uint32_t oi = physicsBones.objIdx[begin];
                    if (IsDisable(oi))
                        continue;
                    for (std::uint32_t bi = begin; bi < end; ++bi)
                    {
                        const float w = physicsBones.invMass[bi];
                        if (w <= Epsilon)
                            continue;
                        if (physicsBones.angularBlendFactor[bi] <= Epsilon)
                            continue;
                        const std::uint32_t pbi = physicsBones.parentBoneIdx[bi];
                        if (pbi == UINT32_MAX)
                            continue;

                        const Vector& worldPosPbi = physicsBones.predPos[pbi];
                        const Vector& worldPosBi = physicsBones.predPos[bi];
                        const Quaternion& worldRotPbi = physicsBones.predRot[pbi];
                        const Quaternion& oldWorldRotBi = physicsBones.predRot[bi];
                        const Quaternion orgWorldRotBi = DirectX::XMQuaternionMultiply(physicsBones.orgLocalRot[bi], worldRotPbi);

                        Vector restDir = DirectX::XMVector3Rotate(physicsBones.orgLocalPos[bi], worldRotPbi);
                        if (DirectX::XMVector3Less(DirectX::XMVector3LengthSq(restDir), vEpsilonSq))
                            restDir = DirectX::XMVector3Rotate(vYone, orgWorldRotBi);
                        restDir = DirectX::XMVector3Normalize(restDir);

                        Vector currentDir = DirectX::XMVectorSubtract(worldPosBi, worldPosPbi);
                        const Vector curLenSq = DirectX::XMVector3LengthSq(currentDir);
                        if (DirectX::XMVector3LessOrEqual(curLenSq, vEpsilonSq))
                            continue;
                        currentDir = DirectX::XMVectorMultiply(currentDir, DirectX::XMVectorReciprocalSqrt(curLenSq));

                        const Vector dot = DirectX::XMVector3Dot(restDir, currentDir);
                        Quaternion qSwingWorld = qZero;
                        if (DirectX::XMVector3Less(dot, vNeg9999))
                        {
                            Vector up = vYone;
                            const Vector upDot = DirectX::XMVectorAbs(DirectX::XMVector3Dot(restDir, up));
                            if (DirectX::XMVector3Greater(upDot, v99))
                                up = vXone;

                            const Vector fallbackAxis = DirectX::XMVector3Normalize(DirectX::XMVector3Cross(up, restDir));
                            qSwingWorld = DirectX::XMQuaternionRotationNormal(fallbackAxis, DirectX::XM_PI);
                        }
                        else
                        {
                            const Vector cross = DirectX::XMVector3Cross(restDir, currentDir);
                            qSwingWorld = DirectX::XMVectorSetW(cross, 1.0f + DirectX::XMVectorGetX(dot));
                            qSwingWorld = DirectX::XMQuaternionNormalize(qSwingWorld);
                        }

                        Quaternion targetRot = DirectX::XMQuaternionMultiply(orgWorldRotBi, qSwingWorld);

                        const Vector rotDot = DirectX::XMQuaternionDot(oldWorldRotBi, targetRot);
                        const Vector isNegative = DirectX::XMVectorLess(rotDot, vZero);
                        targetRot = DirectX::XMVectorSelect(targetRot, DirectX::XMVectorNegate(targetRot), isNegative);
                        physicsBones.predRot[bi] = DirectX::XMQuaternionNormalize(DirectX::XMQuaternionSlerp(oldWorldRotBi, targetRot, physicsBones.angularBlendFactor[bi]));

                        const Quaternion deltaRot = DirectX::XMQuaternionMultiply(DirectX::XMQuaternionConjugate(oldWorldRotBi), physicsBones.predRot[bi]);
                        physicsBones.rot[bi] = DirectX::XMQuaternionNormalize(DirectX::XMQuaternionMultiply(physicsBones.rot[bi], deltaRot));
                    }
                }
            },
            tbb::auto_partitioner()
        );

        if (DEBUG_MODE)
        {
            timeProfiler.End([this](const std::string& name, const double ms) {
                this->LoggingTimeProfiler(name, ms);
            });
        }
    }

    void XPBDWorld::SolveAnimDrive(const float deltaTime, const bool initLambda)
    {
        if (physicsBonesGroup.empty())
            return;

        static Internal::TimeProfiler timeProfiler(__func__);
        if (DEBUG_MODE)
            timeProfiler.Start();

        const float invDeltaTimeSq = reciprocal(deltaTime * deltaTime);

        auto LinearAnimDrive = [this, invDeltaTimeSq, initLambda](const std::uint32_t bi, const std::uint32_t pbi) {
            const bool hasLinearAnimDrive = EpsilonSq < DirectX::XMVectorGetX(DirectX::XMVector3LengthSq(physicsBones.animDriveCompliance[bi]));
            if (!hasLinearAnimDrive)
                return;

            const float w = physicsBones.invMass[bi];
            if (Epsilon >= w)
                return;

            const Vector& parentPos = physicsBones.predPos[pbi];
            const Quaternion& parentRot = physicsBones.predRot[pbi];
            const float parentScale = physicsBones.orgWorldScale[pbi];

            const Vector localPos = DirectX::XMVectorAdd(physicsBones.orgLocalPos[bi], physicsBones.offset[bi]);
            const Vector rotatedLocalPos = DirectX::XMVector3Rotate(DirectX::XMVectorScale(localPos, parentScale), parentRot);
            const Vector targetRestPos = DirectX::XMVectorAdd(parentPos, rotatedLocalPos);

            const Vector dir = DirectX::XMVectorSubtract(physicsBones.predPos[bi], targetRestPos);
            const Vector CSq = DirectX::XMVector3LengthSq(dir);
            if (DirectX::XMVector3LessOrEqual(vEpsilonSq, CSq))
                return;

            const Vector vC = DirectX::XMVectorSqrt(CSq);
            const float C = DirectX::XMVectorGetX(vC);
            const Vector normal = DirectX::XMVectorMultiply(dir, DirectX::XMVectorReciprocal(vC));

            const Vector localDirComp = DirectX::XMVector3Rotate(normal, DirectX::XMQuaternionConjugate(parentRot));
            const float currentComp = DirectX::XMVectorGetX(DirectX::XMVector3Dot(physicsBones.animDriveCompliance[bi], DirectX::XMVectorAbs(localDirComp)));

            const float alphaProxy = currentComp * invDeltaTimeSq;
            const float denom = w + alphaProxy;

            float deltaLambda = 0.0f;
            if (initLambda)
            {
                deltaLambda = -C * reciprocal(denom);
                physicsBones.animDriveLambda[bi] = deltaLambda;
            }
            else
            {
                deltaLambda = (-C - (alphaProxy * physicsBones.animDriveLambda[bi])) * reciprocal(denom);
                physicsBones.animDriveLambda[bi] += deltaLambda;
            }
            physicsBones.predPos[bi] = DirectX::XMVectorAdd(physicsBones.predPos[bi], DirectX::XMVectorScale(normal, deltaLambda * w));
        };

        auto AngularAnimDrive = [this, invDeltaTimeSq, initLambda](const std::uint32_t bi, const std::uint32_t pbi) {
            const bool hasAngularAnimDrive = AngularEpsilonSq < DirectX::XMVectorGetX(DirectX::XMVector3LengthSq(physicsBones.animDriveAngularCompliance[bi]));
            if (!hasAngularAnimDrive)
                return;

            const Vector sumRotInertia = DirectX::XMVectorAdd(physicsBones.invAngularInertiaPositive[bi], physicsBones.invAngularInertiaNegative[bi]);
            const float proxyInvInertia = DirectX::XMVectorGetX(DirectX::XMVector3LengthSq(sumRotInertia));
            if (AngularEpsilonSq >= proxyInvInertia)
                return;

            const Quaternion& parentRot = physicsBones.predRot[pbi];
            const Quaternion& localRestRot = physicsBones.orgLocalRot[bi];
            const Quaternion target = DirectX::XMQuaternionMultiply(localRestRot, parentRot);

            const Quaternion targetInv = DirectX::XMQuaternionConjugate(target);
            Quaternion diff = DirectX::XMQuaternionMultiply(physicsBones.predRot[bi], targetInv);
            if (DirectX::XMVector3Less(DirectX::XMVectorSplatW(diff), vZero))
                diff = DirectX::XMVectorNegate(diff);

            const Vector qVec = DirectX::XMVectorSetW(diff, 0.0f);
            const Vector CSq = DirectX::XMVector3LengthSq(qVec);
            if (DirectX::XMVector3LessOrEqual(CSq, vAngularEpsilonSq))
                return;

            const Vector vC = DirectX::XMVectorSqrt(CSq);
            const Vector localDir = DirectX::XMVectorMultiply(qVec, DirectX::XMVectorReciprocal(vC));
            const Vector localDirAbs = DirectX::XMVectorAbs(localDir);
            const float currentComp = DirectX::XMVectorGetX(DirectX::XMVector3Dot(physicsBones.animDriveAngularCompliance[bi], localDirAbs));
            const Vector worldDir = DirectX::XMVector3Rotate(localDir, target);

            float wRot = 0.0f;
            Vector worldInvTau = vZero;
            const Quaternion invRot = DirectX::XMQuaternionConjugate(physicsBones.predRot[bi]);
            const Vector localTau = DirectX::XMVector3Rotate(worldDir, invRot);
            const Vector isPos = DirectX::XMVectorGreaterOrEqual(localTau, vZero);
            const Vector invInertia = DirectX::XMVectorSelect(physicsBones.invAngularInertiaNegative[bi], physicsBones.invAngularInertiaPositive[bi], isPos);

            const Vector localInvTau = DirectX::XMVectorMultiply(localTau, invInertia);
            worldInvTau = DirectX::XMVector3Rotate(localInvTau, physicsBones.predRot[bi]);
            wRot = DirectX::XMVectorGetX(DirectX::XMVector3Dot(worldDir, worldInvTau));
            if (wRot <= AngularEpsilon)
                return;

            const float C = DirectX::XMVectorGetX(vC);
            const float alphaProxy = currentComp * invDeltaTimeSq;
            const float denom = wRot + alphaProxy;
            float deltaLambda = 0.0f;

            if (initLambda)
            {
                deltaLambda = -C * reciprocal(denom);
                physicsBones.animDriveAngularLambda[bi] = deltaLambda;
            }
            else
            {
                deltaLambda = (-C - (alphaProxy * physicsBones.animDriveAngularLambda[bi])) * reciprocal(denom);
                physicsBones.animDriveAngularLambda[bi] += deltaLambda;
            }

            const Vector dTheta = DirectX::XMVectorScale(worldInvTau, deltaLambda);
            const Vector thetaSq = DirectX::XMVector3LengthSq(dTheta);
            if (DirectX::XMVector3LessOrEqual(thetaSq, vAngularEpsilonSq))
                return;

            const Vector angle = DirectX::XMVectorSqrt(thetaSq);
            const Vector axis = DirectX::XMVectorMultiply(dTheta, DirectX::XMVectorReciprocal(angle));
            const Quaternion qCorr = DirectX::XMQuaternionRotationNormal(axis, DirectX::XMVectorGetX(angle));
            Quaternion newRot = DirectX::XMQuaternionMultiply(physicsBones.predRot[bi], qCorr);
            if (DirectX::XMVector3Less(DirectX::XMQuaternionDot(physicsBones.predRot[bi], newRot), vZero))
                newRot = DirectX::XMVectorNegate(newRot);
            physicsBones.predRot[bi] = DirectX::XMQuaternionNormalize(newRot);
        };


        const std::uint32_t groups = physicsBonesGroup.size() - 1ull;
        tbb::parallel_for(
            tbb::blocked_range<std::uint32_t>(0, groups),
            [&](const tbb::blocked_range<std::uint32_t>& r) {
                for (std::uint32_t g = r.begin(); g != r.end(); ++g)
                {
                    const std::uint32_t begin = physicsBonesGroup[g];
                    const std::uint32_t end = physicsBonesGroup[g + 1];
                    if (begin >= end)
                        continue;
                    const std::uint32_t oi = physicsBones.objIdx[begin];
                    if (IsDisable(oi))
                        continue;

                    for (std::uint32_t bi = begin; bi < end; ++bi)
                    {
                        const std::uint32_t pbi = physicsBones.parentBoneIdx[bi];
                        if (pbi == UINT32_MAX)
                            continue;

                        LinearAnimDrive(bi, pbi);
                        AngularAnimDrive(bi, pbi);
                    }
                }
            },
            tbb::auto_partitioner()
        );

        if (DEBUG_MODE)
        {
            timeProfiler.End([this](const std::string& name, const double ms) {
                this->LoggingTimeProfiler(name, ms);
            });
        }
    }

    void XPBDWorld::SolveShapeMatchingConstraint(const float deltaTime)
    {
        if (shapeMatchingConstraintsGroup.empty())
            return;

        const float dt2 = deltaTime * deltaTime;
        const Vector invDt2 = DirectX::XMVectorReciprocal(DirectX::XMVectorReplicate(dt2));
        tbb::parallel_for(tbb::blocked_range<std::uint32_t>(0, shapeMatchingConstraints.numConstraints, 32),
            [&](const tbb::blocked_range<std::uint32_t>& r) {
                for (std::uint32_t ci = r.begin(); ci != r.end(); ++ci)
                {
                    const auto& cluster = shapeMatchingConstraints.cluster[ci];
                    Vector currentCoM = vZero;
                    float totalMass = 0.0f;
                    for (std::uint32_t i = 0; i < cluster.size; ++i)
                    {
                        const std::uint32_t idx = cluster.offset + i;
                        const std::uint32_t bi = shapeMatchingConstraints.boneIdx[idx];
                        const float mass = shapeMatchingConstraints.boneMass[idx];
                        const Vector& pos = physicsBones.predPos[bi];
                        currentCoM = DirectX::XMVectorAdd(currentCoM, DirectX::XMVectorScale(pos, mass));
                        totalMass += mass;
                    }
                    currentCoM = DirectX::XMVectorMultiply(currentCoM, DirectX::XMVectorReciprocal(DirectX::XMVectorReplicate(totalMass)));

                    Matrix A = vmZeroAll;
                    for (std::uint32_t i = 0; i < cluster.size; ++i)
                    {
                        const std::uint32_t idx = cluster.offset + i;
                        const std::uint32_t bi = shapeMatchingConstraints.boneIdx[idx];
                        const float mass = shapeMatchingConstraints.boneMass[idx];

                        const Vector pi = DirectX::XMVectorSubtract(physicsBones.predPos[bi], currentCoM);
                        const Vector& qi = shapeMatchingConstraints.restRelativePos[idx];

                        const Vector massPI = DirectX::XMVectorScale(pi, mass);

                        const Vector qx = DirectX::XMVectorSplatX(qi);
                        const Vector qy = DirectX::XMVectorSplatY(qi);
                        const Vector qz = DirectX::XMVectorSplatZ(qi);

                        A.r[0] = DirectX::XMVectorAdd(A.r[0], DirectX::XMVectorMultiply(massPI, qx));
                        A.r[1] = DirectX::XMVectorAdd(A.r[1], DirectX::XMVectorMultiply(massPI, qy));
                        A.r[2] = DirectX::XMVectorAdd(A.r[2], DirectX::XMVectorMultiply(massPI, qz));
                    }

                    const Matrix R = ExtractRotationMatrix(A);
                    const Matrix tranR = DirectX::XMMatrixTranspose(R);
                    const Vector& compPos = cluster.compliancePositive;
                    const Vector& compNeg = cluster.complianceNegative;
                    const Vector& inertia = cluster.inertiaScale;
                    for (std::uint32_t i = 0; i < cluster.size; ++i)
                    {
                        const std::uint32_t idx = cluster.offset + i;
                        const std::uint32_t bi = shapeMatchingConstraints.boneIdx[idx];
                        const Vector& qi = shapeMatchingConstraints.restRelativePos[idx];
                        Vector gi = DirectX::XMVector3TransformNormal(qi, R);
                        gi = DirectX::XMVectorAdd(gi, currentCoM);

                        const Vector& currentPos = physicsBones.predPos[bi];
                        Vector deltaWorld = DirectX::XMVectorSubtract(gi, currentPos);
                        Vector deltaLocal = DirectX::XMVector3TransformNormal(deltaWorld, tranR);
                        const Vector cmpMask = DirectX::XMVectorGreaterOrEqual(deltaLocal, vZero);
                        const Vector alpha = DirectX::XMVectorSelect(compNeg, compPos, cmpMask);

                        const Vector alphaDivDt2 = DirectX::XMVectorMultiply(alpha, invDt2);
                        const Vector invMass = DirectX::XMVectorReplicate(physicsBones.invMass[bi]);
                        const Vector denom = DirectX::XMVectorAdd(invMass, alphaDivDt2);
                        const Vector correctionFactor = DirectX::XMVectorMultiply(invMass, DirectX::XMVectorReciprocal(denom));

                        const Vector finalCorrection = DirectX::XMVectorMultiply(correctionFactor, cluster.inertiaScale);
                        deltaLocal = DirectX::XMVectorMultiply(deltaLocal, finalCorrection);
                        deltaWorld = DirectX::XMVector3TransformNormal(deltaLocal, R);
                        physicsBones.predPos[bi] = DirectX::XMVectorAdd(currentPos, deltaWorld);
                    }
                }
            }, 
            tbb::static_partitioner()
        );
    }
    
    void XPBDWorld::SolveDeformConstraint(const float deltaTime)
    {
        if (deformConstraintsGroup.empty())
            return;

        static Internal::TimeProfiler timeProfiler(__func__);
        if (DEBUG_MODE)
            timeProfiler.Start();

        auto CreateAnisoScaleMatrix = [&](const Vector& localDir, float squish, const Vector& vBulge) {
            const Vector nx = DirectX::XMVectorSplatX(localDir);
            const Vector ny = DirectX::XMVectorSplatY(localDir);
            const Vector nz = DirectX::XMVectorSplatZ(localDir);

            const Vector vSquish = DirectX::XMVectorReplicate(squish);
            const Vector s_minus_bx = DirectX::XMVectorSubtract(vSquish, DirectX::XMVectorSplatX(vBulge));
            const Vector s_minus_by = DirectX::XMVectorSubtract(vSquish, DirectX::XMVectorSplatY(vBulge));
            const Vector s_minus_bz = DirectX::XMVectorSubtract(vSquish, DirectX::XMVectorSplatZ(vBulge));

            Vector row0 = DirectX::XMVectorMultiply(DirectX::XMVectorMultiply(nx, localDir), s_minus_bx);
            Vector row1 = DirectX::XMVectorMultiply(DirectX::XMVectorMultiply(ny, localDir), s_minus_by);
            Vector row2 = DirectX::XMVectorMultiply(DirectX::XMVectorMultiply(nz, localDir), s_minus_bz);

            row0 = DirectX::XMVectorMultiplyAdd(vBulge, vmIdentity.r[0], row0);
            row1 = DirectX::XMVectorMultiplyAdd(vBulge, vmIdentity.r[1], row1);
            row2 = DirectX::XMVectorMultiplyAdd(vBulge, vmIdentity.r[2], row2);

            return Matrix{row0, row1, row2, vmIdentity.r[3]};
        };

        tbb::parallel_for(
            tbb::blocked_range<std::uint32_t>(0, deformConstraints.numConstraints, 32),
            [&](const tbb::blocked_range<std::uint32_t>& r) {
                for (std::uint32_t ci = r.begin(); ci != r.end(); ++ci)
                {
                    const std::uint32_t oi = deformConstraints.objIdx[ci];
                    if (IsDisable(oi))
                        continue;
                    const std::uint32_t bi = deformConstraints.boneIdx[ci];
                    const std::uint32_t abi = deformConstraints.anchIdx[ci];
                    if (bi == UINT32_MAX || abi == UINT32_MAX)
                        continue;
                    const float invMass = physicsBones.invMass[bi];
                    const float invMassA = physicsBones.invMass[abi];
                    if (invMass <= Epsilon && invMassA <= Epsilon)
                        continue;

                    const Vector& worldPos = physicsBones.pos[bi];
                    const Quaternion& worldRot = physicsBones.rot[bi];
                    const Quaternion invWorldRot = DirectX::XMQuaternionConjugate(worldRot);

                    Matrix currentMat = vmZeroAll;

                    const Vector& worldPosA = physicsBones.pos[abi];
                    const Quaternion& worldRotA = physicsBones.rot[abi];
                    const Quaternion invWorldRotA = DirectX::XMQuaternionConjugate(worldRotA);
                    Matrix currentMatA = vmZeroAll;
                    std::uint32_t validCount = 0;
                    std::uint32_t validCountA = 0;
                    if (deformConstraints.restLen[ci] > Epsilon)
                    {
                        const Vector dir = DirectX::XMVectorSubtract(worldPosA, worldPos);
                        const Vector currentLenSq = DirectX::XMVector3LengthSq(dir);
                        if (DirectX::XMVector3Greater(currentLenSq, vEpsilonSq))
                        {
                            const Vector currentLen = DirectX::XMVectorSqrt(currentLenSq);
                            Vector primary = DirectX::XMVectorMultiply(currentLen, DirectX::XMVectorReciprocal(DirectX::XMVectorReplicate(deformConstraints.restLen[ci])));
                            primary = DirectX::XMVectorMax(vMinScale, primary);

                            const Vector worldDir = DirectX::XMVectorMultiply(dir, DirectX::XMVectorReciprocal(currentLen));

                            if (invMass > Epsilon)
                            {
                                const Vector localDir = DirectX::XMVector3Rotate(worldDir, invWorldRot);
                                const Vector axisWeights = DirectX::XMVectorMultiply(localDir, localDir);
                                const Vector effSensVec = DirectX::XMVector3Less(primary, vOne) ? DirectX::XMVectorMultiply(physicsBones.deformSquishSensitivity[bi], deformConstraints.squishWeight[ci]) : DirectX::XMVectorMultiply(physicsBones.deformStretchSensitivity[bi], deformConstraints.stretchWeight[ci]);
                                const float effSens = DirectX::XMVectorGetX(DirectX::XMVector3Dot(effSensVec, axisWeights));
                                float distSquish = 1.0f + (DirectX::XMVectorGetX(primary) - 1.0f) * effSens;
                                distSquish = std::max(distSquish, 0.01f);
                                const float maxBulge = rsqrt(distSquish);
                                const Vector maxBulgeDev = DirectX::XMVectorReplicate(maxBulge - 1.0f);
                                const Vector finalBulge = DirectX::XMVectorMultiplyAdd(maxBulgeDev, physicsBones.deformVolumePreservation[bi], vOne);
                                currentMat += CreateAnisoScaleMatrix(localDir, distSquish, finalBulge) - vmIdentity;
                                ++validCount;
                            }
                            if (invMassA > Epsilon)
                            {
                                const Vector localDirA = DirectX::XMVector3Rotate(DirectX::XMVectorNegate(worldDir), invWorldRotA);
                                const Vector axisWeightsA = DirectX::XMVectorMultiply(localDirA, localDirA);
                                const Vector effSensVecA = DirectX::XMVector3Less(primary, vOne) ? DirectX::XMVectorMultiply(physicsBones.deformSquishSensitivity[abi], deformConstraints.squishWeight[ci]) : DirectX::XMVectorMultiply(physicsBones.deformStretchSensitivity[abi], deformConstraints.stretchWeight[ci]);
                                const float effSensA = DirectX::XMVectorGetX(DirectX::XMVector3Dot(effSensVecA, axisWeightsA));
                                float distSquishA = 1.0f + (DirectX::XMVectorGetX(primary) - 1.0f) * effSensA;
                                distSquishA = std::max(distSquishA, 0.01f);
                                const float maxBulgeA = rsqrt(distSquishA);
                                const Vector maxBulgeDevA = DirectX::XMVectorReplicate(maxBulgeA - 1.0f);
                                const Vector finalBulgeA = DirectX::XMVectorMultiplyAdd(maxBulgeDevA, physicsBones.deformVolumePreservation[abi], vOne);
                                currentMatA += CreateAnisoScaleMatrix(localDirA, distSquishA, finalBulgeA) - vmIdentity;
                                ++validCountA;
                            }
                        }
                    }

                    const Quaternion targetRot = DirectX::XMQuaternionMultiply(deformConstraints.restRot[ci], worldRotA);
                    const Quaternion rotDiff = DirectX::XMQuaternionMultiply(worldRot, DirectX::XMQuaternionConjugate(targetRot));
                    const Quaternion rotDiffAligned = DirectX::XMQuaternionMultiply(DirectX::XMQuaternionMultiply(physicsBones.invAlignRot[bi], rotDiff), physicsBones.alignRot[bi]);
                    const Vector qy = DirectX::XMVectorSplatY(rotDiffAligned);
                    const Vector qw = DirectX::XMVectorSplatW(rotDiffAligned);
                    const Vector twistLenSq = DirectX::XMVectorAdd(DirectX::XMVectorMultiply(qy, qy), DirectX::XMVectorMultiply(qw, qw));
                    const Vector maskTwist = DirectX::XMVectorGreater(twistLenSq, vAngularEpsilonSq);
                    const Vector invLen = DirectX::XMVectorReciprocalSqrt(twistLenSq);

                    const Vector qTwistRaw = DirectX::XMVectorMultiply(DirectX::XMVectorAndInt(rotDiffAligned, maskYW), invLen);
                    const Quaternion qTwist = DirectX::XMVectorSelect(vWone, qTwistRaw, maskTwist);

                    const Quaternion invTwist = DirectX::XMQuaternionConjugate(qTwist);
                    const Quaternion qSwing = DirectX::XMQuaternionMultiply(invTwist, rotDiffAligned);
                    const float swingAngle = 2.0f * DirectX::XMScalarACos(std::clamp(DirectX::XMVectorGetW(qSwing), -1.0f, 1.0f));
                    if (swingAngle > AngularEpsilon)
                    {
                        if (invMass > Epsilon)
                        {
                            const Vector swingAxis = DirectX::XMVectorSetW(qSwing, 0.0f);
                            const Vector axisLenSq = DirectX::XMVector3LengthSq(swingAxis);
                            if (DirectX::XMVector3Greater(axisLenSq, vAngularEpsilonSq))
                            {
                                const Vector normSwingAxisAligned = DirectX::XMVectorMultiply(swingAxis, DirectX::XMVectorReciprocalSqrt(axisLenSq));
                                const Vector localRotAxis = DirectX::XMVector3Rotate(normSwingAxisAligned, physicsBones.alignRot[bi]);
                                const Vector effBulgeSens = DirectX::XMVectorMultiply(physicsBones.deformBulgeSensitivity[bi], deformConstraints.bulgeWeight[ci]);
                                const Vector finalBulge = DirectX::XMVectorMultiplyAdd(DirectX::XMVectorReplicate(swingAngle), effBulgeSens, vOne);
                                currentMat += CreateAnisoScaleMatrix(localRotAxis, 1.0f, finalBulge) - vmIdentity;
                                ++validCount;
                            }
                        }
                        if (invMassA > Epsilon)
                        {
                            const Quaternion targetRotA = DirectX::XMQuaternionMultiply(DirectX::XMQuaternionConjugate(deformConstraints.restRot[ci]), worldRot);
                            const Quaternion rotDiffA = DirectX::XMQuaternionMultiply(worldRotA, DirectX::XMQuaternionConjugate(targetRotA));
                            const Quaternion rotDiffAAligned = DirectX::XMQuaternionMultiply(DirectX::XMQuaternionMultiply(physicsBones.invAlignRot[abi], rotDiffA), physicsBones.alignRot[abi]);
                            const Vector qyA = DirectX::XMVectorSplatY(rotDiffAAligned);
                            const Vector qwA = DirectX::XMVectorSplatW(rotDiffAAligned);
                            const Vector twistLenSqA = DirectX::XMVectorAdd(DirectX::XMVectorMultiply(qyA, qyA), DirectX::XMVectorMultiply(qwA, qwA));
                            const Vector maskTwistA = DirectX::XMVectorGreater(twistLenSqA, vAngularEpsilonSq);
                            const Vector invLenA = DirectX::XMVectorReciprocalSqrt(twistLenSqA);

                            const Vector qTwistRawA = DirectX::XMVectorMultiply(DirectX::XMVectorAndInt(rotDiffAAligned, maskYW), invLenA);
                            const Quaternion qTwistA = DirectX::XMVectorSelect(vWone, qTwistRawA, maskTwistA);

                            const Quaternion invTwistA = DirectX::XMQuaternionConjugate(qTwistA);
                            const Quaternion qSwingA = DirectX::XMQuaternionMultiply(invTwistA, rotDiffAAligned);
                            const Vector swingAxisA = DirectX::XMVectorSetW(qSwingA, 0.0f);
                            const Vector axisLenSqA = DirectX::XMVector3LengthSq(swingAxisA);
                            if (DirectX::XMVector3Greater(axisLenSqA, vAngularEpsilonSq))
                            {
                                const Vector normSwingAxisAlignedA = DirectX::XMVectorMultiply(swingAxisA, DirectX::XMVectorReciprocalSqrt(axisLenSqA));
                                const Vector localRotAxisA = DirectX::XMVector3Rotate(normSwingAxisAlignedA, physicsBones.alignRot[abi]);
                                const Vector effBulgeSensA = DirectX::XMVectorMultiply(physicsBones.deformBulgeSensitivity[abi], deformConstraints.bulgeWeight[ci]);
                                const Vector finalBulgeA = DirectX::XMVectorMultiplyAdd(DirectX::XMVectorReplicate(swingAngle), effBulgeSensA, vOne);
                                currentMatA += CreateAnisoScaleMatrix(localRotAxisA, 1.0f, finalBulgeA) - vmIdentity;
                                ++validCountA;
                            }
                        }
                    }

                    if (validCount > 0)
                    {
                        tbb::spin_mutex::scoped_lock sl(physicsBonesLock[bi]);
                        physicsBones.deformScaleCache[bi] += currentMat;
                        physicsBones.deformCount[bi] += validCount;
                    }
                    if (validCountA > 0)
                    {
                        tbb::spin_mutex::scoped_lock sl(physicsBonesLock[abi]);
                        physicsBones.deformScaleCache[abi] += currentMatA;
                        physicsBones.deformCount[abi] += validCountA;
                    }
                }
            },
            tbb::static_partitioner()
        );

        const Vector dt = DirectX::XMVectorReplicate(deltaTime);
        const Vector dtSq = DirectX::XMVectorReplicate(deltaTime * deltaTime);
        tbb::parallel_for(
            tbb::blocked_range<std::uint32_t>(0, physicsBones.numBones, 128),
            [&](const tbb::blocked_range<std::uint32_t>& r) {
                for (std::uint32_t bi = r.begin(); bi != r.end(); ++bi)
                {
                    Matrix targetDevMat = physicsBones.deformScaleCache[bi];
                    std::uint32_t deformCount = physicsBones.deformCount[bi];
                    const Internal::PhysicsBones::CollideCache deformCache = physicsBones.deformCache[bi];

                    physicsBones.deformScaleCache[bi] = vmZeroAll;
                    physicsBones.deformCount[bi] = 0;
                    physicsBones.deformCache[bi] = {};

                    const std::uint32_t oi = physicsBones.objIdx[bi];
                    if (IsDisable(oi) || physicsBones.invMass[bi] <= Epsilon || physicsBones.isParticle[bi])
                        continue;

                    Matrix targetMat = vmIdentity;
                    if (deformCount > 0)
                    {
                        const Vector invCount = DirectX::XMVectorReciprocal(DirectX::XMVectorReplicate(static_cast<float>(deformCount)));
                        targetDevMat.r[0] = DirectX::XMVectorMultiply(targetDevMat.r[0], invCount);
                        targetDevMat.r[1] = DirectX::XMVectorMultiply(targetDevMat.r[1], invCount);
                        targetDevMat.r[2] = DirectX::XMVectorMultiply(targetDevMat.r[2], invCount);
                        targetMat += targetDevMat;
                    }
                    else
                        targetMat = vmIdentity;

                    if (deformCache.depth > Epsilon)
                    {
                        if (DirectX::XMVector3Greater(DirectX::XMVector3LengthSq(deformCache.normal), vEpsilonSq))
                        {
                            const Quaternion invWorldRot = DirectX::XMQuaternionConjugate(physicsBones.rot[bi]);
                            const Vector localNormal = DirectX::XMVector3Rotate(DirectX::XMVector3Normalize(deformCache.normal), invWorldRot);

                            const Vector axisWeights = DirectX::XMVectorMultiply(localNormal, localNormal);
                            const float colSensitivity = DirectX::XMVectorGetX(DirectX::XMVector3Dot(physicsBones.deformSquishSensitivity[bi], axisWeights));

                            const float squishRatio = reciprocal(1.0f + (deformCache.depth * colSensitivity));
                            const float maxBulge = rsqrt(squishRatio);
                            const Vector maxBulgeDev = DirectX::XMVectorReplicate(maxBulge - 1.0f);
                            const Vector finalBulge = DirectX::XMVectorMultiplyAdd(maxBulgeDev, physicsBones.deformVolumePreservation[bi], vOne);
                            const Matrix colMat = CreateAnisoScaleMatrix(localNormal, squishRatio, finalBulge);
                            targetMat = DirectX::XMMatrixMultiply(targetMat, colMat);
                            ++deformCount;
                        }
                    }

                    const Vector tSqX = DirectX::XMVector3LengthSq(targetMat.r[0]);
                    const Vector tSqY = DirectX::XMVector3LengthSq(targetMat.r[1]);
                    const Vector tSqZ = DirectX::XMVector3LengthSq(targetMat.r[2]);
                    const Vector tScaleSqVec = DirectX::XMVectorSet(DirectX::XMVectorGetX(tSqX), DirectX::XMVectorGetX(tSqY), DirectX::XMVectorGetX(tSqZ), 0.0f);
                    const Vector tScaleVec = DirectX::XMVectorMax(DirectX::XMVectorSqrt(tScaleSqVec), vEpsilon);

                    Vector tDeviation = DirectX::XMVectorSubtract(tScaleVec, vOne);
                    const Vector maxRadius = DirectX::XMVectorSubtract(physicsBones.deformMax[bi], vOne);
                    const Vector minRadius = DirectX::XMVectorSubtract(vOne, physicsBones.deformMin[bi]);
                    const Vector tIsNegative = DirectX::XMVectorLess(tDeviation, vZero);

                    Vector ellipseRadii = DirectX::XMVectorSelect(maxRadius, minRadius, tIsNegative);
                    ellipseRadii = DirectX::XMVectorMax(ellipseRadii, vEpsilon);

                    const Vector tNormalizedDev = DirectX::XMVectorMultiply(tDeviation, DirectX::XMVectorReciprocal(ellipseRadii));
                    const Vector tLengthSq = DirectX::XMVector3LengthSq(tNormalizedDev);
                    if (DirectX::XMVector3Greater(tLengthSq, vOne))
                    {
                        const Vector invLength = DirectX::XMVectorReciprocalSqrt(tLengthSq);
                        tDeviation = DirectX::XMVectorMultiply(tDeviation, invLength);
                        const Vector clampedScaleVec = DirectX::XMVectorAdd(tDeviation, vOne);

                        const Vector scaleRatios = DirectX::XMVectorMultiply(clampedScaleVec, DirectX::XMVectorReciprocal(tScaleVec));
                        targetMat.r[0] = DirectX::XMVectorMultiply(targetMat.r[0], DirectX::XMVectorSplatX(scaleRatios));
                        targetMat.r[1] = DirectX::XMVectorMultiply(targetMat.r[1], DirectX::XMVectorSplatY(scaleRatios));
                        targetMat.r[2] = DirectX::XMVectorMultiply(targetMat.r[2], DirectX::XMVectorSplatZ(scaleRatios));
                    }

                    Matrix currentMat = physicsBones.deformScale[bi];
                    Matrix velocityMat = physicsBones.deformVelocityScale[bi];
                    if (deformCache.depth > Epsilon)
                        velocityMat = vmZero;
                    const Vector isSquishTarget = DirectX::XMVectorLess(tScaleVec, vOne);
                    const Vector stiffness = DirectX::XMVectorSelect(physicsBones.deformStretchStiffness[bi], physicsBones.deformSquishStiffness[bi], isSquishTarget);
                    const Vector damping = DirectX::XMVectorSelect(physicsBones.deformStretchDamping[bi], physicsBones.deformSquishDamping[bi], isSquishTarget);
                    const Vector kX = DirectX::XMVectorSplatX(stiffness);
                    const Vector kY = DirectX::XMVectorSplatY(stiffness);
                    const Vector kZ = DirectX::XMVectorSplatZ(stiffness);
                    const Vector dX = DirectX::XMVectorSplatX(damping);
                    const Vector dY = DirectX::XMVectorSplatY(damping);
                    const Vector dZ = DirectX::XMVectorSplatZ(damping);

                    auto ApplySpring = [&](Vector& current, Vector& velocity, const Vector& target, const Vector& k, const Vector& d) {
                        const Vector displacement = DirectX::XMVectorSubtract(target, current);
                        const Vector kDt = DirectX::XMVectorMultiply(k, dt);
                        const Vector springImpulse = DirectX::XMVectorMultiply(kDt, displacement);
                        const Vector numerator = DirectX::XMVectorAdd(velocity, springImpulse);

                        const Vector dDt = DirectX::XMVectorMultiply(d, dt);
                        const Vector kDt2 = DirectX::XMVectorMultiply(k, dtSq);
                        const Vector denominator = DirectX::XMVectorAdd(vOne, DirectX::XMVectorAdd(dDt, kDt2));

                        velocity = DirectX::XMVectorMultiply(numerator, DirectX::XMVectorReciprocal(denominator));
                        current = DirectX::XMVectorMultiplyAdd(velocity, dt, current);
                    };

                    ApplySpring(currentMat.r[0], velocityMat.r[0], targetMat.r[0], kX, dX);
                    ApplySpring(currentMat.r[1], velocityMat.r[1], targetMat.r[1], kY, dY);
                    ApplySpring(currentMat.r[2], velocityMat.r[2], targetMat.r[2], kZ, dZ);

                    const Vector sqX = DirectX::XMVector3LengthSq(currentMat.r[0]);
                    const Vector sqY = DirectX::XMVector3LengthSq(currentMat.r[1]);
                    const Vector sqZ = DirectX::XMVector3LengthSq(currentMat.r[2]);
                    const Vector currentScaleSqVec = DirectX::XMVectorSet(DirectX::XMVectorGetX(sqX), DirectX::XMVectorGetX(sqY), DirectX::XMVectorGetX(sqZ), 0.0f);
                    const Vector currentScaleVec = DirectX::XMVectorMax(DirectX::XMVectorSqrt(currentScaleSqVec), vEpsilon);

                    Vector deviation = DirectX::XMVectorSubtract(currentScaleVec, vOne);
                    const Vector isNegative = DirectX::XMVectorLess(deviation, vZero);

                    Vector currentEllipseRadii = DirectX::XMVectorSelect(maxRadius, minRadius, isNegative);
                    currentEllipseRadii = DirectX::XMVectorMax(currentEllipseRadii, vEpsilon);

                    const Vector normalizedDev = DirectX::XMVectorMultiply(deviation, DirectX::XMVectorReciprocal(currentEllipseRadii));
                    const Vector lengthSq = DirectX::XMVector3LengthSq(normalizedDev);
                    if (DirectX::XMVector3Greater(lengthSq, vOne))
                    {
                        const Vector invLength = DirectX::XMVectorReciprocalSqrt(lengthSq);
                        deviation = DirectX::XMVectorMultiply(deviation, invLength);
                        const Vector clampedScaleVec = DirectX::XMVectorAdd(deviation, vOne);
                        const Vector scaleRatios = DirectX::XMVectorMultiply(clampedScaleVec, DirectX::XMVectorReciprocal(currentScaleVec));
                        currentMat.r[0] = DirectX::XMVectorMultiply(currentMat.r[0], DirectX::XMVectorSplatX(scaleRatios));
                        currentMat.r[1] = DirectX::XMVectorMultiply(currentMat.r[1], DirectX::XMVectorSplatY(scaleRatios));
                        currentMat.r[2] = DirectX::XMVectorMultiply(currentMat.r[2], DirectX::XMVectorSplatZ(scaleRatios));
                    }

                    currentMat.r[3] = vWone;
                    velocityMat.r[3] = vZero;

                    if (IsInvalid(currentMat))
                    {
                        currentMat = vmIdentity;
                        velocityMat = vmZeroAll;
                    }

                    physicsBones.deformVelocityScale[bi] = velocityMat;
                    physicsBones.deformScale[bi] = currentMat;
                }
            },
            tbb::static_partitioner()
        );

        if (DEBUG_MODE)
        {
            timeProfiler.End([this](const std::string& name, const double ms) {
                this->LoggingTimeProfiler(name, ms);
            });
        }
    }

    void XPBDWorld::UpdateBoneVelocity(const float deltaTime)
    {
        static Internal::TimeProfiler timeProfiler(__func__);
        if (DEBUG_MODE)
            timeProfiler.Start();

        const Vector dt = DirectX::XMVectorReplicate(deltaTime);
        const Vector subTimeRatio = DirectX::XMVectorReplicate(deltaTime * 60.0f);
        const Vector invDt = DirectX::XMVectorReciprocal(DirectX::XMVectorReplicate(deltaTime));
        const Vector dbInvDt = DirectX::XMVectorScale(invDt, 2.0f);
        const float gravity = DirectX::XMVectorGetX(DirectX::XMVector3Length(SkyrimGravity));
        const Vector bounceThreshold = DirectX::XMVectorReplicate(-gravity * deltaTime * 2.0f);
        tbb::parallel_for(
            tbb::blocked_range<std::uint32_t>(0, physicsBones.numBones, 128),
            [&](const tbb::blocked_range<std::uint32_t>& r) {
                for (std::uint32_t bi = r.begin(); bi != r.end(); ++bi)
                {
                    const Internal::PhysicsBones::CollideCache frictionCache = physicsBones.frictionCache[bi];
                    physicsBones.frictionCache[bi] = {};

                    const std::uint32_t oi = physicsBones.objIdx[bi];
                    if (IsDisable(oi))
                        continue;
                    if (physicsBones.invMass[bi] <= Epsilon)
                        continue;

                    physicsBones.prevPos[bi] = physicsBones.pos[bi];
                    physicsBones.prevRot[bi] = physicsBones.rot[bi];

                    const Vector deltaPos = DirectX::XMVectorSubtract(physicsBones.predPos[bi], physicsBones.pos[bi]);
                    const Vector relVel = DirectX::XMVectorMultiply(deltaPos, invDt);

                    const Quaternion invRot = DirectX::XMQuaternionConjugate(physicsBones.rot[bi]);
                    const Vector localRelVel = DirectX::XMVector3Rotate(relVel, invRot);
                    
                    const Vector isPosLinVel = DirectX::XMVectorGreaterOrEqual(localRelVel, vZero);
                    const Vector linDamping = DirectX::XMVectorSelect(physicsBones.dampingNegative[bi], physicsBones.dampingPositive[bi], isPosLinVel);
                    const Vector scaledDamping = DirectX::XMVectorSubtract(vOne, DirectX::XMVectorMultiply(linDamping, subTimeRatio));
                    const Vector dampedLocalRelVel = DirectX::XMVectorMultiply(localRelVel, scaledDamping);
                    const Vector dampedRelVel = DirectX::XMVector3Rotate(dampedLocalRelVel, physicsBones.rot[bi]);
                    const Vector& objVel = objectDatas.velocity[oi];
                    Vector posVel = DirectX::XMVectorAdd(dampedRelVel, objVel);

                    // apply centrifugal force
                    {
                        const Vector& rootPos = objectDatas.prevWorldPos[oi];
                        const Vector& omega = objectDatas.omegaWorldRot[oi];
                        const Vector omegaLenSq = DirectX::XMVector3LengthSq(omega);
                        if (DirectX::XMVector3Greater(omegaLenSq, vEpsilonSq))
                        {
                            const Vector r = DirectX::XMVectorSubtract(physicsBones.predPos[bi], rootPos);
                            const Vector omega_x_r = DirectX::XMVector3Cross(omega, r);
                            Vector a_cf = DirectX::XMVector3Cross(omega, omega_x_r);
                            a_cf = DirectX::XMVectorNegate(a_cf);
                            const Vector extraVel = DirectX::XMVectorMultiply(a_cf, dt);
                            posVel = DirectX::XMVectorAdd(posVel, extraVel);
                            /*const Vector a_cor = DirectX::XMVectorMultiply(DirectX::XMVector3Cross(omega, posVel), vNegTwo);
                            posVel = DirectX::XMVectorAdd(posVel, DirectX::XMVectorMultiply(a_cor, dt));*/
                        }
                    }

                    const float depth = frictionCache.depth;
                    const bool hasCollision = Epsilon < depth;
                    const Vector normalImpulse = hasCollision ? DirectX::XMVectorScale(invDt, depth) : vYone;
                    const Vector friction = DirectX::XMVectorReplicate(physicsBones.collisionFriction[bi]);
                    if (hasCollision)
                    {
                        const Vector n = DirectX::XMVector3Normalize(frictionCache.normal);
                        const Vector v_n_mag = DirectX::XMVector3Dot(posVel, n);
                        const Vector v_n = DirectX::XMVectorMultiply(n, v_n_mag);
                        const Vector v_t = DirectX::XMVectorSubtract(posVel, v_n);

                        const Vector preVel = physicsBones.posVel[bi];
                        const Vector v_pre_n_mag = DirectX::XMVector3Dot(preVel, n);

                        const Vector restitution = DirectX::XMVectorReplicate(physicsBones.collisionRestitution[bi]);
                        Vector bounceMag = DirectX::XMVectorMultiply(DirectX::XMVectorNegate(v_pre_n_mag), restitution);

                        const Vector bounceMask = DirectX::XMVectorLess(v_pre_n_mag, bounceThreshold);
                        bounceMag = DirectX::XMVectorSelect(vZero, bounceMag, bounceMask);

                        const Vector v_n_clipped = DirectX::XMVectorMax(v_n_mag, bounceMag);
                        const Vector v_n_final = DirectX::XMVectorMultiply(n, v_n_clipped);
                        const Vector v_t_lenSq = DirectX::XMVector3LengthSq(v_t);
                        const float physicsScale = physicsBones.physicsScale[bi];
                        if (DirectX::XMVector3LessOrEqual(v_t_lenSq, vEpsilonSq))
                            posVel = v_n_final;
                        else
                        {
                            const Vector frictionDrop = DirectX::XMVectorMultiply(friction, normalImpulse);
                            const Vector vt_mag = DirectX::XMVectorSqrt(v_t_lenSq);
                            const Vector scale = DirectX::XMVectorMax(vZero, DirectX::XMVectorMultiply(DirectX::XMVectorSubtract(vt_mag, frictionDrop), DirectX::XMVectorReciprocal(vt_mag)));
                            const Vector v_t_new = DirectX::XMVectorMultiply(v_t, scale);
                            posVel = DirectX::XMVectorAdd(v_t_new, v_n_final);
                        }
                    }
                    physicsBones.posVel[bi] = posVel;

                    physicsBones.pos[bi] = physicsBones.predPos[bi];
                    if (physicsBones.advancedRotation[bi])
                    {
                        Vector dq = DirectX::XMQuaternionMultiply(DirectX::XMQuaternionConjugate(physicsBones.rot[bi]), physicsBones.predRot[bi]);
                        if (DirectX::XMVectorGetW(dq) < 0.0f)
                            dq = DirectX::XMVectorNegate(dq);

                        const Vector worldAngVel = DirectX::XMVectorSetW(DirectX::XMVectorMultiply(DirectX::XMVectorMultiply(dq, vTwo), invDt), 0.0f);
                        const Vector localAngVel = DirectX::XMVector3Rotate(worldAngVel, invRot);
                        const Vector isPosAngVel = DirectX::XMVectorGreaterOrEqual(localAngVel, vZero);
                        const Vector angDamping = DirectX::XMVectorSelect(physicsBones.angularDampingNegative[bi], physicsBones.angularDampingPositive[bi], isPosAngVel);
                        const Vector scaledAngDamping = DirectX::XMVectorSubtract(vOne, DirectX::XMVectorMultiply(angDamping, subTimeRatio));
                        const Vector dampedLocalAngVel = DirectX::XMVectorMultiply(localAngVel, scaledAngDamping);
                        physicsBones.angVel[bi] = DirectX::XMVector3Rotate(dampedLocalAngVel, physicsBones.rot[bi]);
                        physicsBones.rot[bi] = physicsBones.predRot[bi];

                        if (hasCollision)
                        {
                            const Vector angVelLenSq = DirectX::XMVector3LengthSq(physicsBones.angVel[bi]);
                            if (DirectX::XMVector3LessOrEqual(angVelLenSq, vAngularEpsilonSq))
                                physicsBones.angVel[bi] = vZero;
                            else
                            {
                                const Vector angVel_mag = DirectX::XMVectorSqrt(angVelLenSq);
                                const Vector rotFriction = DirectX::XMVectorScale(friction, ANGULAR_FRICTION_SCALE);
                                const Vector rotFrictionDrop = DirectX::XMVectorMultiply(rotFriction, normalImpulse);
                                const Vector rotScale = DirectX::XMVectorMax(vZero, DirectX::XMVectorMultiply(DirectX::XMVectorSubtract(angVel_mag, rotFrictionDrop), DirectX::XMVectorReciprocal(angVel_mag)));
                                physicsBones.angVel[bi] = DirectX::XMVectorMultiply(physicsBones.angVel[bi], rotScale);
                            }
                        }
                    }
                    else
                    {
                        physicsBones.angVel[bi] = vZero;
                        physicsBones.predRot[bi] = physicsBones.rot[bi];
                    }

                    if (IsInvalid(physicsBones.pos[bi]) ||
                        IsInvalid(physicsBones.rot[bi]) ||
                        IsInvalid(physicsBones.posVel[bi]) ||
                        IsInvalid(physicsBones.angVel[bi]))
                    {
                        ResetBone(bi);
                    }
                }
            },
            tbb::static_partitioner()
        );

        if (DEBUG_MODE)
        {
            timeProfiler.End([this](const std::string& name, const double ms) {
                this->LoggingTimeProfiler(name, ms);
            });
        }
    }

    void XPBDWorld::ApplyToSkyrim(const bool syncFrame)
    {
        if (physicsBonesGroup.empty())
            return;

        static Internal::TimeProfiler timeProfiler(__func__);
        if (DEBUG_MODE)
            timeProfiler.Start();

        const float alpha = timeAccumulator * reciprocal(DeltaTime60);
        const std::uint32_t groups = physicsBonesGroup.size() - 1ull; 
        tbb::parallel_for(
            tbb::blocked_range<std::uint32_t>(0, groups),
            [&](const tbb::blocked_range<std::uint32_t>& r) {
                for (std::uint32_t g = r.begin(); g != r.end(); ++g)
                {
                    const std::uint32_t begin = physicsBonesGroup[g];
                    const std::uint32_t end = physicsBonesGroup[g + 1];
                    if (begin >= end)
                        continue;
                    const std::uint32_t oi = physicsBones.objIdx[begin];
                    if (IsDisable(oi))
                        continue;
                    Vector worldPosOffset = vZero;
                    if (syncFrame)
                    {
                        if (RE::TESObjectREFR* object = GetREFR(objectDatas.objectID[oi]); object && object->loadedData && object->loadedData->data3D)
                        {
                            const Vector currentWorldPos = ToVector(object->loadedData->data3D->world.translate);
                            worldPosOffset = DirectX::XMVectorSubtract(currentWorldPos, objectDatas.prevWorldPos[oi]);
                        }
                    }
                    for (std::uint32_t bi = begin; bi < end; ++bi)
                    {
                        if (physicsBones.invMass[bi] <= Epsilon)
                            continue;
                        if (IsSkippedFrame(bi))
                            continue;
                        const std::uint32_t pbi = physicsBones.parentBoneIdx[bi];
                        if (pbi == UINT32_MAX)
                            continue;
                        auto& node = physicsBones.node[bi];
                        if (!node)
                            continue;

                        Vector parentWorldPos = vZero;
                        Quaternion parentWorldRot = qZero;
                        Vector parentWorldScale = vOne;
                        if (node->parent)
                        {
                            parentWorldPos = ToVector(node->parent->world.translate);
                            parentWorldRot = ToQuaternion(node->parent->world.rotate);
                            parentWorldScale = DirectX::XMVectorReplicate(node->parent->world.scale);
                        }
                        else if (std::uint32_t pbi = physicsBones.parentBoneIdx[bi]; pbi != UINT32_MAX)
                        {
                            parentWorldPos = DirectX::XMVectorLerp(physicsBones.prevPos[pbi], physicsBones.pos[pbi], alpha);
                            parentWorldRot = DirectX::XMQuaternionNormalize(DirectX::XMQuaternionSlerp(physicsBones.prevRot[pbi], physicsBones.rot[pbi], alpha));
                            parentWorldScale = DirectX::XMVectorReplicate(physicsBones.orgWorldScale[pbi]);
                        }

                        const Vector physicsWorldPos = DirectX::XMVectorAdd(DirectX::XMVectorLerp(physicsBones.prevPos[bi], physicsBones.pos[bi], alpha), physicsBones.dynamicLinearOffset[bi]);
                        const Quaternion physicsWorldRot = DirectX::XMQuaternionNormalize(DirectX::XMQuaternionMultiply(DirectX::XMQuaternionSlerp(physicsBones.prevRot[bi], physicsBones.rot[bi], alpha), physicsBones.dynamicAngularOffset[bi]));
                        Quaternion finalWorldRot = physicsWorldRot;

                        /*if (physicsBones.angularBlendFactor[bi] > Epsilon)
                        {
                            const Quaternion orgWorldRot = DirectX::XMQuaternionMultiply(physicsBones.orgLocalRot[bi], parentWorldRot);
                        
                            Vector restDir = DirectX::XMVector3Rotate(physicsBones.orgLocalPos[bi], parentWorldRot);
                            if (DirectX::XMVector3Less(DirectX::XMVector3LengthSq(restDir), vEpsilonSq))
                                restDir = DirectX::XMQuaternionMultiply(DirectX::XMVector3Rotate(vYone, physicsBones.orgLocalRot[bi]), parentWorldRot);
                            restDir = DirectX::XMVector3Normalize(restDir);

                            Vector currentDir = DirectX::XMVectorSubtract(physicsWorldPos, parentWorldPos);
                            if (DirectX::XMVector3Less(DirectX::XMVector3LengthSq(currentDir), vEpsilonSq))
                                currentDir = restDir;
                            else
                                currentDir = DirectX::XMVector3Normalize(currentDir);
                            const Vector dirLenSq = DirectX::XMVector3LengthSq(currentDir);
                            if (DirectX::XMVector3Greater(dirLenSq, vEpsilonSq))
                            {
                                const Quaternion invOrgWorldRot = DirectX::XMQuaternionConjugate(orgWorldRot);
                                Quaternion worldOffset = DirectX::XMQuaternionMultiply(invOrgWorldRot, physicsWorldRot);
                                if (DirectX::XMVector3Less(DirectX::XMVectorSplatW(worldOffset), vZero))
                                    worldOffset = DirectX::XMVectorNegate(worldOffset);

                                const Vector offsetXYZ = DirectX::XMVectorAndInt(worldOffset, maskXYZ);
                                const Vector twistDot = DirectX::XMVector3Dot(offsetXYZ, restDir);
                                const Vector twistVec = DirectX::XMVectorMultiply(restDir, twistDot);

                                Quaternion qTwistWorld = DirectX::XMVectorSelect(twistVec, worldOffset, maskW);
                                const Vector twistLenSq = DirectX::XMVector4LengthSq(qTwistWorld);
                                const Vector maskTwist = DirectX::XMVectorGreater(twistLenSq, vAngularEpsilonSq);
                                qTwistWorld = DirectX::XMVectorMultiply(qTwistWorld, DirectX::XMVectorReciprocalSqrt(twistLenSq));
                                qTwistWorld = DirectX::XMVectorSelect(vWone, qTwistWorld, maskTwist);

                                const Vector swingDot = DirectX::XMVector3Dot(restDir, currentDir);
                                const Vector flippedMask = DirectX::XMVectorLess(swingDot, vNeg9999);

                                const Vector swingAxis = DirectX::XMVector3Cross(restDir, currentDir);
                                const Quaternion qSwingRaw = DirectX::XMVectorSelect(swingAxis, DirectX::XMVectorAdd(swingDot, vOne), maskW);

                                Vector up = vYone;
                                const Vector upDot = DirectX::XMVectorAbs(DirectX::XMVector3Dot(restDir, up));
                                if (DirectX::XMVector3Greater(upDot, v99))
                                    up = vXone;
                                const Vector fallbackAxis = DirectX::XMVector3Normalize(DirectX::XMVector3Cross(up, restDir));
                                const Quaternion qSwingFlipped = DirectX::XMQuaternionRotationNormal(fallbackAxis, DirectX::XM_PI);
                                const Quaternion qSwingWorld = DirectX::XMQuaternionNormalize(DirectX::XMVectorSelect(qSwingRaw, qSwingFlipped, flippedMask));

                                Quaternion targetRot = DirectX::XMQuaternionMultiply(qTwistWorld, qSwingWorld);
                                targetRot = DirectX::XMQuaternionNormalize(DirectX::XMQuaternionMultiply(orgWorldRot, targetRot));
                                const Vector vDot = DirectX::XMQuaternionDot(physicsWorldRot, targetRot);
                                const Vector isNegative = DirectX::XMVectorLess(vDot, vZero);
                                targetRot = DirectX::XMVectorSelect(targetRot, DirectX::XMVectorNegate(targetRot), isNegative);
                                finalWorldRot = DirectX::XMQuaternionNormalize(DirectX::XMQuaternionSlerp(physicsWorldRot, targetRot, physicsBones.angularBlendFactor[bi]));
                            }
                        }*/

                        const Vector diff = DirectX::XMVectorSubtract(physicsWorldPos, parentWorldPos);
                        const Vector invRot = DirectX::XMQuaternionConjugate(parentWorldRot);

                        const Vector localPos = DirectX::XMVectorMultiply(DirectX::XMVector3Rotate(diff, invRot), DirectX::XMVectorReciprocal(parentWorldScale));
                        const Quaternion localRot = DirectX::XMQuaternionMultiply(finalWorldRot, invRot);

                        Vector calibWorldPos = DirectX::XMVectorAdd(DirectX::XMVector3Rotate(DirectX::XMVectorMultiply(localPos, parentWorldScale), parentWorldRot), parentWorldPos);
                        if (syncFrame)
                            calibWorldPos = DirectX::XMVectorAdd(calibWorldPos, worldPosOffset);
                        const RE::NiPoint3 localP = ToNiPoint(localPos);
                        const RE::NiMatrix3 localR = ToNiMatrix(localRot);
                        const RE::NiPoint3 worldP = ToNiPoint(calibWorldPos);

                        const Matrix localScaledMat = DirectX::XMMatrixMultiply(physicsBones.deformScale[bi], ToMatrix(localRot));
                        const Matrix calibWorldMat = DirectX::XMMatrixMultiply(localScaledMat, ToMatrix(parentWorldRot));
                        const RE::NiMatrix3 worldR = ToNiMatrix(calibWorldMat);

                        // prevents memory write skips by compiler optimization
                        std::memcpy(&node->local.translate, &localP, sizeof(localP));
                        std::memcpy(&node->local.rotate, &localR, sizeof(localR));
                        std::memcpy(&node->world.translate, &worldP, sizeof(worldP));
                        std::memcpy(&node->world.rotate, &worldR, sizeof(worldR));
                    }
                }
            },
            tbb::auto_partitioner()
        );

        if (DEBUG_MODE)
        {
            timeProfiler.End([this](const std::string& name, const double ms) {
                this->LoggingTimeProfiler(name, ms);
            });
        }
    }

    AABB XPBDWorld::GetObjectAABB(const std::uint32_t objIdx) const
    {
        AABB bounds = AABB();
        bool isFirst = false;

        if (!collidersGroup.empty())
        {
            const std::uint32_t groups = collidersGroup.size() - 1ull;
            for (std::uint32_t g = 0; g < groups; ++g)
            {
                std::uint32_t begin = collidersGroup[g];
                std::uint32_t end = collidersGroup[g + 1];
                if (colliders.objIdx[begin] != objIdx)
                    continue;
                if (begin >= end)
                    break;
                for (std::uint32_t ci = begin; ci < end; ++ci)
                {
                    const std::uint32_t bi = colliders.boneIdx[ci];
                    if (bi == UINT32_MAX)
                        continue;

                    const Vector worldPos = DirectX::XMVectorAdd(physicsBones.predPos[bi], physicsBones.dynamicLinearOffset[bi]);
                    const Quaternion worldRot = DirectX::XMQuaternionMultiply(physicsBones.predRot[bi], physicsBones.dynamicAngularOffset[bi]);
                    AABB worldAABB = colliders.boundingAABB[ci].GetWorldAABB(worldPos, worldRot, physicsBones.orgWorldScale[bi]);
                    worldAABB.Fatten(physicsBones.collisionMargin[bi]);
                    if (!isFirst)
                    {
                        bounds = worldAABB;
                        isFirst = true;
                    }
                    else
                        bounds = bounds.Merge(worldAABB);
                }
                break;
            }
        }
        else if (!physicsBonesGroup.empty()) // generate AABB that objects without collider for object culling
        {
            const std::uint32_t groups = physicsBonesGroup.size() - 1ull;
            for (std::uint32_t g = 0; g < groups; ++g)
            {
                std::uint32_t begin = physicsBonesGroup[g];
                std::uint32_t end = physicsBonesGroup[g + 1];
                if (physicsBones.objIdx[begin] != objIdx)
                    continue;
                if (begin >= end)
                    break;
                for (std::uint32_t bi = begin; bi < end; ++bi)
                {
                    const Vector worldPos = DirectX::XMVectorAdd(physicsBones.predPos[bi], physicsBones.dynamicLinearOffset[bi]);
                    AABB boneAABB(worldPos, worldPos);
                    if (!isFirst)
                    {
                        bounds = boneAABB;
                        isFirst = true;
                    }
                    else
                        bounds = bounds.Merge(boneAABB);
                }
                break;
            }
        }
        return bounds;
    }

    std::vector<AABBPair> XPBDWorld::GetAABBPairs(const std::uint32_t objIdx)
    {
        std::vector<AABBPair> pairs;
        if (objIdx == UINT32_MAX || objIdxToTreeNodeIdx[objIdx] == UINT32_MAX)
            return pairs;
        pairs.reserve(32);
        const AABB objAABB = objectDatas.boundingAABB[objIdx];
        globalAABBTree.QueryPairs(objIdx, objAABB, pairs);
        return pairs;
    }

    bool XPBDWorld::ConvexHullvsConvexHull(const std::uint32_t coiA, const std::uint32_t coiB, Internal::ContactManifold& outManifold)
    {
        const std::uint32_t biA = colliders.boneIdx[coiA];
        const std::uint32_t biB = colliders.boneIdx[coiB];

        const Vector posA = DirectX::XMVectorAdd(physicsBones.predPos[biA], physicsBones.dynamicLinearOffset[biA]);
        const Quaternion rotA = DirectX::XMQuaternionMultiply(physicsBones.predRot[biA], physicsBones.dynamicAngularOffset[biA]);
        const float scaleA = physicsBones.orgWorldScale[biA];

        const Vector posB = DirectX::XMVectorAdd(physicsBones.predPos[biB], physicsBones.dynamicLinearOffset[biB]);
        const Quaternion rotB = DirectX::XMQuaternionMultiply(physicsBones.predRot[biB], physicsBones.dynamicAngularOffset[biB]);
        const float scaleB = physicsBones.orgWorldScale[biB];

        const float marginA = physicsBones.collisionMargin[biA];
        const float marginB = physicsBones.collisionMargin[biB];
        const float sumMargin = marginA + marginB;

        const float rA = colliders.boundingSphere[coiA] * scaleA + marginA;
        const float rB = colliders.boundingSphere[coiB] * scaleB + marginB;
        const float sumR = rA + rB;

        const Vector centerA = DirectX::XMVectorAdd(posA, DirectX::XMVector3Rotate(DirectX::XMVectorScale(colliders.boundingSphereCenter[coiA], scaleA), rotA));
        const Vector centerB = DirectX::XMVectorAdd(posB, DirectX::XMVector3Rotate(DirectX::XMVectorScale(colliders.boundingSphereCenter[coiB], scaleB), rotB));
        const Vector centerToCenter = DirectX::XMVectorSubtract(centerB, centerA);
        const Vector distSq = DirectX::XMVector3LengthSq(centerToCenter);
        if (DirectX::XMVector3Less(DirectX::XMVectorReplicate(sumR * sumR), distSq))
            return false;

        if (DEBUG_MODE)
            std::atomic_ref(totalColCandidates).fetch_add(1, std::memory_order_relaxed);

        DirectX::XMFLOAT3 cA_f3, cB_f3;
        DirectX::XMStoreFloat3(&cA_f3, centerA);
        DirectX::XMStoreFloat3(&cB_f3, centerB);

        DirectX::XMFLOAT3 pA_f3, pB_f3;
        DirectX::XMStoreFloat3(&pA_f3, posA);
        DirectX::XMStoreFloat3(&pB_f3, posB);

        const Quaternion invRotA = DirectX::XMQuaternionConjugate(rotA);
        const Quaternion invRotB = DirectX::XMQuaternionConjugate(rotB);

        const auto& hullA = colliders.convexHullData[coiA];
        const auto& hullB = colliders.convexHullData[coiB];

#if defined(AVX) || defined(AVX2)
        auto hmin256_ps = [](const __m256 v) -> float {
            __m256 shuf = _mm256_permute2f128_ps(v, v, 1);
            __m256 m = _mm256_min_ps(v, shuf);
            shuf = _mm256_shuffle_ps(m, m, _MM_SHUFFLE(2, 3, 0, 1));
            m = _mm256_min_ps(m, shuf);
            shuf = _mm256_shuffle_ps(m, m, _MM_SHUFFLE(1, 0, 3, 2));
            m = _mm256_min_ps(m, shuf);
            return _mm_cvtss_f32(_mm256_castps256_ps128(m));
        };
        auto hmax256_ps = [](const __m256 v) -> float {
            __m256 shuf = _mm256_permute2f128_ps(v, v, 1);
            __m256 m = _mm256_max_ps(v, shuf);
            shuf = _mm256_shuffle_ps(m, m, _MM_SHUFFLE(2, 3, 0, 1));
            m = _mm256_max_ps(m, shuf);
            shuf = _mm256_shuffle_ps(m, m, _MM_SHUFFLE(1, 0, 3, 2));
            m = _mm256_max_ps(m, shuf);
            return _mm_cvtss_f32(_mm256_castps256_ps128(m));
        };
#endif
        float minOverlap = FLT_MAX;
        Vector bestAxis = vZero;
        bool flip = false;

        Vector hist[AXIS_HISTORY_MAX];
        std::uint32_t histCount = 0;
        auto TestAxis = [&](const Vector& inAxis) -> bool {
            const Vector lenSqV = DirectX::XMVector3LengthSq(inAxis);
            if (DirectX::XMVector3Less(lenSqV, vEpsilonSq))
                return true;

            const Vector axis = DirectX::XMVector3Normalize(inAxis);
            for (std::uint32_t i = 0; i < histCount; ++i)
            {
                const Vector dot = DirectX::XMVectorAbs(DirectX::XMVector3Dot(axis, hist[i]));
                if (DirectX::XMVector3Greater(dot, v998))
                    return true;
            }

            if (histCount < AXIS_HISTORY_MAX)
            {
                hist[histCount++] = axis;
            }
            const Vector centerDot = DirectX::XMVectorAbs(DirectX::XMVector3Dot(centerToCenter, axis));
            if (DirectX::XMVector3Less(DirectX::XMVectorReplicate(sumR), centerDot))
                return false;

            const float pAdotAx = DirectX::XMVectorGetX(DirectX::XMVector3Dot(posA, axis));
            const float pBdotAx = DirectX::XMVectorGetX(DirectX::XMVector3Dot(posB, axis));

            const Vector localA = DirectX::XMVector3Rotate(axis, invRotA);
            DirectX::XMFLOAT3 localA_f3;
            DirectX::XMStoreFloat3(&localA_f3, localA);

            const Vector localB = DirectX::XMVector3Rotate(axis, invRotB);
            DirectX::XMFLOAT3 localB_f3;
            DirectX::XMStoreFloat3(&localB_f3, localB);

#if defined(AVX512)
            const __m512 v_axA = _mm512_set1_ps(localA_f3.x);
            const __m512 v_ayA = _mm512_set1_ps(localA_f3.y);
            const __m512 v_azA = _mm512_set1_ps(localA_f3.z);

            const __m512 v_axB = _mm512_set1_ps(localB_f3.x);
            const __m512 v_ayB = _mm512_set1_ps(localB_f3.y);
            const __m512 v_azB = _mm512_set1_ps(localB_f3.z);

            __m512 dotA0 = _mm512_mul_ps(_mm512_load_ps(&hullA.vX[0]), v_axA);
            dotA0 = _mm512_fmadd_ps(_mm512_load_ps(&hullA.vY[0]), v_ayA, dotA0);
            dotA0 = _mm512_fmadd_ps(_mm512_load_ps(&hullA.vZ[0]), v_azA, dotA0);

            __m512 minA;
            __m512 maxA;

            __m512 dotB0 = _mm512_mul_ps(_mm512_load_ps(&hullB.vX[0]), v_axB);
            dotB0 = _mm512_fmadd_ps(_mm512_load_ps(&hullB.vY[0]), v_ayB, dotB0);
            dotB0 = _mm512_fmadd_ps(_mm512_load_ps(&hullB.vZ[0]), v_azB, dotB0);

            __m512 minB;
            __m512 maxB;

            if (COL_VERTEX_MAX > 16)
            {
                __m512 dotA1 = _mm512_mul_ps(_mm512_load_ps(&hullA.vX[16]), v_axA);
                dotA1 = _mm512_fmadd_ps(_mm512_load_ps(&hullA.vY[16]), v_ayA, dotA1);
                dotA1 = _mm512_fmadd_ps(_mm512_load_ps(&hullA.vZ[16]), v_azA, dotA1);

                minA = _mm512_min_ps(dotA0, dotA1);
                maxA = _mm512_max_ps(dotA0, dotA1);

                __m512 dotB1 = _mm512_mul_ps(_mm512_load_ps(&hullB.vX[16]), v_axB);
                dotB1 = _mm512_fmadd_ps(_mm512_load_ps(&hullB.vY[16]), v_ayB, dotB1);
                dotB1 = _mm512_fmadd_ps(_mm512_load_ps(&hullB.vZ[16]), v_azB, dotB1);

                minB = _mm512_min_ps(dotB0, dotB1);
                maxB = _mm512_max_ps(dotB0, dotB1);
            }
            else
            {
                minA = dotA0;
                maxA = dotA0;
                minB = dotB0;
                maxB = dotB0;
            }

            for (std::uint32_t i = 32; i < COL_VERTEX_MAX; i += 16)
            {
                __m512 dotA = _mm512_mul_ps(_mm512_load_ps(&hullA.vX[i]), v_axA);
                dotA = _mm512_fmadd_ps(_mm512_load_ps(&hullA.vY[i]), v_ayA, dotA);
                dotA = _mm512_fmadd_ps(_mm512_load_ps(&hullA.vZ[i]), v_azA, dotA);

                minA = _mm512_min_ps(minA, dotA);
                maxA = _mm512_max_ps(maxA, dotA);

                __m512 dotB = _mm512_mul_ps(_mm512_load_ps(&hullB.vX[i]), v_axB);
                dotB = _mm512_fmadd_ps(_mm512_load_ps(&hullB.vY[i]), v_ayB, dotB);
                dotB = _mm512_fmadd_ps(_mm512_load_ps(&hullB.vZ[i]), v_azB, dotB);

                minB = _mm512_min_ps(minB, dotB);
                maxB = _mm512_max_ps(maxB, dotB);
            }

            const float fminA = _mm512_reduce_min_ps(minA) * scaleA + pAdotAx - marginA;
            const float fmaxA = _mm512_reduce_max_ps(maxA) * scaleA + pAdotAx + marginA;
            const float fminB = _mm512_reduce_min_ps(minB) * scaleB + pBdotAx - marginB;
            const float fmaxB = _mm512_reduce_max_ps(maxB) * scaleB + pBdotAx + marginB;

#elif defined(AVX2)
            const __m256 v_axA = _mm256_set1_ps(localA_f3.x);
            const __m256 v_ayA = _mm256_set1_ps(localA_f3.y);
            const __m256 v_azA = _mm256_set1_ps(localA_f3.z);

            const __m256 v_axB = _mm256_set1_ps(localB_f3.x);
            const __m256 v_ayB = _mm256_set1_ps(localB_f3.y);
            const __m256 v_azB = _mm256_set1_ps(localB_f3.z);

            __m256 dotA0 = _mm256_mul_ps(_mm256_load_ps(&hullA.vX[0]), v_axA);
            dotA0 = _mm256_fmadd_ps(_mm256_load_ps(&hullA.vY[0]), v_ayA, dotA0);
            dotA0 = _mm256_fmadd_ps(_mm256_load_ps(&hullA.vZ[0]), v_azA, dotA0);

            __m256 dotA1 = _mm256_mul_ps(_mm256_load_ps(&hullA.vX[8]), v_axA);
            dotA1 = _mm256_fmadd_ps(_mm256_load_ps(&hullA.vY[8]), v_ayA, dotA1);
            dotA1 = _mm256_fmadd_ps(_mm256_load_ps(&hullA.vZ[8]), v_azA, dotA1);

            __m256 minA = _mm256_min_ps(dotA0, dotA1);
            __m256 maxA = _mm256_max_ps(dotA0, dotA1);

            __m256 dotB0 = _mm256_mul_ps(_mm256_load_ps(&hullB.vX[0]), v_axB);
            dotB0 = _mm256_fmadd_ps(_mm256_load_ps(&hullB.vY[0]), v_ayB, dotB0);
            dotB0 = _mm256_fmadd_ps(_mm256_load_ps(&hullB.vZ[0]), v_azB, dotB0);

            __m256 dotB1 = _mm256_mul_ps(_mm256_load_ps(&hullB.vX[8]), v_axB);
            dotB1 = _mm256_fmadd_ps(_mm256_load_ps(&hullB.vY[8]), v_ayB, dotB1);
            dotB1 = _mm256_fmadd_ps(_mm256_load_ps(&hullB.vZ[8]), v_azB, dotB1);

            __m256 minB = _mm256_min_ps(dotB0, dotB1);
            __m256 maxB = _mm256_max_ps(dotB0, dotB1);

            for (std::uint32_t i = 16; i < COL_VERTEX_MAX; i += 8)
            {
                __m256 dotA = _mm256_mul_ps(_mm256_load_ps(&hullA.vX[i]), v_axA);
                dotA = _mm256_fmadd_ps(_mm256_load_ps(&hullA.vY[i]), v_ayA, dotA);
                dotA = _mm256_fmadd_ps(_mm256_load_ps(&hullA.vZ[i]), v_azA, dotA);

                minA = _mm256_min_ps(dotA, minA);
                maxA = _mm256_max_ps(dotA, maxA);

                __m256 dotB = _mm256_mul_ps(_mm256_load_ps(&hullB.vX[i]), v_axB);
                dotB = _mm256_fmadd_ps(_mm256_load_ps(&hullB.vY[i]), v_ayB, dotB);
                dotB = _mm256_fmadd_ps(_mm256_load_ps(&hullB.vZ[i]), v_azB, dotB);

                minB = _mm256_min_ps(dotB, minB);
                maxB = _mm256_max_ps(dotB, maxB);
            }

            const float fminA = hmin256_ps(minA) * scaleA + pAdotAx - marginA;
            const float fmaxA = hmax256_ps(maxA) * scaleA + pAdotAx + marginA;
            const float fminB = hmin256_ps(minB) * scaleB + pBdotAx - marginB;
            const float fmaxB = hmax256_ps(maxB) * scaleB + pBdotAx + marginB;

#elif defined(AVX)
            const __m256 v_axA = _mm256_set1_ps(localA_f3.x);
            const __m256 v_ayA = _mm256_set1_ps(localA_f3.y);
            const __m256 v_azA = _mm256_set1_ps(localA_f3.z);

            const __m256 v_axB = _mm256_set1_ps(localB_f3.x);
            const __m256 v_ayB = _mm256_set1_ps(localB_f3.y);
            const __m256 v_azB = _mm256_set1_ps(localB_f3.z);

            const __m256 dotA0 = _mm256_add_ps(
                _mm256_add_ps(_mm256_mul_ps(_mm256_load_ps(&hullA.vX[0]), v_axA),
                              _mm256_mul_ps(_mm256_load_ps(&hullA.vY[0]), v_ayA)),
                _mm256_mul_ps(_mm256_load_ps(&hullA.vZ[0]), v_azA));

            const __m256 dotA1 = _mm256_add_ps(
                _mm256_add_ps(_mm256_mul_ps(_mm256_load_ps(&hullA.vX[8]), v_axA),
                              _mm256_mul_ps(_mm256_load_ps(&hullA.vY[8]), v_ayA)),
                _mm256_mul_ps(_mm256_load_ps(&hullA.vZ[8]), v_azA));

            __m256 minA = _mm256_min_ps(dotA0, dotA1);
            __m256 maxA = _mm256_max_ps(dotA0, dotA1);

            const __m256 dotB0 = _mm256_add_ps(
                _mm256_add_ps(_mm256_mul_ps(_mm256_load_ps(&hullB.vX[0]), v_axB),
                              _mm256_mul_ps(_mm256_load_ps(&hullB.vY[0]), v_ayB)),
                _mm256_mul_ps(_mm256_load_ps(&hullB.vZ[0]), v_azB));

            const __m256 dotB1 = _mm256_add_ps(
                _mm256_add_ps(_mm256_mul_ps(_mm256_load_ps(&hullB.vX[8]), v_axB),
                              _mm256_mul_ps(_mm256_load_ps(&hullB.vY[8]), v_ayB)),
                _mm256_mul_ps(_mm256_load_ps(&hullB.vZ[8]), v_azB));

            __m256 minB = _mm256_min_ps(dotB0, dotB1);
            __m256 maxB = _mm256_max_ps(dotB0, dotB1);

            for (std::uint32_t i = 16; i < COL_VERTEX_MAX; i += 8)
            {
                const __m256 dotA = _mm256_add_ps(
                    _mm256_add_ps(_mm256_mul_ps(_mm256_load_ps(&hullA.vX[i]), v_axA),
                                  _mm256_mul_ps(_mm256_load_ps(&hullA.vY[i]), v_ayA)),
                    _mm256_mul_ps(_mm256_load_ps(&hullA.vZ[i]), v_azA));

                minA = _mm256_min_ps(dotA, minA);
                maxA = _mm256_max_ps(dotA, maxA);

                const __m256 dotB = _mm256_add_ps(
                    _mm256_add_ps(_mm256_mul_ps(_mm256_load_ps(&hullB.vX[i]), v_axB),
                                  _mm256_mul_ps(_mm256_load_ps(&hullB.vY[i]), v_ayB)),
                    _mm256_mul_ps(_mm256_load_ps(&hullB.vZ[i]), v_azB));

                minB = _mm256_min_ps(dotB, minB);
                maxB = _mm256_max_ps(dotB, maxB);
            }

            const float fminA = hmin256_ps(minA) * scaleA + pAdotAx - marginA;
            const float fmaxA = hmax256_ps(maxA) * scaleA + pAdotAx + marginA;
            const float fminB = hmin256_ps(minB) * scaleB + pBdotAx - marginB;
            const float fmaxB = hmax256_ps(maxB) * scaleB + pBdotAx + marginB;
#endif

            if (fmaxA < fminB || fmaxB < fminA)
                return false;

            const float overlap1 = fmaxA - fminB;
            const float overlap2 = fmaxB - fminA;
            if (overlap1 < 0.0f || overlap2 < 0.0f)
                return false;

            const float overlap = std::min(overlap1, overlap2);
            if (overlap < minOverlap)
            {
                minOverlap = overlap;
                bestAxis = axis;
                flip = (overlap1 < overlap2);
            }
            return true;
        };

        auto& cache = convexHullCache[Internal::GetCacheKey(coiA, coiB)];
        if (cache.lastFrame == currentFrame - 1ull)
        {
            if (!TestAxis(cache.axis))
                return false;
        }

        const std::uint32_t fCount = std::max(hullA.faceCount, hullB.faceCount);
        for (std::uint32_t i = 0; i < fCount; ++i)
        {
            const Vector nA = DirectX::XMVector3Rotate(DirectX::XMVectorSet(hullA.fX[i], hullA.fY[i], hullA.fZ[i], 0), rotA);
            if (DirectX::XMVector3LessOrEqual(DirectX::XMVector3Dot(nA, centerToCenter), vEpsilon))
                continue;
            if (!TestAxis(nA))
                return false;

            const Vector nB = DirectX::XMVector3Rotate(DirectX::XMVectorSet(hullB.fX[i], hullB.fY[i], hullB.fZ[i], 0), rotB);
            if (!TestAxis(nB))
                return false;
        }

        Vector wEdgeB[COL_EDGE_MAX];
        bool wEdgeBValid[COL_EDGE_MAX] = {false};
        const std::uint32_t eCountB = hullB.edgeCount;
        for (std::uint32_t i = 0; i < eCountB; ++i)
        {
            const Vector eB = DirectX::XMVectorSet(hullB.eX[i], hullB.eY[i], hullB.eZ[i], 0);
            wEdgeB[i] = DirectX::XMVector3Rotate(eB, rotB);
            if (DirectX::XMVector3LessOrEqual(DirectX::XMVector3LengthSq(wEdgeB[i]), vEpsilonSq))
                continue;
            wEdgeBValid[i] = true;
        }

        const std::uint32_t eCountA = hullA.edgeCount;
        for (std::uint32_t eiA = 0; eiA < eCountA; ++eiA)
        {
            const Vector eA = DirectX::XMVectorSet(hullA.eX[eiA], hullA.eY[eiA], hullA.eZ[eiA], 0);
            const Vector wA = DirectX::XMVector3Rotate(eA, rotA);

            if (DirectX::XMVector3LessOrEqual(DirectX::XMVector3LengthSq(wA), vEpsilonSq))
                continue;

            for (std::uint32_t eiB = 0; eiB < eCountB; ++eiB)
            {
                if (!wEdgeBValid[eiB])
                    continue;
                const Vector crossAxis = DirectX::XMVector3Cross(wA, wEdgeB[eiB]);
                if (DirectX::XMVector3LessOrEqual(DirectX::XMVector3LengthSq(crossAxis), vEpsilonSq))
                    continue;
                if (!TestAxis(crossAxis))
                    return false;
            }
        }

        const Vector normal = flip ? DirectX::XMVectorNegate(bestAxis) : bestAxis;
        cache.axis = bestAxis;
        cache.lastFrame = currentFrame;

        // manifold
        Internal::ContactManifold::ContactPoint tempPoints[5];
        std::int32_t tempCount = 0;
        const Vector vSumMargin = DirectX::XMVectorReplicate(-sumMargin);
        for (std::uint32_t i = 0; i < cache.persistentManifold.pointCount; ++i)
        {
            const auto& cp = cache.persistentManifold.points[i];
            const Vector wA = DirectX::XMVectorAdd(posA, DirectX::XMVector3Rotate(DirectX::XMVectorScale(cp.localPointA, scaleA), rotA));
            const Vector wB = DirectX::XMVectorAdd(posB, DirectX::XMVector3Rotate(DirectX::XMVectorScale(cp.localPointB, scaleB), rotB));

            const Vector diff = DirectX::XMVectorSubtract(wA, wB);
            const Vector currentDepth = DirectX::XMVectorNegate(DirectX::XMVector3Dot(diff, normal));
            if (DirectX::XMVector3Less(currentDepth, vSumMargin))
                continue;

            const Vector projDiff = DirectX::XMVectorAdd(diff, DirectX::XMVectorMultiply(normal, currentDepth));
            const Vector driftSq = DirectX::XMVector3LengthSq(projDiff);
            if (DirectX::XMVector3Greater(driftSq, vBreakThresholdSq))
                continue;

            tempPoints[tempCount] = cp;
            tempPoints[tempCount].depth = DirectX::XMVectorGetX(currentDepth);
            tempCount++;
        }

        const std::uint32_t vCountA = hullA.vertexCount;
        const std::uint32_t vCountB = hullB.vertexCount;

        const Vector localNormalA = DirectX::XMVector3Rotate(normal, invRotA);
        DirectX::XMFLOAT3 lnA_f3;
        DirectX::XMStoreFloat3(&lnA_f3, localNormalA);
        const float posDotA = DirectX::XMVectorGetX(DirectX::XMVector3Dot(posA, normal));

        const Vector localNormalB = DirectX::XMVector3Rotate(normal, invRotB);
        DirectX::XMFLOAT3 lnB_f3;
        DirectX::XMStoreFloat3(&lnB_f3, localNormalB);
        const float posDotB = DirectX::XMVectorGetX(DirectX::XMVector3Dot(posB, normal));
        const float tolerance = 0.02f + sumMargin;

        float frameMaxDepth = -1.0f;
        Vector frameBestLA = vZero;
        Vector frameBestLB = vZero;
        auto AddTempPoint = [&](const Vector& lA, const Vector& lB, const float depth) {
            if (depth > frameMaxDepth)
            {
                frameMaxDepth = depth;
                frameBestLA = lA;
                frameBestLB = lB;
            }
        };

        const Matrix matA = ToMatrix(rotA);
        const Matrix matB = ToMatrix(rotB);
        const Matrix matInvA = ToMatrix(invRotA);
        const Matrix matInvB = ToMatrix(invRotB);

        const float invScaleA = reciprocal(scaleA);
        const float invScaleB = reciprocal(scaleB);

#if defined(AVX512)
        const __m512 v_lnAx = _mm512_set1_ps(lnA_f3.x);
        const __m512 v_lnAy = _mm512_set1_ps(lnA_f3.y);
        const __m512 v_lnAz = _mm512_set1_ps(lnA_f3.z);
        const __m512 v_scaleA = _mm512_set1_ps(scaleA);
        const __m512 v_posDotA = _mm512_set1_ps(posDotA);

        __m512 v_minDotA = _mm512_set1_ps(FLT_MAX);
        for (std::uint32_t i = 0; i < COL_VERTEX_MAX; i += 16)
        {
            const __m512 vx = _mm512_load_ps(&hullA.vX[i]);
            const __m512 vy = _mm512_load_ps(&hullA.vY[i]);
            const __m512 vz = _mm512_load_ps(&hullA.vZ[i]);

            __m512 dot = _mm512_fmadd_ps(vz, v_lnAz, _mm512_fmadd_ps(vy, v_lnAy, _mm512_mul_ps(vx, v_lnAx)));
            dot = _mm512_fmadd_ps(dot, v_scaleA, v_posDotA);
            v_minDotA = _mm512_min_ps(v_minDotA, dot);
        }
        const float minDotA = _mm512_reduce_min_ps(v_minDotA);

        const __m512 v_lnBx = _mm512_set1_ps(lnB_f3.x);
        const __m512 v_lnBy = _mm512_set1_ps(lnB_f3.y);
        const __m512 v_lnBz = _mm512_set1_ps(lnB_f3.z);
        const __m512 v_scaleB = _mm512_set1_ps(scaleB);
        const __m512 v_posDotB = _mm512_set1_ps(posDotB);

        __m512 v_maxDotB = _mm512_set1_ps(-FLT_MAX);
        for (std::uint32_t i = 0; i < COL_VERTEX_MAX; i += 16)
        {
            const __m512 vx = _mm512_load_ps(&hullB.vX[i]);
            const __m512 vy = _mm512_load_ps(&hullB.vY[i]);
            const __m512 vz = _mm512_load_ps(&hullB.vZ[i]);

            __m512 dot = _mm512_fmadd_ps(vz, v_lnBz, _mm512_fmadd_ps(vy, v_lnBy, _mm512_mul_ps(vx, v_lnBx)));
            dot = _mm512_fmadd_ps(dot, v_scaleB, v_posDotB);
            v_maxDotB = _mm512_max_ps(v_maxDotB, dot);
        }
        const float maxDotB = _mm512_reduce_max_ps(v_maxDotB);

        const __m512 v_tolA = _mm512_set1_ps(minDotA + tolerance);
        const __m512 v_tolB = _mm512_set1_ps(maxDotB - tolerance);

        float bestPenA = -1.0f;
        float bestPenB = -1.0f;
        int bestIdxA = -1;
        int bestIdxB = -1;

        for (std::uint32_t i = 0; i < COL_VERTEX_MAX; i += 16)
        {
            // hullA
            {
                const __m512 vx = _mm512_load_ps(&hullA.vX[i]);
                const __m512 vy = _mm512_load_ps(&hullA.vY[i]);
                const __m512 vz = _mm512_load_ps(&hullA.vZ[i]);
                __m512 dot = _mm512_fmadd_ps(vz, v_lnAz, _mm512_fmadd_ps(vy, v_lnAy, _mm512_mul_ps(vx, v_lnAx)));
                dot = _mm512_fmadd_ps(dot, v_scaleA, v_posDotA);
                __mmask16 mask = _mm512_cmp_ps_mask(dot, v_tolA, _CMP_LE_OQ);

                if (mask)
                {
                    const __m512 v_pen = _mm512_sub_ps(_mm512_set1_ps(maxDotB), dot);
                    alignas(64) float pens[16];
                    _mm512_store_ps(pens, v_pen);

                    std::uint32_t mask32 = static_cast<std::uint32_t>(mask);
                    while (mask32)
                    {
                        const std::uint32_t k = _tzcnt_u32(mask32);
                        mask32 &= mask32 - 1;

                        const std::uint32_t idx = i + k;
                        if (idx < vCountA && pens[k] > bestPenA)
                        {
                            bestPenA = pens[k];
                            bestIdxA = idx;
                        }
                    }
                }
            }

            // hullB
            {
                const __m512 vx = _mm512_load_ps(&hullB.vX[i]);
                const __m512 vy = _mm512_load_ps(&hullB.vY[i]);
                const __m512 vz = _mm512_load_ps(&hullB.vZ[i]);
                __m512 dot = _mm512_fmadd_ps(vz, v_lnBz, _mm512_fmadd_ps(vy, v_lnBy, _mm512_mul_ps(vx, v_lnBx)));
                dot = _mm512_fmadd_ps(dot, v_scaleB, v_posDotB);

                __mmask16 mask = _mm512_cmp_ps_mask(dot, v_tolB, _CMP_GE_OQ);

                if (mask)
                {
                    const __m512 v_pen = _mm512_sub_ps(dot, _mm512_set1_ps(minDotA));
                    alignas(64) float pens[16];
                    _mm512_store_ps(pens, v_pen);

                    std::uint32_t mask32 = static_cast<std::uint32_t>(mask);
                    while (mask32)
                    {
                        const std::uint32_t k = _tzcnt_u32(mask32);
                        mask32 &= mask32 - 1;

                        const std::uint32_t idx = i + k;
                        if (idx < vCountB && pens[k] > bestPenB)
                        {
                            bestPenB = pens[k];
                            bestIdxB = idx;
                        }
                    }
                }
            }
        }

        if (bestIdxA != -1 && bestPenA > 0.0f)
        {
            const Vector lA = DirectX::XMVectorSet(hullA.vX[bestIdxA], hullA.vY[bestIdxA], hullA.vZ[bestIdxA], 0.0f);
            const Vector rot_lA = DirectX::XMVector3TransformNormal(lA, matA);
            const Vector worldA_i = DirectX::XMVectorAdd(posA, DirectX::XMVectorScale(rot_lA, scaleA));

            const Vector wB = DirectX::XMVectorAdd(worldA_i, DirectX::XMVectorScale(normal, bestPenA));
            const Vector diffB = DirectX::XMVectorSubtract(wB, posB);
            const Vector lB = DirectX::XMVectorScale(DirectX::XMVector3TransformNormal(diffB, matInvB), invScaleB);

            AddTempPoint(lA, lB, bestPenA);
        }
        if (bestIdxB != -1 && bestPenB > 0.0f)
        {
            const Vector lB = DirectX::XMVectorSet(hullB.vX[bestIdxB], hullB.vY[bestIdxB], hullB.vZ[bestIdxB], 0.0f);
            const Vector rot_lB = DirectX::XMVector3TransformNormal(lB, matB);
            const Vector worldB_i = DirectX::XMVectorAdd(posB, DirectX::XMVectorScale(rot_lB, scaleB));

            const Vector wA = DirectX::XMVectorSubtract(worldB_i, DirectX::XMVectorScale(normal, bestPenB));
            const Vector diffA = DirectX::XMVectorSubtract(wA, posA);
            const Vector lA = DirectX::XMVectorScale(DirectX::XMVector3TransformNormal(diffA, matInvA), invScaleA);

            AddTempPoint(lA, lB, bestPenB);
        }

#elif defined(AVX2)
        const __m256 v_lnAx = _mm256_set1_ps(lnA_f3.x);
        const __m256 v_lnAy = _mm256_set1_ps(lnA_f3.y);
        const __m256 v_lnAz = _mm256_set1_ps(lnA_f3.z);
        const __m256 v_scaleA = _mm256_set1_ps(scaleA);
        const __m256 v_posDotA = _mm256_set1_ps(posDotA);

        __m256 v_minDotA = _mm256_set1_ps(FLT_MAX);
        for (std::uint32_t i = 0; i < COL_VERTEX_MAX; i += 8)
        {
            const __m256 vx = _mm256_load_ps(&hullA.vX[i]);
            const __m256 vy = _mm256_load_ps(&hullA.vY[i]);
            const __m256 vz = _mm256_load_ps(&hullA.vZ[i]);
            __m256 dot = _mm256_fmadd_ps(vz, v_lnAz, _mm256_fmadd_ps(vy, v_lnAy, _mm256_mul_ps(vx, v_lnAx)));
            dot = _mm256_fmadd_ps(dot, v_scaleA, v_posDotA);
            v_minDotA = _mm256_min_ps(v_minDotA, dot);
        }
        const float minDotA = hmin256_ps(v_minDotA);

        const __m256 v_lnBx = _mm256_set1_ps(lnB_f3.x);
        const __m256 v_lnBy = _mm256_set1_ps(lnB_f3.y);
        const __m256 v_lnBz = _mm256_set1_ps(lnB_f3.z);
        const __m256 v_scaleB = _mm256_set1_ps(scaleB);
        const __m256 v_posDotB = _mm256_set1_ps(posDotB);

        __m256 v_maxDotB = _mm256_set1_ps(-FLT_MAX);
        for (std::uint32_t i = 0; i < COL_VERTEX_MAX; i += 8)
        {
            const __m256 vx = _mm256_load_ps(&hullB.vX[i]);
            const __m256 vy = _mm256_load_ps(&hullB.vY[i]);
            const __m256 vz = _mm256_load_ps(&hullB.vZ[i]);
            __m256 dot = _mm256_fmadd_ps(vz, v_lnBz, _mm256_fmadd_ps(vy, v_lnBy, _mm256_mul_ps(vx, v_lnBx)));
            dot = _mm256_fmadd_ps(dot, v_scaleB, v_posDotB);
            v_maxDotB = _mm256_max_ps(v_maxDotB, dot);
        }
        const float maxDotB = hmax256_ps(v_maxDotB);

        const __m256 v_tolA = _mm256_set1_ps(minDotA + tolerance);
        const __m256 v_tolB = _mm256_set1_ps(maxDotB - tolerance);

        float bestPenA = -1.0f;
        float bestPenB = -1.0f;
        int bestIdxA = -1;
        int bestIdxB = -1;

        for (std::uint32_t i = 0; i < COL_VERTEX_MAX; i += 8)
        {
            // hullA
            {
                const __m256 vx = _mm256_load_ps(&hullA.vX[i]);
                const __m256 vy = _mm256_load_ps(&hullA.vY[i]);
                const __m256 vz = _mm256_load_ps(&hullA.vZ[i]);
                __m256 dot = _mm256_fmadd_ps(vz, v_lnAz, _mm256_fmadd_ps(vy, v_lnAy, _mm256_mul_ps(vx, v_lnAx)));
                dot = _mm256_fmadd_ps(dot, v_scaleA, v_posDotA);

                const __m256 cmp = _mm256_cmp_ps(dot, v_tolA, _CMP_LE_OQ);
                std::uint32_t mask = _mm256_movemask_ps(cmp);
                if (mask)
                {
                    const __m256 v_pen = _mm256_sub_ps(_mm256_set1_ps(maxDotB), dot);
                    alignas(32) float pens[8];
                    _mm256_store_ps(pens, v_pen);
                    while (mask)
                    {
                        const std::uint32_t k = _tzcnt_u32(mask);
                        mask &= mask - 1;

                        const std::uint32_t idx = i + k;
                        if (idx < vCountA && pens[k] > bestPenA)
                        {
                            bestPenA = pens[k];
                            bestIdxA = idx;
                        }
                    }
                }
            }

            // hullB
            {
                const __m256 vx = _mm256_load_ps(&hullB.vX[i]);
                const __m256 vy = _mm256_load_ps(&hullB.vY[i]);
                const __m256 vz = _mm256_load_ps(&hullB.vZ[i]);
                __m256 dot = _mm256_fmadd_ps(vz, v_lnBz, _mm256_fmadd_ps(vy, v_lnBy, _mm256_mul_ps(vx, v_lnBx)));
                dot = _mm256_fmadd_ps(dot, v_scaleB, v_posDotB);

                const __m256 cmp = _mm256_cmp_ps(dot, v_tolB, _CMP_GE_OQ);
                std::uint32_t mask = _mm256_movemask_ps(cmp);
                if (mask == 0)
                    continue;
                {
                    const __m256 v_pen = _mm256_sub_ps(dot, _mm256_set1_ps(minDotA));
                    alignas(32) float pens[8];
                    _mm256_store_ps(pens, v_pen);
                    while (mask)
                    {
                        const std::uint32_t k = _tzcnt_u32(mask);
                        mask &= mask - 1;

                        const std::uint32_t idx = i + k;
                        if (idx < vCountB && pens[k] > bestPenB)
                        {
                            bestPenB = pens[k];
                            bestIdxB = idx;
                        }
                    }
                }
            }
        }

        if (bestIdxA != -1 && bestPenA > 0.0f)
        {
            const Vector lA = DirectX::XMVectorSet(hullA.vX[bestIdxA], hullA.vY[bestIdxA], hullA.vZ[bestIdxA], 0.0f);
            const Vector rot_lA = DirectX::XMVector3TransformNormal(lA, matA);
            const Vector worldA_i = DirectX::XMVectorAdd(posA, DirectX::XMVectorScale(rot_lA, scaleA));
            const Vector wB = DirectX::XMVectorAdd(worldA_i, DirectX::XMVectorScale(normal, bestPenA));
            const Vector diffB = DirectX::XMVectorSubtract(wB, posB);
            const Vector lB = DirectX::XMVectorScale(DirectX::XMVector3TransformNormal(diffB, matInvB), invScaleB);
            AddTempPoint(lA, lB, bestPenA);
        }
        if (bestIdxB != -1 && bestPenB > 0.0f)
        {
            const Vector lB = DirectX::XMVectorSet(hullB.vX[bestIdxB], hullB.vY[bestIdxB], hullB.vZ[bestIdxB], 0.0f);
            const Vector rot_lB = DirectX::XMVector3TransformNormal(lB, matB);
            const Vector worldB_i = DirectX::XMVectorAdd(posB, DirectX::XMVectorScale(rot_lB, scaleB));
            const Vector wA = DirectX::XMVectorSubtract(worldB_i, DirectX::XMVectorScale(normal, bestPenB));
            const Vector diffA = DirectX::XMVectorSubtract(wA, posA);
            const Vector lA = DirectX::XMVectorScale(DirectX::XMVector3TransformNormal(diffA, matInvA), invScaleA);
            AddTempPoint(lA, lB, bestPenB);
        }

#elif defined(AVX)
        const __m256 v_lnAx = _mm256_set1_ps(lnA_f3.x);
        const __m256 v_lnAy = _mm256_set1_ps(lnA_f3.y);
        const __m256 v_lnAz = _mm256_set1_ps(lnA_f3.z);
        const __m256 v_scaleA = _mm256_set1_ps(scaleA);
        const __m256 v_posDotA = _mm256_set1_ps(posDotA);

        __m256 v_minDotA = _mm256_set1_ps(FLT_MAX);
        for (std::uint32_t i = 0; i < COL_VERTEX_MAX; i += 8)
        {
            const __m256 vx = _mm256_load_ps(&hullA.vX[i]);
            const __m256 vy = _mm256_load_ps(&hullA.vY[i]);
            const __m256 vz = _mm256_load_ps(&hullA.vZ[i]);

            __m256 dot = _mm256_add_ps(_mm256_add_ps(_mm256_mul_ps(vx, v_lnAx), _mm256_mul_ps(vy, v_lnAy)), _mm256_mul_ps(vz, v_lnAz));
            dot = _mm256_add_ps(_mm256_mul_ps(dot, v_scaleA), v_posDotA);
            v_minDotA = _mm256_min_ps(v_minDotA, dot);
        }
        const float minDotA = hmin256_ps(v_minDotA);

        const __m256 v_lnBx = _mm256_set1_ps(lnB_f3.x);
        const __m256 v_lnBy = _mm256_set1_ps(lnB_f3.y);
        const __m256 v_lnBz = _mm256_set1_ps(lnB_f3.z);
        const __m256 v_scaleB = _mm256_set1_ps(scaleB);
        const __m256 v_posDotB = _mm256_set1_ps(posDotB);

        __m256 v_maxDotB = _mm256_set1_ps(-FLT_MAX);
        for (std::uint32_t i = 0; i < COL_VERTEX_MAX; i += 8)
        {
            const __m256 vx = _mm256_load_ps(&hullB.vX[i]);
            const __m256 vy = _mm256_load_ps(&hullB.vY[i]);
            const __m256 vz = _mm256_load_ps(&hullB.vZ[i]);

            __m256 dot = _mm256_add_ps(_mm256_add_ps(_mm256_mul_ps(vx, v_lnBx), _mm256_mul_ps(vy, v_lnBy)), _mm256_mul_ps(vz, v_lnBz));
            dot = _mm256_add_ps(_mm256_mul_ps(dot, v_scaleB), v_posDotB);
            v_maxDotB = _mm256_max_ps(v_maxDotB, dot);
        }
        const float maxDotB = hmax256_ps(v_maxDotB);

        const __m256 v_tolA = _mm256_set1_ps(minDotA + tolerance);
        const __m256 v_tolB = _mm256_set1_ps(maxDotB - tolerance);

        float bestPenA = -1.0f;
        float bestPenB = -1.0f;
        int bestIdxA = -1;
        int bestIdxB = -1;
        for (std::uint32_t i = 0; i < COL_VERTEX_MAX; i += 8)
        {
            // hullA
            {
                const __m256 vx = _mm256_load_ps(&hullA.vX[i]);
                const __m256 vy = _mm256_load_ps(&hullA.vY[i]);
                const __m256 vz = _mm256_load_ps(&hullA.vZ[i]);
                __m256 dot = _mm256_add_ps(_mm256_add_ps(_mm256_mul_ps(vx, v_lnAx), _mm256_mul_ps(vy, v_lnAy)), _mm256_mul_ps(vz, v_lnAz));
                dot = _mm256_add_ps(_mm256_mul_ps(dot, v_scaleA), v_posDotA);

                const __m256 cmp = _mm256_cmp_ps(dot, v_tolA, _CMP_LE_OQ);
                std::uint32_t mask = _mm256_movemask_ps(cmp);
                if (mask)
                {
                    const __m256 v_pen = _mm256_sub_ps(_mm256_set1_ps(maxDotB), dot);
                    alignas(32) float pens[8];
                    _mm256_store_ps(pens, v_pen);
                    while (mask)
                    {
                        const std::uint32_t k = _tzcnt_u32(mask);
                        mask &= mask - 1;
                        const std::uint32_t idx = i + k;
                        if (idx < vCountA && pens[k] > bestPenA)
                        {
                            bestPenA = pens[k];
                            bestIdxA = idx;
                        }
                    }
                }
            }

            // hullB
            {
                const __m256 vx = _mm256_load_ps(&hullB.vX[i]);
                const __m256 vy = _mm256_load_ps(&hullB.vY[i]);
                const __m256 vz = _mm256_load_ps(&hullB.vZ[i]);
                __m256 dot = _mm256_add_ps(_mm256_add_ps(_mm256_mul_ps(vx, v_lnBx), _mm256_mul_ps(vy, v_lnBy)), _mm256_mul_ps(vz, v_lnBz));
                dot = _mm256_add_ps(_mm256_mul_ps(dot, v_scaleB), v_posDotB);

                const __m256 cmp = _mm256_cmp_ps(dot, v_tolB, _CMP_GE_OQ);
                std::uint32_t mask = _mm256_movemask_ps(cmp);
                if (mask)
                {
                    const __m256 v_pen = _mm256_sub_ps(dot, _mm256_set1_ps(minDotA));
                    alignas(32) float pens[8];
                    _mm256_store_ps(pens, v_pen);
                    while (mask)
                    {
                        const std::uint32_t k = _tzcnt_u32(mask);
                        mask &= mask - 1;
                        const std::uint32_t idx = i + k;
                        if (idx < vCountB && pens[k] > bestPenB)
                        {
                            bestPenB = pens[k];
                            bestIdxB = idx;
                        }
                    }
                }
            }
        }

        if (bestIdxA != -1 && bestPenA > 0.0f)
        {
            const Vector lA = DirectX::XMVectorSet(hullA.vX[bestIdxA], hullA.vY[bestIdxA], hullA.vZ[bestIdxA], 0.0f);
            const Vector rot_lA = DirectX::XMVector3TransformNormal(lA, matA);
            const Vector worldA_i = DirectX::XMVectorAdd(posA, DirectX::XMVectorScale(rot_lA, scaleA));
            const Vector wB = DirectX::XMVectorAdd(worldA_i, DirectX::XMVectorScale(normal, bestPenA));
            const Vector diffB = DirectX::XMVectorSubtract(wB, posB);
            const Vector lB = DirectX::XMVectorScale(DirectX::XMVector3TransformNormal(diffB, matInvB), invScaleB);
            AddTempPoint(lA, lB, bestPenA);
        }
        if (bestIdxB != -1 && bestPenB > 0.0f)
        {
            const Vector lB = DirectX::XMVectorSet(hullB.vX[bestIdxB], hullB.vY[bestIdxB], hullB.vZ[bestIdxB], 0.0f);
            const Vector rot_lB = DirectX::XMVector3TransformNormal(lB, matB);
            const Vector worldB_i = DirectX::XMVectorAdd(posB, DirectX::XMVectorScale(rot_lB, scaleB));
            const Vector wA = DirectX::XMVectorSubtract(worldB_i, DirectX::XMVectorScale(normal, bestPenB));
            const Vector diffA = DirectX::XMVectorSubtract(wA, posA);
            const Vector lA = DirectX::XMVectorScale(DirectX::XMVector3TransformNormal(diffA, matInvA), invScaleA);
            AddTempPoint(lA, lB, bestPenB);
        }
#endif

        if (frameMaxDepth > 0.0f)
        {
            bool isDuplicate = false;
            for (std::int32_t i = 0; i < tempCount; ++i)
            {
                const Vector tempPointDistSq = DirectX::XMVector3LengthSq(DirectX::XMVectorSubtract(tempPoints[i].localPointA, frameBestLA));
                if (DirectX::XMVector3Less(tempPointDistSq, vContactMergeThresholdSq))
                {
                    if (frameMaxDepth > tempPoints[i].depth)
                    {
                        tempPoints[i].depth = frameMaxDepth;
                        tempPoints[i].localPointB = frameBestLB;
                    }
                    isDuplicate = true;
                    break;
                }
            }
            if (!isDuplicate && tempCount < 5)
            {
                tempPoints[tempCount++] = {frameBestLA, frameBestLB, frameMaxDepth};
            }
        }

        const std::uint32_t allowPoints = objectDatas.maxManifoldPoints[colliders.objIdx[coiA]];
        const std::uint32_t finalMaxPoints = std::min(allowPoints, 4u);
        if (tempCount <= finalMaxPoints)
        {
            cache.persistentManifold.pointCount = tempCount;
            for (std::int32_t i = 0; i < tempCount; ++i)
            {
                cache.persistentManifold.points[i] = tempPoints[i];
            }
        }
        else
        {
            std::int32_t p1 = 0, p2 = -1, p3 = -1, p4 = -1;
            Vector maxDistSq = vNegOne;
            Vector maxTriAreaSq = vNegOne;
            Vector maxQuadAreaSq = vNegOne;

            for (std::int32_t i = 1; i < tempCount; ++i)
            {
                if (tempPoints[i].depth > tempPoints[p1].depth)
                    p1 = i;
            }

            for (std::int32_t i = 0; i < tempCount; ++i)
            {
                if (i == p1)
                    continue;
                const Vector dSq = DirectX::XMVector3LengthSq(DirectX::XMVectorSubtract(tempPoints[i].localPointA, tempPoints[p1].localPointA));
                if (DirectX::XMVector3Less(maxDistSq, dSq))
                {
                    maxDistSq = dSq;
                    p2 = i;
                }
            }

            if (p2 != -1)
            {
                for (std::int32_t i = 0; i < tempCount; ++i)
                {
                    if (i == p1 || i == p2)
                        continue;
                    const Vector edge1 = DirectX::XMVectorSubtract(tempPoints[i].localPointA, tempPoints[p1].localPointA);
                    const Vector edge2 = DirectX::XMVectorSubtract(tempPoints[p2].localPointA, tempPoints[p1].localPointA);
                    const Vector areaSq = DirectX::XMVector3LengthSq(DirectX::XMVector3Cross(edge1, edge2));
                    if (DirectX::XMVector3Less(maxTriAreaSq, areaSq))
                    {
                        maxTriAreaSq = areaSq;
                        p3 = i;
                    }
                }
            }

            if (p3 != -1)
            {
                for (std::int32_t i = 0; i < tempCount; ++i)
                {
                    if (i == p1 || i == p2 || i == p3)
                        continue;
                    const Vector e1 = DirectX::XMVector3Cross(DirectX::XMVectorSubtract(tempPoints[i].localPointA, tempPoints[p1].localPointA), DirectX::XMVectorSubtract(tempPoints[p2].localPointA, tempPoints[p1].localPointA));
                    const Vector e2 = DirectX::XMVector3Cross(DirectX::XMVectorSubtract(tempPoints[i].localPointA, tempPoints[p2].localPointA), DirectX::XMVectorSubtract(tempPoints[p3].localPointA, tempPoints[p2].localPointA));
                    const Vector e3 = DirectX::XMVector3Cross(DirectX::XMVectorSubtract(tempPoints[i].localPointA, tempPoints[p3].localPointA), DirectX::XMVectorSubtract(tempPoints[p1].localPointA, tempPoints[p3].localPointA));
                    const Vector areaSqSum = DirectX::XMVectorAdd(DirectX::XMVectorAdd(DirectX::XMVector3LengthSq(e1), DirectX::XMVector3LengthSq(e2)), DirectX::XMVector3LengthSq(e3));
                    if (DirectX::XMVector3Less(maxQuadAreaSq, areaSqSum))
                    {
                        maxQuadAreaSq = areaSqSum;
                        p4 = i;
                    }
                }
            }

            std::uint32_t mfCount = 0;
            if (finalMaxPoints >= 1u)
                cache.persistentManifold.points[mfCount++] = tempPoints[p1];
            if (finalMaxPoints >= 2u && p2 != -1)
                cache.persistentManifold.points[mfCount++] = tempPoints[p2];
            if (finalMaxPoints >= 3u && p3 != -1)
                cache.persistentManifold.points[mfCount++] = tempPoints[p3];
            if (finalMaxPoints >= 4u && p4 != -1)
                cache.persistentManifold.points[mfCount++] = tempPoints[p4];
            cache.persistentManifold.pointCount = mfCount;
        }

        cache.persistentManifold.normal = normal;
        outManifold = cache.persistentManifold;
        return true;
    }

    bool XPBDWorld::ConvexHullvsSphere(const std::uint32_t coiHull, const std::uint32_t coiSphere, Internal::ContactManifold& outManifold)
    {
        const std::uint32_t biA = colliders.boneIdx[coiHull];
        const std::uint32_t biB = colliders.boneIdx[coiSphere];

        const Vector posA = DirectX::XMVectorAdd(physicsBones.predPos[biA], physicsBones.dynamicLinearOffset[biA]);
        const Quaternion rotA = DirectX::XMQuaternionMultiply(physicsBones.dynamicAngularOffset[biA], physicsBones.predRot[biA]);
        const float scaleA = physicsBones.orgWorldScale[biA];

        const Vector posB = DirectX::XMVectorAdd(physicsBones.predPos[biB], physicsBones.dynamicLinearOffset[biB]);
        const Quaternion rotB = DirectX::XMQuaternionMultiply(physicsBones.dynamicAngularOffset[biB], physicsBones.predRot[biB]);
        const float scaleB = physicsBones.orgWorldScale[biB];

        const float marginA = physicsBones.collisionMargin[biA];
        const float marginB = physicsBones.collisionMargin[biB];
        const float sumMargin = marginA + marginB;

        const float rA = colliders.boundingSphere[coiHull] * scaleA + marginA;
        const float rB = colliders.boundingSphere[coiSphere] * scaleB + marginB;
        const float sumR = rA + rB;

        const Vector centerA = DirectX::XMVectorAdd(posA, DirectX::XMVector3Rotate(DirectX::XMVectorScale(colliders.boundingSphereCenter[coiHull], scaleA), rotA));
        const Vector centerB = DirectX::XMVectorAdd(posB, DirectX::XMVector3Rotate(DirectX::XMVectorScale(colliders.boundingSphereCenter[coiSphere], scaleB), rotB));
        const Vector distSq = DirectX::XMVector3LengthSq(DirectX::XMVectorSubtract(centerB, centerA));
        if (DirectX::XMVector3Less(DirectX::XMVectorReplicate(sumR * sumR), distSq))
            return false;

        if (DEBUG_MODE)
            std::atomic_ref(totalColCandidates).fetch_add(1, std::memory_order_relaxed);

        const auto& hullA = colliders.convexHullData[coiHull];
        const auto& sphereB = colliders.sphereData[coiSphere];

        const Quaternion invRotA = DirectX::XMQuaternionConjugate(rotA);
        const Quaternion invRotB = DirectX::XMQuaternionConjugate(rotB);

#if defined(AVX) || defined(AVX2)
        auto hmin256_ps = [](const __m256 v) -> float {
            __m256 shuf = _mm256_permute2f128_ps(v, v, 1);
            __m256 m = _mm256_min_ps(v, shuf);
            shuf = _mm256_shuffle_ps(m, m, _MM_SHUFFLE(2, 3, 0, 1));
            m = _mm256_min_ps(m, shuf);
            shuf = _mm256_shuffle_ps(m, m, _MM_SHUFFLE(1, 0, 3, 2));
            m = _mm256_min_ps(m, shuf);
            return _mm_cvtss_f32(_mm256_castps256_ps128(m));
        };
        auto hmax256_ps = [](const __m256 v) -> float {
            __m256 shuf = _mm256_permute2f128_ps(v, v, 1);
            __m256 m = _mm256_max_ps(v, shuf);
            shuf = _mm256_shuffle_ps(m, m, _MM_SHUFFLE(2, 3, 0, 1));
            m = _mm256_max_ps(m, shuf);
            shuf = _mm256_shuffle_ps(m, m, _MM_SHUFFLE(1, 0, 3, 2));
            m = _mm256_max_ps(m, shuf);
            return _mm_cvtss_f32(_mm256_castps256_ps128(m));
        };
 #endif

        const Matrix matA = ToMatrix(rotA);
        const Matrix matB = ToMatrix(rotB);
        const Matrix matInvA = ToMatrix(invRotA);
        const Matrix matInvB = ToMatrix(invRotB);
        const float invScaleA = reciprocal(scaleA);
        const float invScaleB = reciprocal(scaleB);

        auto& cache = convexHullCache[Internal::GetCacheKey(coiHull, coiSphere)];

        bool collidedAny = false;
        Vector finalNormal = vZero;
        float frameMaxDepth = -1.0f;
        Vector frameBestLA = vZero;
        Vector frameBestLB = vZero;

        const float hullBoundingRadius = colliders.boundingSphere[coiHull] * scaleA + marginA;

        for (std::uint32_t sIdx = 0; sIdx < COL_SPHERE_MAX; ++sIdx)
        {
            const Vector lCenterB = DirectX::XMVectorSet(sphereB.cX[sIdx], sphereB.cY[sIdx], sphereB.cZ[sIdx], 0.0f);
            const Vector wCenterB = DirectX::XMVectorAdd(posB, DirectX::XMVector3Rotate(DirectX::XMVectorScale(lCenterB, scaleB), rotB));
            const float wRadiusB = sphereB.radius[sIdx] * scaleB + marginB;

            const Vector centerToCenter = DirectX::XMVectorSubtract(wCenterB, centerA);
            const Vector distSqVec = DirectX::XMVector3LengthSq(centerToCenter);
            const float distSq = DirectX::XMVectorGetX(distSqVec);
            const float subSumR = hullBoundingRadius + wRadiusB;
            if (distSq > subSumR * subSumR)
                continue;

            float minOverlap = FLT_MAX;
            Vector bestAxis = vZero;
            bool flip = false;

            Vector hist[AXIS_HISTORY_MAX];
            std::uint32_t histCount = 0;

            auto TestAxis = [&](const Vector& inAxis) -> bool {
                const Vector lenSqV = DirectX::XMVector3LengthSq(inAxis);
                if (DirectX::XMVector3Less(lenSqV, vEpsilonSq))
                    return true;

                const Vector axis = DirectX::XMVector3Normalize(inAxis);

                for (std::uint32_t i = 0; i < histCount; ++i)
                {
                    const Vector dot = DirectX::XMVectorAbs(DirectX::XMVector3Dot(axis, hist[i]));
                    if (DirectX::XMVector3Greater(dot, v998))
                        return true;
                }

                if (histCount < AXIS_HISTORY_MAX)
                    hist[histCount++] = axis;

                const Vector centerDot = DirectX::XMVectorAbs(DirectX::XMVector3Dot(centerToCenter, axis));
                if (DirectX::XMVector3Less(DirectX::XMVectorReplicate(sumR), centerDot))
                    return false;

                const float pAdotAx = DirectX::XMVectorGetX(DirectX::XMVector3Dot(posA, axis));
                const float pBdotAx = DirectX::XMVectorGetX(DirectX::XMVector3Dot(wCenterB, axis));

                const Vector localA = DirectX::XMVector3Rotate(axis, invRotA);
                DirectX::XMFLOAT3 localA_f3;
                DirectX::XMStoreFloat3(&localA_f3, localA);

#if defined(AVX)
                const __m256 v_axA = _mm256_set1_ps(localA_f3.x);
                const __m256 v_ayA = _mm256_set1_ps(localA_f3.y);
                const __m256 v_azA = _mm256_set1_ps(localA_f3.z);

                const __m256 dotA0 = _mm256_add_ps(
                    _mm256_add_ps(_mm256_mul_ps(_mm256_load_ps(&hullA.vX[0]), v_axA),
                                  _mm256_mul_ps(_mm256_load_ps(&hullA.vY[0]), v_ayA)),
                    _mm256_mul_ps(_mm256_load_ps(&hullA.vZ[0]), v_azA));

                const __m256 dotA1 = _mm256_add_ps(
                    _mm256_add_ps(_mm256_mul_ps(_mm256_load_ps(&hullA.vX[8]), v_axA),
                                  _mm256_mul_ps(_mm256_load_ps(&hullA.vY[8]), v_ayA)),
                    _mm256_mul_ps(_mm256_load_ps(&hullA.vZ[8]), v_azA));

                __m256 minA = _mm256_min_ps(dotA0, dotA1);
                __m256 maxA = _mm256_max_ps(dotA0, dotA1);

                for (std::uint32_t i = 16; i < COL_VERTEX_MAX; i += 8)
                {
                    const __m256 dotA = _mm256_add_ps(
                        _mm256_add_ps(_mm256_mul_ps(_mm256_load_ps(&hullA.vX[i]), v_axA),
                                      _mm256_mul_ps(_mm256_load_ps(&hullA.vY[i]), v_ayA)),
                        _mm256_mul_ps(_mm256_load_ps(&hullA.vZ[i]), v_azA));

                    minA = _mm256_min_ps(dotA, minA);
                    maxA = _mm256_max_ps(dotA, maxA);
                }

                const float fminA = hmin256_ps(minA) * scaleA + pAdotAx - marginA;
                const float fmaxA = hmax256_ps(maxA) * scaleA + pAdotAx + marginA;

#elif defined(AVX2)
                const __m256 v_axA = _mm256_set1_ps(localA_f3.x);
                const __m256 v_ayA = _mm256_set1_ps(localA_f3.y);
                const __m256 v_azA = _mm256_set1_ps(localA_f3.z);

                __m256 dotA0 = _mm256_mul_ps(_mm256_load_ps(&hullA.vX[0]), v_axA);
                dotA0 = _mm256_fmadd_ps(_mm256_load_ps(&hullA.vY[0]), v_ayA, dotA0);
                dotA0 = _mm256_fmadd_ps(_mm256_load_ps(&hullA.vZ[0]), v_azA, dotA0);

                __m256 dotA1 = _mm256_mul_ps(_mm256_load_ps(&hullA.vX[8]), v_axA);
                dotA1 = _mm256_fmadd_ps(_mm256_load_ps(&hullA.vY[8]), v_ayA, dotA1);
                dotA1 = _mm256_fmadd_ps(_mm256_load_ps(&hullA.vZ[8]), v_azA, dotA1);

                __m256 minA = _mm256_min_ps(dotA0, dotA1);
                __m256 maxA = _mm256_max_ps(dotA0, dotA1);

                for (std::uint32_t i = 16; i < COL_VERTEX_MAX; i += 8)
                {
                    __m256 dotA = _mm256_mul_ps(_mm256_load_ps(&hullA.vX[i]), v_axA);
                    dotA = _mm256_fmadd_ps(_mm256_load_ps(&hullA.vY[i]), v_ayA, dotA);
                    dotA = _mm256_fmadd_ps(_mm256_load_ps(&hullA.vZ[i]), v_azA, dotA);

                    minA = _mm256_min_ps(dotA, minA);
                    maxA = _mm256_max_ps(dotA, maxA);
                }

                const float fminA = hmin256_ps(minA) * scaleA + pAdotAx - marginA;
                const float fmaxA = hmax256_ps(maxA) * scaleA + pAdotAx + marginA;

#elif defined(AVX512)
                const __m512 v_axA = _mm512_set1_ps(localA_f3.x);
                const __m512 v_ayA = _mm512_set1_ps(localA_f3.y);
                const __m512 v_azA = _mm512_set1_ps(localA_f3.z);

                __m512 dotA0 = _mm512_mul_ps(_mm512_load_ps(&hullA.vX[0]), v_axA);
                dotA0 = _mm512_fmadd_ps(_mm512_load_ps(&hullA.vY[0]), v_ayA, dotA0);
                dotA0 = _mm512_fmadd_ps(_mm512_load_ps(&hullA.vZ[0]), v_azA, dotA0);

                __m512 minA;
                __m512 maxA;

                if (COL_VERTEX_MAX > 16)
                {
                    __m512 dotA1 = _mm512_mul_ps(_mm512_load_ps(&hullA.vX[16]), v_axA);
                    dotA1 = _mm512_fmadd_ps(_mm512_load_ps(&hullA.vY[16]), v_ayA, dotA1);
                    dotA1 = _mm512_fmadd_ps(_mm512_load_ps(&hullA.vZ[16]), v_azA, dotA1);

                    minA = _mm512_min_ps(dotA0, dotA1);
                    maxA = _mm512_max_ps(dotA0, dotA1);
                }
                else
                {
                    minA = dotA0;
                    maxA = dotA0;
                }

                for (std::uint32_t i = 32; i < COL_VERTEX_MAX; i += 16)
                {
                    __m512 dotA = _mm512_mul_ps(_mm512_load_ps(&hullA.vX[i]), v_axA);
                    dotA = _mm512_fmadd_ps(_mm512_load_ps(&hullA.vY[i]), v_ayA, dotA);
                    dotA = _mm512_fmadd_ps(_mm512_load_ps(&hullA.vZ[i]), v_azA, dotA);

                    minA = _mm512_min_ps(minA, dotA);
                    maxA = _mm512_max_ps(maxA, dotA);
                }

                const float fminA = _mm512_reduce_min_ps(minA) * scaleA + pAdotAx - marginA;
                const float fmaxA = _mm512_reduce_max_ps(maxA) * scaleA + pAdotAx + marginA;
#endif

                const float fminB = pBdotAx - wRadiusB;
                const float fmaxB = pBdotAx + wRadiusB;

                if (fmaxA < fminB || fmaxB < fminA)
                    return false;

                const float overlap1 = fmaxA - fminB;
                const float overlap2 = fmaxB - fminA;
                if (overlap1 < 0.0f || overlap2 < 0.0f)
                    return false;

                const float overlap = std::min(overlap1, overlap2);
                if (overlap < minOverlap)
                {
                    minOverlap = overlap;
                    bestAxis = axis;
                    flip = (overlap1 < overlap2);
                }
                return true;
            };

            if (cache.lastFrame == currentFrame - 1ull)
            {
                if (!TestAxis(cache.axis))
                    continue;
            }

            bool isCollide = true;
            for (std::uint32_t i = 0; i < hullA.faceCount; ++i)
            {
                const Vector nA = DirectX::XMVector3Rotate(DirectX::XMVectorSet(hullA.fX[i], hullA.fY[i], hullA.fZ[i], 0), rotA);
                if (DirectX::XMVector3LessOrEqual(DirectX::XMVector3Dot(nA, centerToCenter), vEpsilon))
                    continue;

                if (!TestAxis(nA))
                {
                    isCollide = false;
                    break;
                }
            }
            if (!isCollide)
                continue;

            for (std::uint32_t i = 0; i < hullA.vertexCount; ++i)
            {
                const Vector lV = DirectX::XMVectorSet(hullA.vX[i], hullA.vY[i], hullA.vZ[i], 0);
                const Vector wV = DirectX::XMVectorAdd(posA, DirectX::XMVector3Rotate(DirectX::XMVectorScale(lV, scaleA), rotA));
                const Vector vToS = DirectX::XMVectorSubtract(wCenterB, wV);

                if (!TestAxis(vToS))
                {
                    isCollide = false;
                    break;
                }
            }
            if (!isCollide)
                continue;

            for (std::uint32_t i = 0; i < hullA.edgeCount; ++i)
            {
                const Vector lE = DirectX::XMVectorSet(hullA.eX[i], hullA.eY[i], hullA.eZ[i], 0);
                const Vector wE = DirectX::XMVector3Rotate(lE, rotA);

                const Vector crossAxis = DirectX::XMVector3Cross(wE, centerToCenter);
                if (DirectX::XMVector3LessOrEqual(DirectX::XMVector3LengthSq(crossAxis), vEpsilonSq))
                    continue;

                if (!TestAxis(crossAxis))
                {
                    isCollide = false;
                    break;
                }
            }
            if (!isCollide)
                continue;

            collidedAny = true;
            const Vector normal = flip ? DirectX::XMVectorNegate(bestAxis) : bestAxis;

            cache.axis = bestAxis;
            cache.lastFrame = currentFrame;

            const Vector worldB_i = DirectX::XMVectorAdd(wCenterB, DirectX::XMVectorScale(normal, wRadiusB));
            const Vector worldA_i = DirectX::XMVectorSubtract(worldB_i, DirectX::XMVectorScale(normal, minOverlap));

            const Vector diffA = DirectX::XMVectorSubtract(worldA_i, posA);
            const Vector lA = DirectX::XMVectorScale(DirectX::XMVector3TransformNormal(diffA, matInvA), invScaleA);

            const Vector diffB = DirectX::XMVectorSubtract(worldB_i, posB);
            const Vector lB = DirectX::XMVectorScale(DirectX::XMVector3TransformNormal(diffB, matInvB), invScaleB);

            if (minOverlap > frameMaxDepth)
            {
                frameMaxDepth = minOverlap;
                frameBestLA = lA;
                frameBestLB = lB;
                finalNormal = normal;
            }
        }
        if (!collidedAny)
            return false;

        Internal::ContactManifold::ContactPoint tempPoints[5];
        std::int32_t tempCount = 0;
        const Vector vSumMargin = DirectX::XMVectorReplicate(-sumMargin);
        for (std::uint32_t i = 0; i < cache.persistentManifold.pointCount; ++i)
        {
            const auto& cp = cache.persistentManifold.points[i];
            const Vector wA = DirectX::XMVectorAdd(posA, DirectX::XMVector3Rotate(DirectX::XMVectorScale(cp.localPointA, scaleA), rotA));
            const Vector wB = DirectX::XMVectorAdd(posB, DirectX::XMVector3Rotate(DirectX::XMVectorScale(cp.localPointB, scaleB), rotB));

            const Vector diff = DirectX::XMVectorSubtract(wA, wB);
            const Vector currentDepth = DirectX::XMVectorNegate(DirectX::XMVector3Dot(diff, finalNormal));
            if (DirectX::XMVector3Less(currentDepth, vSumMargin))
                continue;

            const Vector projDiff = DirectX::XMVectorAdd(diff, DirectX::XMVectorMultiply(finalNormal, currentDepth));
            const Vector driftSq = DirectX::XMVector3LengthSq(projDiff);
            if (DirectX::XMVector3Greater(driftSq, vBreakThresholdSq))
                continue;

            tempPoints[tempCount] = cp;
            tempPoints[tempCount].depth = DirectX::XMVectorGetX(currentDepth);
            tempCount++;
        }
        
        if (frameMaxDepth > 0.0f)
        {
            bool isDuplicate = false;
            for (std::int32_t i = 0; i < tempCount; ++i)
            {
                const Vector tempPointDistSq = DirectX::XMVector3LengthSq(DirectX::XMVectorSubtract(tempPoints[i].localPointA, frameBestLA));
                if (DirectX::XMVector3Less(tempPointDistSq, vContactMergeThresholdSq))
                {
                    if (frameMaxDepth > tempPoints[i].depth)
                    {
                        tempPoints[i].depth = frameMaxDepth;
                        tempPoints[i].localPointB = frameBestLB;
                    }
                    isDuplicate = true;
                    break;
                }
            }
            if (!isDuplicate && tempCount < 5)
            {
                tempPoints[tempCount++] = {frameBestLA, frameBestLB, frameMaxDepth};
            }
        }

        const std::uint32_t allowPoints = objectDatas.maxManifoldPoints[colliders.objIdx[coiHull]];
        const std::uint32_t finalMaxPoints = std::min(allowPoints, 4u);
        if (tempCount <= finalMaxPoints)
        {
            cache.persistentManifold.pointCount = tempCount;
            for (std::int32_t i = 0; i < tempCount; ++i)
            {
                cache.persistentManifold.points[i] = tempPoints[i];
            }
        }
        else
        {
            std::int32_t p1 = 0, p2 = -1, p3 = -1, p4 = -1;
            Vector maxDistSq = vNegOne;
            Vector maxTriAreaSq = vNegOne;
            Vector maxQuadAreaSq = vNegOne;

            for (std::int32_t i = 1; i < tempCount; ++i)
            {
                if (tempPoints[i].depth > tempPoints[p1].depth)
                    p1 = i;
            }

            for (std::int32_t i = 0; i < tempCount; ++i)
            {
                if (i == p1)
                    continue;
                const Vector dSq = DirectX::XMVector3LengthSq(DirectX::XMVectorSubtract(tempPoints[i].localPointA, tempPoints[p1].localPointA));
                if (DirectX::XMVector3Less(maxDistSq, dSq))
                {
                    maxDistSq = dSq;
                    p2 = i;
                }
            }

            if (p2 != -1)
            {
                for (std::int32_t i = 0; i < tempCount; ++i)
                {
                    if (i == p1 || i == p2)
                        continue;
                    const Vector edge1 = DirectX::XMVectorSubtract(tempPoints[i].localPointA, tempPoints[p1].localPointA);
                    const Vector edge2 = DirectX::XMVectorSubtract(tempPoints[p2].localPointA, tempPoints[p1].localPointA);
                    const Vector areaSq = DirectX::XMVector3LengthSq(DirectX::XMVector3Cross(edge1, edge2));
                    if (DirectX::XMVector3Less(maxTriAreaSq, areaSq))
                    {
                        maxTriAreaSq = areaSq;
                        p3 = i;
                    }
                }
            }

            if (p3 != -1)
            {
                for (std::int32_t i = 0; i < tempCount; ++i)
                {
                    if (i == p1 || i == p2 || i == p3)
                        continue;
                    const Vector e1 = DirectX::XMVector3Cross(DirectX::XMVectorSubtract(tempPoints[i].localPointA, tempPoints[p1].localPointA), DirectX::XMVectorSubtract(tempPoints[p2].localPointA, tempPoints[p1].localPointA));
                    const Vector e2 = DirectX::XMVector3Cross(DirectX::XMVectorSubtract(tempPoints[i].localPointA, tempPoints[p2].localPointA), DirectX::XMVectorSubtract(tempPoints[p3].localPointA, tempPoints[p2].localPointA));
                    const Vector e3 = DirectX::XMVector3Cross(DirectX::XMVectorSubtract(tempPoints[i].localPointA, tempPoints[p3].localPointA), DirectX::XMVectorSubtract(tempPoints[p1].localPointA, tempPoints[p3].localPointA));
                    const Vector areaSqSum = DirectX::XMVectorAdd(DirectX::XMVectorAdd(DirectX::XMVector3LengthSq(e1), DirectX::XMVector3LengthSq(e2)), DirectX::XMVector3LengthSq(e3));
                    if (DirectX::XMVector3Less(maxQuadAreaSq, areaSqSum))
                    {
                        maxQuadAreaSq = areaSqSum;
                        p4 = i;
                    }
                }
            }

            std::uint32_t mfCount = 0;
            if (finalMaxPoints >= 1u)
                cache.persistentManifold.points[mfCount++] = tempPoints[p1];
            if (finalMaxPoints >= 2u && p2 != -1)
                cache.persistentManifold.points[mfCount++] = tempPoints[p2];
            if (finalMaxPoints >= 3u && p3 != -1)
                cache.persistentManifold.points[mfCount++] = tempPoints[p3];
            if (finalMaxPoints >= 4u && p4 != -1)
                cache.persistentManifold.points[mfCount++] = tempPoints[p4];
            cache.persistentManifold.pointCount = mfCount;
        }

        cache.persistentManifold.normal = finalNormal;
        outManifold = cache.persistentManifold;
        return true;
    }

    bool XPBDWorld::SpherevsSphere(const std::uint32_t coiA, const std::uint32_t coiB, Internal::ContactManifold& outManifold)
    {
        const std::uint32_t biA = colliders.boneIdx[coiA];
        const std::uint32_t biB = colliders.boneIdx[coiB];

        const Vector posA = DirectX::XMVectorAdd(physicsBones.predPos[biA], physicsBones.dynamicLinearOffset[biA]);
        const Quaternion rotA = DirectX::XMQuaternionMultiply(physicsBones.predRot[biA], physicsBones.dynamicAngularOffset[biA]);
        const float scaleA = physicsBones.orgWorldScale[biA];

        const Vector posB = DirectX::XMVectorAdd(physicsBones.predPos[biB], physicsBones.dynamicLinearOffset[biB]);
        const Quaternion rotB = DirectX::XMQuaternionMultiply(physicsBones.predRot[biB], physicsBones.dynamicAngularOffset[biB]);
        const float scaleB = physicsBones.orgWorldScale[biB];

        const float marginA = physicsBones.collisionMargin[biA];
        const float marginB = physicsBones.collisionMargin[biB];
        const float sumMargin = marginA + marginB;

        if (DEBUG_MODE)
            std::atomic_ref(totalColCandidates).fetch_add(1, std::memory_order_relaxed);

        const auto& sphereA = colliders.sphereData[coiA];
        const auto& sphereB = colliders.sphereData[coiB];

        const Quaternion invRotA = DirectX::XMQuaternionConjugate(rotA);
        const Quaternion invRotB = DirectX::XMQuaternionConjugate(rotB);
        const Matrix matInvA = ToMatrix(invRotA);
        const Matrix matInvB = ToMatrix(invRotB);
        const float invScaleA = reciprocal(scaleA);
        const float invScaleB = reciprocal(scaleB);

        DirectX::XMFLOAT3 pA, pB;
        DirectX::XMStoreFloat3(&pA, posA);
        DirectX::XMStoreFloat3(&pB, posB);

        DirectX::XMFLOAT4X4A matA, matB;
        DirectX::XMStoreFloat4x4A(&matA, ToMatrix(rotA));
        DirectX::XMStoreFloat4x4A(&matB, ToMatrix(rotB));

        alignas(32) float awX[8], awY[8], awZ[8], awR[8];
        alignas(32) float bwX[8], bwY[8], bwZ[8], bwR[8];
        auto TransformSpheresToWorld = [](const SphereData& localData, const DirectX::XMFLOAT3& pos, const DirectX::XMFLOAT4X4A& rot, const float scale, const float margin, float* outX, float* outY, float* outZ, float* outR) {
            const __m256 vScale = _mm256_set1_ps(scale);
            const __m256 vMargin = _mm256_set1_ps(margin);
            const __m256 vPosX = _mm256_set1_ps(pos.x);
            const __m256 vPosY = _mm256_set1_ps(pos.y);
            const __m256 vPosZ = _mm256_set1_ps(pos.z);

            const __m256 lx = _mm256_mul_ps(_mm256_load_ps(localData.cX), vScale);
            const __m256 ly = _mm256_mul_ps(_mm256_load_ps(localData.cY), vScale);
            const __m256 lz = _mm256_mul_ps(_mm256_load_ps(localData.cZ), vScale);
            const __m256 lr = _mm256_load_ps(localData.radius);
#if defined(AVX)
            const __m256 wR = _mm256_add_ps(_mm256_mul_ps(lr, vScale), vMargin);

            const __m256 wX = _mm256_add_ps(_mm256_add_ps(_mm256_mul_ps(lx, _mm256_set1_ps(rot._11)), _mm256_mul_ps(ly, _mm256_set1_ps(rot._21))), _mm256_add_ps(_mm256_mul_ps(lz, _mm256_set1_ps(rot._31)), vPosX));
            const __m256 wY = _mm256_add_ps(_mm256_add_ps(_mm256_mul_ps(lx, _mm256_set1_ps(rot._12)), _mm256_mul_ps(ly, _mm256_set1_ps(rot._22))), _mm256_add_ps(_mm256_mul_ps(lz, _mm256_set1_ps(rot._32)), vPosY));
            const __m256 wZ = _mm256_add_ps(_mm256_add_ps(_mm256_mul_ps(lx, _mm256_set1_ps(rot._13)), _mm256_mul_ps(ly, _mm256_set1_ps(rot._23))), _mm256_add_ps(_mm256_mul_ps(lz, _mm256_set1_ps(rot._33)), vPosZ));
#elif defined(AVX2) || defined(AVX512)
            const __m256 slx = _mm256_mul_ps(lx, vScale);
            const __m256 sly = _mm256_mul_ps(ly, vScale);
            const __m256 slz = _mm256_mul_ps(lz, vScale);

            const __m256 wR = _mm256_fmadd_ps(lr, vScale, vMargin);

            __m256 wX = _mm256_fmadd_ps(slz, _mm256_set1_ps(rot._31), vPosX);
            wX = _mm256_fmadd_ps(sly, _mm256_set1_ps(rot._21), wX);
            wX = _mm256_fmadd_ps(slx, _mm256_set1_ps(rot._11), wX);
            __m256 wY = _mm256_fmadd_ps(slz, _mm256_set1_ps(rot._32), vPosY);
            wY = _mm256_fmadd_ps(sly, _mm256_set1_ps(rot._22), wY);
            wY = _mm256_fmadd_ps(slx, _mm256_set1_ps(rot._12), wY);
            __m256 wZ = _mm256_fmadd_ps(slz, _mm256_set1_ps(rot._33), vPosZ);
            wZ = _mm256_fmadd_ps(sly, _mm256_set1_ps(rot._23), wZ);
            wZ = _mm256_fmadd_ps(slx, _mm256_set1_ps(rot._13), wZ);
#endif
            _mm256_store_ps(outX, wX);
            _mm256_store_ps(outY, wY);
            _mm256_store_ps(outZ, wZ);
            _mm256_store_ps(outR, wR);
        };

        TransformSpheresToWorld(sphereA, pA, matA, scaleA, marginA, awX, awY, awZ, awR);
        TransformSpheresToWorld(sphereB, pB, matB, scaleB, marginB, bwX, bwY, bwZ, bwR);

        bool collidedAny = false;
        Vector finalNormal = vZero;
        float frameMaxDepth = -1.0f;
        Vector frameBestLA = vZero;
        Vector frameBestLB = vZero;

#if defined(AVX) || defined(AVX2)
        for (std::uint32_t a = 0; a < COL_SPHERE_MAX; a += 4)
        {
            const std::uint32_t a0 = a;
            const std::uint32_t a1 = a + 1;
            const std::uint32_t a2 = a + 2;
            const std::uint32_t a3 = a + 3;

            const __m256 v_ax_01 = _mm256_setr_ps(awX[a0], awX[a0], awX[a0], awX[a0], awX[a1], awX[a1], awX[a1], awX[a1]);
            const __m256 v_ay_01 = _mm256_setr_ps(awY[a0], awY[a0], awY[a0], awY[a0], awY[a1], awY[a1], awY[a1], awY[a1]);
            const __m256 v_az_01 = _mm256_setr_ps(awZ[a0], awZ[a0], awZ[a0], awZ[a0], awZ[a1], awZ[a1], awZ[a1], awZ[a1]);
            const __m256 v_ar_01 = _mm256_setr_ps(awR[a0], awR[a0], awR[a0], awR[a0], awR[a1], awR[a1], awR[a1], awR[a1]);

            const __m256 v_ax_23 = _mm256_setr_ps(awX[a2], awX[a2], awX[a2], awX[a2], awX[a3], awX[a3], awX[a3], awX[a3]);
            const __m256 v_ay_23 = _mm256_setr_ps(awY[a2], awY[a2], awY[a2], awY[a2], awY[a3], awY[a3], awY[a3], awY[a3]);
            const __m256 v_az_23 = _mm256_setr_ps(awZ[a2], awZ[a2], awZ[a2], awZ[a2], awZ[a3], awZ[a3], awZ[a3], awZ[a3]);
            const __m256 v_ar_23 = _mm256_setr_ps(awR[a2], awR[a2], awR[a2], awR[a2], awR[a3], awR[a3], awR[a3], awR[a3]);

            for (std::uint32_t b = 0; b < COL_SPHERE_MAX; b += 4)
            {
                const __m256 v_bx = _mm256_broadcast_ps((const __m128*)&bwX[b]);
                const __m256 v_by = _mm256_broadcast_ps((const __m128*)&bwY[b]);
                const __m256 v_bz = _mm256_broadcast_ps((const __m128*)&bwZ[b]);
                const __m256 v_br = _mm256_broadcast_ps((const __m128*)&bwR[b]);

                const __m256 dx01 = _mm256_sub_ps(v_ax_01, v_bx);
                const __m256 dy01 = _mm256_sub_ps(v_ay_01, v_by);
                const __m256 dz01 = _mm256_sub_ps(v_az_01, v_bz);

                const __m256 dx23 = _mm256_sub_ps(v_ax_23, v_bx);
                const __m256 dy23 = _mm256_sub_ps(v_ay_23, v_by);
                const __m256 dz23 = _mm256_sub_ps(v_az_23, v_bz);

#if defined(AVX)
                const __m256 distSq01 = _mm256_add_ps(
                    _mm256_add_ps(_mm256_mul_ps(dx01, dx01), _mm256_mul_ps(dy01, dy01)),
                    _mm256_mul_ps(dz01, dz01));
                const __m256 radSum01 = _mm256_add_ps(v_ar_01, v_br);
                const __m256 radSq01 = _mm256_mul_ps(radSum01, radSum01);
                const std::uint32_t mask01 = _mm256_movemask_ps(_mm256_cmp_ps(distSq01, radSq01, _CMP_LE_OQ));

                const __m256 distSq23 = _mm256_add_ps(
                    _mm256_add_ps(_mm256_mul_ps(dx23, dx23), _mm256_mul_ps(dy23, dy23)),
                    _mm256_mul_ps(dz23, dz23));
                const __m256 radSum23 = _mm256_add_ps(v_ar_23, v_br);
                const __m256 radSq23 = _mm256_mul_ps(radSum23, radSum23);
                const std::uint32_t mask23 = _mm256_movemask_ps(_mm256_cmp_ps(distSq23, radSq23, _CMP_LE_OQ));
#elif defined(AVX2)
                __m256 distSq01 = _mm256_mul_ps(dx01, dx01);
                distSq01 = _mm256_fmadd_ps(dy01, dy01, distSq01);
                distSq01 = _mm256_fmadd_ps(dz01, dz01, distSq01);
                const __m256 radSq01 = _mm256_mul_ps(_mm256_add_ps(v_ar_01, v_br), _mm256_add_ps(v_ar_01, v_br));
                const std::uint32_t mask01 = _mm256_movemask_ps(_mm256_cmp_ps(distSq01, radSq01, _CMP_LE_OQ));

                __m256 distSq23 = _mm256_mul_ps(dx23, dx23);
                distSq23 = _mm256_fmadd_ps(dy23, dy23, distSq23);
                distSq23 = _mm256_fmadd_ps(dz23, dz23, distSq23);
                const __m256 radSq23 = _mm256_mul_ps(_mm256_add_ps(v_ar_23, v_br), _mm256_add_ps(v_ar_23, v_br));
                const std::uint32_t mask23 = _mm256_movemask_ps(_mm256_cmp_ps(distSq23, radSq23, _CMP_LE_OQ));
#endif
                std::uint32_t cmpMask = mask01 | (mask23 << 8);

                if (cmpMask)
                {
                    collidedAny = true;

                    alignas(32) float f_distSq[16];
                    _mm256_store_ps(&f_distSq[0], distSq01);
                    _mm256_store_ps(&f_distSq[8], distSq23);

                    while (cmpMask)
                    {
                        const std::uint32_t bitIdx = _tzcnt_u32(cmpMask);
                        cmpMask &= cmpMask - 1;

                        const uint32_t a_idx = a + (bitIdx >> 2);
                        const uint32_t b_idx = b + (bitIdx & 3);

                        if (a_idx < COL_SPHERE_MAX && b_idx < COL_SPHERE_MAX)
                        {
                            const float dist = std::sqrt(f_distSq[bitIdx]);
                            const float depth = (awR[a_idx] + bwR[b_idx]) - dist;

                            if (depth > frameMaxDepth)
                            {
                                frameMaxDepth = depth;

                                const Vector wCa = DirectX::XMVectorSet(awX[a_idx], awY[a_idx], awZ[a_idx], 0.0f);
                                const Vector wCb = DirectX::XMVectorSet(bwX[b_idx], bwY[b_idx], bwZ[b_idx], 0.0f);

                                if (dist > Epsilon)
                                    finalNormal = DirectX::XMVectorMultiply(DirectX::XMVectorSubtract(wCa, wCb), DirectX::XMVectorReciprocal(DirectX::XMVectorReplicate(dist)));
                                else
                                    finalNormal = vXone;

                                const Vector wB_point = DirectX::XMVectorAdd(wCb, DirectX::XMVectorScale(finalNormal, bwR[b_idx]));
                                const Vector wA_point = DirectX::XMVectorSubtract(wCa, DirectX::XMVectorScale(finalNormal, awR[a_idx]));

                                const Vector diffA = DirectX::XMVectorSubtract(wA_point, posA);
                                frameBestLA = DirectX::XMVectorScale(DirectX::XMVector3TransformNormal(diffA, matInvA), invScaleA);

                                const Vector diffB = DirectX::XMVectorSubtract(wB_point, posB);
                                frameBestLB = DirectX::XMVectorScale(DirectX::XMVector3TransformNormal(diffB, matInvB), invScaleB);
                            }
                        }
                    }
                }
            }
        }
#elif defined(AVX512)
        for (std::uint32_t a = 0; a < COL_SPHERE_MAX; a += 4)
        {
            const std::uint32_t a0 = a;
            const std::uint32_t a1 = a + 1;
            const std::uint32_t a2 = a + 2;
            const std::uint32_t a3 = a + 3;

            const __m512 v_ax = _mm512_setr_ps(
                awX[a0], awX[a0], awX[a0], awX[a0], awX[a1], awX[a1], awX[a1], awX[a1],
                awX[a2], awX[a2], awX[a2], awX[a2], awX[a3], awX[a3], awX[a3], awX[a3]);
            const __m512 v_ay = _mm512_setr_ps(
                awY[a0], awY[a0], awY[a0], awY[a0], awY[a1], awY[a1], awY[a1], awY[a1],
                awY[a2], awY[a2], awY[a2], awY[a2], awY[a3], awY[a3], awY[a3], awY[a3]);
            const __m512 v_az = _mm512_setr_ps(
                awZ[a0], awZ[a0], awZ[a0], awZ[a0], awZ[a1], awZ[a1], awZ[a1], awZ[a1],
                awZ[a2], awZ[a2], awZ[a2], awZ[a2], awZ[a3], awZ[a3], awZ[a3], awZ[a3]);
            const __m512 v_ar = _mm512_setr_ps(
                awR[a0], awR[a0], awR[a0], awR[a0], awR[a1], awR[a1], awR[a1], awR[a1],
                awR[a2], awR[a2], awR[a2], awR[a2], awR[a3], awR[a3], awR[a3], awR[a3]);

            for (std::uint32_t b = 0; b < COL_SPHERE_MAX; b += 4)
            {
                const __m512 v_bx = _mm512_broadcast_f32x4(_mm_load_ps(&bwX[b]));
                const __m512 v_by = _mm512_broadcast_f32x4(_mm_load_ps(&bwY[b]));
                const __m512 v_bz = _mm512_broadcast_f32x4(_mm_load_ps(&bwZ[b]));
                const __m512 v_br = _mm512_broadcast_f32x4(_mm_load_ps(&bwR[b]));

                const __m512 dx = _mm512_sub_ps(v_ax, v_bx);
                const __m512 dy = _mm512_sub_ps(v_ay, v_by);
                const __m512 dz = _mm512_sub_ps(v_az, v_bz);

                __m512 distSq = _mm512_mul_ps(dx, dx);
                distSq = _mm512_fmadd_ps(dy, dy, distSq);
                distSq = _mm512_fmadd_ps(dz, dz, distSq);

                const __m512 radSum = _mm512_add_ps(v_ar, v_br);
                const __m512 radSq = _mm512_mul_ps(radSum, radSum);

                std::uint32_t cmpMask = static_cast<std::uint32_t>(_mm512_cmp_ps_mask(distSq, radSq, _CMP_LE_OQ));

                if (cmpMask)
                {
                    collidedAny = true;
                    alignas(64) float f_distSq[16];
                    _mm512_store_ps(f_distSq, distSq);

                    while (cmpMask)
                    {
                        const std::uint32_t bitIdx = _tzcnt_u32(cmpMask);
                        cmpMask &= cmpMask - 1;

                        const uint32_t a_idx = a + (bitIdx >> 2);
                        const uint32_t b_idx = b + (bitIdx & 3);

                        if (a_idx < COL_SPHERE_MAX && b_idx < COL_SPHERE_MAX)
                        {
                            const float dist = std::sqrt(f_distSq[bitIdx]);
                            const float depth = (awR[a_idx] + bwR[b_idx]) - dist;

                            if (depth > frameMaxDepth)
                            {
                                frameMaxDepth = depth;

                                const Vector wCa = DirectX::XMVectorSet(awX[a_idx], awY[a_idx], awZ[a_idx], 0.0f);
                                const Vector wCb = DirectX::XMVectorSet(bwX[b_idx], bwY[b_idx], bwZ[b_idx], 0.0f);
                                
                                if (dist > Epsilon)
                                    finalNormal = DirectX::XMVectorMultiply(DirectX::XMVectorSubtract(wCa, wCb), DirectX::XMVectorReciprocal(DirectX::XMVectorReplicate(dist)));
                                else
                                    finalNormal = vXone;

                                const Vector wB_point = DirectX::XMVectorAdd(wCb, DirectX::XMVectorScale(finalNormal, bwR[b_idx]));
                                const Vector wA_point = DirectX::XMVectorSubtract(wCa, DirectX::XMVectorScale(finalNormal, awR[a_idx]));

                                const Vector diffA = DirectX::XMVectorSubtract(wA_point, posA);
                                frameBestLA = DirectX::XMVectorScale(DirectX::XMVector3TransformNormal(diffA, matInvA), invScaleA);

                                const Vector diffB = DirectX::XMVectorSubtract(wB_point, posB);
                                frameBestLB = DirectX::XMVectorScale(DirectX::XMVector3TransformNormal(diffB, matInvB), invScaleB);
                            }
                        }
                    }
                }
            }
        }
#endif

        if (!collidedAny)
            return false;

        auto& cache = convexHullCache[Internal::GetCacheKey(coiA, coiB)];
        cache.axis = finalNormal;
        cache.lastFrame = currentFrame;

        Internal::ContactManifold::ContactPoint tempPoints[5];
        std::int32_t tempCount = 0;
        const Vector vSumMargin = DirectX::XMVectorReplicate(-sumMargin);

        for (std::uint32_t i = 0; i < cache.persistentManifold.pointCount; ++i)
        {
            const auto& cp = cache.persistentManifold.points[i];
            const Vector wA = DirectX::XMVectorAdd(posA, DirectX::XMVector3Rotate(DirectX::XMVectorScale(cp.localPointA, scaleA), rotA));
            const Vector wB = DirectX::XMVectorAdd(posB, DirectX::XMVector3Rotate(DirectX::XMVectorScale(cp.localPointB, scaleB), rotB));

            const Vector diff = DirectX::XMVectorSubtract(wA, wB);
            const Vector currentDepth = DirectX::XMVectorNegate(DirectX::XMVector3Dot(diff, finalNormal));

            if (DirectX::XMVector3Less(currentDepth, vSumMargin))
                continue;

            const Vector projDiff = DirectX::XMVectorAdd(diff, DirectX::XMVectorMultiply(finalNormal, currentDepth));
            const Vector driftSq = DirectX::XMVector3LengthSq(projDiff);
            if (DirectX::XMVector3Greater(driftSq, vBreakThresholdSq))
                continue;

            tempPoints[tempCount] = cp;
            tempPoints[tempCount].depth = DirectX::XMVectorGetX(currentDepth);
            tempCount++;
        }

        if (frameMaxDepth > 0.0f)
        {
            bool isDuplicate = false;
            for (std::int32_t i = 0; i < tempCount; ++i)
            {
                const Vector tempPointDistSq = DirectX::XMVector3LengthSq(DirectX::XMVectorSubtract(tempPoints[i].localPointA, frameBestLA));
                if (DirectX::XMVector3Less(tempPointDistSq, vContactMergeThresholdSq))
                {
                    if (frameMaxDepth > tempPoints[i].depth)
                    {
                        tempPoints[i].depth = frameMaxDepth;
                        tempPoints[i].localPointB = frameBestLB;
                    }
                    isDuplicate = true;
                    break;
                }
            }
            if (!isDuplicate && tempCount < 5)
            {
                tempPoints[tempCount++] = {frameBestLA, frameBestLB, frameMaxDepth};
            }
        }

        const std::uint32_t allowPoints = objectDatas.maxManifoldPoints[colliders.objIdx[coiA]];
        const std::uint32_t finalMaxPoints = std::min(allowPoints, 4u);

        if (tempCount <= finalMaxPoints)
        {
            cache.persistentManifold.pointCount = tempCount;
            for (std::int32_t i = 0; i < tempCount; ++i)
                cache.persistentManifold.points[i] = tempPoints[i];
        }
        else
        {
            std::int32_t p1 = 0, p2 = -1, p3 = -1, p4 = -1;
            Vector maxDistSq = vNegOne;
            Vector maxTriAreaSq = vNegOne;
            Vector maxQuadAreaSq = vNegOne;

            for (std::int32_t i = 1; i < tempCount; ++i)
            {
                if (tempPoints[i].depth > tempPoints[p1].depth)
                    p1 = i;
            }

            for (std::int32_t i = 0; i < tempCount; ++i)
            {
                if (i == p1)
                    continue;
                const Vector dSq = DirectX::XMVector3LengthSq(DirectX::XMVectorSubtract(tempPoints[i].localPointA, tempPoints[p1].localPointA));
                if (DirectX::XMVector3Less(maxDistSq, dSq))
                {
                    maxDistSq = dSq;
                    p2 = i;
                }
            }

            if (p2 != -1)
            {
                for (std::int32_t i = 0; i < tempCount; ++i)
                {
                    if (i == p1 || i == p2)
                        continue;
                    const Vector edge1 = DirectX::XMVectorSubtract(tempPoints[i].localPointA, tempPoints[p1].localPointA);
                    const Vector edge2 = DirectX::XMVectorSubtract(tempPoints[p2].localPointA, tempPoints[p1].localPointA);
                    const Vector areaSq = DirectX::XMVector3LengthSq(DirectX::XMVector3Cross(edge1, edge2));
                    if (DirectX::XMVector3Less(maxTriAreaSq, areaSq))
                    {
                        maxTriAreaSq = areaSq;
                        p3 = i;
                    }
                }
            }

            if (p3 != -1)
            {
                for (std::int32_t i = 0; i < tempCount; ++i)
                {
                    if (i == p1 || i == p2 || i == p3)
                        continue;
                    const Vector e1 = DirectX::XMVector3Cross(DirectX::XMVectorSubtract(tempPoints[i].localPointA, tempPoints[p1].localPointA), DirectX::XMVectorSubtract(tempPoints[p2].localPointA, tempPoints[p1].localPointA));
                    const Vector e2 = DirectX::XMVector3Cross(DirectX::XMVectorSubtract(tempPoints[i].localPointA, tempPoints[p2].localPointA), DirectX::XMVectorSubtract(tempPoints[p3].localPointA, tempPoints[p2].localPointA));
                    const Vector e3 = DirectX::XMVector3Cross(DirectX::XMVectorSubtract(tempPoints[i].localPointA, tempPoints[p3].localPointA), DirectX::XMVectorSubtract(tempPoints[p1].localPointA, tempPoints[p3].localPointA));
                    const Vector areaSqSum = DirectX::XMVectorAdd(DirectX::XMVectorAdd(DirectX::XMVector3LengthSq(e1), DirectX::XMVector3LengthSq(e2)), DirectX::XMVector3LengthSq(e3));
                    if (DirectX::XMVector3Less(maxQuadAreaSq, areaSqSum))
                    {
                        maxQuadAreaSq = areaSqSum;
                        p4 = i;
                    }
                }
            }

            std::uint32_t mfCount = 0;
            if (finalMaxPoints >= 1u)
                cache.persistentManifold.points[mfCount++] = tempPoints[p1];
            if (finalMaxPoints >= 2u && p2 != -1)
                cache.persistentManifold.points[mfCount++] = tempPoints[p2];
            if (finalMaxPoints >= 3u && p3 != -1)
                cache.persistentManifold.points[mfCount++] = tempPoints[p3];
            if (finalMaxPoints >= 4u && p4 != -1)
                cache.persistentManifold.points[mfCount++] = tempPoints[p4];
            cache.persistentManifold.pointCount = mfCount;
        }

        cache.persistentManifold.normal = finalNormal;
        outManifold = cache.persistentManifold;
        return true;
    }

    std::uint32_t XPBDWorld::FindObject(RE::TESObjectREFR* object) const
    {
        std::uint32_t objIdx = UINT32_MAX;
        for (std::uint32_t oi = 0; oi < objectDatas.objectID.size(); ++oi)
        {
            if (objectDatas.objectID[oi] != object->formID)
                continue;
            objIdx = oi;
            logger::debug("{:x} : found object {}", object->formID, objIdx);
            break;
        }
        return objIdx;
    }
    std::uint32_t XPBDWorld::AllocateObject(RE::TESObjectREFR* object)
    {
        std::uint32_t objIdx = FindObject(object);

        if (objIdx == UINT32_MAX)
        {
            // find empty slot
            for (std::uint32_t oi = 0; oi < objectDatas.objectID.size(); ++oi)
            {
                if (objectDatas.objectID[oi] != 0)
                    continue;
                objIdx = oi;
                objectDatas.objectID[objIdx] = object->formID;
                objectDatas.roots[objIdx].clear();
                if (object->loadedData && object->loadedData->data3D)
                {
                    objectDatas.prevWorldPos[objIdx] = ToVector(object->loadedData->data3D->world.translate);
                    RE::NiNode* npcNode = GetNPCNode(object->loadedData->data3D.get());
                    objectDatas.prevNPCWorldRot[objIdx] = npcNode ? ToQuaternion(npcNode->world.rotate) : ToQuaternion(RE::NiMatrix3());
                    objectDatas.targetNPCWorldRot[objIdx] = objectDatas.prevNPCWorldRot[objIdx];
                    objectDatas.npcNode[objIdx] = npcNode ? RE::NiPointer(npcNode) : nullptr;
                }
                else
                {
                    objectDatas.prevWorldPos[objIdx] = ToVector(object->GetPosition());
                    objectDatas.prevNPCWorldRot[objIdx] = ToQuaternion(RE::NiMatrix3());
                    objectDatas.targetNPCWorldRot[objIdx] = objectDatas.prevNPCWorldRot[objIdx];
                    objectDatas.npcNode[objIdx] = nullptr;
                }
                objectDatas.deltaWorldPos[objIdx] = vZero;
                objectDatas.deltaWorldRot[objIdx] = qZero;
                objectDatas.omegaWorldRot[objIdx] = vZero;
                if (RE::TESObjectCELL* cell = object->GetParentCell(); cell)
                    objectDatas.bhkWorld[objIdx] = cell->GetbhkWorld();
                else
                    objectDatas.bhkWorld[objIdx] = nullptr;
                objectDatas.velocity[objIdx] = vZero;
                objectDatas.acceleration[objIdx] = vZero;
                objectDatas.boundingAABB[objIdx] = AABB();
                objectDatas.isStatic[objIdx] = 0;
                objectDatas.windMultiplier[objIdx] = vZero;
                objectDatas.maxManifoldPoints[objIdx] = 4;
                objectDatas.isDisable[objIdx] = 0;
                objectDatas.isDisableByToggle[objIdx] = 0;
                objectDatas.randState[objIdx] = rand_Hash(1103515245 + objIdx + objectDatas.objectID[objIdx]);
                boneNameToIdx[objIdx].clear();
                logger::debug("{:x} : add new object {}", object->formID, objIdx);
                break;
            }
            if (objIdx == UINT32_MAX)
            {
                objIdx = objectDatas.objectID.size();
                objectDatas.objectID.push_back(object->formID);
                objectDatas.roots.push_back({});
                bool hasNPCNode = false;
                if (object->loadedData && object->loadedData->data3D)
                {
                    RE::NiAVObject* npcObj = object->loadedData->data3D->GetObjectByName("NPC");
                    RE::NiNode* npcNode = npcObj ? npcObj->AsNode() : nullptr;    
                    if (npcNode && npcNode->parent)
                    {
                        objectDatas.prevWorldPos.push_back(ToVector(object->loadedData->data3D->world.translate));
                        objectDatas.prevNPCWorldRot.push_back(ToQuaternion(npcNode->world.rotate));
                        objectDatas.targetNPCWorldRot.push_back(ToQuaternion(npcNode->world.rotate));
                        objectDatas.npcNode.push_back(RE::NiPointer(npcNode));
                        hasNPCNode = true;
                    }
                }
                if (!hasNPCNode)
                {
                    objectDatas.prevWorldPos.push_back(ToVector(object->GetPosition()));
                    objectDatas.prevNPCWorldRot.push_back(ToQuaternion(RE::NiMatrix3()));
                    objectDatas.targetNPCWorldRot.push_back(ToQuaternion(RE::NiMatrix3()));
                    objectDatas.npcNode.push_back(nullptr);
                }
                objectDatas.deltaWorldPos.push_back(vZero);
                objectDatas.deltaWorldRot.push_back(qZero);
                objectDatas.omegaWorldRot.push_back(vZero);
                if (RE::TESObjectCELL* cell = object->GetParentCell(); cell)
                    objectDatas.bhkWorld.push_back(cell->GetbhkWorld());
                else
                    objectDatas.bhkWorld.push_back(nullptr);
                objectDatas.velocity.push_back(vZero);
                objectDatas.acceleration.push_back(vZero);
                objectDatas.boundingAABB.push_back(AABB());
                objectDatas.isStatic.push_back(0);
                objectDatas.windMultiplier.push_back(vZero);
                objectDatas.maxManifoldPoints.push_back(4);
                objectDatas.isDisable.push_back(0);
                objectDatas.isDisableByToggle.push_back(0);
                objectDatas.randState.push_back(rand_Hash(1103515245 + objIdx + objectDatas.objectID[objIdx]));
                boneNameToIdx.push_back({});
                logger::debug("{:x} : add new object {}", object->formID, objIdx);
            }
        }
        return objIdx;
    }
    std::uint32_t XPBDWorld::FindRoot(const std::uint32_t objIdx, const Internal::ObjectDatas::Root& rootData) const
    {
        if (objIdx == UINT32_MAX)
            return UINT32_MAX;
        std::uint32_t rootIdx = UINT32_MAX;
        auto& root = objectDatas.roots[objIdx];
        for (std::uint32_t ri = 0; ri < root.size(); ++ri)
        {
            if (root[ri] == rootData)
            {
                rootIdx = ri;
                logger::debug("{} : found root {}", objIdx, rootIdx);
                break;
            }
        }
        return rootIdx;
    }
    std::uint32_t XPBDWorld::AllocateRoot(const std::uint32_t objIdx, const Internal::ObjectDatas::Root& rootData)
    {
        std::uint32_t rootIdx = FindRoot(objIdx, rootData);
        if (rootIdx == UINT32_MAX)
        {
            auto& root = objectDatas.roots[objIdx];
            rootIdx = static_cast<std::uint32_t>(root.size());
            root.push_back(rootData);
            logger::debug("{:x} : add new root {}", objIdx, rootIdx);
        }
        return rootIdx;
    }
    
    std::uint32_t XPBDWorld::AllocateBone()
    {
        const std::uint32_t newIdx = physicsBones.numBones++;
        physicsBones.pos.push_back(vZero);
        physicsBones.prevPos.push_back(vZero);
        physicsBones.predPos.push_back(vZero);
        physicsBones.backupPos.push_back(vZero);
        physicsBones.posVel.push_back(vZero);
        
        physicsBones.advancedRotation.push_back(0);
        physicsBones.rot.push_back(qZero);
        physicsBones.prevRot.push_back(qZero);
        physicsBones.predRot.push_back(qZero);
        physicsBones.backupRot.push_back(qZero);
        physicsBones.angVel.push_back(vZero);

        physicsBones.dampingPositive.push_back(vZero);
        physicsBones.dampingNegative.push_back(vZero);
        physicsBones.angularDampingPositive.push_back(vZero);
        physicsBones.angularDampingNegative.push_back(vZero);
        physicsBones.inertiaPositive.push_back(vZero);
        physicsBones.inertiaNegative.push_back(vZero);
        physicsBones.invAngularInertiaPositive.push_back(vZero);
        physicsBones.invAngularInertiaNegative.push_back(vZero);
        physicsBones.angularBlendFactor.push_back(0);
        physicsBones.gravity.push_back(GetSkyrimGravity(1.0f));
        physicsBones.offset.push_back(vZero);
        physicsBones.invMass.push_back(0);
        physicsBones.windFactor.push_back(vZero);
        physicsBones.physicsScale.push_back(0);

        physicsBones.deformMax.push_back(vOne);
        physicsBones.deformMin.push_back(vOne);
        physicsBones.deformVolumePreservation.push_back(vZero);
        physicsBones.deformSquishSensitivity.push_back(vZero);
        physicsBones.deformStretchSensitivity.push_back(vZero);
        physicsBones.deformBulgeSensitivity.push_back(vZero);
        physicsBones.deformSquishStiffness.push_back(vZero);
        physicsBones.deformStretchStiffness.push_back(vZero);
        physicsBones.deformSquishDamping.push_back(vZero);
        physicsBones.deformStretchDamping.push_back(vZero);

        physicsBones.deformScale.push_back(vmIdentity);
        physicsBones.deformScaleCache.push_back(vmZeroAll);
        physicsBones.deformVelocityScale.push_back(vmZeroAll);
        physicsBones.deformCount.push_back(0);

        physicsBones.animDriveCompliance.push_back(vZero);
        physicsBones.animDriveLambda.push_back(0);
        physicsBones.animDriveAngularCompliance.push_back(vZero);
        physicsBones.animDriveAngularLambda.push_back(0);

        physicsBones.linearRotTorque.push_back(vmZero);
        physicsBones.centerOfMass.push_back(vZero);
        physicsBones.dynamicLinearOffset.push_back(vZero);
        physicsBones.dynamicAngularOffset.push_back(qZero);

        physicsBones.collisionMargin.push_back(0);
        physicsBones.collisionShrink.push_back(0);
        physicsBones.collisionFriction.push_back(0);
        physicsBones.collisionCompliance.push_back(0);
        physicsBones.collisionRestitution.push_back(0);

        physicsBones.layerGroup.push_back(0);
        physicsBones.collideLayer.push_back(0);

        physicsBones.frictionCache.push_back({});
        physicsBones.deformCache.push_back({});
        physicsBones.groundCache.push_back({});

        physicsBones.node.push_back(nullptr);
        physicsBones.particleName.push_back("");
        physicsBones.isParticle.push_back(0);
        physicsBones.particleDepth.push_back(0);
        physicsBones.parentBoneIdx.push_back(0);
        physicsBones.objIdx.push_back(UINT32_MAX);
        physicsBones.rootIdx.push_back(UINT32_MAX);
        physicsBones.depth.push_back(0);

        physicsBones.prevNodeWorldPos.push_back(vZero);
        physicsBones.targetNodeWorldPos.push_back(vZero);
        physicsBones.prevNodeWorldRot.push_back(qZero);
        physicsBones.targetNodeWorldRot.push_back(qZero);

        physicsBones.alignRot.push_back(qZero);
        physicsBones.invAlignRot.push_back(qZero);

        physicsBones.orgWorldScale.push_back(1);
        physicsBones.orgLocalPos.push_back(vZero);
        physicsBones.orgLocalRot.push_back(qZero);

        physicsBones.lastFrame.push_back(0);
        return newIdx;
    }
    void XPBDWorld::ReserveBone(std::uint32_t n)
    {
        if (n == 0)
            return;
        n += physicsBones.numBones;
        physicsBones.pos.reserve(n);
        physicsBones.prevPos.reserve(n);
        physicsBones.predPos.reserve(n);
        physicsBones.backupPos.reserve(n);
        physicsBones.posVel.reserve(n);

        physicsBones.advancedRotation.reserve(n);
        physicsBones.rot.reserve(n);
        physicsBones.prevRot.reserve(n);
        physicsBones.predRot.reserve(n);
        physicsBones.backupRot.reserve(n);
        physicsBones.angVel.reserve(n);

        physicsBones.dampingPositive.reserve(n);
        physicsBones.dampingNegative.reserve(n);
        physicsBones.angularDampingPositive.reserve(n);
        physicsBones.angularDampingNegative.reserve(n);
        physicsBones.inertiaPositive.reserve(n);
        physicsBones.inertiaNegative.reserve(n);
        physicsBones.invAngularInertiaPositive.reserve(n);
        physicsBones.invAngularInertiaNegative.reserve(n);
        physicsBones.angularBlendFactor.reserve(n);
        physicsBones.gravity.reserve(n);
        physicsBones.offset.reserve(n);
        physicsBones.invMass.reserve(n);
        physicsBones.windFactor.reserve(n);
        physicsBones.physicsScale.reserve(n);

        physicsBones.deformMax.reserve(n);
        physicsBones.deformMin.reserve(n);
        physicsBones.deformVolumePreservation.reserve(n);
        physicsBones.deformSquishSensitivity.reserve(n);
        physicsBones.deformStretchSensitivity.reserve(n);
        physicsBones.deformBulgeSensitivity.reserve(n);
        physicsBones.deformSquishStiffness.reserve(n);
        physicsBones.deformStretchStiffness.reserve(n);
        physicsBones.deformSquishDamping.reserve(n);
        physicsBones.deformStretchDamping.reserve(n);

        physicsBones.deformScale.reserve(n);
        physicsBones.deformScaleCache.reserve(n);
        physicsBones.deformVelocityScale.reserve(n);
        physicsBones.deformCount.reserve(n);

        physicsBones.animDriveCompliance.reserve(n);
        physicsBones.animDriveLambda.reserve(n);
        physicsBones.animDriveAngularCompliance.reserve(n);
        physicsBones.animDriveAngularLambda.reserve(n);

        physicsBones.linearRotTorque.reserve(n);
        physicsBones.centerOfMass.reserve(n);
        physicsBones.dynamicLinearOffset.reserve(n);
        physicsBones.dynamicAngularOffset.reserve(n);

        physicsBones.collisionMargin.reserve(n);
        physicsBones.collisionShrink.reserve(n);
        physicsBones.collisionFriction.reserve(n);
        physicsBones.collisionCompliance.reserve(n);
        physicsBones.collisionRestitution.reserve(n);

        physicsBones.layerGroup.reserve(n);
        physicsBones.collideLayer.reserve(n);

        physicsBones.frictionCache.reserve(n);
        physicsBones.deformCache.reserve(n);
        physicsBones.groundCache.reserve(n);

        physicsBones.node.reserve(n);
        physicsBones.particleName.reserve(n);
        physicsBones.isParticle.reserve(n);
        physicsBones.particleDepth.reserve(n);
        physicsBones.parentBoneIdx.reserve(n);
        physicsBones.objIdx.reserve(n);
        physicsBones.rootIdx.reserve(n);
        physicsBones.depth.reserve(n);

        physicsBones.prevNodeWorldPos.reserve(n);
        physicsBones.targetNodeWorldPos.reserve(n);
        physicsBones.prevNodeWorldRot.reserve(n);
        physicsBones.targetNodeWorldRot.reserve(n);

        physicsBones.alignRot.reserve(n);
        physicsBones.invAlignRot.reserve(n);

        physicsBones.orgWorldScale.reserve(n);
        physicsBones.orgLocalPos.reserve(n);
        physicsBones.orgLocalRot.reserve(n);

        physicsBones.lastFrame.reserve(n);
    }
    
    std::uint32_t XPBDWorld::AllocateDistanceConstraint()
    {
        const std::uint32_t newIdx = distanceConstraints.numConstraints++;
        distanceConstraints.boneIdx.push_back(UINT32_MAX);
        distanceConstraints.anchIdx.push_back(UINT32_MAX);
        distanceConstraints.objIdx.push_back(UINT32_MAX);
        distanceConstraints.rootIdx.push_back(UINT32_MAX);
        distanceConstraints.colorGraph.push_back(0);

        distanceConstraints.restLen.push_back(0);
        distanceConstraints.complianceSquish.push_back(0);
        distanceConstraints.complianceStretch.push_back(0);
        distanceConstraints.squishMargin.push_back(0);
        distanceConstraints.stretchMargin.push_back(0);
        distanceConstraints.squishLimit.push_back(0);
        distanceConstraints.stretchLimit.push_back(0);
        distanceConstraints.squishDamping.push_back(0);
        distanceConstraints.stretchDamping.push_back(0);

        distanceConstraints.lambda.push_back(0);
        return newIdx;
    }
    void XPBDWorld::ReserveDistanceConstraint(std::uint32_t n)
    {
        if (n == 0)
            return;
        n += distanceConstraints.numConstraints;
        distanceConstraints.boneIdx.reserve(n);
        distanceConstraints.anchIdx.reserve(n);
        distanceConstraints.objIdx.reserve(n);
        distanceConstraints.rootIdx.reserve(n);
        distanceConstraints.colorGraph.reserve(n);

        distanceConstraints.restLen.reserve(n);
        distanceConstraints.complianceSquish.reserve(n);
        distanceConstraints.complianceStretch.reserve(n);
        distanceConstraints.squishMargin.reserve(n);
        distanceConstraints.stretchMargin.reserve(n);
        distanceConstraints.squishLimit.reserve(n);
        distanceConstraints.stretchLimit.reserve(n);
        distanceConstraints.squishDamping.reserve(n);
        distanceConstraints.stretchDamping.reserve(n);

        distanceConstraints.lambda.reserve(n);
    }

    std::uint32_t XPBDWorld::AllocateAngularConstraint()
    {
        const std::uint32_t newIdx = angularConstraints.numConstraints++;
        angularConstraints.boneIdx.push_back(UINT32_MAX);
        angularConstraints.anchIdx.push_back(UINT32_MAX);
        angularConstraints.objIdx.push_back(UINT32_MAX);
        angularConstraints.rootIdx.push_back(UINT32_MAX);
        angularConstraints.colorGraph.push_back(0);

        angularConstraints.restRot.push_back(qZero);
        angularConstraints.compliancePositive.push_back(vZero);
        angularConstraints.complianceNegative.push_back(vZero);
        angularConstraints.marginPositive.push_back(vZero);
        angularConstraints.marginNegative.push_back(vZero);
        angularConstraints.limitPositive.push_back(vInf);
        angularConstraints.limitNegative.push_back(vNegInf);
        angularConstraints.dampingPositive.push_back(vZero);
        angularConstraints.dampingNegative.push_back(vZero);

        angularConstraints.lambda.push_back(0);

        return newIdx;
    }
    void XPBDWorld::ReserveAngularConstraint(std::uint32_t n)
    {
        if (n == 0)
            return;
        n += angularConstraints.numConstraints;
        angularConstraints.boneIdx.reserve(n);
        angularConstraints.anchIdx.reserve(n);
        angularConstraints.objIdx.reserve(n);
        angularConstraints.rootIdx.reserve(n);
        angularConstraints.colorGraph.reserve(n);

        angularConstraints.restRot.reserve(n);
        angularConstraints.compliancePositive.reserve(n);
        angularConstraints.complianceNegative.reserve(n);
        angularConstraints.marginPositive.reserve(n);
        angularConstraints.marginNegative.reserve(n);
        angularConstraints.limitPositive.reserve(n);
        angularConstraints.limitNegative.reserve(n);
        angularConstraints.dampingPositive.reserve(n);
        angularConstraints.dampingNegative.reserve(n);

        angularConstraints.lambda.reserve(n);
    }

    std::uint32_t XPBDWorld::AllocateConeConstraint()
    {
        const std::uint32_t newIdx = coneConstraints.numConstraints++;
        coneConstraints.boneIdx.push_back(UINT32_MAX);
        coneConstraints.anchIdx.push_back(UINT32_MAX);
        coneConstraints.objIdx.push_back(UINT32_MAX);
        coneConstraints.rootIdx.push_back(UINT32_MAX);
        coneConstraints.colorGraph.push_back(0);

        coneConstraints.restDir.push_back(vZero);
        coneConstraints.alignRot.push_back(vZero);
        coneConstraints.invAlignRot.push_back(vZero);
        coneConstraints.compliancePositive.push_back(vZero);
        coneConstraints.complianceNegative.push_back(vZero);
        coneConstraints.marginPositive.push_back(vZero);
        coneConstraints.marginNegative.push_back(vZero);
        coneConstraints.limitPositive.push_back(vInf);
        coneConstraints.limitNegative.push_back(vNegInf);
        coneConstraints.dampingPositive.push_back(vZero);
        coneConstraints.dampingNegative.push_back(vZero);

        coneConstraints.lambda.push_back(0);

        return newIdx;
    }
    void XPBDWorld::ReserveConeConstraint(std::uint32_t n)
    {
        if (n == 0)
            return;
        n += coneConstraints.numConstraints;
        coneConstraints.boneIdx.reserve(n);
        coneConstraints.anchIdx.reserve(n);
        coneConstraints.objIdx.reserve(n);
        coneConstraints.rootIdx.reserve(n);
        coneConstraints.colorGraph.reserve(n);

        coneConstraints.restDir.reserve(n);
        coneConstraints.alignRot.reserve(n);
        coneConstraints.invAlignRot.reserve(n);
        coneConstraints.compliancePositive.reserve(n);
        coneConstraints.complianceNegative.reserve(n);
        coneConstraints.marginPositive.reserve(n);
        coneConstraints.marginNegative.reserve(n);
        coneConstraints.limitPositive.reserve(n);
        coneConstraints.limitNegative.reserve(n);
        coneConstraints.dampingPositive.reserve(n);
        coneConstraints.dampingNegative.reserve(n);

        coneConstraints.lambda.reserve(n);
    }

    std::uint32_t XPBDWorld::AllocateDeformConstraint()
    {
        const std::uint32_t newIdx = deformConstraints.numConstraints++;
        deformConstraints.boneIdx.push_back(UINT32_MAX);
        deformConstraints.anchIdx.push_back(UINT32_MAX);
        deformConstraints.objIdx.push_back(UINT32_MAX);
        deformConstraints.rootIdx.push_back(UINT32_MAX);

        deformConstraints.restLen.push_back(0);
        deformConstraints.restRot.push_back(qZero);
        deformConstraints.squishWeight.push_back(vZero);
        deformConstraints.stretchWeight.push_back(vZero);
        deformConstraints.bulgeWeight.push_back(vZero);
        return newIdx;
    }
    void XPBDWorld::ReserveDeformConstraint(std::uint32_t n)
    {
        if (n == 0)
            return;
        n += deformConstraints.numConstraints;
        deformConstraints.boneIdx.reserve(n);
        deformConstraints.anchIdx.reserve(n);
        deformConstraints.objIdx.reserve(n);
        deformConstraints.rootIdx.reserve(n);

        deformConstraints.restLen.reserve(n);
        deformConstraints.restRot.reserve(n);
        deformConstraints.squishWeight.reserve(n);
        deformConstraints.stretchWeight.reserve(n);
        deformConstraints.bulgeWeight.reserve(n);
    }

    std::uint32_t XPBDWorld::AllocateShapeMatchingConstraint()
    {
        const std::uint32_t newIdx = shapeMatchingConstraints.numConstraints++;
        shapeMatchingConstraints.objIdx.push_back(UINT32_MAX);
        shapeMatchingConstraints.rootIdx.push_back(UINT32_MAX);
        shapeMatchingConstraints.cluster.push_back({});
        shapeMatchingConstraints.cluster[newIdx].offset = shapeMatchingConstraints.boneIdx.size();
        shapeMatchingConstraints.cluster[newIdx].size = 0;
        return newIdx;
    }
    std::uint32_t XPBDWorld::AllocateShapeMatchingConstraintCluster(std::uint32_t ci)
    {
        const std::uint32_t newIdx = shapeMatchingConstraints.cluster[ci].offset + shapeMatchingConstraints.cluster[ci].size++;
        shapeMatchingConstraints.boneIdx.push_back(UINT32_MAX);
        shapeMatchingConstraints.boneMass.push_back(0);
        shapeMatchingConstraints.restRelativePos.push_back(vZero);
        return newIdx;
    }
    void XPBDWorld::ReserveShapeMatchingConstraint(std::uint32_t n)
    {
        if (n == 0)
            return;
        n += shapeMatchingConstraints.numConstraints;
        shapeMatchingConstraints.objIdx.reserve(n);
        shapeMatchingConstraints.rootIdx.reserve(n);
        shapeMatchingConstraints.cluster.reserve(n);
    }
    void XPBDWorld::ReserveShapeMatchingConstraintCluster(std::uint32_t n)
    {
        if (n == 0)
            return;
        n += shapeMatchingConstraints.boneIdx.size();
        shapeMatchingConstraints.boneIdx.reserve(n);
        shapeMatchingConstraints.boneMass.reserve(n);
        shapeMatchingConstraints.restRelativePos.reserve(n);
    }

    std::uint32_t XPBDWorld::AllocateCollider()
    {
        const std::uint32_t newIdx = colliders.numColliders++;
        colliders.boneIdx.push_back(UINT32_MAX);
        colliders.objIdx.push_back(UINT32_MAX);
        colliders.rootIdx.push_back(UINT32_MAX);

        colliders.noCollideCount.push_back(0);
        for (std::uint32_t i = 0; i < NOCOLLIDE_MAX; ++i)
        {
            colliders.noCollideBoneIdx.push_back(UINT32_MAX);
        }

        colliders.boundingAABB.push_back(AABB());
        colliders.boundingSphere.push_back(0);
        colliders.boundingSphereCenter.push_back(vZero);

        colliders.colliderType.push_back(ColliderType::kNone);
        colliders.convexHullData.push_back(ConvexHullDataBatch());
        colliders.sphereData.push_back(SphereData());

        return newIdx;
    }
    void XPBDWorld::ReserveCollider(std::uint32_t n)
    {
        if (n == 0)
            return;
        n += colliders.numColliders;
        colliders.boneIdx.reserve(n);
        colliders.objIdx.reserve(n);
        colliders.rootIdx.reserve(n);

        colliders.noCollideCount.reserve(n);
        const std::uint32_t nn = n * NOCOLLIDE_MAX;
        colliders.noCollideBoneIdx.reserve(nn);

        colliders.boundingAABB.reserve(n);
        colliders.boundingSphere.reserve(n);
        colliders.boundingSphereCenter.reserve(n);

        colliders.colliderType.reserve(n);
        colliders.convexHullData.reserve(n);
        colliders.sphereData.reserve(n);
    }

    void XPBDWorld::ReorderMaps()
    {
        // logger::info("Reorder maps...");

        threadPool->Execute([&] {
            { // PhysicsBones
                std::vector<std::uint32_t> oldToNewBoneIdx(physicsBones.numBones, UINT32_MAX);
                physicsBonesOrder.resize(physicsBones.numBones);
                std::iota(physicsBonesOrder.begin(), physicsBonesOrder.end(), 0);
                std::ranges::sort(physicsBonesOrder, [&](std::uint32_t a, std::uint32_t b) {
                    if ((physicsBones.node[a] == nullptr && !physicsBones.isParticle[a]) != (physicsBones.node[b] == nullptr && !physicsBones.isParticle[b]))
                    {
                        return (physicsBones.node[a] != nullptr || physicsBones.isParticle[a]) > (physicsBones.node[b] != nullptr || physicsBones.isParticle[b]);
                    }
                    const std::uint32_t objIdxA = physicsBones.objIdx[a];
                    const std::uint32_t objIdxB = physicsBones.objIdx[b];
                    if (objIdxA != objIdxB)
                        return objIdxA < objIdxB;
                    if (physicsBones.rootIdx[a] != physicsBones.rootIdx[b])
                        return physicsBones.rootIdx[a] < physicsBones.rootIdx[b];
                    if (physicsBones.depth[a] != physicsBones.depth[b])
                        return physicsBones.depth[a] < physicsBones.depth[b];
                    const bool isParticleA = physicsBones.isParticle[a];
                    const bool isParticleB = physicsBones.isParticle[b];
                    if (isParticleA != isParticleB)
                        return !isParticleA > !isParticleB;
                    if (isParticleA && isParticleB)
                        return physicsBones.particleDepth[a] < physicsBones.particleDepth[b];
                    return false;
                });

                physicsBonesGroup.clear();
                physicsBonesRoots.clear();
                std::uint32_t currentObjIdx = UINT32_MAX;
                std::uint32_t currentRootIdx = UINT32_MAX;
                std::uint32_t validCount = 0;
                for (std::uint32_t bi = 0; bi < physicsBones.numBones; ++bi)
                {
                    const auto& orig = physicsBonesOrder[bi];
                    if ((physicsBones.node[orig] == nullptr && !physicsBones.isParticle[orig]) ||
                        physicsBones.objIdx[orig] == UINT32_MAX ||
                        physicsBones.rootIdx[orig] == UINT32_MAX)
                        break;
                    if (currentObjIdx != physicsBones.objIdx[orig])
                    {
                        currentObjIdx = physicsBones.objIdx[orig];
                        physicsBonesGroup.push_back(bi);
                    }
                    if (currentRootIdx != physicsBones.rootIdx[orig])
                    {
                        currentRootIdx = physicsBones.rootIdx[orig];
                        physicsBonesRoots.push_back(bi);
                    }
                    oldToNewBoneIdx[orig] = bi;
                    validCount++;
                }
                if (!physicsBonesGroup.empty())
                    physicsBonesGroup.push_back(validCount);
                if (!physicsBonesRoots.empty())
                    physicsBonesRoots.push_back(validCount);

                {
                    const Internal::PhysicsBones tmpPhysicsBones = physicsBones;
                    tbb::parallel_for(
                        tbb::blocked_range<std::uint32_t>(0, validCount, 128),
                        [&](const tbb::blocked_range<std::uint32_t>& r) {
                            for (std::uint32_t i = r.begin(); i != r.end(); ++i)
                            {
                                const std::uint32_t srcIdx = physicsBonesOrder[i];

                                physicsBones.pos[i] = tmpPhysicsBones.pos[srcIdx];
                                physicsBones.prevPos[i] = tmpPhysicsBones.prevPos[srcIdx];
                                physicsBones.predPos[i] = tmpPhysicsBones.predPos[srcIdx];
                                physicsBones.backupPos[i] = tmpPhysicsBones.backupPos[srcIdx];
                                physicsBones.posVel[i] = tmpPhysicsBones.posVel[srcIdx];

                                physicsBones.advancedRotation[i] = tmpPhysicsBones.advancedRotation[srcIdx];
                                physicsBones.rot[i] = tmpPhysicsBones.rot[srcIdx];
                                physicsBones.predRot[i] = tmpPhysicsBones.predRot[srcIdx];
                                physicsBones.backupRot[i] = tmpPhysicsBones.backupRot[srcIdx];
                                physicsBones.angVel[i] = tmpPhysicsBones.angVel[srcIdx];

                                physicsBones.dampingPositive[i] = tmpPhysicsBones.dampingPositive[srcIdx];
                                physicsBones.dampingNegative[i] = tmpPhysicsBones.dampingNegative[srcIdx];
                                physicsBones.angularDampingPositive[i] = tmpPhysicsBones.angularDampingPositive[srcIdx];
                                physicsBones.angularDampingNegative[i] = tmpPhysicsBones.angularDampingNegative[srcIdx];
                                physicsBones.inertiaPositive[i] = tmpPhysicsBones.inertiaPositive[srcIdx];
                                physicsBones.inertiaNegative[i] = tmpPhysicsBones.inertiaNegative[srcIdx];
                                physicsBones.invAngularInertiaPositive[i] = tmpPhysicsBones.invAngularInertiaPositive[srcIdx];
                                physicsBones.invAngularInertiaNegative[i] = tmpPhysicsBones.invAngularInertiaNegative[srcIdx];
                                physicsBones.angularBlendFactor[i] = tmpPhysicsBones.angularBlendFactor[srcIdx];
                                physicsBones.gravity[i] = tmpPhysicsBones.gravity[srcIdx];
                                physicsBones.offset[i] = tmpPhysicsBones.offset[srcIdx];
                                physicsBones.invMass[i] = tmpPhysicsBones.invMass[srcIdx];
                                physicsBones.windFactor[i] = tmpPhysicsBones.windFactor[srcIdx];
                                physicsBones.physicsScale[i] = tmpPhysicsBones.physicsScale[srcIdx];

                                physicsBones.deformMax[i] = tmpPhysicsBones.deformMax[srcIdx];
                                physicsBones.deformMin[i] = tmpPhysicsBones.deformMin[srcIdx];
                                physicsBones.deformVolumePreservation[i] = tmpPhysicsBones.deformVolumePreservation[srcIdx];
                                physicsBones.deformSquishSensitivity[i] = tmpPhysicsBones.deformSquishSensitivity[srcIdx];
                                physicsBones.deformStretchSensitivity[i] = tmpPhysicsBones.deformStretchSensitivity[srcIdx];
                                physicsBones.deformBulgeSensitivity[i] = tmpPhysicsBones.deformBulgeSensitivity[srcIdx];
                                physicsBones.deformSquishStiffness[i] = tmpPhysicsBones.deformSquishStiffness[srcIdx];
                                physicsBones.deformStretchStiffness[i] = tmpPhysicsBones.deformStretchStiffness[srcIdx];
                                physicsBones.deformSquishDamping[i] = tmpPhysicsBones.deformSquishDamping[srcIdx];
                                physicsBones.deformStretchDamping[i] = tmpPhysicsBones.deformStretchDamping[srcIdx];

                                physicsBones.deformScale[i] = tmpPhysicsBones.deformScale[srcIdx];
                                physicsBones.deformScaleCache[i] = tmpPhysicsBones.deformScaleCache[srcIdx];
                                physicsBones.deformVelocityScale[i] = tmpPhysicsBones.deformVelocityScale[srcIdx];
                                physicsBones.deformCount[i] = tmpPhysicsBones.deformCount[srcIdx];

                                physicsBones.animDriveCompliance[i] = tmpPhysicsBones.animDriveCompliance[srcIdx];
                                physicsBones.animDriveLambda[i] = tmpPhysicsBones.animDriveLambda[srcIdx];
                                physicsBones.animDriveAngularCompliance[i] = tmpPhysicsBones.animDriveAngularCompliance[srcIdx];
                                physicsBones.animDriveAngularLambda[i] = tmpPhysicsBones.animDriveAngularLambda[srcIdx];

                                physicsBones.linearRotTorque[i] = tmpPhysicsBones.linearRotTorque[srcIdx];
                                physicsBones.centerOfMass[i] = tmpPhysicsBones.centerOfMass[srcIdx];
                                physicsBones.dynamicLinearOffset[i] = tmpPhysicsBones.dynamicLinearOffset[srcIdx];
                                physicsBones.dynamicAngularOffset[i] = tmpPhysicsBones.dynamicAngularOffset[srcIdx];

                                physicsBones.collisionMargin[i] = tmpPhysicsBones.collisionMargin[srcIdx];
                                physicsBones.collisionShrink[i] = tmpPhysicsBones.collisionShrink[srcIdx];
                                physicsBones.collisionFriction[i] = tmpPhysicsBones.collisionFriction[srcIdx];
                                physicsBones.collisionCompliance[i] = tmpPhysicsBones.collisionCompliance[srcIdx];
                                physicsBones.collisionRestitution[i] = tmpPhysicsBones.collisionRestitution[srcIdx];

                                physicsBones.layerGroup[i] = tmpPhysicsBones.layerGroup[srcIdx];
                                physicsBones.collideLayer[i] = tmpPhysicsBones.collideLayer[srcIdx];

                                physicsBones.frictionCache[i] = tmpPhysicsBones.frictionCache[srcIdx];
                                physicsBones.deformCache[i] = tmpPhysicsBones.deformCache[srcIdx];
                                physicsBones.groundCache[i] = tmpPhysicsBones.groundCache[srcIdx];

                                physicsBones.node[i] = tmpPhysicsBones.node[srcIdx];
                                physicsBones.particleName[i] = tmpPhysicsBones.particleName[srcIdx];
                                physicsBones.isParticle[i] = tmpPhysicsBones.isParticle[srcIdx];
                                physicsBones.particleDepth[i] = tmpPhysicsBones.particleDepth[srcIdx];
                                physicsBones.parentBoneIdx[i] = tmpPhysicsBones.parentBoneIdx[srcIdx];
                                physicsBones.objIdx[i] = tmpPhysicsBones.objIdx[srcIdx];
                                physicsBones.rootIdx[i] = tmpPhysicsBones.rootIdx[srcIdx];
                                physicsBones.depth[i] = tmpPhysicsBones.depth[srcIdx];

                                physicsBones.prevNodeWorldPos[i] = tmpPhysicsBones.prevNodeWorldPos[srcIdx];
                                physicsBones.targetNodeWorldPos[i] = tmpPhysicsBones.targetNodeWorldPos[srcIdx];
                                physicsBones.prevNodeWorldRot[i] = tmpPhysicsBones.prevNodeWorldRot[srcIdx];
                                physicsBones.targetNodeWorldRot[i] = tmpPhysicsBones.targetNodeWorldRot[srcIdx];

                                physicsBones.alignRot[i] = tmpPhysicsBones.alignRot[srcIdx];
                                physicsBones.invAlignRot[i] = tmpPhysicsBones.invAlignRot[srcIdx];

                                physicsBones.orgWorldScale[i] = tmpPhysicsBones.orgWorldScale[srcIdx];
                                physicsBones.orgLocalPos[i] = tmpPhysicsBones.orgLocalPos[srcIdx];
                                physicsBones.orgLocalRot[i] = tmpPhysicsBones.orgLocalRot[srcIdx];

                                physicsBones.lastFrame[i] = tmpPhysicsBones.lastFrame[srcIdx];
                            }
                        },
                        tbb::static_partitioner()
                    );

                    physicsBones.pos.resize(validCount);
                    physicsBones.predPos.resize(validCount);
                    physicsBones.backupPos.resize(validCount);
                    physicsBones.posVel.resize(validCount);

                    physicsBones.advancedRotation.resize(validCount);
                    physicsBones.rot.resize(validCount);
                    physicsBones.predRot.resize(validCount);
                    physicsBones.angVel.resize(validCount);

                    physicsBones.dampingPositive.resize(validCount);
                    physicsBones.dampingNegative.resize(validCount);
                    physicsBones.angularDampingPositive.resize(validCount);
                    physicsBones.angularDampingNegative.resize(validCount);
                    physicsBones.inertiaPositive.resize(validCount);
                    physicsBones.inertiaNegative.resize(validCount);
                    physicsBones.invAngularInertiaPositive.resize(validCount);
                    physicsBones.invAngularInertiaNegative.resize(validCount);
                    physicsBones.angularBlendFactor.resize(validCount);
                    physicsBones.gravity.resize(validCount);
                    physicsBones.offset.resize(validCount);
                    physicsBones.invMass.resize(validCount);
                    physicsBones.windFactor.resize(validCount);
                    physicsBones.physicsScale.resize(validCount);

                    physicsBones.deformMax.resize(validCount);
                    physicsBones.deformMin.resize(validCount);
                    physicsBones.deformVolumePreservation.resize(validCount);
                    physicsBones.deformSquishSensitivity.resize(validCount);
                    physicsBones.deformStretchSensitivity.resize(validCount);
                    physicsBones.deformBulgeSensitivity.resize(validCount);
                    physicsBones.deformSquishStiffness.resize(validCount);
                    physicsBones.deformStretchStiffness.resize(validCount);
                    physicsBones.deformSquishDamping.resize(validCount);
                    physicsBones.deformStretchDamping.resize(validCount);

                    physicsBones.deformScale.resize(validCount);
                    physicsBones.deformScaleCache.resize(validCount);
                    physicsBones.deformVelocityScale.resize(validCount);
                    physicsBones.deformCount.resize(validCount);

                    physicsBones.animDriveCompliance.resize(validCount);
                    physicsBones.animDriveLambda.resize(validCount);
                    physicsBones.animDriveAngularCompliance.resize(validCount);
                    physicsBones.animDriveAngularLambda.resize(validCount);

                    physicsBones.linearRotTorque.resize(validCount);
                    physicsBones.centerOfMass.resize(validCount);
                    physicsBones.dynamicLinearOffset.resize(validCount);
                    physicsBones.dynamicAngularOffset.resize(validCount);

                    physicsBones.collisionMargin.resize(validCount);
                    physicsBones.collisionShrink.resize(validCount);
                    physicsBones.collisionFriction.resize(validCount);
                    physicsBones.collisionCompliance.resize(validCount);
                    physicsBones.collisionRestitution.resize(validCount);

                    physicsBones.layerGroup.resize(validCount);
                    physicsBones.collideLayer.resize(validCount);

                    physicsBones.frictionCache.resize(validCount);
                    physicsBones.deformCache.resize(validCount);
                    physicsBones.groundCache.resize(validCount);

                    physicsBones.node.resize(validCount);
                    physicsBones.particleName.resize(validCount);
                    physicsBones.isParticle.resize(validCount);
                    physicsBones.particleDepth.resize(validCount);
                    physicsBones.parentBoneIdx.resize(validCount);
                    physicsBones.objIdx.resize(validCount);
                    physicsBones.rootIdx.resize(validCount);
                    physicsBones.depth.resize(validCount);

                    physicsBones.prevNodeWorldPos.resize(validCount);
                    physicsBones.targetNodeWorldPos.resize(validCount);
                    physicsBones.prevNodeWorldRot.resize(validCount);
                    physicsBones.targetNodeWorldRot.resize(validCount);

                    physicsBones.alignRot.resize(validCount);
                    physicsBones.invAlignRot.resize(validCount);

                    physicsBones.orgWorldScale.resize(validCount);
                    physicsBones.orgLocalPos.resize(validCount);
                    physicsBones.orgLocalRot.resize(validCount);

                    physicsBones.lastFrame.resize(validCount);

                    physicsBonesLock = std::make_unique<tbb::spin_mutex[]>(validCount);
                }
                tbb::parallel_invoke(
                    [&] {
                        physicsBones.numBones = validCount;
                        std::iota(physicsBonesOrder.begin(), physicsBonesOrder.end(), 0);
                        physicsBonesOrder.resize(validCount);

                        // caching bones
                        if (!physicsBonesGroup.empty())
                        {
                            const std::uint32_t groups = physicsBonesGroup.size() - 1;
                            for (std::uint32_t g = 0; g < groups; ++g)
                            {
                                const std::uint32_t begin = physicsBonesGroup[g];
                                const std::uint32_t end = physicsBonesGroup[g + 1];
                                const std::uint32_t oi = physicsBones.objIdx[begin];
                                boneNameToIdx[oi].clear();
                                for (std::uint32_t bi = begin; bi < end; ++bi)
                                {
                                    if (auto& node = physicsBones.node[bi]; node && !node->name.empty())
                                        boneNameToIdx[oi][node->name.c_str()] = bi;
                                    else if (physicsBones.isParticle[bi])
                                        boneNameToIdx[oi][physicsBones.particleName[bi]] = bi;
                                }
                            }
                        }
                    },
                    [&] {
                        for (std::uint32_t i = 0; i < validCount; ++i)
                        {
                            if (physicsBones.parentBoneIdx[i] != UINT32_MAX)
                                physicsBones.parentBoneIdx[i] = oldToNewBoneIdx[physicsBones.parentBoneIdx[i]];
                        }
                    },
                    [&] {
                        for (std::uint32_t i = 0; i < distanceConstraints.numConstraints; ++i)
                        {
                            if (distanceConstraints.boneIdx[i] != UINT32_MAX)
                                distanceConstraints.boneIdx[i] = oldToNewBoneIdx[distanceConstraints.boneIdx[i]];
                            if (distanceConstraints.anchIdx[i] != UINT32_MAX)
                                distanceConstraints.anchIdx[i] = oldToNewBoneIdx[distanceConstraints.anchIdx[i]];
                        }
                    },
                    [&] {
                        for (std::uint32_t i = 0; i < angularConstraints.numConstraints; ++i)
                        {
                            if (angularConstraints.boneIdx[i] != UINT32_MAX)
                                angularConstraints.boneIdx[i] = oldToNewBoneIdx[angularConstraints.boneIdx[i]];
                            if (angularConstraints.anchIdx[i] != UINT32_MAX)
                                angularConstraints.anchIdx[i] = oldToNewBoneIdx[angularConstraints.anchIdx[i]];
                        }
                    },
                    [&] {
                        for (std::uint32_t i = 0; i < coneConstraints.numConstraints; ++i)
                        {
                            if (coneConstraints.boneIdx[i] != UINT32_MAX)
                                coneConstraints.boneIdx[i] = oldToNewBoneIdx[coneConstraints.boneIdx[i]];
                            if (coneConstraints.anchIdx[i] != UINT32_MAX)
                                coneConstraints.anchIdx[i] = oldToNewBoneIdx[coneConstraints.anchIdx[i]];
                        }
                    },
                    [&] {
                        for (std::uint32_t i = 0; i < deformConstraints.numConstraints; ++i)
                        {
                            if (deformConstraints.boneIdx[i] != UINT32_MAX)
                                deformConstraints.boneIdx[i] = oldToNewBoneIdx[deformConstraints.boneIdx[i]];
                            if (deformConstraints.anchIdx[i] != UINT32_MAX)
                                deformConstraints.anchIdx[i] = oldToNewBoneIdx[deformConstraints.anchIdx[i]];
                        }
                    },
                    [&] {
                        for (std::uint32_t i = 0; i < shapeMatchingConstraints.numConstraints; ++i)
                        {
                            if (shapeMatchingConstraints.boneIdx[i] != UINT32_MAX)
                                shapeMatchingConstraints.boneIdx[i] = oldToNewBoneIdx[shapeMatchingConstraints.boneIdx[i]];
                        }
                    },
                    [&] {
                        for (std::uint32_t i = 0; i < colliders.numColliders; ++i)
                        {
                            if (colliders.boneIdx[i] != UINT32_MAX)
                                colliders.boneIdx[i] = oldToNewBoneIdx[colliders.boneIdx[i]];

                            const std::uint32_t noColBase = static_cast<std::uint32_t>(i) * NOCOLLIDE_MAX;
                            for (std::uint32_t nc = 0; nc < NOCOLLIDE_MAX; ++nc)
                            {
                                const std::uint32_t ncIdx = noColBase + nc;
                                if (colliders.noCollideBoneIdx[ncIdx] != UINT32_MAX)
                                {
                                    colliders.noCollideBoneIdx[ncIdx] = oldToNewBoneIdx[colliders.noCollideBoneIdx[ncIdx]];
                                }
                            }
                        }
                    },
                    [&] {
                        driver->UpdateNewBoneIndex(nullptr, oldToNewBoneIdx);
                    }
                );
            }

            auto UpdateConsGroup = [](auto& cons, std::vector<std::uint32_t>& group, const std::vector<std::uint32_t>& order) {
                group.clear();
                std::uint32_t currentObjIdx = UINT32_MAX;
                std::uint32_t validCount = 0;
                for (std::uint32_t coi = 0; coi < cons.numConstraints; ++coi)
                {
                    const auto& oi = order[coi];
                    if (cons.objIdx[oi] == UINT32_MAX ||
                        cons.rootIdx[oi] == UINT32_MAX)
                        break;
                    if (cons.boneIdx[oi] == UINT32_MAX || cons.anchIdx[oi] == UINT32_MAX)
                        break;
                    if (currentObjIdx != cons.objIdx[oi])
                    {
                        currentObjIdx = cons.objIdx[oi];
                        group.push_back(coi);
                    }
                    validCount++;
                }
                if (!group.empty())
                    group.push_back(validCount);
                return validCount;
            };

            tbb::parallel_invoke( 
                [&] { // DistanceConstraints
                    distanceConstraintsOrder.resize(distanceConstraints.numConstraints);
                    std::iota(distanceConstraintsOrder.begin(), distanceConstraintsOrder.end(), 0);
                    std::ranges::sort(distanceConstraintsOrder, [&](std::uint32_t a, std::uint32_t b) {
                        if ((distanceConstraints.boneIdx[a] == UINT32_MAX) != (distanceConstraints.boneIdx[b] == UINT32_MAX))
                            return (distanceConstraints.boneIdx[a] != UINT32_MAX) > (distanceConstraints.boneIdx[b] != UINT32_MAX);
                        if ((distanceConstraints.anchIdx[a] == UINT32_MAX) != (distanceConstraints.anchIdx[b] == UINT32_MAX))
                            return (distanceConstraints.anchIdx[a] != UINT32_MAX) > (distanceConstraints.anchIdx[b] != UINT32_MAX);
                        if (distanceConstraints.objIdx[a] != distanceConstraints.objIdx[b])
                            return distanceConstraints.objIdx[a] < distanceConstraints.objIdx[b];
                        if (distanceConstraints.rootIdx[a] != distanceConstraints.rootIdx[b])
                            return distanceConstraints.rootIdx[a] < distanceConstraints.rootIdx[b];
                        if (distanceConstraints.boneIdx[a] == distanceConstraints.boneIdx[b])
                            return distanceConstraints.anchIdx[a] < distanceConstraints.anchIdx[b];
                        return distanceConstraints.boneIdx[a] < distanceConstraints.boneIdx[b];
                    });

                    const std::uint32_t validCount = UpdateConsGroup(distanceConstraints, distanceConstraintsGroup, distanceConstraintsOrder);

                    {
                        const Internal::DistanceConstraints tmpDistCons = distanceConstraints;
                        tbb::parallel_for(
                            tbb::blocked_range<std::uint32_t>(0, validCount, 128),
                            [&](const tbb::blocked_range<std::uint32_t>& r) {
                                for (std::uint32_t i = r.begin(); i != r.end(); ++i)
                                {
                                    const std::uint32_t srcIdx = distanceConstraintsOrder[i];

                                    distanceConstraints.boneIdx[i] = tmpDistCons.boneIdx[srcIdx];
                                    distanceConstraints.anchIdx[i] = tmpDistCons.anchIdx[srcIdx];
                                    distanceConstraints.objIdx[i] = tmpDistCons.objIdx[srcIdx];
                                    distanceConstraints.rootIdx[i] = tmpDistCons.rootIdx[srcIdx];
                                    distanceConstraints.colorGraph[i] = tmpDistCons.colorGraph[srcIdx];

                                    distanceConstraints.restLen[i] = tmpDistCons.restLen[srcIdx];
                                    distanceConstraints.complianceSquish[i] = tmpDistCons.complianceSquish[srcIdx];
                                    distanceConstraints.complianceStretch[i] = tmpDistCons.complianceStretch[srcIdx];
                                    distanceConstraints.squishMargin[i] = tmpDistCons.squishMargin[srcIdx];
                                    distanceConstraints.stretchMargin[i] = tmpDistCons.stretchMargin[srcIdx];
                                    distanceConstraints.squishLimit[i] = tmpDistCons.squishLimit[srcIdx];
                                    distanceConstraints.stretchLimit[i] = tmpDistCons.stretchLimit[srcIdx];
                                    distanceConstraints.squishDamping[i] = tmpDistCons.squishDamping[srcIdx];
                                    distanceConstraints.stretchDamping[i] = tmpDistCons.stretchDamping[srcIdx];
                                }
                            },
                            tbb::static_partitioner());
                    }
                    distanceConstraints.boneIdx.resize(validCount);
                    distanceConstraints.anchIdx.resize(validCount);
                    distanceConstraints.objIdx.resize(validCount);
                    distanceConstraints.rootIdx.resize(validCount);
                    distanceConstraints.colorGraph.resize(validCount);

                    distanceConstraints.restLen.resize(validCount);
                    distanceConstraints.complianceSquish.resize(validCount);
                    distanceConstraints.complianceStretch.resize(validCount);
                    distanceConstraints.squishMargin.resize(validCount);
                    distanceConstraints.stretchMargin.resize(validCount);
                    distanceConstraints.squishLimit.resize(validCount);
                    distanceConstraints.stretchLimit.resize(validCount);
                    distanceConstraints.squishDamping.resize(validCount);
                    distanceConstraints.stretchDamping.resize(validCount);

                    distanceConstraints.lambda.resize(validCount);

                    distanceConstraints.numConstraints = validCount;
                    distanceConstraintsOrder.resize(validCount);
                    std::iota(distanceConstraintsOrder.begin(), distanceConstraintsOrder.end(), 0);
                },
                [&] { // AngularConstraints
                    angularConstraintsOrder.resize(angularConstraints.numConstraints);
                    std::iota(angularConstraintsOrder.begin(), angularConstraintsOrder.end(), 0);
                    std::ranges::sort(angularConstraintsOrder, [&](std::uint32_t a, std::uint32_t b) {
                        if ((angularConstraints.boneIdx[a] == UINT32_MAX) != (angularConstraints.boneIdx[b] == UINT32_MAX))
                            return (angularConstraints.boneIdx[a] != UINT32_MAX) > (angularConstraints.boneIdx[b] != UINT32_MAX);
                        if ((angularConstraints.anchIdx[a] == UINT32_MAX) != (angularConstraints.anchIdx[b] == UINT32_MAX))
                            return (angularConstraints.anchIdx[a] != UINT32_MAX) > (angularConstraints.anchIdx[b] != UINT32_MAX);
                        if (angularConstraints.objIdx[a] != angularConstraints.objIdx[b])
                            return angularConstraints.objIdx[a] < angularConstraints.objIdx[b];
                        if (angularConstraints.rootIdx[a] != angularConstraints.rootIdx[b])
                            return angularConstraints.rootIdx[a] < angularConstraints.rootIdx[b];
                        if (angularConstraints.boneIdx[a] == angularConstraints.boneIdx[b])
                            return angularConstraints.anchIdx[a] < angularConstraints.anchIdx[b];
                        return angularConstraints.boneIdx[a] < angularConstraints.boneIdx[b];
                    });

                    const std::uint32_t validCount = UpdateConsGroup(angularConstraints, angularConstraintsGroup, angularConstraintsOrder);

                    {
                        const Internal::AngularConstraints tmpAngCons = angularConstraints;
                        tbb::parallel_for(
                            tbb::blocked_range<std::uint32_t>(0, validCount, 128),
                            [&](const tbb::blocked_range<std::uint32_t>& r) {
                                for (std::uint32_t i = r.begin(); i != r.end(); ++i)
                                {
                                    const std::uint32_t srcIdx = angularConstraintsOrder[i];

                                    angularConstraints.boneIdx[i] = tmpAngCons.boneIdx[srcIdx];
                                    angularConstraints.anchIdx[i] = tmpAngCons.anchIdx[srcIdx];
                                    angularConstraints.objIdx[i] = tmpAngCons.objIdx[srcIdx];
                                    angularConstraints.rootIdx[i] = tmpAngCons.rootIdx[srcIdx];
                                    angularConstraints.colorGraph[i] = tmpAngCons.colorGraph[srcIdx];

                                    angularConstraints.restRot[i] = tmpAngCons.restRot[srcIdx];
                                    angularConstraints.compliancePositive[i] = tmpAngCons.compliancePositive[srcIdx];
                                    angularConstraints.complianceNegative[i] = tmpAngCons.complianceNegative[srcIdx];
                                    angularConstraints.marginPositive[i] = tmpAngCons.marginPositive[srcIdx];
                                    angularConstraints.marginNegative[i] = tmpAngCons.marginNegative[srcIdx];
                                    angularConstraints.limitPositive[i] = tmpAngCons.limitPositive[srcIdx];
                                    angularConstraints.limitNegative[i] = tmpAngCons.limitNegative[srcIdx];
                                    angularConstraints.dampingPositive[i] = tmpAngCons.dampingPositive[srcIdx];
                                    angularConstraints.dampingNegative[i] = tmpAngCons.dampingNegative[srcIdx];
                                }
                            },
                            tbb::static_partitioner());

                        angularConstraints.boneIdx.resize(validCount);
                        angularConstraints.anchIdx.resize(validCount);
                        angularConstraints.objIdx.resize(validCount);
                        angularConstraints.rootIdx.resize(validCount);
                        angularConstraints.colorGraph.resize(validCount);

                        angularConstraints.restRot.resize(validCount);
                        angularConstraints.compliancePositive.resize(validCount);
                        angularConstraints.complianceNegative.resize(validCount);
                        angularConstraints.marginPositive.resize(validCount);
                        angularConstraints.marginNegative.resize(validCount);
                        angularConstraints.limitPositive.resize(validCount);
                        angularConstraints.limitNegative.resize(validCount);
                        angularConstraints.dampingPositive.resize(validCount);
                        angularConstraints.dampingNegative.resize(validCount);

                        angularConstraints.lambda.resize(validCount);
                    }

                    angularConstraints.numConstraints = validCount;
                    angularConstraintsOrder.resize(validCount);
                    std::iota(angularConstraintsOrder.begin(), angularConstraintsOrder.end(), 0);
                },
                [&] { // ConeConstraints
                    coneConstraintsOrder.resize(coneConstraints.numConstraints);
                    std::iota(coneConstraintsOrder.begin(), coneConstraintsOrder.end(), 0);
                    std::ranges::sort(coneConstraintsOrder, [&](std::uint32_t a, std::uint32_t b) {
                        if ((coneConstraints.boneIdx[a] == UINT32_MAX) != (coneConstraints.boneIdx[b] == UINT32_MAX))
                            return (coneConstraints.boneIdx[a] != UINT32_MAX) > (coneConstraints.boneIdx[b] != UINT32_MAX);
                        if ((coneConstraints.anchIdx[a] == UINT32_MAX) != (coneConstraints.anchIdx[b] == UINT32_MAX))
                            return (coneConstraints.anchIdx[a] != UINT32_MAX) > (coneConstraints.anchIdx[b] != UINT32_MAX);
                        if (coneConstraints.objIdx[a] != coneConstraints.objIdx[b])
                            return coneConstraints.objIdx[a] < coneConstraints.objIdx[b];
                        if (coneConstraints.rootIdx[a] != coneConstraints.rootIdx[b])
                            return coneConstraints.rootIdx[a] < coneConstraints.rootIdx[b];
                        if (coneConstraints.boneIdx[a] == coneConstraints.boneIdx[b])
                            return coneConstraints.anchIdx[a] < coneConstraints.anchIdx[b];
                        return coneConstraints.boneIdx[a] < coneConstraints.boneIdx[b];
                    });

                    const std::uint32_t validCount = UpdateConsGroup(coneConstraints, coneConstraintsGroup, coneConstraintsOrder);

                    {
                        const Internal::ConeConstraints tmpConeCons = coneConstraints;
                        tbb::parallel_for(
                            tbb::blocked_range<std::uint32_t>(0, validCount, 128),
                            [&](const tbb::blocked_range<std::uint32_t>& r) {
                                for (std::uint32_t i = r.begin(); i != r.end(); ++i)
                                {
                                    const std::uint32_t srcIdx = coneConstraintsOrder[i];

                                    coneConstraints.boneIdx[i] = tmpConeCons.boneIdx[srcIdx];
                                    coneConstraints.anchIdx[i] = tmpConeCons.anchIdx[srcIdx];
                                    coneConstraints.objIdx[i] = tmpConeCons.objIdx[srcIdx];
                                    coneConstraints.rootIdx[i] = tmpConeCons.rootIdx[srcIdx];
                                    coneConstraints.colorGraph[i] = tmpConeCons.colorGraph[srcIdx];

                                    coneConstraints.restDir[i] = tmpConeCons.restDir[srcIdx];
                                    coneConstraints.alignRot[i] = tmpConeCons.alignRot[srcIdx];
                                    coneConstraints.invAlignRot[i] = tmpConeCons.invAlignRot[srcIdx];

                                    coneConstraints.compliancePositive[i] = tmpConeCons.compliancePositive[srcIdx];
                                    coneConstraints.complianceNegative[i] = tmpConeCons.complianceNegative[srcIdx];
                                    coneConstraints.marginPositive[i] = tmpConeCons.marginPositive[srcIdx];
                                    coneConstraints.marginNegative[i] = tmpConeCons.marginNegative[srcIdx];
                                    coneConstraints.limitPositive[i] = tmpConeCons.limitPositive[srcIdx];
                                    coneConstraints.limitNegative[i] = tmpConeCons.limitNegative[srcIdx];
                                    coneConstraints.dampingPositive[i] = tmpConeCons.dampingPositive[srcIdx];
                                    coneConstraints.dampingNegative[i] = tmpConeCons.dampingNegative[srcIdx];
                                }
                            },
                            tbb::static_partitioner());

                        coneConstraints.boneIdx.resize(validCount);
                        coneConstraints.anchIdx.resize(validCount);
                        coneConstraints.objIdx.resize(validCount);
                        coneConstraints.rootIdx.resize(validCount);
                        coneConstraints.colorGraph.resize(validCount);

                        coneConstraints.restDir.resize(validCount);
                        coneConstraints.alignRot.resize(validCount);
                        coneConstraints.invAlignRot.resize(validCount);

                        coneConstraints.compliancePositive.resize(validCount);
                        coneConstraints.complianceNegative.resize(validCount);
                        coneConstraints.marginPositive.resize(validCount);
                        coneConstraints.marginNegative.resize(validCount);
                        coneConstraints.limitPositive.resize(validCount);
                        coneConstraints.limitNegative.resize(validCount);
                        coneConstraints.dampingPositive.resize(validCount);
                        coneConstraints.dampingNegative.resize(validCount);

                        coneConstraints.lambda.resize(validCount);
                    }

                    coneConstraints.numConstraints = validCount;
                    coneConstraintsOrder.resize(validCount);
                    std::iota(coneConstraintsOrder.begin(), coneConstraintsOrder.end(), 0);
                },
                [&] { // DeformConstraints
                    deformConstraintsOrder.resize(deformConstraints.numConstraints);
                    std::iota(deformConstraintsOrder.begin(), deformConstraintsOrder.end(), 0);
                    std::ranges::sort(deformConstraintsOrder, [&](std::uint32_t a, std::uint32_t b) {
                        if ((deformConstraints.boneIdx[a] == UINT32_MAX) != (deformConstraints.boneIdx[b] == UINT32_MAX))
                            return (deformConstraints.boneIdx[a] != UINT32_MAX) > (deformConstraints.boneIdx[b] != UINT32_MAX);
                        if ((deformConstraints.anchIdx[a] == UINT32_MAX) != (deformConstraints.anchIdx[b] == UINT32_MAX))
                            return (deformConstraints.anchIdx[a] != UINT32_MAX) > (deformConstraints.anchIdx[b] != UINT32_MAX);
                        if (deformConstraints.objIdx[a] != deformConstraints.objIdx[b])
                            return deformConstraints.objIdx[a] < deformConstraints.objIdx[b];
                        if (deformConstraints.rootIdx[a] != deformConstraints.rootIdx[b])
                            return deformConstraints.rootIdx[a] < deformConstraints.rootIdx[b];
                        if (deformConstraints.boneIdx[a] == deformConstraints.boneIdx[b])
                            return deformConstraints.anchIdx[a] < deformConstraints.anchIdx[b];
                        return deformConstraints.boneIdx[a] < deformConstraints.boneIdx[b];
                    });

                    const std::uint32_t validCount = UpdateConsGroup(deformConstraints, deformConstraintsGroup, deformConstraintsOrder);

                    {
                        const Internal::DeformConstraints tmpDeformCons = deformConstraints;
                        tbb::parallel_for(
                            tbb::blocked_range<std::uint32_t>(0, validCount, 128),
                            [&](const tbb::blocked_range<std::uint32_t>& r) {
                                for (std::uint32_t i = r.begin(); i != r.end(); ++i)
                                {
                                    const std::uint32_t srcIdx = deformConstraintsOrder[i];

                                    deformConstraints.boneIdx[i] = tmpDeformCons.boneIdx[srcIdx];
                                    deformConstraints.anchIdx[i] = tmpDeformCons.anchIdx[srcIdx];
                                    deformConstraints.objIdx[i] = tmpDeformCons.objIdx[srcIdx];
                                    deformConstraints.rootIdx[i] = tmpDeformCons.rootIdx[srcIdx];

                                    deformConstraints.restLen[i] = tmpDeformCons.restLen[srcIdx];
                                    deformConstraints.restRot[i] = tmpDeformCons.restRot[srcIdx];
                                    deformConstraints.squishWeight[i] = tmpDeformCons.squishWeight[srcIdx];
                                    deformConstraints.stretchWeight[i] = tmpDeformCons.stretchWeight[srcIdx];
                                    deformConstraints.bulgeWeight[i] = tmpDeformCons.bulgeWeight[srcIdx];
                                }
                            },
                            tbb::static_partitioner());

                        deformConstraints.boneIdx.resize(validCount);
                        deformConstraints.anchIdx.resize(validCount);
                        deformConstraints.objIdx.resize(validCount);
                        deformConstraints.rootIdx.resize(validCount);

                        deformConstraints.restLen.resize(validCount);
                        deformConstraints.restRot.resize(validCount);
                        deformConstraints.squishWeight.resize(validCount);
                        deformConstraints.stretchWeight.resize(validCount);
                        deformConstraints.bulgeWeight.resize(validCount);
                    }

                    deformConstraints.numConstraints = validCount;
                    deformConstraintsOrder.resize(validCount);
                    std::iota(deformConstraintsOrder.begin(), deformConstraintsOrder.end(), 0);
                },
                [&] { // ShapeMatchingConstraints
                    shapeMatchingConstraintsOrder.resize(shapeMatchingConstraints.numConstraints);
                    std::iota(shapeMatchingConstraintsOrder.begin(), shapeMatchingConstraintsOrder.end(), 0);
                    std::ranges::sort(shapeMatchingConstraintsOrder, [&](std::uint32_t a, std::uint32_t b) {
                        if (shapeMatchingConstraints.objIdx[a] != shapeMatchingConstraints.objIdx[b])
                            return shapeMatchingConstraints.objIdx[a] < shapeMatchingConstraints.objIdx[b];
                        if (shapeMatchingConstraints.rootIdx[a] != shapeMatchingConstraints.rootIdx[b])
                            return shapeMatchingConstraints.rootIdx[a] < shapeMatchingConstraints.rootIdx[b];
                        return a < b;
                    });

                    std::uint32_t validClusterCount = 0;
                    std::uint32_t totalValidBones = 0;
                    std::vector<std::uint32_t> validOrder;
                    validOrder.reserve(shapeMatchingConstraints.numConstraints);
                    std::vector<std::uint32_t> newOffsets(shapeMatchingConstraints.numConstraints);
                    std::vector<std::uint32_t> newSizes(shapeMatchingConstraints.numConstraints);
                    for (std::uint32_t i = 0; i < shapeMatchingConstraints.numConstraints; ++i)
                    {
                        const std::uint32_t srcIdx = shapeMatchingConstraintsOrder[i];

                        if (shapeMatchingConstraints.objIdx[srcIdx] == UINT32_MAX ||
                            shapeMatchingConstraints.rootIdx[srcIdx] == UINT32_MAX)
                            break;

                        const auto& oldCluster = shapeMatchingConstraints.cluster[srcIdx];
                        std::uint32_t aliveBones = 0;
                        for (std::uint32_t s = 0; s < oldCluster.size; ++s)
                        {
                            if (shapeMatchingConstraints.boneIdx[oldCluster.offset + s] != UINT32_MAX)
                                aliveBones++;
                        }

                        if (aliveBones >= 3)
                        {
                            newOffsets[validClusterCount] = totalValidBones;
                            newSizes[validClusterCount] = aliveBones;
                            totalValidBones += aliveBones;

                            validOrder.push_back(srcIdx);
                            validClusterCount++;
                        }
                    }

                    if (validClusterCount == 0)
                    {
                        shapeMatchingConstraints.numConstraints = validClusterCount;
                        shapeMatchingConstraintsGroup.clear();
                        return;
                    }

                    std::vector<std::uint32_t> newObjIdx(validClusterCount);
                    std::vector<std::uint32_t> newRootIdx(validClusterCount);
                    std::vector<Internal::ShapeMatchingConstraints::ClusterData> newCluster(validClusterCount);
                    std::vector<std::uint32_t> newBoneIdx(totalValidBones);
                    std::vector<Vector> newRestRelativePos(totalValidBones);
                    std::vector<float> newBoneMass(totalValidBones);
                    tbb::parallel_for(
                        tbb::blocked_range<std::uint32_t>(0, validClusterCount, 64),
                        [&](const tbb::blocked_range<std::uint32_t>& r) {
                            for (std::uint32_t i = r.begin(); i != r.end(); ++i)
                            {
                                const std::uint32_t srcIdx = validOrder[i];

                                newObjIdx[i] = shapeMatchingConstraints.objIdx[srcIdx];
                                newRootIdx[i] = shapeMatchingConstraints.rootIdx[srcIdx];

                                newCluster[i] = shapeMatchingConstraints.cluster[srcIdx];
                                newCluster[i].offset = newOffsets[i];
                                newCluster[i].size = newSizes[i];

                                const std::uint32_t srcStrideBase = shapeMatchingConstraints.cluster[srcIdx].offset;
                                const std::uint32_t oldSize = shapeMatchingConstraints.cluster[srcIdx].size;

                                std::uint32_t dstA = newOffsets[i];
                                for (std::uint32_t s = 0; s < oldSize; ++s)
                                {
                                    const std::uint32_t srcA = srcStrideBase + s;

                                    if (shapeMatchingConstraints.boneIdx[srcA] != UINT32_MAX)
                                    {
                                        newBoneIdx[dstA] = shapeMatchingConstraints.boneIdx[srcA];
                                        newRestRelativePos[dstA] = shapeMatchingConstraints.restRelativePos[srcA];
                                        newBoneMass[dstA] = shapeMatchingConstraints.boneMass[srcA];
                                        dstA++;
                                    }
                                }
                            }
                        },
                        tbb::static_partitioner()
                    );

                    shapeMatchingConstraintsGroup.clear();
                    std::uint32_t currentObjIdx = UINT32_MAX;
                    for (std::uint32_t i = 0; i < validClusterCount; ++i)
                    {
                        if (currentObjIdx != newObjIdx[i])
                        {
                            currentObjIdx = newObjIdx[i];
                            shapeMatchingConstraintsGroup.push_back(i);
                        }
                    }
                    shapeMatchingConstraintsGroup.push_back(validClusterCount);

                    shapeMatchingConstraints.numConstraints = validClusterCount;
                    shapeMatchingConstraints.objIdx = std::move(newObjIdx);
                    shapeMatchingConstraints.rootIdx = std::move(newRootIdx);
                    shapeMatchingConstraints.cluster = std::move(newCluster);
                    shapeMatchingConstraints.boneIdx = std::move(newBoneIdx);
                    shapeMatchingConstraints.boneMass = std::move(newBoneMass);
                    shapeMatchingConstraints.restRelativePos = std::move(newRestRelativePos);

                    shapeMatchingConstraintsOrder.resize(validClusterCount);
                    std::iota(shapeMatchingConstraintsOrder.begin(), shapeMatchingConstraintsOrder.end(), 0);
                },
                [&] { // collider
                    collidersOrder.resize(colliders.numColliders);
                    std::iota(collidersOrder.begin(), collidersOrder.end(), 0);
                    std::ranges::sort(collidersOrder, [&](std::uint32_t a, std::uint32_t b) {
                        const std::uint32_t biA = colliders.boneIdx[a];
                        const std::uint32_t biB = colliders.boneIdx[b];
                        if ((biA == UINT32_MAX) != (biB == UINT32_MAX))
                            return (biA != UINT32_MAX) > (biB != UINT32_MAX);
                        if (biA == UINT32_MAX || biB == UINT32_MAX)
                            return biA < biB;
                        const std::uint32_t objIdxA = colliders.objIdx[a];
                        const std::uint32_t objIdxB = colliders.objIdx[b];
                        if (objIdxA != objIdxB)
                            return objIdxA < objIdxB;
                        if (colliders.rootIdx[a] != colliders.rootIdx[b])
                            return colliders.rootIdx[a] < colliders.rootIdx[b];
                        if (physicsBones.depth[biA] != physicsBones.depth[biB])
                            return physicsBones.depth[biA] > physicsBones.depth[biB];
                        return biA < biB;
                    });

                    collidersGroup.clear();
                    collidersRoots.clear();
                    std::uint32_t currentObjIdx = UINT32_MAX;
                    std::uint32_t currentRootIdx = UINT32_MAX;
                    std::uint32_t validCount = 0;
                    for (std::uint32_t ci = 0; ci < colliders.numColliders; ++ci)
                    {
                        const auto& coi = collidersOrder[ci];
                        if (colliders.objIdx[coi] == UINT32_MAX ||
                            colliders.rootIdx[coi] == UINT32_MAX)
                            break;
                        if (currentObjIdx != colliders.objIdx[coi])
                        {
                            currentObjIdx = colliders.objIdx[coi];
                            collidersGroup.push_back(ci);
                        }
                        if (currentRootIdx != colliders.rootIdx[coi])
                        {
                            currentRootIdx = colliders.rootIdx[coi];
                            collidersRoots.push_back(ci);
                        }
                        validCount++;
                    }
                    if (!collidersGroup.empty())
                        collidersGroup.push_back(validCount);
                    if (!collidersRoots.empty())
                        collidersRoots.push_back(validCount);

                    {
                        const Internal::Colliders tmpColliders = colliders;
                        tbb::parallel_for(
                            tbb::blocked_range<std::uint32_t>(0, validCount, 128),
                            [&](const tbb::blocked_range<std::uint32_t>& r) {
                                for (std::uint32_t i = r.begin(); i != r.end(); ++i)
                                {
                                    const std::uint32_t srcIdx = collidersOrder[i];

                                    colliders.boneIdx[i] = tmpColliders.boneIdx[srcIdx];
                                    colliders.objIdx[i] = tmpColliders.objIdx[srcIdx];
                                    colliders.rootIdx[i] = tmpColliders.rootIdx[srcIdx];

                                    colliders.noCollideCount[i] = tmpColliders.noCollideCount[srcIdx];
                                    const std::uint32_t dstStrideBase = i * NOCOLLIDE_MAX;
                                    const std::uint32_t srcStrideBase = srcIdx * NOCOLLIDE_MAX;
                                    for (std::uint32_t s = 0; s < NOCOLLIDE_MAX; ++s)
                                    {
                                        const std::uint32_t dstA = dstStrideBase + s;
                                        const std::uint32_t srcA = srcStrideBase + s;
                                        colliders.noCollideBoneIdx[dstA] = tmpColliders.noCollideBoneIdx[srcA];
                                    }

                                    colliders.boundingAABB[i] = tmpColliders.boundingAABB[srcIdx];
                                    colliders.boundingSphere[i] = tmpColliders.boundingSphere[srcIdx];
                                    colliders.boundingSphereCenter[i] = tmpColliders.boundingSphereCenter[srcIdx];

                                    colliders.colliderType[i] = tmpColliders.colliderType[srcIdx];
                                    colliders.convexHullData[i] = tmpColliders.convexHullData[srcIdx];
                                    colliders.sphereData[i] = tmpColliders.sphereData[srcIdx];
                                }
                            },
                            tbb::static_partitioner());

                        colliders.boneIdx.resize(validCount);
                        colliders.objIdx.resize(validCount);
                        colliders.rootIdx.resize(validCount);

                        colliders.noCollideCount.resize(validCount);
                        const std::uint32_t validNoCollideCount = validCount * NOCOLLIDE_MAX;
                        colliders.noCollideBoneIdx.resize(validNoCollideCount);

                        colliders.boundingAABB.resize(validCount);
                        colliders.boundingSphere.resize(validCount);
                        colliders.boundingSphereCenter.resize(validCount);

                        colliders.colliderType.resize(validCount);
                        colliders.convexHullData.resize(validCount);
                        colliders.sphereData.resize(validCount);
                    }

                    colliders.numColliders = validCount;
                    collidersOrder.resize(validCount);
                    std::iota(collidersOrder.begin(), collidersOrder.end(), 0);

                    std::vector<std::uint8_t> isParent(physicsBones.numBones, 0);
                    for (std::uint32_t ci = 0; ci < colliders.numColliders; ++ci)
                    {
                        const std::uint32_t bi = colliders.boneIdx[ci];
                        if (bi == UINT32_MAX)
                            continue;
                        const std::uint32_t pbi = physicsBones.parentBoneIdx[bi];
                        if (pbi == UINT32_MAX)
                            continue;
                        isParent[pbi] = 1;
                    }

                    collidersLeafs.clear();
                    for (std::uint32_t ci = 0; ci < colliders.numColliders; ++ci)
                    {
                        const std::uint32_t bi = colliders.boneIdx[ci];
                        if (bi == UINT32_MAX)
                            continue;
                        if (physicsBones.invMass[bi] <= Epsilon)
                            continue;
                        if (isParent[bi] != 0)
                            continue;
                        collidersLeafs.push_back(ci);
                    }
                },
                [&] { // driver
                    driver->ReorderMaps(nullptr, GetContext());
                }
            );
            // logger::info("Reorder maps done");
            convexHullCache.clear();
            manifoldCache.clear();
            objectHashesSmall.clear();
            objectHashesLarge.clear();
            colorGraphDirty = true;
            orderDirty = false;
        });
    }

    void XPBDWorld::BuildConstraintColorGraph()
    {
        const std::uint32_t numBones = physicsBones.numBones;
        auto assignColorGraph = [numBones](auto& cons, std::vector<std::uint32_t>& order) {
            cons.colorGraph.assign(cons.numConstraints, 0);
            std::vector<std::vector<std::uint32_t>> boneToConsIdx(numBones);
            for (std::uint32_t ci = 0; ci < cons.numConstraints; ++ci)
            {
                const std::uint32_t bi = cons.boneIdx[ci];
                if (bi < numBones)
                    boneToConsIdx[bi].push_back(ci);
                const std::uint32_t ai = cons.anchIdx[ci];
                if (ai < numBones)
                    boneToConsIdx[ai].push_back(ci);
            }
            for (std::uint32_t ci = 0; ci < cons.numConstraints; ++ci)
            {
                std::vector<bool> usedColors;
                auto markUsed = [&](const std::uint32_t bi) {
                    if (bi >= numBones)
                        return;
                    for (std::uint32_t adjConsIdx : boneToConsIdx[bi])
                    {
                        if (adjConsIdx < ci)
                        {
                            std::uint32_t color = cons.colorGraph[adjConsIdx];
                            if (color >= usedColors.size())
                            {
                                usedColors.resize(color + 1, false);
                            }
                            usedColors[color] = true;
                        }
                    }
                };
                markUsed(cons.boneIdx[ci]);
                markUsed(cons.anchIdx[ci]);
                std::uint32_t color = 0;
                while (color < usedColors.size() && usedColors[color])
                {
                    color++;
                }
                cons.colorGraph[ci] = color;
            }

            order.resize(cons.numConstraints);
            std::iota(order.begin(), order.end(), 0);
            std::ranges::sort(order, [&](std::uint32_t a, std::uint32_t b) {
                if (cons.objIdx[a] != cons.objIdx[b])
                    return cons.objIdx[a] < cons.objIdx[b];
                if (cons.colorGraph[a] != cons.colorGraph[b])
                    return cons.colorGraph[a] < cons.colorGraph[b];
                return a < b;
            });
        };

        auto makeGroup = [](auto& cons, std::vector<std::uint32_t>& group, std::vector<std::uint32_t>& colorGroup) {
            group.clear();
            colorGroup.clear();

            std::uint32_t currentObjIdx = UINT32_MAX;
            std::uint32_t currentColor = UINT32_MAX;

            for (std::uint32_t i = 0; i < cons.numConstraints; ++i)
            {
                if (cons.objIdx[i] == UINT32_MAX)
                    break;

                if (currentObjIdx != cons.objIdx[i])
                {
                    currentObjIdx = cons.objIdx[i];
                    group.push_back(i);

                    currentColor = cons.colorGraph[i];
                    colorGroup.push_back(i);
                }
                else if (currentColor != cons.colorGraph[i])
                {
                    currentColor = cons.colorGraph[i];
                    colorGroup.push_back(i);
                }
            }
            if (cons.numConstraints > 0)
            {
                group.push_back(cons.numConstraints);
                colorGroup.push_back(cons.numConstraints);
            }
        };

        threadPool->Execute([&] {
            tbb::parallel_invoke(
                [&] { // 1. Distance Constraints
                    assignColorGraph(distanceConstraints, distanceConstraintsOrder);
                    const Internal::DistanceConstraints tmpConstraints = distanceConstraints;
                    tbb::parallel_for(
                        tbb::blocked_range<std::uint32_t>(0, distanceConstraints.numConstraints, 128),
                        [&](const tbb::blocked_range<std::uint32_t>& r) {
                            for (std::uint32_t i = r.begin(); i != r.end(); ++i)
                            {
                                const std::uint32_t srcIdx = distanceConstraintsOrder[i];

                                distanceConstraints.boneIdx[i] = tmpConstraints.boneIdx[srcIdx];
                                distanceConstraints.anchIdx[i] = tmpConstraints.anchIdx[srcIdx];
                                distanceConstraints.objIdx[i] = tmpConstraints.objIdx[srcIdx];
                                distanceConstraints.rootIdx[i] = tmpConstraints.rootIdx[srcIdx];
                                distanceConstraints.colorGraph[i] = tmpConstraints.colorGraph[srcIdx];

                                distanceConstraints.restLen[i] = tmpConstraints.restLen[srcIdx];
                                distanceConstraints.complianceSquish[i] = tmpConstraints.complianceSquish[srcIdx];
                                distanceConstraints.complianceStretch[i] = tmpConstraints.complianceStretch[srcIdx];
                                distanceConstraints.squishMargin[i] = tmpConstraints.squishMargin[srcIdx];
                                distanceConstraints.stretchMargin[i] = tmpConstraints.stretchMargin[srcIdx];
                                distanceConstraints.squishDamping[i] = tmpConstraints.squishDamping[srcIdx];
                                distanceConstraints.stretchDamping[i] = tmpConstraints.stretchDamping[srcIdx];

                                distanceConstraints.lambda[i] = tmpConstraints.lambda[srcIdx];
                            }
                        },
                        tbb::static_partitioner());
                    std::iota(distanceConstraintsOrder.begin(), distanceConstraintsOrder.end(), 0);
                    makeGroup(distanceConstraints, distanceConstraintsGroup, distanceConstraintsColorGroup);
                },
                [&] { // 2. Angular Constraints
                    assignColorGraph(angularConstraints, angularConstraintsOrder);
                    const Internal::AngularConstraints tmpAngCons = angularConstraints;
                    tbb::parallel_for(
                        tbb::blocked_range<std::uint32_t>(0, angularConstraints.numConstraints, 128),
                        [&](const tbb::blocked_range<std::uint32_t>& r) {
                            for (std::uint32_t i = r.begin(); i != r.end(); ++i)
                            {
                                const std::uint32_t srcIdx = angularConstraintsOrder[i];

                                angularConstraints.boneIdx[i] = tmpAngCons.boneIdx[srcIdx];
                                angularConstraints.anchIdx[i] = tmpAngCons.anchIdx[srcIdx];
                                angularConstraints.objIdx[i] = tmpAngCons.objIdx[srcIdx];
                                angularConstraints.rootIdx[i] = tmpAngCons.rootIdx[srcIdx];
                                angularConstraints.colorGraph[i] = tmpAngCons.colorGraph[srcIdx];

                                angularConstraints.restRot[i] = tmpAngCons.restRot[srcIdx];
                                angularConstraints.compliancePositive[i] = tmpAngCons.compliancePositive[srcIdx];
                                angularConstraints.complianceNegative[i] = tmpAngCons.complianceNegative[srcIdx];
                                angularConstraints.marginPositive[i] = tmpAngCons.marginPositive[srcIdx];
                                angularConstraints.marginNegative[i] = tmpAngCons.marginNegative[srcIdx];
                                angularConstraints.dampingPositive[i] = tmpAngCons.dampingPositive[srcIdx];
                                angularConstraints.dampingNegative[i] = tmpAngCons.dampingNegative[srcIdx];

                                angularConstraints.lambda[i] = tmpAngCons.lambda[srcIdx];
                            }
                        },
                        tbb::static_partitioner());
                    std::iota(angularConstraintsOrder.begin(), angularConstraintsOrder.end(), 0);
                    makeGroup(angularConstraints, angularConstraintsGroup, angularConstraintsColorGroup);
                },
                [&] { // 2. Cone Constraints
                    assignColorGraph(coneConstraints, coneConstraintsOrder);
                    const Internal::ConeConstraints tmpConeCons = coneConstraints;
                    tbb::parallel_for(
                        tbb::blocked_range<std::uint32_t>(0, coneConstraints.numConstraints, 128),
                        [&](const tbb::blocked_range<std::uint32_t>& r) {
                            for (std::uint32_t i = r.begin(); i != r.end(); ++i)
                            {
                                const std::uint32_t srcIdx = coneConstraintsOrder[i];

                                coneConstraints.boneIdx[i] = tmpConeCons.boneIdx[srcIdx];
                                coneConstraints.anchIdx[i] = tmpConeCons.anchIdx[srcIdx];
                                coneConstraints.objIdx[i] = tmpConeCons.objIdx[srcIdx];
                                coneConstraints.rootIdx[i] = tmpConeCons.rootIdx[srcIdx];
                                coneConstraints.colorGraph[i] = tmpConeCons.colorGraph[srcIdx];

                                coneConstraints.restDir[i] = tmpConeCons.restDir[srcIdx];
                                coneConstraints.alignRot[i] = tmpConeCons.alignRot[srcIdx];
                                coneConstraints.invAlignRot[i] = tmpConeCons.invAlignRot[srcIdx];
                                coneConstraints.compliancePositive[i] = tmpConeCons.compliancePositive[srcIdx];
                                coneConstraints.complianceNegative[i] = tmpConeCons.complianceNegative[srcIdx];
                                coneConstraints.marginPositive[i] = tmpConeCons.marginPositive[srcIdx];
                                coneConstraints.marginNegative[i] = tmpConeCons.marginNegative[srcIdx];
                                coneConstraints.dampingPositive[i] = tmpConeCons.dampingPositive[srcIdx];
                                coneConstraints.dampingNegative[i] = tmpConeCons.dampingNegative[srcIdx];

                                coneConstraints.lambda[i] = tmpConeCons.lambda[srcIdx];
                            }
                        },
                        tbb::static_partitioner());
                    std::iota(coneConstraintsOrder.begin(), coneConstraintsOrder.end(), 0);
                    makeGroup(coneConstraints, coneConstraintsGroup, coneConstraintsColorGroup);
                }
            );
            colorGraphDirty = false;
        });
    }

    void XPBDWorld::CheckUpdate()
    {
        if (orderDirty)
            ReorderMaps();
        if (colorGraphDirty)
            BuildConstraintColorGraph();
    }

    void XPBDWorld::SetBone(const std::uint32_t bi, const PhysicsInput::Bone& bone)
    {
        float physicsScale = 1.0f;
        if (std::uint32_t pbi = physicsBones.parentBoneIdx[bi]; physicsBones.isParticle[bi] && pbi != UINT32_MAX)
            physicsScale = physicsBones.physicsScale[pbi];
        else
            physicsScale = bone.GetPhysicsScaleByVolume();
        const Vector vPhysicsScale = DirectX::XMVectorReplicate(physicsScale);

        physicsBones.dampingPositive[bi] = ToVector(bone.dampingPositive);
        physicsBones.dampingNegative[bi] = ToVector(bone.dampingNegative);
        physicsBones.angularDampingPositive[bi] = ToVector(bone.angularDampingPositive);
        physicsBones.angularDampingNegative[bi] = ToVector(bone.angularDampingNegative);

        const float mass = bone.mass * physicsScale;
        if (mass <= Epsilon)
        {
            physicsBones.invMass[bi] = 0.0f;
            physicsBones.inertiaPositive[bi] = vZero;
            physicsBones.inertiaNegative[bi] = vZero;
            physicsBones.invAngularInertiaPositive[bi] = vZero;
            physicsBones.invAngularInertiaNegative[bi] = vZero;
        }
        else
        {
            physicsBones.invMass[bi] = reciprocal(mass);
            const Vector vMass = DirectX::XMVectorReplicate(mass);
            physicsBones.inertiaPositive[bi] = ToVector(bone.inertiaPositive);
            physicsBones.inertiaNegative[bi] = ToVector(bone.inertiaNegative);
            physicsBones.invAngularInertiaPositive[bi] = GetInvInertia(vMass, ToVector(bone.angularInertiaPositive));
            physicsBones.invAngularInertiaNegative[bi] = GetInvInertia(vMass, ToVector(bone.angularInertiaNegative));
        }
        physicsBones.angularBlendFactor[bi] = bone.angularBlendFactor;
        physicsBones.gravity[bi] = DirectX::XMVectorMultiply(GetSkyrimGravity(bone.gravity), vPhysicsScale);
        physicsBones.windFactor[bi] = ToVector(bone.windFactor);
        physicsBones.physicsScale[bi] = physicsScale;
        physicsBones.centerOfMass[bi] = ToVector(bone.centerOfMass);

        physicsBones.deformMax[bi] = ToVector(bone.deformMax);
        physicsBones.deformMin[bi] = ToVector(bone.deformMin);
        physicsBones.deformVolumePreservation[bi] = ToVector(bone.deformVolumePreservation);
        physicsBones.deformSquishSensitivity[bi] = ToVector(bone.deformSquishSensitivity);
        physicsBones.deformStretchSensitivity[bi] = ToVector(bone.deformStretchSensitivity);
        physicsBones.deformBulgeSensitivity[bi] = ToVector(bone.deformBulgeSensitivity);
        physicsBones.deformSquishStiffness[bi] = ToVector(bone.deformSquishStiffness);
        physicsBones.deformStretchStiffness[bi] = ToVector(bone.deformStretchStiffness);
        physicsBones.deformSquishDamping[bi] = ToVector(bone.deformSquishDamping);
        physicsBones.deformStretchDamping[bi] = ToVector(bone.deformStretchDamping);

        physicsBones.animDriveCompliance[bi] = ToVector(bone.animDriveCompliance);
        physicsBones.animDriveAngularCompliance[bi] = ToVector(bone.animDriveAngularCompliance);

        physicsBones.linearRotTorque[bi] = NiPoin3x3ToMatrix(bone.linearRotTorque, physicsScale);

        physicsBones.collisionMargin[bi] = (bone.collisionMargin < Epsilon ? 0.0f : bone.collisionMargin);
        physicsBones.collisionShrink[bi] = (bone.collisionMargin < COL_MARGIN_MIN ? COL_MARGIN_MIN - bone.collisionMargin : 0.0f);
        physicsBones.collisionFriction[bi] = bone.collisionFriction;
        physicsBones.collisionCompliance[bi] = bone.collisionCompliance;
        physicsBones.collisionRestitution[bi] = bone.collisionRestitution;
        physicsBones.layerGroup[bi] = bone.collisionLayerGroup;
        physicsBones.collideLayer[bi] = bone.collisionCollideLayer;
    }

    void XPBDWorld::SetDistanceConstraint(const std::uint32_t ci, const PhysicsInput::DistanceConstraint& data, const float physicsScale)
    {
        distanceConstraints.complianceSquish[ci] = data.complianceSquish;
        distanceConstraints.complianceStretch[ci] = data.complianceStretch;
        distanceConstraints.squishMargin[ci] = data.squishMargin;
        distanceConstraints.stretchMargin[ci] = data.stretchMargin;
        distanceConstraints.squishLimit[ci] = data.squishLimit;
        distanceConstraints.stretchLimit[ci] = data.stretchLimit;
        ClampZeroToInfinity(distanceConstraints.squishLimit[ci]);
        ClampZeroToInfinity(distanceConstraints.stretchLimit[ci]);
        distanceConstraints.squishDamping[ci] = data.squishDamping;
        distanceConstraints.stretchDamping[ci] = data.stretchDamping;
    }

    void XPBDWorld::SetAngularConstraint(const std::uint32_t ci, const PhysicsInput::AngularConstraint& data, const float physicsScale)
    {
        angularConstraints.compliancePositive[ci] = ToVector(data.compliancePositive);
        angularConstraints.complianceNegative[ci] = ToVector(data.complianceNegative);
        angularConstraints.marginPositive[ci] = ToVector(data.marginPositive);
        angularConstraints.marginNegative[ci] = ToVector(data.marginNegative);
        angularConstraints.limitPositive[ci] = ToVector(data.limitPositive);
        angularConstraints.limitNegative[ci] = ToVector(data.limitNegative);
        ClampZeroToInfinityRot(angularConstraints.limitPositive[ci], false);
        ClampZeroToInfinityRot(angularConstraints.limitNegative[ci], true);
        angularConstraints.dampingPositive[ci] = ToVector(data.dampingPositive);
        angularConstraints.dampingNegative[ci] = ToVector(data.dampingNegative);
    }

    void XPBDWorld::SetConeConstraint(const std::uint32_t ci, const PhysicsInput::ConeConstraint& data, const float physicsScale)
    {
        coneConstraints.compliancePositive[ci] = ToVector(data.compliancePositive);
        coneConstraints.complianceNegative[ci] = ToVector(data.complianceNegative);
        coneConstraints.marginPositive[ci] = ToVector(data.marginPositive);
        coneConstraints.marginNegative[ci] = ToVector(data.marginNegative);
        coneConstraints.limitPositive[ci] = ToVector(data.limitPositive);
        coneConstraints.limitNegative[ci] = ToVector(data.limitNegative);
        ClampZeroToOneRadian(coneConstraints.limitPositive[ci], false);
        ClampZeroToOneRadian(coneConstraints.limitNegative[ci], true);
        coneConstraints.dampingPositive[ci] = ToVector(data.dampingPositive);
        coneConstraints.dampingNegative[ci] = ToVector(data.dampingNegative);
    }

    void XPBDWorld::SetDeformConstraint(const std::uint32_t ci, const PhysicsInput::DeformConstraint& data, const float physicsScale)
    {
        deformConstraints.squishWeight[ci] = ToVector(data.squishWeight);
        deformConstraints.stretchWeight[ci] = ToVector(data.stretchWeight);
        deformConstraints.bulgeWeight[ci] = ToVector(data.bulgeWeight);
    }

    void XPBDWorld::SetShapeMatchingConstraint(const std::uint32_t ci, const PhysicsInput::ShapeMatchingConstraint& data, const float physicsScale)
    {
        shapeMatchingConstraints.cluster[ci].compliancePositive = ToVector(data.compliancePositive);
        shapeMatchingConstraints.cluster[ci].complianceNegative = ToVector(data.complianceNegative);
        shapeMatchingConstraints.cluster[ci].inertiaScale = ToVector(data.inertiaScale);
    }

    void XPBDWorld::LoggingTimeProfiler(const std::string& funcName, const double ms) const
    {
        logger::info("{} time: {:.3f}ms ({} bones / {} distanceConstrants / {} angularConstrants / {} coneConstraints / {} deformConstraints / {} colliders)", funcName, ms,
                      physicsBones.numBones, distanceConstraints.numConstraints, angularConstraints.numConstraints, coneConstraints.numConstraints, deformConstraints.numConstraints, colliders.numColliders);
    }

    void XPBDWorld::UpdateChildTreeData(RE::NiNode* node, RE::NiUpdateData ::Flag flag) const
    {
        if (!node)
            return;

        auto& children = node->GetChildren();
        for (auto& child : children)
        {
            if (!child)
                continue;
            RE::NiNode* childNode = child->AsNode();
            if (!childNode)
                continue;
            RE::NiUpdateData ctx = {0.0f, flag};
            childNode->UpdateWorldData(&ctx);
            UpdateChildTreeData(childNode, flag);
        }
    }
    void XPBDWorld::UpdateChildTreeWorldTransforms(RE::NiNode* node) const
    {
        if (!node)
            return;

        const RE::NiPoint3 worldPos = node->world.translate;
        const RE::NiMatrix3 worldRot = node->world.rotate;
        const float worldScale = node->world.scale;
        auto& children = node->GetChildren();
        for (auto& child : children)
        {
            if (!child)
                continue;
            RE::NiNode* childNode = child->AsNode();
            if (!childNode)
                continue;
            const RE::NiPoint3 childWorldPos = worldPos + (worldRot * (childNode->local.translate * worldScale));
            const RE::NiMatrix3 childWorldRot = worldRot * childNode->local.rotate;
            std::memcpy(&childNode->world.translate, &childWorldPos, sizeof(childWorldPos));
            std::memcpy(&childNode->world.rotate, &childWorldRot, sizeof(childWorldRot));
        }
    }
} // namespace MXPBD
