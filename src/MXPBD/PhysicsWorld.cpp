#include "MXPBD/PhysicsWorld.h"

namespace MXPBD
{
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

    void XPBDWorld::AddPhysics(RE::TESObjectREFR* object, RE::NiNode* rootNode, const RootType rootType, const PhysicsInput& input)
    {
        if (!object || !rootNode)
            return;
        if (input.bones.empty() && input.constraints.empty() && input.colliders.datas.empty())
            return;
        if (rootType == XPBDWorld::RootType::kNone)
            return;

        WaitForPhysicsWorldAsync();

        const ObjectDatas::Root newRoot = {.type = rootType, .bipedSlot = input.bipedSlot};

        // logger::info("{:x} : adding physics for {} bones, {} constraints, {} colliders", object->formID, input.bones.size(), input.constraints.size(), input.convexHullColliders.colliders.size());

        std::lock_guard lg(lock);
        if (orderDirty)
        {
            ReorderMaps();
            orderDirty = false;
        }

        std::uint32_t objIdx = AllocateObject(object);
        std::uint32_t rootIdx = AllocateRoot(objIdx, newRoot);

        // caching bones
        std::unordered_map<std::string, std::uint32_t> boneNameToIdx;
        if (!physicsBonesGroup.empty())
        {
            const std::uint32_t groups = physicsBonesGroup.size() - 1;
            for (std::uint32_t g = 0; g < groups; ++g)
            {
                const std::uint32_t begin = physicsBonesGroup[g];
                const std::uint32_t end = physicsBonesGroup[g + 1];
                if (physicsBones.objIdx[begin] != objIdx)
                    continue;
                for (std::uint32_t bi = begin; bi < end; ++bi)
                {
                    if (auto& node = physicsBones.node[bi]; node && !node->name.empty())
                    {
                        boneNameToIdx[node->name.c_str()] = bi;
                    }
                    else if (physicsBones.isParticle[bi])
                    {
                        boneNameToIdx[physicsBones.particleName[bi]] = bi;
                    }
                }
            }
        }

        if (rootType == XPBDWorld::RootType::kSkeleton || rootType == XPBDWorld::RootType::kFacegen || rootType == XPBDWorld::RootType::kCloth || rootType == XPBDWorld::RootType::kWeapon)
        {
            const std::unordered_map<std::string, std::uint32_t> existsBones = boneNameToIdx;
            std::unordered_map<std::string, std::vector<std::string>> particleParentToName;
            for (const auto& bone : input.bones)
            {
                if (!bone.second.isParticle)
                    continue;
                if (existsBones.find(bone.first) != existsBones.end())
                    continue;
                particleParentToName[bone.second.parentBoneName].push_back(bone.first);
            }

            ReserveBone(input.bones.size());
            ReserveConstraint(input.constraints.size());
            ReserveAngularConstraint(input.angularConstraints.size());
            ReserveDeformConstraint(input.deformConstraints.size());

            // add bone data
            Mus::nif::VisitObjects(rootNode, [&](RE::NiAVObject* node, std::uint32_t depth) {
                if (!node || node->name.empty())
                    return true;

                std::uint32_t bi = 0;
                if (auto boneIdxIt = boneNameToIdx.find(node->name.c_str()); boneIdxIt != boneNameToIdx.end())
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
                        auto pit = boneNameToIdx.find(parent->name.c_str());
                        while (pit == boneNameToIdx.end())
                        {
                            parent = parent->parent;
                            if (!parent->name.empty())
                                pit = boneNameToIdx.find(parent->name.c_str());
                        }
                        if (pit != boneNameToIdx.end())
                            physicsBones.parentBoneIdx[bi] = pit->second;
                    }
                    physicsBones.objIdx[bi] = objIdx;
                    physicsBones.rootIdx[bi] = rootIdx;
                    physicsBones.depth[bi] = depth;

                    physicsBones.prevNodeWorldPos[bi] = physicsBones.pos[bi];
                    physicsBones.targetNodeWorldPos[bi] = physicsBones.pos[bi];
                    physicsBones.prevNodeWorldRot[bi] = physicsBones.rot[bi];
                    physicsBones.targetNodeWorldRot[bi] = physicsBones.rot[bi];

                    physicsBones.orgWorldScale[bi] = node->world.scale;
                    physicsBones.orgLocalPos[bi] = ToVector(node->local.translate);
                    physicsBones.orgLocalRot[bi] = ToQuaternion(node->local.rotate);

                    SetBone(bi, found->second);

                    boneNameToIdx[found->first] = bi;
                }

                if (auto pptn = particleParentToName.find(node->name.c_str()); pptn != particleParentToName.end())
                {
                    auto func = [this, &object, &input, objIdx, rootIdx, depth, &boneNameToIdx, &particleParentToName](auto&& func, const std::string& parentName, const std::vector<std::string>& pptnList, const std::uint32_t parentIdx, std::uint32_t particleDepth) -> void {
                        for (const auto& particleName : pptnList)
                        {
                            const auto pit = input.bones.find(particleName);
                            if (pit == input.bones.end())
                                continue;
                            if (boneNameToIdx.find(particleName) != boneNameToIdx.end())
                                continue;

                            const float physicsScale = physicsBones.physicsScale[parentIdx];
                            const Vector vPhysicsScale = DirectX::XMVectorReplicate(physicsScale);
                            logger::info("{:x} : add physics particle({}) bone {}{} for {}", object->formID, particleDepth, particleName,
                                          pit->second.enableDynamicVolume ? "(PhysicsScale : " + std::to_string(physicsScale) + ")" : ""
                                          , parentName);
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

                            boneNameToIdx[pit->first] = particleBi;

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

            // add constraint
            for (const auto& constraint : input.constraints)
            {
                if (existsBones.find(constraint.first) != existsBones.end())
                    continue; // already added
                auto bit = boneNameToIdx.find(constraint.first);
                if (bit == boneNameToIdx.end())
                {
                    logger::error("{:x} : Unable to get physics node {} for constrant", object->formID, constraint.first);
                    continue;
                }

                const std::uint32_t ci = AllocateConstraint();
                constraints.boneIdx[ci] = bit->second;
                constraints.objIdx[ci] = objIdx;
                constraints.rootIdx[ci] = rootIdx;

                const float physicsScale = physicsBones.physicsScale[bit->second];
                std::uint32_t validAnchorCount = 0;
                const std::uint32_t aiBase = static_cast<std::uint32_t>(ci) * ANCHOR_MAX;
                for (std::uint32_t a = 0; a < constraint.second.anchors.size(); ++a)
                {
                    const std::string& anchorName = constraint.second.anchors[a].anchorBoneName;
                    auto abit = boneNameToIdx.find(anchorName);
                    if (abit == boneNameToIdx.end())
                    {
                        logger::error("{:x} : Unable to get anchor node {} for {}", object->formID, anchorName, constraint.first);
                        continue;
                    }
                    if (validAnchorCount >= ANCHOR_MAX)
                    {
                        logger::error("{:x} : Unable to add anchor node {} for {} due to reached maximum anchor count", object->formID, anchorName, constraint.first);
                        continue;
                    }
                    logger::info("{:x} : add constraint {}({}) on {}", object->formID, constraint.first, validAnchorCount, anchorName);

                    const std::uint32_t ai = aiBase + validAnchorCount;
                    constraints.anchData[ai].anchIdx = abit->second;
                    const Vector tPos = physicsBones.pos[bit->second];
                    const Vector aPos = physicsBones.pos[abit->second];
                    constraints.anchData[ai].restLen = DirectX::XMVectorGetX(DirectX::XMVector3Length(DirectX::XMVectorSubtract(tPos, aPos)));
                    SetConstraint(ai, constraint.second.anchors[a], physicsScale);
                    validAnchorCount++;
                }
                constraints.numAnchors[ci] = validAnchorCount;
            }

            // add angular constraint
            for (const auto& angularConstraint : input.angularConstraints)
            {
                if (existsBones.find(angularConstraint.first) != existsBones.end())
                    continue; // already added
                auto bit = boneNameToIdx.find(angularConstraint.first);
                if (bit == boneNameToIdx.end())
                    continue;

                const std::uint32_t aci = AllocateAngularConstraint();
                angularConstraints.boneIdx[aci] = bit->second;
                angularConstraints.objIdx[aci] = objIdx;
                angularConstraints.rootIdx[aci] = rootIdx;

                physicsBones.advancedRotation[bit->second] = 1;

                const float physicsScale = physicsBones.physicsScale[bit->second];

                std::uint32_t validAnchorCount = 0;
                const std::uint32_t aiBase = static_cast<std::uint32_t>(aci) * ANCHOR_MAX;
                for (std::uint32_t a = 0; a < angularConstraint.second.anchors.size(); ++a)
                {
                    const std::string& anchorName = angularConstraint.second.anchors[a].anchorBoneName;
                    auto abit = boneNameToIdx.find(anchorName);
                    if (abit == boneNameToIdx.end())
                    {
                        logger::error("{:x} : Unable to get anchor node {} for {}", object->formID, anchorName, angularConstraint.first);
                        continue;
                    }
                    if (validAnchorCount >= ANCHOR_MAX)
                    {
                        logger::error("{:x} : Unable to add anchor node {} for {} due to reached maximum anchor count", object->formID, anchorName, angularConstraint.first);
                        continue;
                    }
                    logger::info("{:x} : add angular constraint {}({}) on {}", object->formID, angularConstraint.first, validAnchorCount, anchorName);
                    const std::uint32_t ai = aiBase + validAnchorCount;
                    angularConstraints.anchData[ai].anchIdx = abit->second;
                    physicsBones.advancedRotation[abit->second] = 1;
                    const Quaternion childRot = physicsBones.rot[bit->second];
                    const Quaternion anchorRot = physicsBones.rot[abit->second];
                    const Quaternion anchorRotInv = DirectX::XMQuaternionInverse(anchorRot);
                    angularConstraints.anchData[ai].restRot = DirectX::XMQuaternionNormalize(DirectX::XMQuaternionMultiply(childRot, anchorRotInv));
                    SetAngularConstraint(ai, angularConstraint.second.anchors[a], physicsScale);
                    validAnchorCount++;
                }
                angularConstraints.numAnchors[aci] = validAnchorCount;
            }

            // add deform constraint
            for (const auto& deformConstraint : input.deformConstraints)
            {
                if (existsBones.find(deformConstraint.first) != existsBones.end())
                    continue; // already added
                auto bit = boneNameToIdx.find(deformConstraint.first);
                if (bit == boneNameToIdx.end())
                    continue;

                const std::uint32_t dci = AllocateDeformConstraint();
                deformConstraints.boneIdx[dci] = bit->second;
                deformConstraints.objIdx[dci] = objIdx;
                deformConstraints.rootIdx[dci] = rootIdx;

                const float physicsScale = physicsBones.physicsScale[bit->second];

                std::uint32_t validAnchorCount = 0;
                const std::uint32_t aiBase = static_cast<std::uint32_t>(dci) * ANCHOR_MAX;
                for (std::uint32_t a = 0; a < deformConstraint.second.anchors.size(); ++a)
                {
                    const std::string& anchorName = deformConstraint.second.anchors[a].anchorBoneName;
                    auto abit = boneNameToIdx.find(anchorName);
                    if (abit == boneNameToIdx.end())
                    {
                        logger::error("{:x} : Unable to get anchor node {} for {}", object->formID, anchorName, deformConstraint.first);
                        continue;
                    }
                    if (validAnchorCount >= ANCHOR_MAX)
                    {
                        logger::error("{:x} : Unable to add anchor node {} for {} due to reached maximum anchor count", object->formID, anchorName, deformConstraint.first);
                        continue;
                    }
                    logger::info("{:x} : add deform constraint {}({}) on {}", object->formID, deformConstraint.first, validAnchorCount, anchorName);
                    const std::uint32_t ai = aiBase + validAnchorCount;
                    deformConstraints.anchData[ai].anchIdx = abit->second;
                    const Vector tPos = physicsBones.pos[bit->second];
                    const Vector aPos = physicsBones.pos[abit->second];
                    deformConstraints.anchData[ai].restLen = DirectX::XMVectorGetX(DirectX::XMVector3Length(DirectX::XMVectorSubtract(tPos, aPos)));
                    const Quaternion childRot = physicsBones.rot[bit->second];
                    const Quaternion anchorRot = physicsBones.rot[abit->second];
                    const Quaternion anchorRotInv = DirectX::XMQuaternionInverse(anchorRot);
                    deformConstraints.anchData[ai].restRot = DirectX::XMQuaternionNormalize(DirectX::XMQuaternionMultiply(childRot, anchorRotInv));
                    SetDeformConstraint(ai, deformConstraint.second.anchors[a], physicsScale);
                    validAnchorCount++;
                }
                deformConstraints.numAnchors[dci] = validAnchorCount;
            }
        }
        else if (rootType == XPBDWorld::RootType::kCollider)
        {
            if (!input.colliders.datas.empty())
            {
                ReserveCollider(input.colliders.datas.size());
                RemoveCollider(object, newRoot);

                if (colliders.numColliders == 0)
                    convexHullCache.reserve(input.colliders.datas.size() * 8);

                std::uint32_t addedColCount = 0;
                for (auto& collider : input.colliders.datas)
                {
                    const auto& boneName = collider.boneName;
                    auto bit = boneNameToIdx.find(boneName);
                    if (bit == boneNameToIdx.end())
                        continue;

                    const std::uint32_t coi = AllocateCollider();
                    colliders.boneIdx[coi] = bit->second;
                    colliders.objIdx[coi] = objIdx;
                    colliders.rootIdx[coi] = rootIdx;
                    colliders.colliderType[coi] = collider.colliderType;

                    if (collider.colliderType == ColliderType::kConvexHull)
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
                    else if (collider.colliderType == ColliderType::kSphere)
                    {
                        logger::info("{:x} => add sphere collider {} / colGroup {:x} / colLayer {:x}", object->formID, boneName, physicsBones.layerGroup[bit->second], physicsBones.collideLayer[bit->second]);
                        colliders.sphereData[coi] = collider.sphereData;

                        addedColCount++;

                        AABB aabb = AABB();
                        for (std::uint32_t s = 0; s < collider.sphereData.sphereCount; ++s)
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
                        for (std::uint32_t s = 0; s < collider.sphereData.sphereCount; ++s)
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

                    std::uint32_t ncCount = 0;
                    {
                        auto& node = physicsBones.node[bit->second];
                        if (node && node->parent && !node->parent->name.empty())
                        {
                            const std::string ncName = node->parent->name.c_str();
                            auto ncBit = boneNameToIdx.find(ncName);
                            if (ncBit != boneNameToIdx.end())
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
                            auto ncBit = boneNameToIdx.find(ncName);
                            if (ncBit == boneNameToIdx.end())
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
            }
        }
        ReorderMaps();
        orderDirty = false;
    }

    void XPBDWorld::UpdatePhysicsSetting(RE::TESObjectREFR* object, const PhysicsInput& input, bool reset)
    {
        if (!object)
            return;

        WaitForPhysicsWorldAsync();

        if (reset)
            Reset(object);

        // logger::info("{:x} : Replacing physics setting...", object->formID);
        std::lock_guard lg(lock);
        if (orderDirty)
        {
            ReorderMaps();
            orderDirty = false;
        }

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

        if (physicsBonesGroup.empty())
            return;

        std::unordered_map<std::string, std::uint32_t> boneNameToIdx;
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
                        boneNameToIdx[node->name.c_str()] = bi;

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
                        boneNameToIdx[physicsBones.particleName[bi]] = bi;

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
        if (!constraintsGroup.empty())
        {
            const std::uint32_t groups = constraintsGroup.size() - 1ull;
            for (std::uint32_t g = 0; g < groups; ++g)
            {
                const std::uint32_t begin = constraintsGroup[g];
                const std::uint32_t end = constraintsGroup[g + 1];
                if (begin >= end)
                    continue;
                if (constraints.objIdx[begin] != currentObjIdx)
                    continue;
                logger::debug("{:x} : Found constraints group", object->formID);
                for (std::uint32_t ci = begin; ci < end; ++ci)
                {
                    const std::uint32_t bi = constraints.boneIdx[ci];
                    if (bi == UINT32_MAX)
                        continue;
                    std::string nodeName;
                    if (auto& node = physicsBones.node[bi]; node && !node->name.empty())
                        nodeName = node->name.c_str();
                    else if (!physicsBones.particleName[bi].empty())
                        nodeName = physicsBones.particleName[bi];
                    else
                        continue;
                    auto found = input.constraints.find(nodeName);
                    if (found == input.constraints.end())
                        continue;
                    logger::debug("{:x} : Found bone {} for constraints", object->formID, nodeName);
                    const float physicsScale = physicsBones.physicsScale[bi];
                    const std::uint32_t aiBase = ci * ANCHOR_MAX;
                    for (std::uint32_t a = 0; a < ANCHOR_MAX; ++a)
                    {
                        const std::uint32_t ai = aiBase + a;
                        const std::uint32_t abi = constraints.anchData[ai].anchIdx;
                        if (abi == UINT32_MAX)
                            continue;
                        std::string anchorName;
                        if (auto& node = physicsBones.node[abi]; node && !node->name.empty())
                            anchorName = node->name.c_str();
                        else if (!physicsBones.particleName[abi].empty())
                            anchorName = physicsBones.particleName[abi];
                        else
                            continue;
                        auto ait = std::find_if(found->second.anchors.begin(), found->second.anchors.end(), [&anchorName](const PhysicsInput::Constraint::AnchorData& data) {
                            return anchorName == data.anchorBoneName;
                        });
                        if (ait == found->second.anchors.end())
                            continue;
                        SetConstraint(ai, *ait, physicsScale);
                    }
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
                for (std::uint32_t aci = begin; aci < end; ++aci)
                {
                    const std::uint32_t bi = angularConstraints.boneIdx[aci];
                    if (bi == UINT32_MAX)
                        continue;
                    std::string nodeName;
                    if (auto& node = physicsBones.node[bi]; node && !node->name.empty())
                        nodeName = node->name.c_str();
                    else if (!physicsBones.particleName[bi].empty())
                        nodeName = physicsBones.particleName[bi];
                    else
                        continue;
                    auto found = input.angularConstraints.find(nodeName);
                    if (found == input.angularConstraints.end())
                        continue;
                    const auto boneIt = input.bones.find(nodeName);
                    if (boneIt == input.bones.end())
                        continue;
                    logger::debug("{:x} : Found bone {} for angular constraints", object->formID, nodeName);
                    const float physicsScale = physicsBones.physicsScale[bi];
                    const std::uint32_t aiBase = aci * ANCHOR_MAX;
                    for (std::uint32_t a = 0; a < ANCHOR_MAX; ++a)
                    {
                        const std::uint32_t ai = aiBase + a;
                        const std::uint32_t abi = angularConstraints.anchData[ai].anchIdx;
                        if (abi == UINT32_MAX)
                            continue;
                        std::string anchorName;
                        if (auto& node = physicsBones.node[abi]; node && !node->name.empty())
                            anchorName = node->name.c_str();
                        else if (!physicsBones.particleName[abi].empty())
                            anchorName = physicsBones.particleName[abi];
                        else
                            continue;
                        auto ait = std::find_if(found->second.anchors.begin(), found->second.anchors.end(), [&anchorName](const PhysicsInput::AngularConstraint::AnchorData& data) {
                            return anchorName == data.anchorBoneName;
                        });
                        if (ait == found->second.anchors.end())
                            continue;
                        SetAngularConstraint(ai, *ait, physicsScale);
                    }
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
                for (std::uint32_t aci = begin; aci < end; ++aci)
                {
                    const std::uint32_t bi = deformConstraints.boneIdx[aci];
                    if (bi == UINT32_MAX)
                        continue;
                    std::string nodeName;
                    if (auto& node = physicsBones.node[bi]; node && !node->name.empty())
                        nodeName = node->name.c_str();
                    else if (!physicsBones.particleName[bi].empty())
                        nodeName = physicsBones.particleName[bi];
                    else
                        continue;
                    auto found = input.deformConstraints.find(nodeName);
                    if (found == input.deformConstraints.end())
                        continue;
                    const auto boneIt = input.bones.find(nodeName);
                    if (boneIt == input.bones.end())
                        continue;
                    logger::debug("{:x} : Found bone {} for deform constraints", object->formID, nodeName);
                    const float physicsScale = physicsBones.physicsScale[bi];
                    const std::uint32_t aiBase = aci * ANCHOR_MAX;
                    for (std::uint32_t a = 0; a < ANCHOR_MAX; ++a)
                    {
                        const std::uint32_t ai = aiBase + a;
                        const std::uint32_t abi = deformConstraints.anchData[ai].anchIdx;
                        if (abi == UINT32_MAX)
                            continue;
                        std::string anchorName;
                        if (auto& node = physicsBones.node[abi]; node && !node->name.empty())
                            anchorName = node->name.c_str();
                        else if (!physicsBones.particleName[abi].empty())
                            anchorName = physicsBones.particleName[abi];
                        else
                            continue;
                        auto ait = std::find_if(found->second.anchors.begin(), found->second.anchors.end(), [&anchorName](const PhysicsInput::DeformConstraint::AnchorData& data) {
                            return anchorName == data.anchorBoneName;
                        });
                        if (ait == found->second.anchors.end())
                            continue;
                        SetDeformConstraint(ai, *ait, physicsScale);
                    }
                }
            }
        }
    }

    void XPBDWorld::ResetAll()
    {
        WaitForPhysicsWorldAsync();

        std::lock_guard lg(lock);

        for (std::uint32_t oi = 0; oi < objectDatas.objectID.size(); ++oi)
        {
            if (RE::TESObjectREFR* object = GetREFR(objectDatas.objectID[oi]); object)
            {
                auto npcNode = GetNPCNode(object);
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
                            for (std::uint32_t bi = begin; bi < end; ++bi)
                            {
                                ResetBone(bi);
                            }
                            UpdateChildTreeData(GetNPCNodeByObjectIndex(physicsBones.objIdx[begin]));
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

        // find object
        std::uint32_t objIdx = UINT32_MAX;
        for (std::uint32_t oi = 0; oi < objectDatas.objectID.size(); ++oi)
        {
            if (objectDatas.objectID[oi] == object->formID)
            {
                objIdx = oi;
                auto npcNode = GetNPCNode(object);
                if (npcNode)
                    objectDatas.prevWorldPos[oi] = ToVector(npcNode->world.translate);
                else
                    objectDatas.prevWorldPos[oi] = ToVector(object->GetPosition());
                objectDatas.acceleration[oi] = vZero;
                break;
            }
        }
        if (objIdx == UINT32_MAX)
            return;

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
                if (physicsBones.objIdx[begin] != objIdx)
                    continue;
                for (std::uint32_t bi = begin; bi < end; ++bi)
                {
                    ResetBone(bi);
                }
                UpdateChildTreeData(GetNPCNodeByObjectIndex(physicsBones.objIdx[begin]));
                for (std::uint32_t bi = begin; bi < end; ++bi)
                {
                    ResetParticleBone(bi);
                }
                break;
            }
        }
    }

    void XPBDWorld::Reset(const RE::FormID objectID)
    {
        RE::TESObjectREFR* object = GetREFR(objectID);
        if (!object)
            return;
        Reset(object);
    }

    void XPBDWorld::ResetBone(const std::uint32_t bi)
    {
        if (auto& node = physicsBones.node[bi]; node)
        {
            const RE::NiPoint3 oldLocalPos = ToPoint3(physicsBones.orgLocalPos[bi]);
            const RE::NiMatrix3 oldLocalRot = ToNiMatrix(physicsBones.orgLocalRot[bi]);
            memcpy(&node->local.translate, &oldLocalPos, sizeof(oldLocalPos));
            memcpy(&node->local.rotate, &oldLocalRot, sizeof(oldLocalRot));

            const Vector offset = DirectX::XMVector3Rotate(DirectX::XMVectorScale(physicsBones.offset[bi], physicsBones.orgWorldScale[bi]), ToQuaternion(node->world.rotate));
            physicsBones.pos[bi] = DirectX::XMVectorAdd(ToVector(node->world.translate), offset);
            physicsBones.rot[bi] = ToQuaternion(node->world.rotate);
        }
        else if (physicsBones.isParticle[bi] && physicsBones.parentBoneIdx[bi] != UINT32_MAX)
        {
            const std::uint32_t pbi = physicsBones.parentBoneIdx[bi];
            const Vector offset = DirectX::XMVector3Rotate(DirectX::XMVectorScale(physicsBones.offset[bi], physicsBones.orgWorldScale[pbi]), physicsBones.rot[pbi]);
            physicsBones.pos[bi] = DirectX::XMVectorAdd(physicsBones.pos[pbi], offset);
            physicsBones.rot[bi] = physicsBones.rot[pbi];
        }

        physicsBones.prevPos[bi] = physicsBones.pos[bi];
        physicsBones.predPos[bi] = physicsBones.pos[bi];
        physicsBones.posVel[bi] = vZero;

        physicsBones.prevRot[bi] = physicsBones.rot[bi];
        physicsBones.predRot[bi] = physicsBones.rot[bi];
        physicsBones.backupRot[bi] = physicsBones.rot[bi];
        physicsBones.angVel[bi] = vZero;

        physicsBones.deformScale[bi] = vmZeroAll;
        physicsBones.deformScaleCache[bi] = vmZeroAll;
        physicsBones.deformVelocityScale[bi] = vmZeroAll;

        isResetBones = true;
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
            physicsBones.posVel[bi] = vZero;

            physicsBones.rot[bi] = physicsBones.rot[pbi];
            physicsBones.prevRot[bi] = physicsBones.rot[bi];
            physicsBones.predRot[bi] = physicsBones.rot[bi];
            physicsBones.backupRot[bi] = physicsBones.rot[bi];
            physicsBones.angVel[bi] = vZero;
        }
    }

    void XPBDWorld::RemovePhysics(const RE::FormID objectID)
    {
        WaitForPhysicsWorldAsync();
        Reset(objectID);
        std::lock_guard lg(lock);
        RemoveDataList removeList;
        for (std::uint32_t i = 0; i < objectDatas.objectID.size(); ++i)
        {
            if (objectDatas.objectID[i] != objectID)
                continue;
            objectDatas.objectID[i] = 0;
            objectDatas.isDisable[i] = true;
            objectDatas.isDisableByToggle[i] = true;
            for (std::uint32_t ri = 0; ri < objectDatas.roots[i].size(); ++ri)
            {
                removeList.insert(RemoveData(i, ri));
            }
            objectDatas.roots[i].clear();
            RemovePhysics(removeList);
            break;
        }
    }

    void XPBDWorld::RemovePhysics(const RE::FormID objectID, const RootType rootType, const std::uint32_t bipedSlot)
    {
        const ObjectDatas::Root targetRoot = {.type = rootType, .bipedSlot = bipedSlot};
        WaitForPhysicsWorldAsync();
        std::lock_guard lg(lock);
        RemoveDataList removeList;
        for (std::uint32_t i = 0; i < objectDatas.objectID.size(); ++i)
        {
            if (objectDatas.objectID[i] != objectID)
                continue;
            auto& root = objectDatas.roots[i];
            for (std::uint32_t ri = 0; ri < root.size(); ++ri)
            {
                if (root[ri] != targetRoot)
                    continue;
                removeList.insert(RemoveData(i, ri));
                root[ri].type = RootType::kNone;
                break;
            }
            RemovePhysics(removeList);
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

    void XPBDWorld::RemovePhysics(const RemoveDataList& removeList)
    {
        if (removeList.empty())
            return;

        threadPool->Execute([&] {
            // remove nodes
            for (std::uint32_t bi = 0; bi != physicsBones.numBones; ++bi)
            {
                if (removeList.count(RemoveData(physicsBones.objIdx[bi], physicsBones.rootIdx[bi])) > 0)
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
                    if (physicsBones.objIdx[pbi] == UINT32_MAX || physicsBones.rootIdx[pbi] == UINT32_MAX || removeList.count(RemoveData(physicsBones.objIdx[pbi], physicsBones.rootIdx[pbi])) > 0)
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
                // remove constraints
                for (std::uint32_t i = 0; i < constraints.numConstraints; ++i)
                {
                    if (constraints.boneIdx[i] != UINT32_MAX && (!physicsBones.node[constraints.boneIdx[i]] && physicsBones.particleName[constraints.boneIdx[i]].empty()))
                    {
                        constraints.boneIdx[i] = UINT32_MAX;
                        constraints.objIdx[i] = UINT32_MAX;
                        constraints.rootIdx[i] = UINT32_MAX;
                    }
                }
            }, [&] {
                // remove angular constraints
                for (std::uint32_t i = 0; i < angularConstraints.numConstraints; ++i)
                {
                    if (angularConstraints.boneIdx[i] != UINT32_MAX && (!physicsBones.node[angularConstraints.boneIdx[i]] && physicsBones.particleName[angularConstraints.boneIdx[i]].empty()))
                    {
                        angularConstraints.boneIdx[i] = UINT32_MAX;
                        angularConstraints.objIdx[i] = UINT32_MAX;
                        angularConstraints.rootIdx[i] = UINT32_MAX;
                    }
                }
            }, [&] {
                // remove deform constraints
                for (std::uint32_t i = 0; i < deformConstraints.numConstraints; ++i)
                {
                    if (deformConstraints.boneIdx[i] != UINT32_MAX && (!physicsBones.node[deformConstraints.boneIdx[i]] && physicsBones.particleName[deformConstraints.boneIdx[i]].empty()))
                    {
                        deformConstraints.boneIdx[i] = UINT32_MAX;
                        deformConstraints.objIdx[i] = UINT32_MAX;
                        deformConstraints.rootIdx[i] = UINT32_MAX;
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
            });
        });
        orderDirty = true;
    }

    void XPBDWorld::RemoveCollider(RE::TESObjectREFR* object, const ObjectDatas::Root& targetRoot)
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
        const bool isDtZero = deltaTime <= Epsilon;
        std::lock_guard lg(lock);
        if (isResetBones && isDtZero)
            return;
        isResetBones = false;

        if (orderDirty)
        {
            ReorderMaps();
            orderDirty = false;
        }
        if (isNeedColorGraphUpdate)
        {
            BuildConstraintColorGraph();
            isNeedColorGraphUpdate = false;
        }

        static TimeProfiler timeProfiler(__func__);
        timeProfiler.Start();

        threadPool->Execute([&] {
            if (!isDtZero)
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
                    ClampObjectRotation(preCalcStepCount);
                    PrefetchBoneDatas();
                    groundCache.resize(physicsBones.numBones);
                    ObjectCulling();
                }
                std::uint32_t stepCount = 0;
                while (timeAccumulator >= DeltaTime60)
                {
                    const float subStepTime = (stepCount + 1) * DeltaTime60;
                    const float alpha = std::clamp((subStepTime - prevTimeAccumulator) * reciprocal(fixedDeltaTime), 0.0f, 1.0f);
                    const float nextAlpha = std::clamp((subStepTime + DeltaTime60 - prevTimeAccumulator) * reciprocal(fixedDeltaTime), 0.0f, 1.0f);
                    InterpolateBoneDatas(alpha, nextAlpha);
                    UpdateGlobalAABBTree();
                    UpdateWindStrength();

                    const std::uint32_t minExpectedCollisionCount = colliders.numColliders * 4;
                    if (manifoldCacheCount > collideMaxObserved)
                        collideMaxObserved = manifoldCacheCount;
                    else
                        collideMaxObserved = static_cast<std::uint32_t>(collideMaxObserved * 0.98f);
                    convexHullCache.reserve(static_cast<std::uint32_t>(collideMaxObserved * 1.3f));
                    expectedCollisionCount = std::max(static_cast<std::uint32_t>(collideMaxObserved * 1.3f), minExpectedCollisionCount);
                    manifoldCache.resize(expectedCollisionCount);
                    manifoldCacheCount = 0;

                    PredictBones(DeltaTime60);
                    tbb::parallel_invoke(
                        [&] {
                            CreateLocalSpatialHash();
                            GenerateCollisionManifolds();
                        },
                        [&] {
                            GenerateGroundCache(1.0f);
                        });
                    for (std::uint32_t i = 0; i < ITERATION_MAX; ++i)
                    {
                        SolveCachedCollisions(DeltaTime60);
                        SolveCachedGroundCollisions(DeltaTime60);
                        SolveConstraints(DeltaTime60, i == 0);
                        SolveAnimDrive(DeltaTime60, i == 0);
                    }
                    SolveDeformConstraint(DeltaTime60);
                    UpdateBoneVelocity(DeltaTime60);

                    timeAccumulator -= DeltaTime60;
                    stepCount++;
                    currentFrame++;
                }
            }
            ApplyToSkyrim(false);
        });

        timeProfiler.End(this);
    }

    void XPBDWorld::RunPhysicsWorldAsync(const float deltaTime)
    {
        // logger::info("{}", __func__);
        std::lock_guard lg(lock);
        if (isResetBones && deltaTime <= Epsilon)
            return;
        isResetBones = false;

        if (orderDirty)
        {
            ReorderMaps();
            orderDirty = false;
        }
        if (isNeedColorGraphUpdate)
        {
            BuildConstraintColorGraph();
            isNeedColorGraphUpdate = false;
        }

        static TimeProfiler timeProfiler(__func__);
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
                    groundCache.resize(physicsBones.numBones);
                    GenerateGroundCache(preCalcStepCount);
                    UpdateWindStrength();
                }
                isTaskLoading = true;
                backGroundTask.run([this, prevTimeAccumulator, fixedDeltaTime] {
                    std::uint32_t stepCount = 0;
                    while (timeAccumulator >= DeltaTime60)
                    {
                        const float subStepTime = (stepCount + 1) * DeltaTime60;
                        const float alpha = std::clamp((subStepTime - prevTimeAccumulator) * reciprocal(fixedDeltaTime), 0.0f, 1.0f);
                        const float nextAlpha = std::clamp((subStepTime + DeltaTime60 - prevTimeAccumulator) * reciprocal(fixedDeltaTime), 0.0f, 1.0f);
                        InterpolateBoneDatas(alpha, nextAlpha);
                        UpdateGlobalAABBTree();

                        const std::uint32_t minExpectedCollisionCount = colliders.numColliders * 4;
                        if (manifoldCacheCount > collideMaxObserved)
                            collideMaxObserved = manifoldCacheCount;
                        else
                            collideMaxObserved = static_cast<std::uint32_t>(collideMaxObserved * 0.98f);
                        convexHullCache.reserve(static_cast<std::uint32_t>(collideMaxObserved * 1.3f));
                        expectedCollisionCount = std::max(static_cast<std::uint32_t>(collideMaxObserved * 1.3f), minExpectedCollisionCount);
                        manifoldCache.resize(expectedCollisionCount);
                        manifoldCacheCount = 0;

                        PredictBones(DeltaTime60);
                        CreateLocalSpatialHash();
                        GenerateCollisionManifolds();
                        for (std::uint32_t i = 0; i < ITERATION_MAX; ++i)
                        {
                            SolveCachedCollisions(DeltaTime60);
                            SolveCachedGroundCollisions(DeltaTime60);
                            SolveConstraints(DeltaTime60, i == 0);
                            SolveAnimDrive(DeltaTime60, i == 0);
                        }
                        SolveDeformConstraint(DeltaTime60);
                        UpdateBoneVelocity(DeltaTime60);

                        timeAccumulator -= DeltaTime60;
                        stepCount++;
                        currentFrame++;
                    }
                });
            }
        });

        timeProfiler.End(this);
    }

    void XPBDWorld::WaitForPhysicsWorldAsync()
    {
        if (!isTaskLoading)
            return;
        // logger::info("{}", __func__);

        static TimeProfiler timeProfiler(__func__);
        timeProfiler.Start();

        std::lock_guard lg(lock);
        threadPoolAsync->Execute([&] { backGroundTask.wait(); });
        threadPool->Execute([&] {
            ClampObjectRotation(preCalcStepCount);
            ApplyToSkyrim(true);
        });
        isTaskLoading = false;

        timeProfiler.End(this);
    }

    void XPBDWorld::UpdateObjectData(const float deltaTime)
    {
        if (physicsBonesGroup.empty())
            return;
        // logger::info("{}", __func__);

        static TimeProfiler timeProfiler(__func__);
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

        timeProfiler.End(this);
    }

    void XPBDWorld::ClampObjectRotation(const float stepCount)
    {
        if (physicsBonesGroup.empty() || ROTATION_CLAMP <= Epsilon)
            return;
        // logger::info("{}", __func__);

        static TimeProfiler timeProfiler(__func__);
        timeProfiler.Start();

        const float clamp = ROTATION_CLAMP * stepCount;
        const std::uint32_t groups = physicsBonesGroup.size() - 1;
        for (std::uint32_t g = 0; g < groups; ++g)
        {
            const std::uint32_t begin = physicsBonesGroup[g];
            const std::uint32_t oi = physicsBones.objIdx[begin];
            if (IsDisable(oi))
                continue;
            if (objectDatas.objectID[oi] != 0x14) // isPlayer
                continue;
            RE::Actor* object = GetActor(objectDatas.objectID[oi]);
            if (!object || !object->loadedData || !object->loadedData->data3D)
                continue;
            RE::NiNode* npcNode = GetNPCNode(object->loadedData->data3D.get());
            if (!npcNode || !npcNode->parent)
                continue;
            if (RE::ActorState* state = object->AsActorState(); state && state->IsWeaponDrawn())
                continue;
            if (RE::PlayerCamera* playerCamera = RE::PlayerCamera::GetSingleton(); playerCamera)
            {
                if (playerCamera->IsInFirstPerson() || playerCamera->IsInFreeCameraMode())
                    continue;
            }

            const Quaternion q_prev = objectDatas.prevNPCWorldRot[oi];
            const Quaternion q_inv = DirectX::XMQuaternionInverse(q_prev);
            const Quaternion q_curr = ToQuaternion(npcNode->world.rotate);
            const Quaternion q_diff = DirectX::XMQuaternionMultiply(q_inv, q_curr);
            const Quaternion q_target = DirectX::XMQuaternionNormalize(DirectX::XMQuaternionMultiply(q_diff, objectDatas.targetNPCWorldRot[oi]));
            objectDatas.targetNPCWorldRot[oi] = q_target;

            Quaternion q_delta = DirectX::XMQuaternionMultiply(q_inv, q_target);
            if (DirectX::XMVectorGetW(q_delta) < 0.0f)
                q_delta = DirectX::XMVectorNegate(q_delta);

            const float cosHalfAngle = std::clamp(DirectX::XMVectorGetW(q_delta), -1.0f, 1.0f);
            const float angle = 2.0f * std::acos(cosHalfAngle);
            const float t = (angle > clamp) ? (clamp * reciprocal(angle)) : 1.0f;
            const Quaternion q_final = DirectX::XMQuaternionNormalize(DirectX::XMQuaternionSlerp(q_prev, q_target, t));
            const RE::NiMatrix3 npc_world = ToNiMatrix(q_final);
            const RE::NiMatrix3 npc_local = npcNode->parent->world.rotate.Transpose() * npc_world;
            std::memcpy(&npcNode->local.rotate, &npc_local, sizeof(npc_local));
            objectDatas.prevNPCWorldRot[oi] = q_final;
            std::memcpy(&npcNode->world.rotate, &npc_world, sizeof(npc_world));

            RE::NiUpdateData ctx = {0.0f, RE::NiUpdateData ::Flag::kDirty};
            npcNode->UpdateWorldData(&ctx);
            UpdateChildTreeData(npcNode);

            objectDatas.deltaWorldRot[oi] = DirectX::XMQuaternionSlerp(qZero, DirectX::XMQuaternionMultiply(q_inv, q_final), reciprocal(stepCount));
        }

        timeProfiler.End(this);
    }

    void XPBDWorld::PrefetchBoneDatas()
    {
        // logger::info("{}", __func__);

        static TimeProfiler timeProfiler(__func__);
        timeProfiler.Start();

        tbb::parallel_for(
            tbb::blocked_range<std::uint32_t>(0, physicsBones.numBones, 32),
            [&](const tbb::blocked_range<std::uint32_t>& r) {
                for (std::uint32_t bi = r.begin(); bi != r.end(); ++bi)
                {
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

        timeProfiler.End(this);
    }

    void XPBDWorld::InterpolateBoneDatas(const float alpha, const float nextAlpha)
    {
        // logger::info("{}", __func__);

        static TimeProfiler timeProfiler(__func__);
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
                            physicsBones.pos[bi] = DirectX::XMVectorAdd(interPos, offset);
                            physicsBones.prevPos[bi] = physicsBones.pos[bi];

                            const Vector nextInterPos = DirectX::XMVectorLerp(physicsBones.prevNodeWorldPos[bi], physicsBones.targetNodeWorldPos[bi], nextAlpha);
                            const Quaternion nextInterRot = DirectX::XMQuaternionSlerp(physicsBones.prevNodeWorldRot[bi], physicsBones.targetNodeWorldRot[bi], nextAlpha);
                            const Vector nextOffset = DirectX::XMVector3Rotate(DirectX::XMVectorScale(physicsBones.offset[bi], physicsBones.orgWorldScale[bi]), nextInterRot);

                            physicsBones.predPos[bi] = DirectX::XMVectorAdd(nextInterPos, nextOffset);
                            physicsBones.predRot[bi] = nextInterRot;

                            physicsBones.backupRot[bi] = physicsBones.rot[bi];
                            physicsBones.rot[bi] = interRot;
                            physicsBones.prevRot[bi] = physicsBones.rot[bi];
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
                                physicsBones.pos[bi] = DirectX::XMVectorAdd(physicsBones.pos[pbi], offset);
                                physicsBones.prevPos[bi] = physicsBones.pos[bi];

                                const Vector nextOffset = DirectX::XMVector3Rotate(DirectX::XMVectorScale(physicsBones.offset[bi], physicsBones.orgWorldScale[pbi]), physicsBones.predRot[pbi]);
                                physicsBones.predPos[bi] = DirectX::XMVectorAdd(physicsBones.predPos[pbi], nextOffset);
                                physicsBones.predRot[bi] = physicsBones.predRot[pbi];

                                physicsBones.backupRot[bi] = physicsBones.rot[bi];
                                physicsBones.rot[bi] = physicsBones.rot[pbi];
                                physicsBones.prevRot[bi] = physicsBones.rot[bi];
                            }
                            physicsBones.rot[bi] = physicsBones.rot[pbi];
                            physicsBones.orgWorldScale[bi] = physicsBones.orgWorldScale[pbi];
                        }
                    }
                }
            },
            tbb::static_partitioner());

        timeProfiler.End(this);
    }

    void XPBDWorld::UpdateGlobalAABBTree()
    {
        // logger::info("{}", __func__);

        static TimeProfiler timeProfiler(__func__);
        timeProfiler.Start();

        if (objIdxToTreeNodeIdx.size() < objectDatas.objectID.size())
        {
            objIdxToTreeNodeIdx.resize(objectDatas.objectID.size(), UINT32_MAX);
        }

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
            {
                objIdxToTreeNodeIdx[oi] = globalAABBTree.UpdateLeaf(objIdxToTreeNodeIdx[oi], objectDatas.boundingAABB[oi]);
            }
        }

        timeProfiler.End(this);
    }

    void XPBDWorld::ObjectCulling()
    {
        RE::NiCamera* niCam = RE::Main::WorldRootCamera();
        if (!niCam)
            return;

        static TimeProfiler timeProfiler(__func__);
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
                    if (objectDatas.objectID[oi] == 0 || objectDatas.objectID[oi] == 0x14)
                        continue;
                    if (objectDatas.isDisableByToggle[oi])
                    {
                        objectDatas.isDisable[oi] = objectDatas.isDisableByToggle[oi];
                        continue;
                    }
                    const AABB& boundingAABB = objectDatas.boundingAABB[oi];
                    if (boundingAABB.IsInvalid())
                        continue;

                    // culling by distance
                    const Vector closePt = DirectX::XMVectorMax(boundingAABB.min, DirectX::XMVectorMin(camPos, boundingAABB.max));
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
                    objectDatas.isDisable[oi] = isOutside ? 1 : 0;

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

        timeProfiler.End(this);
    }

    void XPBDWorld::UpdateWindStrength()
    {
        if (physicsBonesGroup.empty() || windSpeed <= Epsilon)
            return;

        static TimeProfiler timeProfiler(__func__);
        timeProfiler.Start();

        const std::uint32_t groups = physicsBonesGroup.size() - 1u;
        const std::uint32_t quality = std::max(1u, std::min(WIND_DETECT_QUALITY, groups));
        const std::uint32_t chunkSize = (groups + quality - 1u) * reciprocal(quality);
        const std::uint32_t gBegin = (currentFrame % quality) * chunkSize;
        const std::uint32_t gEnd = std::min(gBegin + chunkSize, groups);
        if (gBegin >= groups)
            return;
        const Vector windVector = DirectX::XMVectorSet(sin(windAngle), std::cos(windAngle), 0.0f, 0.0f);

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
            const Vector havokFrom = DirectX::XMVectorScale(from, SkyrimWorldScale);
            const Vector windRayTo = DirectX::XMVectorScale(windVector, -WIND_DETECT_RANGE);
            const Vector to = DirectX::XMVectorAdd(from, windRayTo);
            const Vector havokTo = DirectX::XMVectorScale(to, SkyrimWorldScale);
            
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
            const float currentMultiplier = objectDatas.windMultiplier[oi];
            float responseSpeed = 0.02f + (windSpeed * 0.5f);
            if (windMultiplier > currentMultiplier)
                responseSpeed *= 2.5f;
            else
                responseSpeed *= 1.2f;
            responseSpeed = std::clamp(responseSpeed, 0.01f, 1.0f);
            objectDatas.windMultiplier[oi] = currentMultiplier + (windMultiplier - currentMultiplier) * responseSpeed;
        }

        timeProfiler.End(this);
    }

    void XPBDWorld::PredictBones(const float deltaTime)
    {
        // logger::info("{}", __func__);

        static TimeProfiler timeProfiler(__func__);
        timeProfiler.Start();

        const Vector dt = DirectX::XMVectorReplicate(deltaTime);
        const Vector invDt = DirectX::XMVectorReciprocal(dt);
        const Vector invDtDb = DirectX::XMVectorScale(invDt, 2.0f);
        const float flutterNoise = 1.0f + 0.5f * sin(currentFrame * 0.1f);
        const float windX = sin(windAngle) * windSpeed * WIND_MULTIPLIER;
        const float windY = std::cos(windAngle) * windSpeed * WIND_MULTIPLIER;
        const float windZFlutter = windSpeed * flutterNoise * WIND_MULTIPLIER;
        const float windTimeFreq = currentFrame * 0.05f;
        tbb::parallel_for(
            tbb::blocked_range<std::uint32_t>(0, physicsBones.numBones, 128),
            [&](const tbb::blocked_range<std::uint32_t>& r) {
                for (std::uint32_t bi = r.begin(); bi != r.end(); ++bi)
                {
                    const std::uint32_t oi = physicsBones.objIdx[bi];
                    if (IsDisable(oi))
                        continue;
                    if (physicsBones.invMass[bi] <= Epsilon)
                        continue;
                    const std::uint32_t pbi = physicsBones.parentBoneIdx[bi];
                    if (pbi == UINT32_MAX)
                        continue;

                    Vector linearDeltaVel = vZero;
                    const Quaternion invRot = DirectX::XMQuaternionInverse(physicsBones.rot[pbi]);

                    // apply linear velocity
                    {
                        // inertia
                        {
                            const Vector worldAcc = objectDatas.acceleration[oi];
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

                        // inertia correction
                        {
                            const Vector worldVel = objectDatas.velocity[oi];
                            const Vector localVel = DirectX::XMVector3Rotate(worldVel, invRot);
                            const Vector isPositive = DirectX::XMVectorGreaterOrEqual(localVel, vZero);
                            const Vector currentCorrection = DirectX::XMVectorSelect(physicsBones.inertiaCorrectionNegative[bi], physicsBones.inertiaCorrectionPositive[bi], isPositive);
                            const Vector localCorection = DirectX::XMVectorMultiply(localVel, currentCorrection);
                            const Vector worldCorection = DirectX::XMVector3Rotate(localCorection, physicsBones.rot[pbi]);
                            linearDeltaVel = DirectX::XMVectorAdd(linearDeltaVel, worldCorection);
                            physicsBones.posVel[bi] = DirectX::XMVectorAdd(physicsBones.posVel[bi], worldCorection);
                        }
                    }

                    // apply wind
                    if (Epsilon < objectDatas.windMultiplier[oi])
                    {
                        const float posNoise = sin(windTimeFreq + (DirectX::XMVectorGetX(physicsBones.pos[bi]) * 0.02f));
                        const float windZ = windZFlutter * (0.3f + posNoise * 0.4f);
                        const Vector worldWind = DirectX::XMVectorSet(windX, windY, windZ, 0.0f);
                        const Vector localWind = DirectX::XMVector3Rotate(worldWind, invRot);
                        const Vector localWindScale = DirectX::XMVectorScale(physicsBones.windFactor[bi], objectDatas.windMultiplier[oi] * physicsBones.invMass[bi]);
                        const Vector localWindForce = DirectX::XMVectorMultiply(localWind, DirectX::XMVectorMultiply(localWindScale, dt));
                        const Vector worldWindForce = DirectX::XMVector3Rotate(localWindForce, physicsBones.rot[bi]);
                        linearDeltaVel = DirectX::XMVectorAdd(linearDeltaVel, worldWindForce);
                        physicsBones.posVel[bi] = DirectX::XMVectorAdd(physicsBones.posVel[bi], worldWindForce);
                    }

                    // apply centrifugal force
                    {
                        const Quaternion q_delta = objectDatas.deltaWorldRot[oi];
                        const Vector axisRaw = DirectX::XMVectorSetW(q_delta, 0.0f);
                        const Vector axisLenSq = DirectX::XMVector3LengthSq(axisRaw);
                        if (DirectX::XMVector3Greater(axisLenSq, vEpsilon))
                        {
                            const float w = std::clamp(DirectX::XMVectorGetW(q_delta), -1.0f, 1.0f);
                            const float angle = 2.0f * std::acos(w);
                            const Vector axis = DirectX::XMVectorMultiply(axisRaw, DirectX::XMVectorReciprocalSqrt(axisLenSq));
                            const Vector omega = DirectX::XMVectorMultiply(axis, DirectX::XMQuaternionMultiply(DirectX::XMVectorReplicate(angle), invDt));
                            std::uint32_t rootPbi = pbi;
                            while (rootPbi != UINT32_MAX && Epsilon < physicsBones.invMass[rootPbi])
                            {
                                rootPbi = physicsBones.parentBoneIdx[rootPbi];
                            }
                            const Vector rootPos = (rootPbi != UINT32_MAX) ? physicsBones.pos[rootPbi] : objectDatas.prevWorldPos[oi];
                            const Vector r = DirectX::XMVectorSubtract(physicsBones.pos[bi], rootPos);
                            const Vector tangential = DirectX::XMVector3Cross(omega, r);
                            const Vector centrifugalAccel = DirectX::XMVectorScale(DirectX::XMVector3Cross(tangential, omega), 0.15f);
                            const Vector centrifugalDeltaVel = DirectX::XMVectorMultiply(centrifugalAccel, dt);
                            linearDeltaVel = DirectX::XMVectorAdd(linearDeltaVel, centrifugalDeltaVel);
                            physicsBones.posVel[bi] = DirectX::XMVectorAdd(physicsBones.posVel[bi], centrifugalDeltaVel);
                        }
                    }

                    // predic linear
                    {
                        const Vector relVel = DirectX::XMVectorSubtract(physicsBones.posVel[bi], objectDatas.velocity[oi]);
                        physicsBones.predPos[bi] = DirectX::XMVectorMultiplyAdd(relVel, DirectX::XMVectorReplicate(deltaTime), physicsBones.pos[bi]);
                    }

                    // apply angular velocity
                    if (physicsBones.advancedRotation[bi])
                    {
                        // linear rot torque
                        const Quaternion invRotBi = DirectX::XMQuaternionConjugate(physicsBones.rot[bi]);
                        const Vector localVel = DirectX::XMVector3Rotate(physicsBones.posVel[bi], invRotBi);
                        const Vector localFakeTorque = DirectX::XMVector3TransformNormal(localVel, physicsBones.linearRotTorque[bi]);
                        if (DirectX::XMVector3Greater(DirectX::XMVector3LengthSq(localFakeTorque), vEpsilon))
                        {
                            const Vector worldFakeTorque = DirectX::XMVector3Rotate(localFakeTorque, physicsBones.rot[bi]);
                            physicsBones.angVel[bi] = DirectX::XMVectorAdd(physicsBones.angVel[bi], worldFakeTorque);
                        }
                    }

                    // predic rotation
                    {
                        std::uint32_t rootPbi = pbi;
                        while (rootPbi != UINT32_MAX && Epsilon < physicsBones.invMass[rootPbi])
                        {
                            rootPbi = physicsBones.parentBoneIdx[rootPbi];
                        }
                        if (rootPbi != UINT32_MAX)
                        {
                            const Quaternion delta = DirectX::XMQuaternionMultiply(DirectX::XMQuaternionConjugate(physicsBones.backupRot[rootPbi]), physicsBones.rot[rootPbi]);
                            const Vector magSq = DirectX::XMVector3LengthSq(DirectX::XMVectorSetW(delta, 0.0f));
                            if (DirectX::XMVector3Greater(magSq, vEpsilon))
                            {
                                const Vector pivot = physicsBones.predPos[rootPbi];
                                const Vector localPred = DirectX::XMVectorSubtract(physicsBones.predPos[bi], pivot);
                                physicsBones.predPos[bi] = DirectX::XMVectorAdd(pivot, DirectX::XMVector3Rotate(localPred, delta));
                                const Vector localPos = DirectX::XMVectorSubtract(physicsBones.pos[bi], pivot);
                                physicsBones.pos[bi] = DirectX::XMVectorAdd(pivot, DirectX::XMVector3Rotate(localPos, delta));
                                physicsBones.posVel[bi] = DirectX::XMVector3Rotate(physicsBones.posVel[bi], delta);
                                physicsBones.angVel[bi] = DirectX::XMVector3Rotate(physicsBones.angVel[bi], delta);
                                physicsBones.rot[bi] = DirectX::XMQuaternionNormalize(DirectX::XMQuaternionMultiply(physicsBones.rot[bi], delta));
                            }
                            const Quaternion q = physicsBones.rot[bi];
                            const Quaternion w = DirectX::XMVectorMultiply(physicsBones.angVel[bi], DirectX::XMVectorMultiply(dt, vHalf));
                            physicsBones.predRot[bi] = DirectX::XMQuaternionNormalize(DirectX::XMVectorAdd(q, DirectX::XMQuaternionMultiply(q, w)));
                        }
                    }

                    // linear limit
                    {
                        const Vector limitNegative = physicsBones.limitNegative[bi];
                        const Vector limitPositive = physicsBones.limitPositive[bi];

                        const Vector parentPos = physicsBones.predPos[pbi];
                        const Quaternion parentRot = physicsBones.predRot[pbi];
                        const float parentScale = physicsBones.orgWorldScale[pbi];

                        const Vector rotatedOffset = DirectX::XMVector3Rotate(DirectX::XMVectorScale(physicsBones.orgLocalPos[bi], parentScale), parentRot);
                        const Vector targetRestPos = DirectX::XMVectorAdd(parentPos, rotatedOffset);

                        const Vector worldOffset = DirectX::XMVectorSubtract(physicsBones.predPos[bi], targetRestPos);
                        const Vector localDeviation = DirectX::XMVector3Rotate(worldOffset, invRot);

                        const Vector isPositive = DirectX::XMVectorGreaterOrEqual(localDeviation, vZero);

                        const Vector currentRadii = DirectX::XMVectorSelect(limitNegative, limitPositive, isPositive);

                        const Vector normalizedDev = DirectX::XMVectorMultiply(localDeviation,DirectX::XMVectorReciprocal(currentRadii));
                        const Vector sqLength = DirectX::XMVector3LengthSq(normalizedDev);
                        if (DirectX::XMVector3Greater(sqLength, vOne))
                        {
                            const Vector clampedLocalDeviation = DirectX::XMVectorMultiply(localDeviation, DirectX::XMVectorReciprocal(sqLength));
                            const Vector clampedWorldOffset = DirectX::XMVector3Rotate(clampedLocalDeviation, parentRot);
                            physicsBones.predPos[bi] = DirectX::XMVectorAdd(targetRestPos, clampedWorldOffset);
                            physicsBones.posVel[bi] = DirectX::XMVectorMultiply(physicsBones.posVel[bi], vVelReduce);
                        }
                    }

                    // angular limit
                    if (physicsBones.advancedRotation[bi])
                    {
                        const Vector limitNegative = physicsBones.angularLimitNegative[bi];
                        const Vector limitPositive = physicsBones.angularLimitPositive[bi];

                        const Quaternion parentRot = physicsBones.predRot[pbi];
                        const Quaternion localOffset = physicsBones.orgLocalRot[bi];

                        const Quaternion restWorldRot = DirectX::XMQuaternionMultiply(physicsBones.orgLocalRot[bi], parentRot);
                        const Quaternion invRestWorldRot = DirectX::XMQuaternionConjugate(restWorldRot);

                        const Quaternion currentWorldRot = physicsBones.predRot[bi];
                        const Quaternion diff = DirectX::XMQuaternionMultiply(currentWorldRot, invRestWorldRot);
                        const Vector qy = DirectX::XMVectorSplatY(diff);
                        const Vector qw = DirectX::XMVectorSplatW(diff);
                        const Vector twistLenSq = DirectX::XMVectorAdd(DirectX::XMVectorMultiply(qy, qy), DirectX::XMVectorMultiply(qw, qw));
                        const Vector maskTwist = DirectX::XMVectorGreater(twistLenSq, vEpsilon);
                        const Vector invLen = DirectX::XMVectorReciprocalSqrt(twistLenSq);
                        const Vector qTwistRaw = DirectX::XMVectorMultiply(DirectX::XMVectorAndInt(diff, maskYW), invLen);
                        Quaternion qTwist = DirectX::XMVectorSelect(vWone, qTwistRaw, maskTwist);

                        const Vector twistAngle = DirectX::XMVectorScale(DirectX::XMVectorATan2(DirectX::XMVectorSplatY(qTwist), DirectX::XMVectorSplatW(qTwist)), 2.0f);
                        const Vector clampedTwistAngle = DirectX::XMVectorClamp(twistAngle, DirectX::XMVectorSplatY(limitNegative), DirectX::XMVectorSplatY(limitPositive));

                        const Vector halfTwist = DirectX::XMVectorScale(clampedTwistAngle, 0.5f);
                        Vector sinT, cosT;
                        DirectX::XMVectorSinCos(&sinT, &cosT, halfTwist);
                        qTwist = DirectX::XMVectorSelect(sinT, cosT, maskW);

                        const Quaternion invTwist = DirectX::XMQuaternionConjugate(qTwist);
                        Quaternion qSwing = DirectX::XMQuaternionMultiply(invTwist, diff);

                        const Vector swingAxis = DirectX::XMVectorAndInt(qSwing, maskXZ);
                        const Vector swingSinSq = DirectX::XMVector3LengthSq(swingAxis);

                        const Vector maskSwing = DirectX::XMVectorGreater(swingSinSq, vEpsilon);
                        const Vector swingSin = DirectX::XMVectorSqrt(swingSinSq);

                        const Vector swingAngle = DirectX::XMVectorScale(DirectX::XMVectorATan2(swingSin, DirectX::XMVectorSplatW(qSwing)), 2.0f);
                        const Vector normSwingAxis = DirectX::XMVectorMultiply(swingAxis, DirectX::XMVectorReciprocal(swingSin));

                        const Vector isPosX = DirectX::XMVectorGreater(DirectX::XMVectorSplatX(normSwingAxis), vZero);
                        const Vector isPosZ = DirectX::XMVectorGreater(DirectX::XMVectorSplatZ(normSwingAxis), vZero);

                        const Vector limitX = DirectX::XMVectorSelect(DirectX::XMVectorNegate(limitNegative), limitPositive, isPosX);
                        const Vector limitZ = DirectX::XMVectorSelect(DirectX::XMVectorNegate(limitNegative), limitPositive, isPosZ);

                        const Vector a = DirectX::XMVectorMax(DirectX::XMVectorAbs(DirectX::XMVectorSplatX(limitX)), vEpsilon);
                        const Vector b = DirectX::XMVectorMax(DirectX::XMVectorAbs(DirectX::XMVectorSplatZ(limitZ)), vEpsilon);

                        const Vector a2 = DirectX::XMVectorMultiply(a, a);
                        const Vector b2 = DirectX::XMVectorMultiply(b, b);
                        const Vector x2 = DirectX::XMVectorMultiply(DirectX::XMVectorSplatX(normSwingAxis), DirectX::XMVectorSplatX(normSwingAxis));
                        const Vector z2 = DirectX::XMVectorMultiply(DirectX::XMVectorSplatZ(normSwingAxis), DirectX::XMVectorSplatZ(normSwingAxis));

                        const Vector denomSq = DirectX::XMVectorAdd(DirectX::XMVectorMultiply(a2, z2), DirectX::XMVectorMultiply(b2, x2));
                        const Vector ellipseMaxAngle = DirectX::XMVectorMultiply(DirectX::XMVectorMultiply(a, b), DirectX::XMVectorReciprocalSqrt(denomSq));

                        const Vector absSwingAngle = DirectX::XMVectorAbs(swingAngle);
                        const Vector exceedsMask = DirectX::XMVectorGreater(absSwingAngle, ellipseMaxAngle);

                        const Vector signMask = DirectX::XMVectorAndInt(swingAngle, maskSign);
                        const Vector clampedSwingAngle = DirectX::XMVectorSelect(swingAngle, DirectX::XMVectorOrInt(ellipseMaxAngle, signMask), exceedsMask);

                        const Vector halfSwing = DirectX::XMVectorScale(clampedSwingAngle, 0.5f);
                        Vector sinS, cosS;
                        DirectX::XMVectorSinCos(&sinS, &cosS, halfSwing);

                        const Quaternion qSwingClamped = DirectX::XMVectorSelect(DirectX::XMVectorMultiply(normSwingAxis, sinS), cosS, maskW);
                        qSwing = DirectX::XMVectorSelect(qSwing, qSwingClamped, DirectX::XMVectorAndInt(maskSwing, exceedsMask));
                        const Quaternion clampedDiff = DirectX::XMQuaternionMultiply(qTwist, qSwing);
                        if (!DirectX::XMQuaternionEqual(diff, clampedDiff))
                        {
                            physicsBones.predRot[bi] = DirectX::XMQuaternionNormalize(DirectX::XMQuaternionMultiply(clampedDiff, restWorldRot));
                            const Quaternion deltaQ = DirectX::XMQuaternionMultiply(physicsBones.predRot[bi], DirectX::XMQuaternionConjugate(physicsBones.rot[bi]));
                            const Vector velocityMultiplier = DirectX::XMVectorMultiply(invDtDb, vVelReduce);
                            Vector deltaOmega = DirectX::XMVectorMultiply(deltaQ, velocityMultiplier);
                            deltaOmega = DirectX::XMVectorAndInt(deltaOmega, maskXYZ);
                            physicsBones.angVel[bi] = DirectX::XMVectorAdd(physicsBones.angVel[bi], deltaOmega);
                        }
                    }

                    // init
                    groundCache[bi].isCast = false;
                }
            },
            tbb::static_partitioner()
        );

        timeProfiler.End(this);
    }

    void XPBDWorld::CreateLocalSpatialHash()
    {
        if (collidersGroup.empty())
            return;
        // logger::info("{}", __func__);
        if (objectHashesSmall.size() < objectDatas.objectID.size())
            objectHashesSmall.resize(objectDatas.objectID.size());
        if (objectHashesLarge.size() < objectDatas.objectID.size())
            objectHashesLarge.resize(objectDatas.objectID.size());

        static TimeProfiler timeProfiler(__func__);
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
                    for (std::uint32_t ci = begin; ci < end; ++ci)
                    {
                        const std::uint32_t bi = colliders.boneIdx[ci];
                        const float radius = colliders.boundingSphere[ci] * physicsBones.orgWorldScale[bi] + physicsBones.collisionMargin[bi];
                        const Vector worldCenter = DirectX::XMVectorAdd(physicsBones.predPos[bi], DirectX::XMVector3Rotate(DirectX::XMVectorScale(colliders.boundingSphereCenter[ci], physicsBones.orgWorldScale[bi]), physicsBones.predRot[bi]));
                        if (radius <= SMALL_GRID_SIZE * 0.5f)
                        {
                            const std::uint32_t hashHigh = localHashSmall.HashWorldCoordsHigh(worldCenter);
                            localHashSmall.cellCount[hashHigh]++;
                            const std::uint32_t hashLow = localHashSmall.HashWorldCoordsLow(worldCenter);
                            localHashSmall.cellCount[hashLow]++;
                        }
                        else
                        {
                            const std::uint32_t hashHigh = localHashLarge.HashWorldCoordsHigh(worldCenter);
                            localHashLarge.cellCount[hashHigh]++;
                            const std::uint32_t hashLow = localHashLarge.HashWorldCoordsLow(worldCenter);
                            localHashLarge.cellCount[hashLow]++;
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
                        const std::uint32_t bi = colliders.boneIdx[ci];
                        const float radius = colliders.boundingSphere[ci] * physicsBones.orgWorldScale[bi] + physicsBones.collisionMargin[bi];
                        const Vector worldCenter = DirectX::XMVectorAdd(physicsBones.predPos[bi], DirectX::XMVector3Rotate(DirectX::XMVectorScale(colliders.boundingSphereCenter[ci], physicsBones.orgWorldScale[bi]), physicsBones.predRot[bi]));
                        if (radius <= SMALL_GRID_SIZE * 0.5f)
                        {
                            const std::uint32_t hashHigh = localHashSmall.HashWorldCoordsHigh(worldCenter);
                            const std::uint32_t offsetHigh = localHashSmall.cell[hashHigh] + localHashSmall.cellCount[hashHigh]++;
                            localHashSmall.entries[offsetHigh] = ci;
                            const std::uint32_t hashLow = localHashSmall.HashWorldCoordsLow(worldCenter);
                            const std::uint32_t offsetLow = localHashSmall.cell[hashLow] + localHashSmall.cellCount[hashLow]++;
                            localHashSmall.entries[offsetLow] = ci;
                        }
                        else
                        {
                            const std::uint32_t hashHigh = localHashLarge.HashWorldCoordsHigh(worldCenter);
                            const std::uint32_t offsetHigh = localHashLarge.cell[hashHigh] + localHashLarge.cellCount[hashHigh]++;
                            localHashLarge.entries[offsetHigh] = ci;
                            const std::uint32_t hashLow = localHashLarge.HashWorldCoordsLow(worldCenter);
                            const std::uint32_t offsetLow = localHashLarge.cell[hashLow] + localHashLarge.cellCount[hashLow]++;
                            localHashLarge.entries[offsetLow] = ci;
                        }
                    }
                }
            },
            tbb::auto_partitioner()
        );

        timeProfiler.End(this);
    }

    void XPBDWorld::GenerateCollisionManifolds()
    {
        if (collidersGroup.empty())
            return;
        // logger::info("{}", __func__);

        static TimeProfiler timeProfiler(__func__);
        timeProfiler.Start();

        auto AddManifold = [&](const std::uint32_t coiA, const std::uint32_t coiB) {
            ContactManifold manifold;
            bool isCollide = false;
            if (colliders.colliderType[coiA] == ColliderType::kSphere)
            {
                if (colliders.colliderType[coiB] == ColliderType::kSphere)
                    isCollide = SpherevsSphere(coiA, coiB, manifold);
                else if (colliders.colliderType[coiB] == ColliderType::kConvexHull)
                    isCollide = ConvexHullvsSphere(coiB, coiA, manifold);
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
                manifoldCache[idx] = {coiA, coiB, manifold};
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
                        const Vector worldCenterA = DirectX::XMVectorAdd(physicsBones.predPos[biA], DirectX::XMVector3Rotate(DirectX::XMVectorScale(colliders.boundingSphereCenter[coiA], physicsBones.orgWorldScale[biA]), physicsBones.predRot[biA]));
                        checkedB.clear();
                        auto CheckCell = [&](const std::uint32_t hash, const LocalSpatialHash& ownHash) {
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
                const AABB objAABB = objectDatas.boundingAABB[oi];
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
                                    const Vector worldCenterA = DirectX::XMVectorAdd(physicsBones.predPos[biA], DirectX::XMVector3Rotate(DirectX::XMVectorScale(colliders.boundingSphereCenter[coiA], physicsBones.orgWorldScale[biA]), physicsBones.predRot[biA]));
                                    checkedB.clear();
                                    auto CheckCell = [&](const std::uint32_t hash, const LocalSpatialHash& anotherHash) {
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

        timeProfiler.End(this);

        {
            colCandidatesStackCount++;
            if (colCandidatesStackCount >= 1000)
            {
                logger::debug("total collide candidate count {}", static_cast<std::uint32_t>(std::floor(totalColCandidates * 0.001f)));
                totalColCandidates = 0;
                colCandidatesStackCount = 0;
            }

            static double totalValidCollisions = 0;
            static std::uint32_t colSolveCount = 0;
            totalValidCollisions += std::min(manifoldCacheCount, expectedCollisionCount);
            colSolveCount++;
            if (colSolveCount >= 1000)
            {
                logger::debug("total actual collide count {}", static_cast<std::uint32_t>(std::floor(totalValidCollisions * 0.001f)));
                totalValidCollisions = 0;
                colSolveCount = 0;
            }
        }
    }

    void XPBDWorld::GenerateGroundCache(const float stepCount)
    {
        if (collidersLeafs.empty() || GROUND_DETECT_RANGE <= Epsilon)
            return;
        // logger::info("{}", __func__);

        static TimeProfiler timeProfiler(__func__);
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
        for (std::uint32_t ci = begin; ci < end; ++ci)
        {
            const std::uint32_t oi = colliders.objIdx[begin];
            if (IsDisable(oi))
                continue;
            RE::bhkWorld* bhkWorld = objectDatas.bhkWorld[oi];
            if (!bhkWorld)
                continue;

            const std::uint32_t bi = colliders.boneIdx[ci];
            if (bi == UINT32_MAX)
                continue;
            if (groundCache[bi].isCast)
                continue;

            const AABB& localAABB = colliders.boundingAABB[ci];
            AABB worldAABB = colliders.boundingAABB[ci].GetWorldAABB(physicsBones.predPos[bi], physicsBones.predRot[bi], physicsBones.orgWorldScale[bi]);
            worldAABB.Fatten(physicsBones.collisionMargin[bi]);
            const Vector worldBottomCenter = DirectX::XMVectorSubtract(worldAABB.GetCenter(), DirectX::XMVectorSet(0.0f, 0.0f, DirectX::XMVectorGetZ(worldAABB.min), 0.0f));

            const Vector from = DirectX::XMVectorAdd(worldBottomCenter, groundRayFrom);
            const Vector havokFrom = DirectX::XMVectorScale(from, SkyrimWorldScale);
            const Vector to = DirectX::XMVectorAdd(worldBottomCenter, groundRayTo);
            const Vector havokTo = DirectX::XMVectorScale(to, SkyrimWorldScale);

            RE::bhkPickData pickData;
            pickData.rayInput.from = havokFrom;
            pickData.rayInput.to = havokTo;

            GroundHitCollector hitCollector;
            hitCollector.Reset();
            pickData.rayHitCollectorA8 = reinterpret_cast<RE::hkpClosestRayHitCollector*>(&hitCollector);
            bhkWorld->PickObject(pickData);
            groundCache[bi].isCast = true;
            if (hitCollector.rayHit.HasHit())
            {
                const float fromZ = DirectX::XMVectorGetZ(from);
                const float toZ = DirectX::XMVectorGetZ(to);
                groundCache[bi].hasHit = true;
                groundCache[bi].height = fromZ + (toZ - fromZ) * hitCollector.rayHit.hitFraction;
                groundCache[bi].normal = DirectX::XMVectorSetW(hitCollector.rayHit.normal.quad, 0.0f);
            }
            std::uint32_t pbi = physicsBones.parentBoneIdx[bi];
            while (pbi != UINT32_MAX)
            {
                tbb::spin_mutex::scoped_lock sl(physicsBonesLock[pbi]);
                if (groundCache[pbi].isCast)
                {
                    if (groundCache[pbi].hasHit && groundCache[pbi].height >= groundCache[bi].height)
                        break;
                }
                groundCache[pbi] = groundCache[bi];
                pbi = physicsBones.parentBoneIdx[pbi];
            }
        }

        timeProfiler.End(this);
    }

    void XPBDWorld::SolveCachedCollisions(const float deltaTime)
    {
        const std::uint32_t validCollisions = std::min(manifoldCacheCount, expectedCollisionCount);
        if (validCollisions == 0)
            return;

        static TimeProfiler timeProfiler(__func__);
        timeProfiler.Start();

        const float InvDtSq = reciprocal(deltaTime * deltaTime);
        auto SolveManifold = [&](const std::uint32_t coiA, const std::uint32_t coiB, const ContactManifold& manifold) {
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
            if (wA + wB + proxyInvInertiaA + proxyInvInertiaB <= Epsilon)
                return;

            tbb::spin_mutex::scoped_lock lock1, lock2;
            if (biA < biB)
            {
                if (Epsilon < wA || Epsilon < proxyInvInertiaA)
                    lock1.acquire(physicsBonesLock[biA]);
                if (Epsilon < wB || Epsilon < proxyInvInertiaB)
                    lock2.acquire(physicsBonesLock[biB]);
            }
            else
            {
                if (Epsilon < wB || Epsilon < proxyInvInertiaB)
                    lock1.acquire(physicsBonesLock[biB]);
                if (Epsilon < wA || Epsilon < proxyInvInertiaA)
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

            const Vector normal = manifold.normal;
            const float compliance = std::max(physicsBones.collisionCompliance[biA], physicsBones.collisionCompliance[biB]);
            const float alphaProxy = compliance * InvDtSq;

            for (std::uint32_t i = 0; i < manifold.pointCount; ++i)
            {
                const auto& cp = manifold.points[i];
                const Vector pA = physicsBones.predPos[biA];
                const Quaternion qA = physicsBones.predRot[biA];
                const Vector rA = DirectX::XMVector3Rotate(DirectX::XMVectorScale(cp.localPointA, physicsBones.orgWorldScale[biA]), qA);
                const Vector wPtA = DirectX::XMVectorAdd(pA, rA);

                const Vector pB = physicsBones.predPos[biB];
                const Quaternion qB = physicsBones.predRot[biB];
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
                if (Epsilon < proxyInvInertiaA)
                {
                    const Quaternion invRotA = DirectX::XMQuaternionConjugate(qA);
                    const Vector localRAxN = DirectX::XMVector3Rotate(rAxN, invRotA);
                    const Vector isPositiveA = DirectX::XMVectorGreaterOrEqual(localRAxN, vZero);
                    const Vector invInertiaA = DirectX::XMVectorSelect(physicsBones.invAngularInertiaNegative[biA], physicsBones.invAngularInertiaPositive[biA], isPositiveA);
                    const Vector localInertiaA = DirectX::XMVectorMultiply(localRAxN, invInertiaA);
                    worldInvTauA = DirectX::XMVector3Rotate(localInertiaA, qA);
                    wRotA = DirectX::XMVectorGetX(DirectX::XMVector3Dot(rAxN, worldInvTauA)) * biasA;

                    const float maxRotWeightA = wA * 2.0f;
                    if (wRotA > maxRotWeightA && wA > Epsilon)
                    {
                        const float scale = maxRotWeightA * reciprocal(wRotA);
                        wRotA = maxRotWeightA;
                        worldInvTauA = DirectX::XMVectorScale(worldInvTauA, scale);
                    }
                }

                float wRotB = 0.0f;
                Vector worldInvTauB = vZero;
                if (Epsilon < proxyInvInertiaB)
                {
                    const Quaternion invRotB = DirectX::XMQuaternionConjugate(qB);
                    const Vector localRBxN = DirectX::XMVector3Rotate(rBxN, invRotB);
                    const Vector isPositiveB = DirectX::XMVectorGreaterOrEqual(localRBxN, vZero);
                    const Vector invInertiaB = DirectX::XMVectorSelect( physicsBones.invAngularInertiaNegative[biB], physicsBones.invAngularInertiaPositive[biB], isPositiveB);
                    const Vector localInertiaB = DirectX::XMVectorMultiply(localRBxN, invInertiaB);
                    worldInvTauB = DirectX::XMVector3Rotate(localInertiaB, qB);
                    wRotB = DirectX::XMVectorGetX(DirectX::XMVector3Dot(rBxN, worldInvTauB)) * biasB;

                    const float maxRotWeightB = wB * 2.0f;
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
                if (Epsilon < proxyInvInertiaA)
                {
                    const Vector dThetaA = DirectX::XMVectorScale(worldInvTauA, relaxedLambda * rotConfidence * biasA);
                    const Vector thetaSqA = DirectX::XMVector3LengthSq(dThetaA);
                    if (DirectX::XMVector3Greater(thetaSqA, vEpsilon))
                    {
                        const Vector angleA = DirectX::XMVectorSqrt(thetaSqA);
                        const Vector axisA = DirectX::XMVectorMultiply(dThetaA, DirectX::XMVectorReciprocal(angleA));
                        const Quaternion qCorrA = DirectX::XMQuaternionRotationNormal(axisA, DirectX::XMVectorGetX(angleA));
                        physicsBones.predRot[biA] = DirectX::XMQuaternionNormalize(DirectX::XMQuaternionMultiply(physicsBones.predRot[biA], qCorrA));
                    }
                }

                if (Epsilon < wB)
                    physicsBones.predPos[biB] = DirectX::XMVectorSubtract(physicsBones.predPos[biB], DirectX::XMVectorScale(pCorrN, wB));
                if (Epsilon < proxyInvInertiaB)
                {
                    const Vector dThetaB = DirectX::XMVectorScale(worldInvTauB, -relaxedLambda * rotConfidence * biasB);
                    const Vector thetaSqB = DirectX::XMVector3LengthSq(dThetaB);
                    if (DirectX::XMVector3Greater(thetaSqB, vEpsilon))
                    {
                        const Vector angleB = DirectX::XMVectorSqrt(thetaSqB);
                        const Vector axisB = DirectX::XMVectorMultiply(dThetaB, DirectX::XMVectorReciprocal(angleB));
                        const Quaternion qCorrB = DirectX::XMQuaternionRotationNormal(axisB, DirectX::XMVectorGetX(angleB));
                        physicsBones.predRot[biB] = DirectX::XMQuaternionNormalize(DirectX::XMQuaternionMultiply(physicsBones.predRot[biB], qCorrB));
                    }
                }

                const float actualPush = relaxedLambda * wNormSum;
                const float squishDepth = currentDepth - actualPush;
                if (Epsilon < wA || Epsilon < proxyInvInertiaA)
                {
                    physicsBones.collideCache[biA].normal = DirectX::XMVectorAdd(physicsBones.collideCache[biA].normal, normal);
                    physicsBones.collideCache[biA].depth = std::max(physicsBones.collideCache[biA].depth, currentDepth);

                    if (currentDepth > physicsBones.deformCache[biA].depth)
                    {
                        physicsBones.deformCache[biA].normal = normal;
                        physicsBones.deformCache[biA].depth = squishDepth;
                    }
                }
                if (Epsilon < wB || Epsilon < proxyInvInertiaB)
                {
                    physicsBones.collideCache[biB].normal = DirectX::XMVectorAdd(physicsBones.collideCache[biB].normal, DirectX::XMVectorNegate(normal));
                    physicsBones.collideCache[biB].depth = std::max(physicsBones.collideCache[biB].depth, currentDepth);

                    if (currentDepth > physicsBones.deformCache[biB].depth)
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

        timeProfiler.End(this);
    }

    void XPBDWorld::SolveCachedGroundCollisions(const float deltaTime)
    {
        if (collidersLeafs.empty() || GROUND_DETECT_RANGE <= Epsilon)
            return;
        // logger::info("{}", __func__);

        static TimeProfiler timeProfiler(__func__);
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
                    if (!groundCache[bi].hasHit)
                        continue;
                    const float w = physicsBones.invMass[bi];
                    if (w <= Epsilon)
                        continue;

                    const Vector pos = physicsBones.predPos[bi];

                    const float groundHeight = groundCache[bi].height;
                    const Vector normal = groundCache[bi].normal;
                    const float scale = physicsBones.orgWorldScale[bi];
                    const Quaternion rot = physicsBones.predRot[bi];
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
                    const Vector worldInvTau = DirectX::XMVector3Rotate(localInertia, rot);
                    const float wRot = DirectX::XMVectorGetX(DirectX::XMVector3Dot(rXN, worldInvTau));
                    const float wSum = w + wRot;

                    const float alphaProxy = physicsBones.collisionCompliance[bi] * invDtSq;
                    const float lambda = currentDepth * reciprocal(wSum + alphaProxy);
                    const float relaxedLambda = lambda * COL_CONVERGENCE;
                    const Vector correction = DirectX::XMVectorScale(normal, relaxedLambda * w);
                    physicsBones.predPos[bi] = DirectX::XMVectorAdd(physicsBones.predPos[bi], correction);

                    if (Epsilon < wRot)
                    {
                        const Vector dTheta = DirectX::XMVectorScale(worldInvTau, relaxedLambda);
                        const Vector theta = DirectX::XMVectorSetW(dTheta, 0.0f);
                        const Vector dq = DirectX::XMVectorMultiply(DirectX::XMQuaternionMultiply(physicsBones.predRot[bi], theta), vHalf);
                        physicsBones.predRot[bi] = DirectX::XMQuaternionNormalize(DirectX::XMVectorAdd(physicsBones.predRot[bi], dq));
                    }

                    physicsBones.collideCache[bi].normal = DirectX::XMVectorAdd(physicsBones.collideCache[bi].normal, DirectX::XMVectorNegate(normal));
                    physicsBones.collideCache[bi].depth = std::max(physicsBones.collideCache[bi].depth, currentDepth);
                }
            },
            tbb::auto_partitioner()
        );

        timeProfiler.End(this);
    }

    void XPBDWorld::SolveConstraints(const float deltaTime, const bool initLambda)
    {
        if (constraintsGroup.empty() && angularConstraintsGroup.empty())
            return;
        // logger::info("{}", __func__);

        static TimeProfiler timeProfiler(__func__);
        timeProfiler.Start();

        const float invDtSq = reciprocal(deltaTime * deltaTime);
        tbb::parallel_invoke([&] {
            if (!constraintsGroup.empty() && !constraintsColorGroup.empty())
            {
                const std::uint32_t groups = constraintsGroup.size() - 1ull;
                tbb::parallel_for(
                    tbb::blocked_range<std::uint32_t>(0, groups),
                    [&](const tbb::blocked_range<std::uint32_t>& r) {
                        for (std::uint32_t g = r.begin(); g != r.end(); ++g)
                        {
                            const std::uint32_t begin = constraintsGroup[g];
                            const std::uint32_t end = constraintsGroup[g + 1];
                            if (begin >= end)
                                continue;
                            const std::uint32_t oi = constraints.objIdx[begin];
                            if (IsDisable(oi))
                                continue;

                            auto ccgIt = std::lower_bound(constraintsColorGroup.begin(), constraintsColorGroup.end(), begin);
                            std::uint32_t ccgi = std::distance(constraintsColorGroup.begin(), ccgIt);
                            const std::uint32_t colorGroups = constraintsColorGroup.size() - 1ull;
                            while (ccgi < colorGroups && constraintsColorGroup[ccgi] < end)
                            {
                                std::uint32_t c_begin = constraintsColorGroup[ccgi];
                                std::uint32_t c_end = std::min(static_cast<std::uint32_t>(constraintsColorGroup[ccgi + 1]), end);

                                for (std::uint32_t ci = c_begin; ci < c_end; ++ci)
                                {
                                    const std::uint32_t bi = constraints.boneIdx[ci];
                                    const float& invMass = physicsBones.invMass[bi];
                                    if (invMass <= Epsilon)
                                        continue;

                                    const std::uint32_t numAnchors = constraints.numAnchors[ci];
                                    const std::uint32_t aiBase = ci * ANCHOR_MAX;
                                    for (std::uint32_t anchor = 0; anchor < numAnchors; ++anchor)
                                    {
                                        const std::uint32_t ai = aiBase + anchor;
                                        auto& anchData = constraints.anchData[ai];
                                        const std::uint32_t& abi = anchData.anchIdx;
                                        if (abi == UINT32_MAX)
                                            continue;

                                        const float anchInvMass = physicsBones.invMass[abi];
                                        const float wSum = invMass + anchInvMass;
                                        if (wSum <= Epsilon)
                                            continue;

                                        const Vector dir = DirectX::XMVectorSubtract(physicsBones.predPos[bi], physicsBones.predPos[abi]);
                                        const float distSq = DirectX::XMVectorGetX(DirectX::XMVector3LengthSq(dir));
                                        if (distSq < Epsilon)
                                            continue;

                                        const float invDist = rsqrt(distSq);
                                        const Vector normal = DirectX::XMVectorScale(dir, invDist);
                                        const float dist = distSq * invDist;

                                        const float C = dist - anchData.restLen;
                                        const bool isLessThanZero = C < 0.0f;
                                        const float currentComp = isLessThanZero ? anchData.complianceSquish : anchData.complianceStretch;
                                        const float currentLimit = isLessThanZero ? anchData.squishMargin : anchData.stretchMargin;
                                        const float currentDamping = isLessThanZero ? anchData.squishDamping : anchData.stretchDamping;
                                        float currentFactor = 1.0f;
                                        if (Epsilon < currentLimit)
                                        {
                                            const float ratio = std::min(std::abs(C) * reciprocal(currentLimit), 1.0f);
                                            const float ratioCubic = ratio * ratio * ratio;
                                            currentFactor = std::max(1.0f - ratioCubic, Epsilon);
                                        }
                                        const float alphaProxy = (currentComp * currentFactor) * invDtSq;
                                        const float denom = wSum + alphaProxy;
                                        float deltaLambda = 0.0f;
                                        if (initLambda)
                                        {
                                            deltaLambda = -C * reciprocal(denom);
                                            anchData.lambda = deltaLambda;
                                        }
                                        else
                                        {
                                            deltaLambda = (-C - (alphaProxy * anchData.lambda)) * reciprocal(denom);
                                            anchData.lambda += deltaLambda;
                                        }

                                        const float correctionMagBi = deltaLambda * invMass;
                                        physicsBones.predPos[bi] = DirectX::XMVectorAdd(physicsBones.predPos[bi], DirectX::XMVectorScale(normal, correctionMagBi));
                                        if (Epsilon < anchInvMass)
                                        {
                                            const float correctionMagAbi = deltaLambda * anchInvMass;
                                            physicsBones.predPos[abi] = DirectX::XMVectorSubtract(physicsBones.predPos[abi], DirectX::XMVectorScale(normal, correctionMagAbi));
                                        }

                                        const float invWSum = reciprocal(wSum);
                                        if (Epsilon < currentDamping)
                                        {
                                            const Vector deltaBi = DirectX::XMVectorSubtract(physicsBones.predPos[bi], physicsBones.pos[bi]);
                                            const Vector deltaAbi = DirectX::XMVectorSubtract(physicsBones.predPos[abi], physicsBones.pos[abi]);

                                            const Vector relDelta = DirectX::XMVectorSubtract(deltaBi, deltaAbi);

                                            const float projDelta = DirectX::XMVectorGetX(DirectX::XMVector3Dot(relDelta, normal));
                                            const float dampLambda = (-projDelta * currentDamping) * invWSum;
                                            const Vector dampCorrection = DirectX::XMVectorScale(normal, dampLambda);

                                            physicsBones.predPos[bi] = DirectX::XMVectorMultiplyAdd(dampCorrection, DirectX::XMVectorReplicate(invMass), physicsBones.predPos[bi]);
                                            if (Epsilon < anchInvMass)
                                            {
                                                const Vector correctionA = DirectX::XMVectorScale(dampCorrection, anchInvMass);
                                                physicsBones.predPos[abi] = DirectX::XMVectorSubtract(physicsBones.predPos[abi], correctionA);
                                            }
                                        }
                                    }
                                }
                                ccgi++;
                            }
                        }
                    },
                    tbb::auto_partitioner()
                );
            } }, [&] {
            if (!angularConstraintsGroup.empty() && !angularConstraintsColorGroup.empty())
            {
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
                            while (ccgi < colorGroups && angularConstraintsColorGroup[ccgi] < end)
                            {
                                std::uint32_t c_begin = angularConstraintsColorGroup[ccgi];
                                std::uint32_t c_end = std::min(static_cast<std::uint32_t>(angularConstraintsColorGroup[ccgi + 1]), end);

                                for (std::uint32_t ci = c_begin; ci < c_end; ++ci)
                                {
                                    const std::uint32_t bi = angularConstraints.boneIdx[ci];
                                    const float invMass = physicsBones.invMass[bi];
                                    const Vector sumRotInertia = DirectX::XMVectorAdd(physicsBones.invAngularInertiaPositive[bi], physicsBones.invAngularInertiaNegative[bi]);
                                    const float proxyInvInertia = DirectX::XMVectorGetX(DirectX::XMVector3LengthSq(sumRotInertia));

                                    const std::uint32_t numAnchors = angularConstraints.numAnchors[ci];
                                    const std::uint32_t aciBase = ci * ANCHOR_MAX;
                                    for (std::uint32_t anchor = 0; anchor < numAnchors; ++anchor)
                                    {
                                        const std::uint32_t ai = aciBase + anchor;
                                        auto& anchData = angularConstraints.anchData[ai];
                                        const std::uint32_t& abi = anchData.anchIdx;
                                        if (abi == UINT32_MAX)
                                            continue;
                                        const float invMassA = physicsBones.invMass[abi];
                                        if (invMass + invMassA <= Epsilon)
                                            continue;

                                        const Vector sumRotInertiaA = DirectX::XMVectorAdd(physicsBones.invAngularInertiaPositive[abi], physicsBones.invAngularInertiaNegative[abi]);
                                        const float proxyInvInertiaA = DirectX::XMVectorGetX(DirectX::XMVector3LengthSq(sumRotInertiaA));
                                        if (proxyInvInertia + proxyInvInertiaA <= Epsilon)
                                            continue;

                                        const Quaternion target = DirectX::XMQuaternionMultiply(anchData.restRot, physicsBones.predRot[abi]);
                                        const Quaternion targetInv = DirectX::XMQuaternionConjugate(target);
                                        const Quaternion diff = DirectX::XMQuaternionMultiply(physicsBones.predRot[bi], targetInv);
                                        Vector diffVec = DirectX::XMVectorSetW(diff, 0.0f);
                                        if (DirectX::XMVectorGetW(diff) < 0.0f)
                                            diffVec = DirectX::XMVectorNegate(diffVec);
                                        const Vector localOmega = DirectX::XMVectorScale(diffVec, 2.0f);

                                        const Vector excessPos = DirectX::XMVectorMax(vZero, DirectX::XMVectorSubtract(localOmega, anchData.marginPositive));
                                        const Vector excessNeg = DirectX::XMVectorMin(vZero, DirectX::XMVectorAdd(localOmega, anchData.marginNegative));
                                        const Vector localViolation = DirectX::XMVectorAdd(excessPos, excessNeg);
                                        const Vector CSq = DirectX::XMVector3LengthSq(localViolation);
                                        if (DirectX::XMVector3LessOrEqual(CSq, vEpsilon))
                                            continue;

                                        const Vector vC = DirectX::XMVectorSqrt(CSq);
                                        const float C = DirectX::XMVectorGetX(vC);

                                        const Vector localDir = DirectX::XMVectorScale(localViolation, reciprocal(C));
                                        const Vector localDirAbs = DirectX::XMVectorAbs(localDir);
                                        const Vector isPositive = DirectX::XMVectorGreaterOrEqual(localViolation, vZero);

                                        const Vector compVec = DirectX::XMVectorSelect(anchData.complianceNegative, anchData.compliancePositive, isPositive);
                                        const float currentComp = DirectX::XMVectorGetX(DirectX::XMVector3Dot(compVec, localDirAbs));

                                        const Vector limitVec = DirectX::XMVectorSelect(anchData.marginNegative, anchData.marginPositive, isPositive);
                                        const float currentLimit = DirectX::XMVectorGetX(DirectX::XMVector3Dot(limitVec, localDirAbs));

                                        const Vector dampVec = DirectX::XMVectorSelect(anchData.dampingNegative, anchData.dampingPositive, isPositive);
                                        const float currentDamping = DirectX::XMVectorGetX(DirectX::XMVector3Dot(dampVec, localDirAbs));

                                        const Vector worldDir = DirectX::XMVector3Rotate(localDir, target);

                                        float wRotA = 0.0f;
                                        Vector worldInvTauA = vZero;
                                        if (Epsilon < proxyInvInertiaA)
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
                                        if (Epsilon < proxyInvInertia)
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
                                        if (wSum <= Epsilon)
                                            continue;

                                        float currentFactor = 1.0f;
                                        if (Epsilon < currentLimit)
                                        {
                                            const float ratio = std::min(std::abs(C) * reciprocal(currentLimit), 1.0f);
                                            const float ratioCubic = ratio * ratio * ratio;
                                            currentFactor = std::max(1.0f - ratioCubic, Epsilon);
                                        }

                                        const float alphaProxy = (currentComp * currentFactor) * invDtSq;
                                        const float denom = wSum + alphaProxy;
                                        float deltaLambda = 0.0f;
                                        if (initLambda)
                                        {
                                            deltaLambda = -C * reciprocal(denom);
                                            anchData.lambda = deltaLambda;
                                        }
                                        else
                                        {
                                            deltaLambda = (-C - (alphaProxy * anchData.lambda)) * reciprocal(denom);
                                            anchData.lambda += deltaLambda;
                                        }

                                        if (Epsilon < proxyInvInertiaA)
                                        {
                                            const Vector dThetaA = DirectX::XMVectorScale(worldInvTauA, -deltaLambda);
                                            const Vector thetaSqA = DirectX::XMVector3LengthSq(dThetaA);
                                            if (DirectX::XMVector3Greater(thetaSqA, vEpsilon))
                                            {
                                                const Vector angleA = DirectX::XMVectorSqrt(thetaSqA);
                                                const Vector axisA = DirectX::XMVectorMultiply(dThetaA, DirectX::XMVectorReciprocal(angleA));
                                                const Quaternion qCorrA = DirectX::XMQuaternionRotationNormal(axisA, DirectX::XMVectorGetX(angleA));
                                                physicsBones.predRot[abi] = DirectX::XMQuaternionNormalize(DirectX::XMQuaternionMultiply(physicsBones.predRot[abi], qCorrA));
                                            }
                                        }
                                        if (Epsilon < proxyInvInertia)
                                        {
                                            const Vector dTheta = DirectX::XMVectorScale(worldInvTau, deltaLambda);
                                            const Vector thetaSq = DirectX::XMVector3LengthSq(dTheta);
                                            if (DirectX::XMVector3Greater(thetaSq, vEpsilon))
                                            {
                                                const Vector angle = DirectX::XMVectorSqrt(thetaSq);
                                                const Vector axis = DirectX::XMVectorMultiply(dTheta, DirectX::XMVectorReciprocal(angle));
                                                const Quaternion qCorr = DirectX::XMQuaternionRotationNormal(axis, DirectX::XMVectorGetX(angle));
                                                physicsBones.predRot[bi] = DirectX::XMQuaternionNormalize(DirectX::XMQuaternionMultiply(physicsBones.predRot[bi], qCorr));
                                            }
                                        }

                                        if (Epsilon < currentDamping)
                                        {
                                            Quaternion dq = DirectX::XMQuaternionMultiply(DirectX::XMQuaternionConjugate(physicsBones.rot[bi]), physicsBones.predRot[bi]);
                                            if (DirectX::XMVectorGetW(dq) < 0.0f)
                                                dq = DirectX::XMVectorNegate(dq);
                                            const Vector angVel = DirectX::XMVectorSetW(DirectX::XMVectorScale(dq, 2.0f), 0.0f);

                                            Quaternion dqA = DirectX::XMQuaternionMultiply(DirectX::XMQuaternionConjugate(physicsBones.rot[abi]), physicsBones.predRot[abi]);
                                            if (DirectX::XMVectorGetW(dqA) < 0.0f)
                                                dqA = DirectX::XMVectorNegate(dqA);
                                            const Vector angVelA = DirectX::XMVectorSetW(DirectX::XMVectorScale(dqA, 2.0f), 0.0f);

                                            const Vector relAngVel = DirectX::XMVectorSubtract(angVel, angVelA);
                                            const float relAngVelDot = DirectX::XMVectorGetX(DirectX::XMVector3Dot(relAngVel, worldDir));
                                            const float angDampLambda = (-relAngVelDot * currentDamping) * reciprocal(wSum);

                                            if (Epsilon < proxyInvInertiaA)
                                            {
                                                const Vector dThetaA = DirectX::XMVectorScale(worldInvTauA, -angDampLambda);
                                                const Vector thetaSqA = DirectX::XMVector3LengthSq(dThetaA);
                                                if (DirectX::XMVector3Greater(thetaSqA, vEpsilon))
                                                {
                                                    const Vector angleA = DirectX::XMVectorSqrt(thetaSqA);
                                                    const Vector axisA = DirectX::XMVectorMultiply(dThetaA, DirectX::XMVectorReciprocal(angleA));
                                                    const Quaternion qCorrA = DirectX::XMQuaternionRotationNormal(axisA, DirectX::XMVectorGetX(angleA));
                                                    physicsBones.predRot[abi] = DirectX::XMQuaternionNormalize(DirectX::XMQuaternionMultiply(physicsBones.predRot[abi], qCorrA));
                                                }
                                            }
                                            if (Epsilon < proxyInvInertia)
                                            {
                                                const Vector dTheta = DirectX::XMVectorScale(worldInvTau, angDampLambda);
                                                const Vector thetaSq = DirectX::XMVector3LengthSq(dTheta);
                                                if (DirectX::XMVector3Greater(thetaSq, vEpsilon))
                                                {
                                                    const Vector angle = DirectX::XMVectorSqrt(thetaSq);
                                                    const Vector axis = DirectX::XMVectorMultiply(dTheta, DirectX::XMVectorReciprocal(angle));
                                                    const Quaternion qCorr = DirectX::XMQuaternionRotationNormal(axis, DirectX::XMVectorGetX(angle));
                                                    physicsBones.predRot[bi] = DirectX::XMQuaternionNormalize(DirectX::XMQuaternionMultiply(physicsBones.predRot[bi], qCorr));
                                                }
                                            }
                                        }
                                    }
                                }
                                ccgi++;
                            }
                        }
                    },
                    tbb::auto_partitioner()
                );
            } });

        timeProfiler.End(this);
    }

    void XPBDWorld::SolveAnimDrive(const float deltaTime, const bool initLambda)
    {
        if (physicsBonesGroup.empty())
            return;

        static TimeProfiler timeProfiler(__func__);
        timeProfiler.Start();

        const float invDtSq = reciprocal(deltaTime * deltaTime);
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

                        const bool hasLinearAnimDrive = Epsilon < DirectX::XMVectorGetX(DirectX::XMVector3LengthSq(physicsBones.animDriveCompliance[bi]));
                        if (hasLinearAnimDrive)
                        {
                            const float wBi = physicsBones.invMass[bi];
                            const float wPbi = physicsBones.invMass[pbi];
                            const float wSum = wBi + wPbi;
                            if (Epsilon < wSum)
                            {
                                const Vector parentPos = physicsBones.predPos[pbi];
                                const Quaternion parentRot = physicsBones.predRot[pbi];
                                const float parentScale = physicsBones.orgWorldScale[pbi];

                                const Vector localPos = DirectX::XMVectorAdd(physicsBones.orgLocalPos[bi], physicsBones.offset[bi]);
                                const Vector rotatedLocalPos = DirectX::XMVector3Rotate(DirectX::XMVectorScale(localPos, parentScale), parentRot);
                                const Vector targetRestPos = DirectX::XMVectorAdd(parentPos, rotatedLocalPos);

                                const Vector dir = DirectX::XMVectorSubtract(physicsBones.predPos[bi], targetRestPos);
                                const Vector CSq = DirectX::XMVector3LengthSq(dir);
                                if (DirectX::XMVector3Greater(CSq, vEpsilon))
                                {
                                    const Vector vC = DirectX::XMVectorSqrt(CSq);
                                    const float C = DirectX::XMVectorGetX(vC);
                                    const Vector normal = DirectX::XMVectorMultiply(dir, DirectX::XMVectorReciprocal(CSq));

                                    const Vector localDirComp = DirectX::XMVector3Rotate(normal, DirectX::XMQuaternionConjugate(parentRot));
                                    const float currentComp = DirectX::XMVectorGetX(DirectX::XMVector3Dot(physicsBones.animDriveCompliance[bi], DirectX::XMVectorAbs(localDirComp)));

                                    const float alphaProxy = currentComp * invDtSq;
                                    const float denom = wSum + alphaProxy;

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

                                    if (Epsilon < wBi)
                                        physicsBones.predPos[bi] = DirectX::XMVectorAdd(physicsBones.predPos[bi], DirectX::XMVectorScale(normal, deltaLambda * wBi));
                                    if (Epsilon < wPbi)
                                        physicsBones.predPos[pbi] = DirectX::XMVectorAdd(physicsBones.predPos[pbi], DirectX::XMVectorScale(normal, -deltaLambda * wPbi));
                                }
                            }
                        }

                        const bool hasAngularAnimDrive = Epsilon < DirectX::XMVectorGetX(DirectX::XMVector3LengthSq(physicsBones.animDriveAngularCompliance[bi]));
                        if (hasAngularAnimDrive)
                        {
                            const Vector sumRotBi = DirectX::XMVectorAdd(physicsBones.invAngularInertiaPositive[bi], physicsBones.invAngularInertiaNegative[bi]);
                            const float proxyRotBi = DirectX::XMVectorGetX(DirectX::XMVector3LengthSq(sumRotBi));
                            const Vector sumRotPbi = DirectX::XMVectorAdd(physicsBones.invAngularInertiaPositive[pbi], physicsBones.invAngularInertiaNegative[pbi]);
                            const float proxyRotPbi = DirectX::XMVectorGetX(DirectX::XMVector3LengthSq(sumRotPbi));
                            if (Epsilon < proxyRotBi + proxyRotPbi)
                            {
                                const Quaternion parentRot = physicsBones.predRot[pbi];
                                const Quaternion localRestRot = physicsBones.orgLocalRot[bi];
                                const Quaternion targetRestRot = DirectX::XMQuaternionMultiply(localRestRot, parentRot);

                                const Quaternion targetInv = DirectX::XMQuaternionConjugate(targetRestRot);
                                const Quaternion diff = DirectX::XMQuaternionMultiply(physicsBones.predRot[bi], targetInv);

                                Vector omega = DirectX::XMVectorGetW(diff) < 0.0f ? DirectX::XMVectorNegate(diff) : diff;
                                omega = DirectX::XMVectorSetW(omega, 0.0f);
                                const Vector CSq = DirectX::XMVector3LengthSq(omega);
                                if (DirectX::XMVector3Greater(CSq, vEpsilon))
                                {
                                    const Vector vC = DirectX::XMVectorSqrt(CSq);
                                    const float C = DirectX::XMVectorGetX(vC);
                                    const Vector localDir = DirectX::XMVectorScale(omega, reciprocal(C));
                                    const Vector worldDir = DirectX::XMVector3Rotate(localDir, targetRestRot);

                                    const float currentComp = DirectX::XMVectorGetX(DirectX::XMVector3Dot(physicsBones.animDriveAngularCompliance[bi], DirectX::XMVectorAbs(localDir)));

                                    float wRotBi = 0.0f;
                                    Vector worldInvTauBi = vZero;
                                    if (Epsilon < proxyRotBi)
                                    {
                                        const Quaternion invRotBi = DirectX::XMQuaternionConjugate(physicsBones.predRot[bi]);
                                        const Vector localTauBi = DirectX::XMVector3Rotate(worldDir, invRotBi);
                                        const Vector isPosBi = DirectX::XMVectorGreaterOrEqual(localTauBi, vZero);
                                        const Vector invRotInertiaBi = DirectX::XMVectorSelect(physicsBones.invAngularInertiaNegative[bi], physicsBones.invAngularInertiaPositive[bi], isPosBi);

                                        const Vector localInvTauBi = DirectX::XMVectorMultiply(localTauBi, invRotInertiaBi);
                                        worldInvTauBi = DirectX::XMVector3Rotate(localInvTauBi, physicsBones.predRot[bi]);
                                        wRotBi = DirectX::XMVectorGetX(DirectX::XMVector3Dot(worldDir, worldInvTauBi));
                                    }

                                    float wRotPbi = 0.0f;
                                    Vector worldInvTauPbi = vZero;
                                    if (Epsilon < proxyRotPbi)
                                    {
                                        const Quaternion invRotPbi = DirectX::XMQuaternionConjugate(physicsBones.predRot[pbi]);
                                        const Vector localTauPbi = DirectX::XMVector3Rotate(worldDir, invRotPbi);
                                        const Vector isPosPbi = DirectX::XMVectorGreaterOrEqual(localTauPbi, vZero);
                                        const Vector invRotInertiaPbi = DirectX::XMVectorSelect(physicsBones.invAngularInertiaNegative[pbi], physicsBones.invAngularInertiaPositive[pbi], isPosPbi);

                                        const Vector localInvTauPbi = DirectX::XMVectorMultiply(localTauPbi, invRotInertiaPbi);
                                        worldInvTauPbi = DirectX::XMVector3Rotate(localInvTauPbi, physicsBones.predRot[pbi]);
                                        wRotPbi = DirectX::XMVectorGetX(DirectX::XMVector3Dot(worldDir, worldInvTauPbi));
                                    }

                                    const float wSum = wRotBi + wRotPbi;
                                    if (Epsilon < wSum)
                                    {
                                        const float alphaProxy = currentComp * invDtSq;
                                        const float denom = wSum + alphaProxy;

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

                                        if (Epsilon < proxyRotBi)
                                        {
                                            const Vector dThetaBi = DirectX::XMVectorScale(worldInvTauBi, deltaLambda);
                                            const Vector thetaSqBi = DirectX::XMVector3LengthSq(dThetaBi);
                                            if (DirectX::XMVector3Greater(thetaSqBi, vEpsilon))
                                            {
                                                const Vector angleBi = DirectX::XMVectorSqrt(thetaSqBi);
                                                const Vector axisBi = DirectX::XMVectorMultiply(dThetaBi, DirectX::XMVectorReciprocal(angleBi));
                                                const Quaternion qCorrBi = DirectX::XMQuaternionRotationNormal(axisBi, DirectX::XMVectorGetX(angleBi));
                                                physicsBones.predRot[bi] = DirectX::XMQuaternionNormalize(DirectX::XMQuaternionMultiply(physicsBones.predRot[bi], qCorrBi));
                                            }
                                        }

                                        if (Epsilon < proxyRotPbi)
                                        {
                                            const Vector dThetaPbi = DirectX::XMVectorScale(worldInvTauPbi, -deltaLambda);
                                            const Vector thetaSqPbi = DirectX::XMVector3LengthSq(dThetaPbi);
                                            if (DirectX::XMVector3Greater(thetaSqPbi, vEpsilon))
                                            {
                                                const Vector anglePbi = DirectX::XMVectorSqrt(thetaSqPbi);
                                                const Vector axisPbi = DirectX::XMVectorMultiply(dThetaPbi, DirectX::XMVectorReciprocal(anglePbi));
                                                const Quaternion qCorrPbi = DirectX::XMQuaternionRotationNormal(axisPbi, DirectX::XMVectorGetX(anglePbi));
                                                physicsBones.predRot[pbi] = DirectX::XMQuaternionNormalize(DirectX::XMQuaternionMultiply(physicsBones.predRot[pbi], qCorrPbi));
                                            }
                                        }
                                    }
                                }
                            }
                        }
                    }
                }
            },
            tbb::auto_partitioner());

        timeProfiler.End(this);
    }

    
    void XPBDWorld::SolveDeformConstraint(const float deltaTime)
    {
        if (deformConstraintsGroup.empty())
            return;

        static TimeProfiler timeProfiler(__func__);
        timeProfiler.Start();

        const Vector dt = DirectX::XMVectorReplicate(deltaTime);
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
                    const float invMass = physicsBones.invMass[bi];
                    const Vector worldPos = physicsBones.pos[bi];
                    const Quaternion worldRot = physicsBones.rot[bi];
                    const Quaternion invWorldRot = DirectX::XMQuaternionConjugate(worldRot);

                    Matrix currentMat = vmZeroAll;
                    std::uint32_t validCount = 0;
                    const std::uint32_t numAnchors = deformConstraints.numAnchors[ci];
                    const std::uint32_t aciBase = ci * ANCHOR_MAX;
                    for (std::uint32_t anchor = 0; anchor < numAnchors; ++anchor)
                    {
                        const std::uint32_t ai = aciBase + anchor;
                        auto& anchData = deformConstraints.anchData[ai];
                        const std::uint32_t& abi = anchData.anchIdx;

                        if (abi == UINT32_MAX)
                            continue;

                        const float invMassA = physicsBones.invMass[abi];
                        if (invMass + invMassA <= Epsilon)
                            continue;

                        const Vector worldPosA = physicsBones.pos[abi];
                        const Quaternion worldRotA = physicsBones.rot[abi];
                        const Quaternion invWorldRotA = DirectX::XMQuaternionConjugate(worldRotA);
                        Matrix currentMatA = vmZeroAll;
                        std::uint32_t validCountA = 0;
                        if (anchData.restLen > Epsilon)
                        {
                            const Vector dir = DirectX::XMVectorSubtract(worldPosA, worldPos);
                            const Vector currentLenSq = DirectX::XMVector3LengthSq(dir);
                            if (DirectX::XMVector3Greater(currentLenSq, vEpsilon))
                            {
                                const Vector currentLen = DirectX::XMVectorSqrt(currentLenSq);
                                Vector primary = DirectX::XMVectorMultiply(currentLen, DirectX::XMVectorReciprocal(DirectX::XMVectorReplicate(anchData.restLen)));
                                primary = DirectX::XMVectorMax(vMinScale, primary);

                                const Vector worldDir = DirectX::XMVectorMultiply(dir, DirectX::XMVectorReciprocal(currentLen));

                                if (invMass > Epsilon)
                                {
                                    const Vector localDir = DirectX::XMVector3Rotate(worldDir, invWorldRot);
                                    const Vector axisWeights = DirectX::XMVectorMultiply(localDir, localDir);
                                    const Vector effSensVec = DirectX::XMVector3Less(primary, vOne) ? DirectX::XMVectorMultiply(physicsBones.deformSquishSensitivity[bi], anchData.squishWeight) : DirectX::XMVectorMultiply(physicsBones.deformStretchSensitivity[bi], anchData.stretchWeight);
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
                                    const Vector effSensVecA = DirectX::XMVector3Less(primary, vOne) ? DirectX::XMVectorMultiply(physicsBones.deformSquishSensitivity[abi], anchData.squishWeight) : DirectX::XMVectorMultiply(physicsBones.deformStretchSensitivity[abi], anchData.stretchWeight);
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

                        const Quaternion targetRot = DirectX::XMQuaternionMultiply(anchData.restRot, worldRotA);
                        const Quaternion rotDiff = DirectX::XMQuaternionMultiply(DirectX::XMQuaternionConjugate(targetRot), worldRot);
                        const float w = std::clamp(DirectX::XMVectorGetW(rotDiff), -1.0f, 1.0f);
                        const float angle = 2.0f * std::acos(std::abs(w));
                        if (angle > Epsilon)
                        {
                            if (invMass > Epsilon)
                            {
                                const Vector rotAxis = DirectX::XMVectorSetW(rotDiff, 0.0f);
                                const Vector axisLenSq = DirectX::XMVector3LengthSq(rotAxis);
                                if (DirectX::XMVector3Greater(axisLenSq, vEpsilon))
                                {
                                    const Vector localRotAxis = DirectX::XMVectorMultiply(rotAxis, DirectX::XMVectorReciprocalSqrt(axisLenSq));
                                    const Vector effBulgeSens = DirectX::XMVectorMultiply(physicsBones.deformBulgeSensitivity[bi], anchData.bulgeWeight);
                                    const Vector finalBulge = DirectX::XMVectorMultiplyAdd(DirectX::XMVectorReplicate(angle), effBulgeSens, vOne);
                                    currentMat += CreateAnisoScaleMatrix(localRotAxis, 1.0f, finalBulge) - vmIdentity;
                                    ++validCount;
                                }
                            }
                            if (invMassA > Epsilon)
                            {
                                const Quaternion targetRotA = DirectX::XMQuaternionMultiply(DirectX::XMQuaternionConjugate(anchData.restRot), worldRot);
                                const Quaternion rotDiffA = DirectX::XMQuaternionMultiply(DirectX::XMQuaternionConjugate(targetRotA), worldRotA);
                                const Vector rotAxisA = DirectX::XMVectorSetW(rotDiffA, 0.0f);
                                const Vector axisLenSqA = DirectX::XMVector3LengthSq(rotAxisA);
                                if (DirectX::XMVector3Greater(axisLenSqA, vEpsilon))
                                {
                                    const Vector localRotAxisA = DirectX::XMVectorMultiply(rotAxisA, DirectX::XMVectorReciprocalSqrt(axisLenSqA));
                                    const Vector effBulgeSensA = DirectX::XMVectorMultiply(physicsBones.deformBulgeSensitivity[abi], anchData.bulgeWeight);
                                    const Vector finalBulgeA = DirectX::XMVectorMultiplyAdd(DirectX::XMVectorReplicate(angle), effBulgeSensA, vOne);
                                    currentMatA += CreateAnisoScaleMatrix(localRotAxisA, 1.0f, finalBulgeA) - vmIdentity;
                                    ++validCountA;
                                }
                            }
                        }

                        if (validCountA > 0)
                        {
                            tbb::spin_mutex::scoped_lock sl(physicsBonesLock[abi]);
                            physicsBones.deformScaleCache[abi] += currentMatA;
                            physicsBones.deformCount[abi] += validCountA;
                        }
                    }

                    if (validCount > 0)
                    {
                        tbb::spin_mutex::scoped_lock sl(physicsBonesLock[bi]);
                        physicsBones.deformScaleCache[bi] += currentMat;
                        physicsBones.deformCount[bi] += validCount;
                    }
                }
            },
            tbb::static_partitioner());

        tbb::parallel_for(
            tbb::blocked_range<std::uint32_t>(0, physicsBones.numBones, 128),
            [&](const tbb::blocked_range<std::uint32_t>& r) {
                for (std::uint32_t bi = r.begin(); bi != r.end(); ++bi)
                {
                    Matrix targetDevMat = physicsBones.deformScaleCache[bi];
                    std::uint32_t deformCount = physicsBones.deformCount[bi];
                    const PhysicsBones::CollideCache deformCache = physicsBones.deformCache[bi];

                    physicsBones.deformScaleCache[bi] = vmZeroAll;
                    physicsBones.deformCount[bi] = 0;
                    physicsBones.deformCache[bi] = {};

                    const std::uint32_t oi = physicsBones.objIdx[bi];
                    if (IsDisable(oi) || physicsBones.invMass[bi] <= Epsilon || physicsBones.isParticle[bi])
                        continue;

                    Matrix targetMat = vmIdentity;
                    if (deformCount > 0)
                    {
                        Vector invCount = DirectX::XMVectorReciprocal(DirectX::XMVectorReplicate(static_cast<float>(deformCount)));
                        targetDevMat.r[0] = DirectX::XMVectorMultiply(targetDevMat.r[0], invCount);
                        targetDevMat.r[1] = DirectX::XMVectorMultiply(targetDevMat.r[1], invCount);
                        targetDevMat.r[2] = DirectX::XMVectorMultiply(targetDevMat.r[2], invCount);
                        targetMat += targetDevMat;
                    }
                    if (deformCount == 0)
                        targetMat = vmIdentity;

                    if (deformCache.depth > Epsilon)
                    {
                        if (DirectX::XMVector3Greater(DirectX::XMVector3LengthSq(deformCache.normal), vEpsilon))
                        {
                            const Quaternion invWorldRot = DirectX::XMQuaternionConjugate(physicsBones.rot[bi]);
                            const Vector localNormal = DirectX::XMVector3Rotate(DirectX::XMVector3Normalize(deformCache.normal), invWorldRot);

                            const Vector axisWeights = DirectX::XMVectorMultiply(localNormal, localNormal);
                            const float colSensitivity = DirectX::XMVectorGetX(DirectX::XMVector3Dot(physicsBones.deformSquishSensitivity[bi], axisWeights));

                            float squishRatio = 1.0f - (deformCache.depth * colSensitivity);
                            squishRatio = std::max(squishRatio, 0.01f);
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
                        const Vector clampedScaleVec = DirectX::XMVectorMultiplyAdd(tDeviation, ellipseRadii, vOne);

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
                        const Vector springForce = DirectX::XMVectorMultiply(displacement, k);
                        const Vector dampingForce = DirectX::XMVectorMultiply(velocity, d);
                        const Vector acceleration = DirectX::XMVectorSubtract(springForce, dampingForce);
                        velocity = DirectX::XMVectorMultiplyAdd(acceleration, dt, velocity);
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
                        const Vector clampedScaleVec = DirectX::XMVectorMultiplyAdd(deviation, currentEllipseRadii, vOne);

                        const Vector scaleRatios = DirectX::XMVectorMultiply(clampedScaleVec, DirectX::XMVectorReciprocal(currentScaleVec));
                        currentMat.r[0] = DirectX::XMVectorMultiply(currentMat.r[0], DirectX::XMVectorSplatX(scaleRatios));
                        currentMat.r[1] = DirectX::XMVectorMultiply(currentMat.r[1], DirectX::XMVectorSplatY(scaleRatios));
                        currentMat.r[2] = DirectX::XMVectorMultiply(currentMat.r[2], DirectX::XMVectorSplatZ(scaleRatios));

                        /*const Vector bounceFactor = DirectX::XMVectorReplicate(-physicsBones.collisionRestitution[bi]);
                        velocityMat.r[0] = DirectX::XMVectorMultiply(velocityMat.r[0], bounceFactor);
                        velocityMat.r[1] = DirectX::XMVectorMultiply(velocityMat.r[1], bounceFactor);
                        velocityMat.r[2] = DirectX::XMVectorMultiply(velocityMat.r[2], bounceFactor);*/
                    }

                    currentMat.r[3] = vWone;
                    velocityMat.r[3] = vZero;

                    /*if (DirectX::XMMatrixIsNaN(currentMat))
                    {
                        currentMat = vmIdentity;
                        velocityMat = vmZeroAll;
                    }*/

                    physicsBones.deformVelocityScale[bi] = velocityMat;
                    /*const Vector finalSqX = DirectX::XMVector3LengthSq(currentMat.r[0]);
                    const Vector finalSqY = DirectX::XMVector3LengthSq(currentMat.r[1]);
                    const Vector finalSqZ = DirectX::XMVector3LengthSq(currentMat.r[2]);
                    const Vector finalScaleSqVec = DirectX::XMVectorSet(DirectX::XMVectorGetX(finalSqX), DirectX::XMVectorGetX(finalSqY), DirectX::XMVectorGetX(finalSqZ), 0.0f);
                    const Vector finalScaleVec = DirectX::XMVectorMax(DirectX::XMVectorSqrt(finalScaleSqVec), vEpsilon);
                    currentMat = DirectX::XMMatrixScalingFromVector(finalScaleVec);*/
                    physicsBones.deformScale[bi] = currentMat;
                }
            },
            tbb::static_partitioner()
        );

        timeProfiler.End(this);
    }

    void XPBDWorld::UpdateBoneVelocity(const float deltaTime)
    {
        // logger::info("{}", __func__);

        static TimeProfiler timeProfiler(__func__);
        timeProfiler.Start();

        const Vector invDt = DirectX::XMVectorReciprocal(DirectX::XMVectorReplicate(deltaTime));
        const Vector dbInvDt = DirectX::XMVectorScale(invDt, 2.0f);
        const float gravity = DirectX::XMVectorGetX(DirectX::XMVector3Length(SkyrimGravity));
        const Vector bounceThreshold = DirectX::XMVectorReplicate(-gravity * deltaTime * 2.0f);
        tbb::parallel_for(
            tbb::blocked_range<std::uint32_t>(0, physicsBones.numBones, 128),
            [&](const tbb::blocked_range<std::uint32_t>& r) {
                for (std::uint32_t bi = r.begin(); bi != r.end(); ++bi)
                {
                    const PhysicsBones::CollideCache collideCache = physicsBones.collideCache[bi];
                    physicsBones.collideCache[bi] = {};

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
                    const Vector linDampingFactor = DirectX::XMVectorSubtract(vOne, linDamping);

                    const Vector dampedLocalRelVel = DirectX::XMVectorMultiply(localRelVel, linDampingFactor);
                    const Vector dampedRelVel = DirectX::XMVector3Rotate(dampedLocalRelVel, physicsBones.rot[bi]);
                    const Vector objVel = objectDatas.velocity[oi];
                    Vector posVel = DirectX::XMVectorAdd(dampedRelVel, objVel);

                    const float depth = collideCache.depth;
                    const Vector normalImpulse = DirectX::XMVectorScale(invDt, depth);
                    const Vector friction = DirectX::XMVectorReplicate(physicsBones.collisionFriction[bi]);
                    const bool hasCollision = Epsilon < depth;
                    if (hasCollision)
                    {
                        const Vector n = DirectX::XMVector3Normalize(collideCache.normal);
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
                        if (DirectX::XMVector3LessOrEqual(v_t_lenSq, vEpsilon))
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
                        const Vector angDampingFactor = DirectX::XMVectorSubtract(vOne, angDamping);
                        const Vector dampedLocalAngVel = DirectX::XMVectorMultiply(localAngVel, angDampingFactor);
                        physicsBones.angVel[bi] = DirectX::XMVector3Rotate(dampedLocalAngVel, physicsBones.rot[bi]);
                        physicsBones.rot[bi] = physicsBones.predRot[bi];

                        if (hasCollision)
                        {
                            const Vector angVelLenSq = DirectX::XMVector3LengthSq(physicsBones.angVel[bi]);
                            if (DirectX::XMVector3LessOrEqual(angVelLenSq, vEpsilon))
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
                }
            },
            tbb::static_partitioner()
        );

        timeProfiler.End(this);
    }

    void XPBDWorld::ApplyToSkyrim(const bool syncFrame)
    {
        if (physicsBonesGroup.empty())
            return;
        // logger::info("{}", __func__);

        static TimeProfiler timeProfiler(__func__);
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

                        const Vector physicsWorldPos = DirectX::XMVectorLerp(physicsBones.prevPos[bi], physicsBones.pos[bi], alpha);
                        const Quaternion physicsWorldRot = DirectX::XMQuaternionNormalize(DirectX::XMQuaternionSlerp(physicsBones.prevRot[bi], physicsBones.rot[bi], alpha));
                        const Quaternion orgWorldRot = DirectX::XMQuaternionMultiply(physicsBones.orgLocalRot[bi], parentWorldRot);

                        Quaternion finalWorldRot = physicsWorldRot;

                        Vector restDir = DirectX::XMVector3Rotate(physicsBones.orgLocalPos[bi], parentWorldRot);
                        if (DirectX::XMVector3Less(DirectX::XMVector3LengthSq(restDir), vEpsilon))
                            restDir = DirectX::XMQuaternionMultiply(DirectX::XMVector3Rotate(vYone, physicsBones.orgLocalRot[bi]), parentWorldRot);
                        restDir = DirectX::XMVector3Normalize(restDir);

                        Vector currentDir = DirectX::XMVectorSubtract(physicsWorldPos, parentWorldPos);
                        if (DirectX::XMVector3Less(DirectX::XMVector3LengthSq(currentDir), vEpsilon))
                            currentDir = restDir;
                        else
                            currentDir = DirectX::XMVector3Normalize(currentDir);
                        const Vector dirLenSq = DirectX::XMVector3LengthSq(currentDir);
                        if (physicsBones.angularBlendFactor[bi] > Epsilon && DirectX::XMVector3Greater(dirLenSq, vEpsilon))
                        {
                            const Quaternion invOrgWorldRot = DirectX::XMQuaternionConjugate(orgWorldRot);
                            const Quaternion worldOffset = DirectX::XMQuaternionMultiply(invOrgWorldRot, physicsWorldRot);

                            const Vector offsetXYZ = DirectX::XMVectorAndInt(worldOffset, maskXYZ);
                            const Vector twistDot = DirectX::XMVector3Dot(offsetXYZ, restDir);
                            const Vector twistVec = DirectX::XMVectorMultiply(restDir, twistDot);

                            Quaternion qTwistWorld = DirectX::XMVectorSelect(twistVec, worldOffset, maskW);
                            const Vector twistLenSq = DirectX::XMVector4LengthSq(qTwistWorld);
                            const Vector maskTwist = DirectX::XMVectorGreater(twistLenSq, vEpsilon);
                            qTwistWorld = DirectX::XMVectorMultiply(qTwistWorld, DirectX::XMVectorReciprocalSqrt(twistLenSq));
                            qTwistWorld = DirectX::XMVectorSelect(DirectX::g_XMIdentityR3, qTwistWorld, maskTwist);

                            const Vector localDot = DirectX::XMVector3Dot(restDir, currentDir);
                            const Vector flippedMask = DirectX::XMVectorLess(localDot, vDotOppositeThreshold);

                            const Vector swingAxis = DirectX::XMVector3Cross(restDir, currentDir);
                            const Quaternion qSwingRaw = DirectX::XMVectorSelect(swingAxis, DirectX::XMVectorAdd(localDot, DirectX::g_XMOne), maskW);

                            Vector vUp = vYone;
                            const Vector upDot = DirectX::XMVectorAbs(DirectX::XMVector3Dot(restDir, vUp));
                            if (DirectX::XMVectorGetX(upDot) > 0.99f)
                                vUp = vXone;
                            const Vector fallbackAxis = DirectX::XMVector3Normalize(DirectX::XMVector3Cross(vUp, restDir));
                            const Quaternion qSwingFlipped = DirectX::XMQuaternionRotationNormal(fallbackAxis, DirectX::XM_PI);
                            const Quaternion qSwingWorld = DirectX::XMQuaternionNormalize(DirectX::XMVectorSelect(qSwingRaw, qSwingFlipped, flippedMask));

                            Quaternion targetRot = DirectX::XMQuaternionMultiply(qTwistWorld, qSwingWorld);
                            targetRot = DirectX::XMQuaternionNormalize(DirectX::XMQuaternionMultiply(orgWorldRot, targetRot));
                            const Vector vDot = DirectX::XMQuaternionDot(physicsWorldRot, targetRot);
                            const Vector isNegative = DirectX::XMVectorLess(vDot, vZero);
                            targetRot = DirectX::XMVectorSelect(targetRot, DirectX::XMVectorNegate(targetRot), isNegative);

                            finalWorldRot = DirectX::XMQuaternionNormalize(DirectX::XMQuaternionSlerp(physicsWorldRot, targetRot, physicsBones.angularBlendFactor[bi]));
                        }

                        const Vector diff = DirectX::XMVectorSubtract(physicsWorldPos, parentWorldPos);
                        const Vector invRot = DirectX::XMQuaternionInverse(parentWorldRot);

                        const Vector localPos = DirectX::XMVectorMultiply(DirectX::XMVector3Rotate(diff, invRot), DirectX::XMVectorReciprocal(parentWorldScale));
                        const Quaternion localRot = DirectX::XMQuaternionMultiply(finalWorldRot, invRot);

                        Vector calibWorldPos = DirectX::XMVectorAdd(DirectX::XMVector3Rotate(DirectX::XMVectorMultiply(localPos, parentWorldScale), parentWorldRot), parentWorldPos);
                        if (syncFrame)
                            calibWorldPos = DirectX::XMVectorAdd(calibWorldPos, worldPosOffset);
                        const RE::NiPoint3 localP = ToPoint3(localPos);
                        const RE::NiMatrix3 localR = ToNiMatrix(localRot);
                        const RE::NiPoint3 worldP = ToPoint3(calibWorldPos);

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
        
        timeProfiler.End(this);
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

                    AABB worldAABB = colliders.boundingAABB[ci].GetWorldAABB(physicsBones.predPos[bi], physicsBones.predRot[bi], physicsBones.orgWorldScale[bi]);
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
                    AABB boneAABB(physicsBones.predPos[bi], physicsBones.predPos[bi]);
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

    bool XPBDWorld::ConvexHullvsConvexHull(const std::uint32_t coiA, const std::uint32_t coiB, ContactManifold& outManifold)
    {
        const std::uint32_t biA = colliders.boneIdx[coiA];
        const std::uint32_t biB = colliders.boneIdx[coiB];

        const Vector posA = physicsBones.predPos[biA];
        const Quaternion rotA = physicsBones.predRot[biA];
        const float scaleA = physicsBones.orgWorldScale[biA];

        const Vector posB = physicsBones.predPos[biB];
        const Quaternion rotB = physicsBones.predRot[biB];
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
        Vector bestAxis = DirectX::XMVectorZero();
        bool flip = false;

        Vector hist[AXIS_HISTORY_MAX];
        std::uint32_t histCount = 0;
        auto TestAxis = [&](const Vector& inAxis) -> bool {
            const Vector lenSqV = DirectX::XMVector3LengthSq(inAxis);
            if (DirectX::XMVector3Less(lenSqV, vEpsilon))
                return true;

            const Vector axis = DirectX::XMVector3Normalize(inAxis);
            for (std::uint32_t i = 0; i < histCount; ++i)
            {
                const Vector dot = DirectX::XMVectorAbs(DirectX::XMVector3Dot(axis, hist[i]));
                if (DirectX::XMVector3Greater(dot, vAxisSimilarityLimit))
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

        auto& cache = convexHullCache[GetCacheKey(coiA, coiB)];
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
            if (DirectX::XMVector3LessOrEqual(DirectX::XMVector3LengthSq(wEdgeB[i]), vEpsilon))
                continue;
            wEdgeBValid[i] = true;
        }

        const std::uint32_t eCountA = hullA.edgeCount;
        for (std::uint32_t eiA = 0; eiA < eCountA; ++eiA)
        {
            const Vector eA = DirectX::XMVectorSet(hullA.eX[eiA], hullA.eY[eiA], hullA.eZ[eiA], 0);
            const Vector wA = DirectX::XMVector3Rotate(eA, rotA);

            if (DirectX::XMVector3LessOrEqual(DirectX::XMVector3LengthSq(wA), vEpsilon))
                continue;

            for (std::uint32_t eiB = 0; eiB < eCountB; ++eiB)
            {
                if (!wEdgeBValid[eiB])
                    continue;
                const Vector crossAxis = DirectX::XMVector3Cross(wA, wEdgeB[eiB]);
                if (DirectX::XMVector3LessOrEqual(DirectX::XMVector3LengthSq(crossAxis), vEpsilon))
                    continue;
                if (!TestAxis(crossAxis))
                    return false;
            }
        }

        const Vector normal = flip ? DirectX::XMVectorNegate(bestAxis) : bestAxis;
        cache.axis = bestAxis;
        cache.lastFrame = currentFrame;

        // manifold
        ContactManifold::ContactPoint tempPoints[5];
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

        alignas(16) DirectX::XMFLOAT4X4A fMatA, fMatB, fMatInvA, fMatInvB;
        DirectX::XMStoreFloat4x4A(&fMatA, matA);
        DirectX::XMStoreFloat4x4A(&fMatB, matB);
        DirectX::XMStoreFloat4x4A(&fMatInvA, matInvA);
        DirectX::XMStoreFloat4x4A(&fMatInvB, matInvB);

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

    bool XPBDWorld::ConvexHullvsSphere(const std::uint32_t coiHull, const std::uint32_t coiSphere, ContactManifold& outManifold)
    {
        const std::uint32_t biA = colliders.boneIdx[coiHull];
        const std::uint32_t biB = colliders.boneIdx[coiSphere];

        const Vector posA = physicsBones.predPos[biA];
        const Quaternion rotA = physicsBones.predRot[biA];
        const float scaleA = physicsBones.orgWorldScale[biA];

        const Vector posB = physicsBones.predPos[biB];
        const Quaternion rotB = physicsBones.predRot[biB];
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

        auto& cache = convexHullCache[GetCacheKey(coiHull, coiSphere)];

        bool collidedAny = false;
        Vector finalNormal = vZero;
        float frameMaxDepth = -1.0f;
        Vector frameBestLA = vZero;
        Vector frameBestLB = vZero;

        for (std::uint32_t sIdx = 0; sIdx < sphereB.sphereCount; ++sIdx)
        {
            const Vector lCenterB = DirectX::XMVectorSet(sphereB.cX[sIdx], sphereB.cY[sIdx], sphereB.cZ[sIdx], 0.0f);
            const Vector wCenterB = DirectX::XMVectorAdd(posB, DirectX::XMVector3Rotate(DirectX::XMVectorScale(lCenterB, scaleB), rotB));
            const float wRadiusB = sphereB.radius[sIdx] * scaleB + marginB;

            const Vector centerToCenter = DirectX::XMVectorSubtract(wCenterB, posA);

            float minOverlap = FLT_MAX;
            Vector bestAxis = DirectX::XMVectorZero();
            bool flip = false;

            Vector hist[AXIS_HISTORY_MAX];
            std::uint32_t histCount = 0;

            auto TestAxis = [&](const Vector& inAxis) -> bool {
                const Vector lenSqV = DirectX::XMVector3LengthSq(inAxis);
                if (DirectX::XMVector3Less(lenSqV, vEpsilon))
                    return true;

                const Vector axis = DirectX::XMVector3Normalize(inAxis);

                for (std::uint32_t i = 0; i < histCount; ++i)
                {
                    const Vector dot = DirectX::XMVectorAbs(DirectX::XMVector3Dot(axis, hist[i]));
                    if (DirectX::XMVector3Greater(dot, vAxisSimilarityLimit))
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
                if (DirectX::XMVector3LessOrEqual(DirectX::XMVector3LengthSq(crossAxis), vEpsilon))
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

            const Vector worldB_i = DirectX::XMVectorSubtract(wCenterB, DirectX::XMVectorScale(normal, wRadiusB));
            const Vector worldA_i = DirectX::XMVectorAdd(worldB_i, DirectX::XMVectorScale(normal, minOverlap));

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

        ContactManifold::ContactPoint tempPoints[5];
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

    bool XPBDWorld::SpherevsSphere(const std::uint32_t coiA, const std::uint32_t coiB, ContactManifold& outManifold)
    {
        const std::uint32_t biA = colliders.boneIdx[coiA];
        const std::uint32_t biB = colliders.boneIdx[coiB];

        const Vector posA = physicsBones.predPos[biA];
        const Quaternion rotA = physicsBones.predRot[biA];
        const float scaleA = physicsBones.orgWorldScale[biA];

        const Vector posB = physicsBones.predPos[biB];
        const Quaternion rotB = physicsBones.predRot[biB];
        const float scaleB = physicsBones.orgWorldScale[biB];

        const float marginA = physicsBones.collisionMargin[biA];
        const float marginB = physicsBones.collisionMargin[biB];
        const float sumMargin = marginA + marginB;

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

        const std::uint32_t activeMaskB = (1u << sphereB.sphereCount) - 1;

        bool collidedAny = false;
        Vector finalNormal = vZero;
        float frameMaxDepth = -1.0f;
        Vector frameBestLA = vZero;
        Vector frameBestLB = vZero;

#if defined(AVX) || defined(AVX2)
        for (std::uint32_t a = 0; a < sphereA.sphereCount; a += 4)
        {
            const std::uint32_t a0 = a;
            const std::uint32_t a1 = (a + 1 < sphereA.sphereCount) ? a + 1 : a;
            const std::uint32_t a2 = (a + 2 < sphereA.sphereCount) ? a + 2 : a;
            const std::uint32_t a3 = (a + 3 < sphereA.sphereCount) ? a + 3 : a;

            const __m256 v_ax_01 = _mm256_setr_ps(awX[a0], awX[a0], awX[a0], awX[a0], awX[a1], awX[a1], awX[a1], awX[a1]);
            const __m256 v_ay_01 = _mm256_setr_ps(awY[a0], awY[a0], awY[a0], awY[a0], awY[a1], awY[a1], awY[a1], awY[a1]);
            const __m256 v_az_01 = _mm256_setr_ps(awZ[a0], awZ[a0], awZ[a0], awZ[a0], awZ[a1], awZ[a1], awZ[a1], awZ[a1]);
            const __m256 v_ar_01 = _mm256_setr_ps(awR[a0], awR[a0], awR[a0], awR[a0], awR[a1], awR[a1], awR[a1], awR[a1]);

            const __m256 v_ax_23 = _mm256_setr_ps(awX[a2], awX[a2], awX[a2], awX[a2], awX[a3], awX[a3], awX[a3], awX[a3]);
            const __m256 v_ay_23 = _mm256_setr_ps(awY[a2], awY[a2], awY[a2], awY[a2], awY[a3], awY[a3], awY[a3], awY[a3]);
            const __m256 v_az_23 = _mm256_setr_ps(awZ[a2], awZ[a2], awZ[a2], awZ[a2], awZ[a3], awZ[a3], awZ[a3], awZ[a3]);
            const __m256 v_ar_23 = _mm256_setr_ps(awR[a2], awR[a2], awR[a2], awR[a2], awR[a3], awR[a3], awR[a3], awR[a3]);

            for (std::uint32_t b = 0; b < sphereB.sphereCount; b += 4)
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

                        const std::uint32_t a_idx = a + (bitIdx / 4);
                        const std::uint32_t b_idx = b + (bitIdx % 4);

                        if (a_idx < sphereA.sphereCount && b_idx < sphereB.sphereCount)
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
        for (std::uint32_t a = 0; a < sphereA.sphereCount; a += 4)
        {
            const std::uint32_t a0 = a;
            const std::uint32_t a1 = (a + 1 < sphereA.sphereCount) ? a + 1 : a;
            const std::uint32_t a2 = (a + 2 < sphereA.sphereCount) ? a + 2 : a;
            const std::uint32_t a3 = (a + 3 < sphereA.sphereCount) ? a + 3 : a;

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

            for (std::uint32_t b = 0; b < sphereB.sphereCount; b += 4)
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

                        const std::uint32_t a_idx = a + (bitIdx / 4);
                        const std::uint32_t b_idx = b + (bitIdx % 4);

                        if (a_idx < sphereA.sphereCount && b_idx < sphereB.sphereCount)
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

        auto& cache = convexHullCache[GetCacheKey(coiA, coiB)];
        cache.axis = finalNormal;
        cache.lastFrame = currentFrame;

        ContactManifold::ContactPoint tempPoints[5];
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
        return false;
    }

    std::uint32_t XPBDWorld::AllocateObject(RE::TESObjectREFR* object)
    {
        std::uint32_t objIdx = UINT32_MAX;
        // find exist slot
        for (std::uint32_t oi = 0; oi < objectDatas.objectID.size(); ++oi)
        {
            if (objectDatas.objectID[oi] != object->formID)
                continue;
            objIdx = oi;
            logger::debug("{:x} : found object {}", object->formID, objIdx);
            break;
        }

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
                }
                else
                {
                    objectDatas.prevWorldPos[objIdx] = ToVector(object->GetPosition());
                    objectDatas.prevNPCWorldRot[objIdx] = ToQuaternion(RE::NiMatrix3());
                    objectDatas.targetNPCWorldRot[objIdx] = objectDatas.prevNPCWorldRot[objIdx];
                }
                objectDatas.deltaWorldPos[objIdx] = vZero;
                objectDatas.deltaWorldRot[objIdx] = qZero;
                if (RE::TESObjectCELL* cell = object->GetParentCell(); cell)
                    objectDatas.bhkWorld[objIdx] = cell->GetbhkWorld();
                else
                    objectDatas.bhkWorld[objIdx] = nullptr;
                objectDatas.velocity[objIdx] = vZero;
                objectDatas.acceleration[objIdx] = vZero;
                objectDatas.boundingAABB[objIdx] = AABB();
                objectDatas.isStatic[objIdx] = 0;
                objectDatas.windMultiplier[objIdx] = 0.0f;
                objectDatas.maxManifoldPoints[objIdx] = 4;
                objectDatas.isDisable[objIdx] = 0;
                objectDatas.isDisableByToggle[objIdx] = 0;
                objectDatas.randState[objIdx] = rand_Hash(1103515245 + objIdx + objectDatas.objectID[objIdx]);
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
                    if (npcObj && npcObj->parent)
                    {
                        objectDatas.prevWorldPos.push_back(ToVector(object->loadedData->data3D->world.translate));
                        objectDatas.prevNPCWorldRot.push_back(ToQuaternion(npcObj->world.rotate));
                        objectDatas.targetNPCWorldRot.push_back(ToQuaternion(npcObj->world.rotate));
                        hasNPCNode = true;
                    }
                }
                if (!hasNPCNode)
                {
                    objectDatas.prevWorldPos.push_back(ToVector(object->GetPosition()));
                    objectDatas.prevNPCWorldRot.push_back(ToQuaternion(RE::NiMatrix3()));
                    objectDatas.targetNPCWorldRot.push_back(ToQuaternion(RE::NiMatrix3()));
                }
                objectDatas.deltaWorldPos.push_back(vZero);
                objectDatas.deltaWorldRot.push_back(qZero);
                if (RE::TESObjectCELL* cell = object->GetParentCell(); cell)
                    objectDatas.bhkWorld.push_back(cell->GetbhkWorld());
                else
                    objectDatas.bhkWorld.push_back(nullptr);
                objectDatas.velocity.push_back(vZero);
                objectDatas.acceleration.push_back(vZero);
                objectDatas.boundingAABB.push_back(AABB());
                objectDatas.isStatic.push_back(0);
                objectDatas.windMultiplier.push_back(0);
                objectDatas.maxManifoldPoints.push_back(4);
                objectDatas.isDisable.push_back(0);
                objectDatas.isDisableByToggle.push_back(0);
                objectDatas.randState.push_back(rand_Hash(1103515245 + objIdx + objectDatas.objectID[objIdx]));
                logger::debug("{:x} : add new object {}", object->formID, objIdx);
            }
        }
        return objIdx;
    }
    std::uint32_t XPBDWorld::AllocateRoot(const std::uint32_t objIdx, const ObjectDatas::Root& rootData)
    {
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
        if (rootIdx == UINT32_MAX)
        {
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
        physicsBones.limitPositive.push_back(vZero);
        physicsBones.limitNegative.push_back(vZero);
        physicsBones.angularLimitPositive.push_back(vZero);
        physicsBones.angularLimitNegative.push_back(vZero);
        physicsBones.inertiaPositive.push_back(vZero);
        physicsBones.inertiaNegative.push_back(vZero);
        physicsBones.invAngularInertiaPositive.push_back(vZero);
        physicsBones.invAngularInertiaNegative.push_back(vZero);
        physicsBones.inertiaCorrectionPositive.push_back(vZero);
        physicsBones.inertiaCorrectionNegative.push_back(vZero);
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

        physicsBones.collisionMargin.push_back(0);
        physicsBones.collisionShrink.push_back(0);
        physicsBones.collisionFriction.push_back(0);
        physicsBones.collisionCompliance.push_back(0);
        physicsBones.collisionRestitution.push_back(0);

        physicsBones.layerGroup.push_back(0);
        physicsBones.collideLayer.push_back(0);

        physicsBones.collideCache.push_back({});
        physicsBones.deformCache.push_back({});

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

        physicsBones.orgWorldScale.push_back(1);
        physicsBones.orgLocalPos.push_back(vZero);
        physicsBones.orgLocalRot.push_back(qZero);
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
        physicsBones.limitPositive.reserve(n);
        physicsBones.limitNegative.reserve(n);
        physicsBones.angularLimitPositive.reserve(n);
        physicsBones.angularLimitNegative.reserve(n);
        physicsBones.inertiaPositive.reserve(n);
        physicsBones.inertiaNegative.reserve(n);
        physicsBones.invAngularInertiaPositive.reserve(n);
        physicsBones.invAngularInertiaNegative.reserve(n);
        physicsBones.inertiaCorrectionPositive.reserve(n);
        physicsBones.inertiaCorrectionNegative.reserve(n);
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

        physicsBones.collisionMargin.reserve(n);
        physicsBones.collisionShrink.reserve(n);
        physicsBones.collisionFriction.reserve(n);
        physicsBones.collisionCompliance.reserve(n);
        physicsBones.collisionRestitution.reserve(n);

        physicsBones.layerGroup.reserve(n);
        physicsBones.collideLayer.reserve(n);

        physicsBones.collideCache.reserve(n);
        physicsBones.deformCache.reserve(n);

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

        physicsBones.orgWorldScale.reserve(n);
        physicsBones.orgLocalPos.reserve(n);
        physicsBones.orgLocalRot.reserve(n);
    }
    
    std::uint32_t XPBDWorld::AllocateConstraint()
    {
        const std::uint32_t newIdx = constraints.numConstraints++;
        constraints.boneIdx.push_back(UINT32_MAX);
        constraints.objIdx.push_back(UINT32_MAX);
        constraints.rootIdx.push_back(UINT32_MAX);
        constraints.colorGraph.push_back(0);
        constraints.numAnchors.push_back(0);
        for (std::uint32_t i = 0; i < ANCHOR_MAX; ++i)
        {
            constraints.anchData.push_back({});
        }
        return newIdx;
    }
    void XPBDWorld::ReserveConstraint(std::uint32_t n)
    {
        if (n == 0)
            return;
        n += constraints.numConstraints;
        constraints.boneIdx.reserve(n);
        constraints.objIdx.reserve(n);
        constraints.rootIdx.reserve(n);
        constraints.colorGraph.reserve(n);
        constraints.numAnchors.reserve(n);
        const std::uint32_t an = n * ANCHOR_MAX;
        constraints.anchData.reserve(an);
    }

    std::uint32_t XPBDWorld::AllocateAngularConstraint()
    {
        const std::uint32_t newIdx = angularConstraints.numConstraints++;
        angularConstraints.boneIdx.push_back(UINT32_MAX);
        angularConstraints.objIdx.push_back(UINT32_MAX);
        angularConstraints.rootIdx.push_back(UINT32_MAX);
        angularConstraints.colorGraph.push_back(0);
        angularConstraints.numAnchors.push_back(0);
        for (std::uint32_t i = 0; i < ANCHOR_MAX; ++i)
        {
            angularConstraints.anchData.push_back({});
        }
        return newIdx;
    }
    void XPBDWorld::ReserveAngularConstraint(std::uint32_t n)
    {
        if (n == 0)
            return;
        n += angularConstraints.numConstraints;
        angularConstraints.boneIdx.reserve(n);
        angularConstraints.objIdx.reserve(n);
        angularConstraints.rootIdx.reserve(n);
        angularConstraints.colorGraph.reserve(n);
        angularConstraints.numAnchors.reserve(n);
        const std::uint32_t an = n * ANCHOR_MAX;
        angularConstraints.anchData.reserve(an);
    }

    std::uint32_t XPBDWorld::AllocateDeformConstraint()
    {
        const std::uint32_t newIdx = deformConstraints.numConstraints++;
        deformConstraints.boneIdx.push_back(UINT32_MAX);
        deformConstraints.objIdx.push_back(UINT32_MAX);
        deformConstraints.rootIdx.push_back(UINT32_MAX);
        deformConstraints.numAnchors.push_back(0);
        for (std::uint32_t i = 0; i < ANCHOR_MAX; ++i)
        {
            deformConstraints.anchData.push_back({});
        }
        return newIdx;
    }
    void XPBDWorld::ReserveDeformConstraint(std::uint32_t n)
    {
        if (n == 0)
            return;
        n += deformConstraints.numConstraints;
        deformConstraints.boneIdx.reserve(n);
        deformConstraints.objIdx.reserve(n);
        deformConstraints.rootIdx.reserve(n);
        deformConstraints.numAnchors.reserve(n);
        const std::uint32_t an = n * ANCHOR_MAX;
        deformConstraints.anchData.reserve(an);
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
                    const auto& ori = physicsBonesOrder[bi];
                    if ((physicsBones.node[ori] == nullptr && !physicsBones.isParticle[ori]) ||
                        physicsBones.objIdx[ori] == UINT32_MAX ||
                        physicsBones.rootIdx[ori] == UINT32_MAX)
                        break;
                    if (currentObjIdx != physicsBones.objIdx[ori])
                    {
                        currentObjIdx = physicsBones.objIdx[ori];
                        physicsBonesGroup.push_back(bi);
                    }
                    if (currentRootIdx != physicsBones.rootIdx[ori])
                    {
                        currentRootIdx = physicsBones.rootIdx[ori];
                        physicsBonesRoots.push_back(bi);
                    }
                    oldToNewBoneIdx[ori] = bi;
                    validCount++;
                }
                if (!physicsBonesGroup.empty())
                    physicsBonesGroup.push_back(validCount);
                if (!physicsBonesRoots.empty())
                    physicsBonesRoots.push_back(validCount);

                {
                    const PhysicsBones tmpPhysicsBones = physicsBones;
                    tbb::parallel_for(
                        tbb::blocked_range<std::uint32_t>(0, validCount, 128),
                        [&](const tbb::blocked_range<std::uint32_t>& r) {
                            for (std::uint32_t i = r.begin(); i != r.end(); ++i)
                            {
                                const std::uint32_t srcIdx = physicsBonesOrder[i];

                                physicsBones.pos[i] = tmpPhysicsBones.pos[srcIdx];
                                physicsBones.prevPos[i] = tmpPhysicsBones.prevPos[srcIdx];
                                physicsBones.predPos[i] = tmpPhysicsBones.predPos[srcIdx];
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
                                physicsBones.limitPositive[i] = tmpPhysicsBones.limitPositive[srcIdx];
                                physicsBones.limitNegative[i] = tmpPhysicsBones.limitNegative[srcIdx];
                                physicsBones.angularLimitPositive[i] = tmpPhysicsBones.angularLimitPositive[srcIdx];
                                physicsBones.angularLimitNegative[i] = tmpPhysicsBones.angularLimitNegative[srcIdx];
                                physicsBones.inertiaPositive[i] = tmpPhysicsBones.inertiaPositive[srcIdx];
                                physicsBones.inertiaNegative[i] = tmpPhysicsBones.inertiaNegative[srcIdx];
                                physicsBones.invAngularInertiaPositive[i] = tmpPhysicsBones.invAngularInertiaPositive[srcIdx];
                                physicsBones.invAngularInertiaNegative[i] = tmpPhysicsBones.invAngularInertiaNegative[srcIdx];
                                physicsBones.inertiaCorrectionPositive[i] = tmpPhysicsBones.inertiaCorrectionPositive[srcIdx];
                                physicsBones.inertiaCorrectionNegative[i] = tmpPhysicsBones.inertiaCorrectionNegative[srcIdx];
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

                                physicsBones.collisionMargin[i] = tmpPhysicsBones.collisionMargin[srcIdx];
                                physicsBones.collisionShrink[i] = tmpPhysicsBones.collisionShrink[srcIdx];
                                physicsBones.collisionFriction[i] = tmpPhysicsBones.collisionFriction[srcIdx];
                                physicsBones.collisionCompliance[i] = tmpPhysicsBones.collisionCompliance[srcIdx];
                                physicsBones.collisionRestitution[i] = tmpPhysicsBones.collisionRestitution[srcIdx];

                                physicsBones.layerGroup[i] = tmpPhysicsBones.layerGroup[srcIdx];
                                physicsBones.collideLayer[i] = tmpPhysicsBones.collideLayer[srcIdx];

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

                                physicsBones.orgWorldScale[i] = tmpPhysicsBones.orgWorldScale[srcIdx];
                                physicsBones.orgLocalPos[i] = tmpPhysicsBones.orgLocalPos[srcIdx];
                                physicsBones.orgLocalRot[i] = tmpPhysicsBones.orgLocalRot[srcIdx];
                            }
                        },
                        tbb::static_partitioner()
                    );

                    physicsBones.pos.resize(validCount);
                    physicsBones.predPos.resize(validCount);
                    physicsBones.posVel.resize(validCount);

                    physicsBones.advancedRotation.resize(validCount);
                    physicsBones.rot.resize(validCount);
                    physicsBones.predRot.resize(validCount);
                    physicsBones.angVel.resize(validCount);

                    physicsBones.dampingPositive.resize(validCount);
                    physicsBones.dampingNegative.resize(validCount);
                    physicsBones.angularDampingPositive.resize(validCount);
                    physicsBones.angularDampingNegative.resize(validCount);
                    physicsBones.limitPositive.resize(validCount);
                    physicsBones.limitNegative.resize(validCount);
                    physicsBones.angularLimitPositive.resize(validCount);
                    physicsBones.angularLimitNegative.resize(validCount);
                    physicsBones.inertiaPositive.resize(validCount);
                    physicsBones.inertiaNegative.resize(validCount);
                    physicsBones.invAngularInertiaPositive.resize(validCount);
                    physicsBones.invAngularInertiaNegative.resize(validCount);
                    physicsBones.inertiaCorrectionPositive.resize(validCount);
                    physicsBones.inertiaCorrectionNegative.resize(validCount);
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

                    physicsBones.collisionMargin.resize(validCount);
                    physicsBones.collisionShrink.resize(validCount);
                    physicsBones.collisionFriction.resize(validCount);
                    physicsBones.collisionCompliance.resize(validCount);
                    physicsBones.collisionRestitution.resize(validCount);

                    physicsBones.layerGroup.resize(validCount);
                    physicsBones.collideLayer.resize(validCount);

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

                    physicsBones.orgWorldScale.resize(validCount);
                    physicsBones.orgLocalPos.resize(validCount);
                    physicsBones.orgLocalRot.resize(validCount);

                    physicsBonesLock = std::make_unique<tbb::spin_mutex[]>(validCount);
                }
                tbb::parallel_invoke(
                    [&] {
                        physicsBones.numBones = validCount;
                        std::iota(physicsBonesOrder.begin(), physicsBonesOrder.end(), 0);
                        physicsBonesOrder.resize(validCount);
                    },
                    [&] {
                        for (std::uint32_t i = 0; i < validCount; ++i)
                        {
                            if (physicsBones.parentBoneIdx[i] != UINT32_MAX)
                            {
                                physicsBones.parentBoneIdx[i] = oldToNewBoneIdx[physicsBones.parentBoneIdx[i]];
                            }
                        }
                    },
                    [&] {
                        for (std::uint32_t i = 0; i < constraints.numConstraints; ++i)
                        {
                            if (constraints.boneIdx[i] != UINT32_MAX)
                                constraints.boneIdx[i] = oldToNewBoneIdx[constraints.boneIdx[i]];

                            for (std::uint32_t a = 0; a < ANCHOR_MAX; ++a)
                            {
                                std::uint32_t ai = static_cast<std::uint32_t>(i) * ANCHOR_MAX + a;
                                if (constraints.anchData[ai].anchIdx != UINT32_MAX)
                                    constraints.anchData[ai].anchIdx = oldToNewBoneIdx[constraints.anchData[ai].anchIdx];
                            }
                        }
                    },
                    [&] {
                        for (std::uint32_t i = 0; i < angularConstraints.numConstraints; ++i)
                        {
                            if (angularConstraints.boneIdx[i] != UINT32_MAX)
                                angularConstraints.boneIdx[i] = oldToNewBoneIdx[angularConstraints.boneIdx[i]];

                            for (std::uint32_t a = 0; a < ANCHOR_MAX; ++a)
                            {
                                std::uint32_t ai = static_cast<std::uint32_t>(i) * ANCHOR_MAX + a;
                                if (angularConstraints.anchData[ai].anchIdx != UINT32_MAX)
                                    angularConstraints.anchData[ai].anchIdx = oldToNewBoneIdx[angularConstraints.anchData[ai].anchIdx];
                            }
                        }
                    },
                    [&] {
                        for (std::uint32_t i = 0; i < deformConstraints.numConstraints; ++i)
                        {
                            if (deformConstraints.boneIdx[i] != UINT32_MAX)
                                deformConstraints.boneIdx[i] = oldToNewBoneIdx[deformConstraints.boneIdx[i]];

                            for (std::uint32_t a = 0; a < ANCHOR_MAX; ++a)
                            {
                                std::uint32_t ai = static_cast<std::uint32_t>(i) * ANCHOR_MAX + a;
                                if (deformConstraints.anchData[ai].anchIdx != UINT32_MAX)
                                    deformConstraints.anchData[ai].anchIdx = oldToNewBoneIdx[deformConstraints.anchData[ai].anchIdx];
                            }
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
                    }
                );
            }
            tbb::parallel_invoke( 
                [&] { // Constraints
                    constraintsOrder.resize(constraints.numConstraints);
                    std::iota(constraintsOrder.begin(), constraintsOrder.end(), 0);
                    std::ranges::sort(constraintsOrder, [&](std::uint32_t a, std::uint32_t b) {
                        if ((constraints.boneIdx[a] == UINT32_MAX) != (constraints.boneIdx[b] == UINT32_MAX))
                            return (constraints.boneIdx[a] != UINT32_MAX) > (constraints.boneIdx[b] != UINT32_MAX);
                        const std::uint32_t objIdxA = constraints.objIdx[a];
                        const std::uint32_t objIdxB = constraints.objIdx[b];
                        if (objIdxA != objIdxB)
                            return objIdxA < objIdxB;
                        if (constraints.rootIdx[a] != constraints.rootIdx[b])
                            return constraints.rootIdx[a] < constraints.rootIdx[b];
                        return constraints.boneIdx[a] < constraints.boneIdx[b];
                    });

                    constraintsGroup.clear();
                    std::uint32_t currentObjIdx = UINT32_MAX;
                    std::uint32_t validCount = 0;
                    for (std::uint32_t coi = 0; coi < constraints.numConstraints; ++coi)
                    {
                        const auto& oi = constraintsOrder[coi];
                        if (constraints.objIdx[oi] == UINT32_MAX ||
                            constraints.rootIdx[oi] == UINT32_MAX)
                            break;
                        if (currentObjIdx != constraints.objIdx[oi])
                        {
                            currentObjIdx = constraints.objIdx[oi];
                            constraintsGroup.push_back(coi);
                        }
                        validCount++;
                    }
                    if (!constraintsGroup.empty())
                        constraintsGroup.push_back(validCount);

                    {
                        const Constraints tmpConstraints = constraints;
                        tbb::parallel_for(
                            tbb::blocked_range<std::uint32_t>(0, validCount, 128),
                            [&](const tbb::blocked_range<std::uint32_t>& r) {
                                for (std::uint32_t i = r.begin(); i != r.end(); ++i)
                                {
                                    const std::uint32_t srcIdx = constraintsOrder[i];

                                    constraints.boneIdx[i] = tmpConstraints.boneIdx[srcIdx];
                                    constraints.objIdx[i] = tmpConstraints.objIdx[srcIdx];
                                    constraints.rootIdx[i] = tmpConstraints.rootIdx[srcIdx];
                                    constraints.colorGraph[i] = tmpConstraints.colorGraph[srcIdx];
                                    constraints.numAnchors[i] = tmpConstraints.numAnchors[srcIdx];

                                    // anchor
                                    {
                                        const std::uint32_t dstStrideBase = i * ANCHOR_MAX;
                                        const std::uint32_t srcStrideBase = srcIdx * ANCHOR_MAX;
                                        for (std::uint32_t s = 0; s < ANCHOR_MAX; ++s)
                                        {
                                            const std::uint32_t dstA = dstStrideBase + s;
                                            const std::uint32_t srcA = srcStrideBase + s;
                                            constraints.anchData[dstA] = tmpConstraints.anchData[srcA];
                                        }
                                    }
                                }
                            },
                            tbb::static_partitioner());
                    }
                    constraints.boneIdx.resize(validCount);
                    constraints.objIdx.resize(validCount);
                    constraints.rootIdx.resize(validCount);
                    constraints.colorGraph.resize(validCount);
                    constraints.numAnchors.resize(validCount);

                    const std::uint32_t validStrideCount = validCount * ANCHOR_MAX;
                    constraints.anchData.resize(validStrideCount);

                    constraints.numConstraints = validCount;
                    std::iota(constraintsOrder.begin(), constraintsOrder.end(), 0);
                    constraintsOrder.resize(validCount);
                },
                [&] { // AngularConstraints
                    angularConstraintsOrder.resize(angularConstraints.numConstraints);
                    std::iota(angularConstraintsOrder.begin(), angularConstraintsOrder.end(), 0);
                    std::ranges::sort(angularConstraintsOrder, [&](std::uint32_t a, std::uint32_t b) {
                        if ((angularConstraints.boneIdx[a] == UINT32_MAX) != (angularConstraints.boneIdx[b] == UINT32_MAX))
                            return (angularConstraints.boneIdx[a] != UINT32_MAX) > (angularConstraints.boneIdx[b] != UINT32_MAX);
                        const std::uint32_t objIdxA = angularConstraints.objIdx[a];
                        const std::uint32_t objIdxB = angularConstraints.objIdx[b];
                        if (objIdxA != objIdxB)
                            return objIdxA < objIdxB;
                        if (angularConstraints.rootIdx[a] != angularConstraints.rootIdx[b])
                            return angularConstraints.rootIdx[a] < angularConstraints.rootIdx[b];
                        return angularConstraints.boneIdx[a] < angularConstraints.boneIdx[b];
                    });

                    angularConstraintsGroup.clear();
                    std::uint32_t currentObjIdx = UINT32_MAX;
                    std::uint32_t validCount = 0;
                    for (std::uint32_t coi = 0; coi < angularConstraints.numConstraints; ++coi)
                    {
                        const auto& oi = angularConstraintsOrder[coi];
                        if (angularConstraints.objIdx[oi] == UINT32_MAX ||
                            angularConstraints.rootIdx[oi] == UINT32_MAX)
                            break;
                        if (currentObjIdx != angularConstraints.objIdx[oi])
                        {
                            currentObjIdx = angularConstraints.objIdx[oi];
                            angularConstraintsGroup.push_back(coi);
                        }
                        validCount++;
                    }
                    if (!angularConstraintsGroup.empty())
                        angularConstraintsGroup.push_back(validCount);

                    {
                        const AngularConstraints tmpAngCons = angularConstraints;
                        tbb::parallel_for(
                            tbb::blocked_range<std::uint32_t>(0, validCount, 128),
                            [&](const tbb::blocked_range<std::uint32_t>& r) {
                                for (std::uint32_t i = r.begin(); i != r.end(); ++i)
                                {
                                    const std::uint32_t srcIdx = angularConstraintsOrder[i];

                                    angularConstraints.boneIdx[i] = tmpAngCons.boneIdx[srcIdx];
                                    angularConstraints.objIdx[i] = tmpAngCons.objIdx[srcIdx];
                                    angularConstraints.rootIdx[i] = tmpAngCons.rootIdx[srcIdx];
                                    angularConstraints.colorGraph[i] = tmpAngCons.colorGraph[srcIdx];
                                    angularConstraints.numAnchors[i] = tmpAngCons.numAnchors[srcIdx];

                                    const std::uint32_t dstStrideBase = i * ANCHOR_MAX;
                                    const std::uint32_t srcStrideBase = srcIdx * ANCHOR_MAX;
                                    for (std::uint32_t s = 0; s < ANCHOR_MAX; ++s)
                                    {
                                        const std::uint32_t dstA = dstStrideBase + s;
                                        const std::uint32_t srcA = srcStrideBase + s;
                                        angularConstraints.anchData[dstA] = tmpAngCons.anchData[srcA];
                                    }
                                }
                            },
                            tbb::static_partitioner());

                        angularConstraints.boneIdx.resize(validCount);
                        angularConstraints.objIdx.resize(validCount);
                        angularConstraints.rootIdx.resize(validCount);
                        angularConstraints.colorGraph.resize(validCount);
                        angularConstraints.numAnchors.resize(validCount);

                        const std::uint32_t validStrideCount = validCount * ANCHOR_MAX;
                        angularConstraints.anchData.resize(validStrideCount);
                    }

                    angularConstraints.numConstraints = validCount;
                    std::iota(angularConstraintsOrder.begin(), angularConstraintsOrder.end(), 0);
                    angularConstraintsOrder.resize(validCount);
                },
                [&] { // DeformConstraints
                    deformConstraintsOrder.resize(deformConstraints.numConstraints);
                    std::iota(deformConstraintsOrder.begin(), deformConstraintsOrder.end(), 0);
                    std::ranges::sort(deformConstraintsOrder, [&](std::uint32_t a, std::uint32_t b) {
                        if ((deformConstraints.boneIdx[a] == UINT32_MAX) != (deformConstraints.boneIdx[b] == UINT32_MAX))
                            return (deformConstraints.boneIdx[a] != UINT32_MAX) > (deformConstraints.boneIdx[b] != UINT32_MAX);
                        const std::uint32_t objIdxA = deformConstraints.objIdx[a];
                        const std::uint32_t objIdxB = deformConstraints.objIdx[b];
                        if (objIdxA != objIdxB)
                            return objIdxA < objIdxB;
                        if (deformConstraints.rootIdx[a] != deformConstraints.rootIdx[b])
                            return deformConstraints.rootIdx[a] < deformConstraints.rootIdx[b];
                        return deformConstraints.boneIdx[a] < deformConstraints.boneIdx[b];
                    });

                    deformConstraintsGroup.clear();
                    std::uint32_t currentObjIdx = UINT32_MAX;
                    std::uint32_t validCount = 0;
                    for (std::uint32_t coi = 0; coi < deformConstraints.numConstraints; ++coi)
                    {
                        const auto& oi = deformConstraintsOrder[coi];
                        if (deformConstraints.objIdx[oi] == UINT32_MAX ||
                            deformConstraints.rootIdx[oi] == UINT32_MAX)
                            break;
                        if (currentObjIdx != deformConstraints.objIdx[oi])
                        {
                            currentObjIdx = deformConstraints.objIdx[oi];
                            deformConstraintsGroup.push_back(coi);
                        }
                        validCount++;
                    }
                    if (!deformConstraintsGroup.empty())
                        deformConstraintsGroup.push_back(validCount);

                    {
                        const DeformConstraints tmpDeformCons = deformConstraints;
                        tbb::parallel_for(
                            tbb::blocked_range<std::uint32_t>(0, validCount, 128),
                            [&](const tbb::blocked_range<std::uint32_t>& r) {
                                for (std::uint32_t i = r.begin(); i != r.end(); ++i)
                                {
                                    const std::uint32_t srcIdx = deformConstraintsOrder[i];

                                    deformConstraints.boneIdx[i] = tmpDeformCons.boneIdx[srcIdx];
                                    deformConstraints.objIdx[i] = tmpDeformCons.objIdx[srcIdx];
                                    deformConstraints.rootIdx[i] = tmpDeformCons.rootIdx[srcIdx];
                                    deformConstraints.numAnchors[i] = tmpDeformCons.numAnchors[srcIdx];

                                    const std::uint32_t dstStrideBase = i * ANCHOR_MAX;
                                    const std::uint32_t srcStrideBase = srcIdx * ANCHOR_MAX;
                                    for (std::uint32_t s = 0; s < ANCHOR_MAX; ++s)
                                    {
                                        const std::uint32_t dstA = dstStrideBase + s;
                                        const std::uint32_t srcA = srcStrideBase + s;
                                        deformConstraints.anchData[dstA] = tmpDeformCons.anchData[srcA];
                                    }
                                }
                            },
                            tbb::static_partitioner());

                        deformConstraints.boneIdx.resize(validCount);
                        deformConstraints.objIdx.resize(validCount);
                        deformConstraints.rootIdx.resize(validCount);
                        deformConstraints.numAnchors.resize(validCount);

                        const std::uint32_t validStrideCount = validCount * ANCHOR_MAX;
                        deformConstraints.anchData.resize(validStrideCount);
                    }

                    deformConstraints.numConstraints = validCount;
                    std::iota(deformConstraintsOrder.begin(), deformConstraintsOrder.end(), 0);
                    deformConstraintsOrder.resize(validCount);
                },
                [&] { // ConvexHullCollider
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
                        const Colliders tmpColliders = colliders;
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
                    std::iota(collidersOrder.begin(), collidersOrder.end(), 0);
                    collidersOrder.resize(validCount);

                    
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
                }
            );
            // logger::info("Reorder maps done");
            isNeedColorGraphUpdate = true;
        });
    }

    void XPBDWorld::BuildConstraintColorGraph()
    {
        const std::uint32_t numBones = physicsBones.numBones;
        threadPool->Execute([&] {
            tbb::parallel_invoke(
                [&] { // constraint
                    constraints.colorGraph.assign(constraints.numConstraints, 0);
                    std::vector<std::vector<std::uint32_t>> boneToConsIdx(numBones);
                    for (std::uint32_t ci = 0; ci < constraints.numConstraints; ++ci)
                    {
                        const std::uint32_t bi = constraints.boneIdx[ci];
                        if (bi < numBones)
                            boneToConsIdx[bi].push_back(ci);

                        const std::uint32_t aiBase = ci * ANCHOR_MAX;
                        for (std::uint32_t a = 0; a < constraints.numAnchors[ci]; ++a)
                        {
                            std::uint32_t anchIdx = constraints.anchData[aiBase + a].anchIdx;
                            if (anchIdx < numBones)
                            {
                                boneToConsIdx[anchIdx].push_back(ci);
                            }
                        }
                    }

                    for (std::uint32_t ci = 0; ci < constraints.numConstraints; ++ci)
                    {
                        std::vector<bool> usedColors;
                        auto markUsed = [&](const std::uint32_t bone) {
                            if (bone >= numBones)
                                return;
                            for (std::uint32_t adjCons : boneToConsIdx[bone])
                            {
                                if (adjCons < ci)
                                {
                                    std::uint32_t color = constraints.colorGraph[adjCons];
                                    if (color >= usedColors.size())
                                    {
                                        usedColors.resize(color + 1, false);
                                    }
                                    usedColors[color] = true;
                                }
                            }
                        };

                        markUsed(constraints.boneIdx[ci]);
                        const std::uint32_t aiBase = ci * ANCHOR_MAX;
                        for (std::uint32_t a = 0; a < constraints.numAnchors[ci]; ++a)
                        {
                            markUsed(constraints.anchData[aiBase + a].anchIdx);
                        }

                        std::uint32_t color = 0;
                        while (color < usedColors.size() && usedColors[color])
                        {
                            color++;
                        }
                        constraints.colorGraph[ci] = color;
                    }

                    // sort
                    constraintsOrder.resize(constraints.numConstraints);
                    std::iota(constraintsOrder.begin(), constraintsOrder.end(), 0);
                    std::ranges::sort(constraintsOrder, [&](std::uint32_t a, std::uint32_t b) {
                        if (constraints.objIdx[a] != constraints.objIdx[b])
                            return constraints.objIdx[a] < constraints.objIdx[b];
                        if (constraints.colorGraph[a] != constraints.colorGraph[b])
                            return constraints.colorGraph[a] < constraints.colorGraph[b];
                        return a < b;
                    });
                    const Constraints tmpConstraints = constraints;
                    tbb::parallel_for(
                        tbb::blocked_range<std::uint32_t>(0, constraints.numConstraints, 128),
                        [&](const tbb::blocked_range<std::uint32_t>& r) {
                            for (std::uint32_t i = r.begin(); i != r.end(); ++i)
                            {
                                const std::uint32_t srcIdx = constraintsOrder[i];

                                constraints.boneIdx[i] = tmpConstraints.boneIdx[srcIdx];
                                constraints.objIdx[i] = tmpConstraints.objIdx[srcIdx];
                                constraints.rootIdx[i] = tmpConstraints.rootIdx[srcIdx];
                                constraints.colorGraph[i] = tmpConstraints.colorGraph[srcIdx];
                                constraints.numAnchors[i] = tmpConstraints.numAnchors[srcIdx];

                                const std::uint32_t dstStrideBase = i * ANCHOR_MAX;
                                const std::uint32_t srcStrideBase = srcIdx * ANCHOR_MAX;
                                for (std::uint32_t s = 0; s < ANCHOR_MAX; ++s)
                                {
                                    const std::uint32_t dstA = dstStrideBase + s;
                                    const std::uint32_t srcA = srcStrideBase + s;
                                    constraints.anchData[dstA] = tmpConstraints.anchData[srcA];
                                }
                            }
                        },
                        tbb::static_partitioner()
                    );
                    std::iota(constraintsOrder.begin(), constraintsOrder.end(), 0);

                    // Rebuild color graph group
                    constraintsGroup.clear();
                    constraintsColorGroup.clear();

                    std::uint32_t currentObjIdx = UINT32_MAX;
                    std::uint32_t currentColor = UINT32_MAX;

                    for (std::uint32_t i = 0; i < constraints.numConstraints; ++i)
                    {
                        if (constraints.objIdx[i] == UINT32_MAX)
                            break;

                        if (currentObjIdx != constraints.objIdx[i])
                        {
                            currentObjIdx = constraints.objIdx[i];
                            constraintsGroup.push_back(i);

                            currentColor = constraints.colorGraph[i];
                            constraintsColorGroup.push_back(i);
                        }
                        else if (currentColor != constraints.colorGraph[i])
                        {
                            currentColor = constraints.colorGraph[i];
                            constraintsColorGroup.push_back(i);
                        }
                    }
                    if (constraints.numConstraints > 0)
                    {
                        constraintsGroup.push_back(constraints.numConstraints);
                        constraintsColorGroup.push_back(constraints.numConstraints);
                    }
                },
                [&] { // angular constraint
                    angularConstraints.colorGraph.assign(angularConstraints.numConstraints, 0);
                    std::vector<std::vector<std::uint32_t>> boneToAngConsIdx(numBones);
                    for (std::uint32_t ci = 0; ci < angularConstraints.numConstraints; ++ci)
                    {
                        std::uint32_t bi = angularConstraints.boneIdx[ci];
                        if (bi < numBones)
                            boneToAngConsIdx[bi].push_back(ci);

                        const std::uint32_t aiBase = ci * ANCHOR_MAX;
                        for (std::uint32_t a = 0; a < angularConstraints.numAnchors[ci]; ++a)
                        {
                            const std::uint32_t abi = angularConstraints.anchData[aiBase + a].anchIdx;
                            if (abi < numBones)
                            {
                                boneToAngConsIdx[abi].push_back(ci);
                            }
                        }
                    }

                    for (std::uint32_t ci = 0; ci < angularConstraints.numConstraints; ++ci)
                    {
                        std::vector<bool> usedColors;
                        auto markUsed = [&](std::uint32_t bone) {
                            if (bone >= numBones)
                                return;
                            for (std::uint32_t adjConstraint : boneToAngConsIdx[bone])
                            {
                                if (adjConstraint < ci)
                                {
                                    std::uint32_t color = angularConstraints.colorGraph[adjConstraint];
                                    if (color >= usedColors.size())
                                    {
                                        usedColors.resize(color + 1, false);
                                    }
                                    usedColors[color] = true;
                                }
                            }
                        };

                        markUsed(angularConstraints.boneIdx[ci]);
                        const std::uint32_t aiBase = ci * ANCHOR_MAX;
                        for (std::uint32_t a = 0; a < angularConstraints.numAnchors[ci]; ++a)
                        {
                            markUsed(angularConstraints.anchData[aiBase + a].anchIdx);
                        }

                        std::uint32_t color = 0;
                        while (color < usedColors.size() && usedColors[color])
                        {
                            color++;
                        }
                        angularConstraints.colorGraph[ci] = color;
                    }

                    // sort
                    angularConstraintsOrder.resize(angularConstraints.numConstraints);
                    std::iota(angularConstraintsOrder.begin(), angularConstraintsOrder.end(), 0);
                    std::ranges::sort(angularConstraintsOrder, [&](std::uint32_t a, std::uint32_t b) {
                        if (angularConstraints.objIdx[a] != angularConstraints.objIdx[b])
                            return angularConstraints.objIdx[a] < angularConstraints.objIdx[b];
                        if (angularConstraints.colorGraph[a] != angularConstraints.colorGraph[b])
                            return angularConstraints.colorGraph[a] < angularConstraints.colorGraph[b];
                        return a < b;
                    });
                    const AngularConstraints tmpAngCons = angularConstraints;
                    tbb::parallel_for(
                        tbb::blocked_range<std::uint32_t>(0, angularConstraints.numConstraints, 128),
                        [&](const tbb::blocked_range<std::uint32_t>& r) {
                            for (std::uint32_t i = r.begin(); i != r.end(); ++i)
                            {
                                const std::uint32_t srcIdx = angularConstraintsOrder[i];

                                angularConstraints.boneIdx[i] = tmpAngCons.boneIdx[srcIdx];
                                angularConstraints.objIdx[i] = tmpAngCons.objIdx[srcIdx];
                                angularConstraints.rootIdx[i] = tmpAngCons.rootIdx[srcIdx];
                                angularConstraints.colorGraph[i] = tmpAngCons.colorGraph[srcIdx];
                                angularConstraints.numAnchors[i] = tmpAngCons.numAnchors[srcIdx];

                                const std::uint32_t dstStrideBase = i * ANCHOR_MAX;
                                const std::uint32_t srcStrideBase = srcIdx * ANCHOR_MAX;
                                for (std::uint32_t s = 0; s < ANCHOR_MAX; ++s)
                                {
                                    const std::uint32_t dstA = dstStrideBase + s;
                                    const std::uint32_t srcA = srcStrideBase + s;
                                    angularConstraints.anchData[dstA] = tmpAngCons.anchData[srcA];
                                }
                            }
                        },
                        tbb::static_partitioner()
                    );
                    std::iota(angularConstraintsOrder.begin(), angularConstraintsOrder.end(), 0);

                    // Rebuild color graph group
                    angularConstraintsGroup.clear();
                    angularConstraintsColorGroup.clear();

                    std::uint32_t currentObjIdx = UINT32_MAX;
                    std::uint32_t currentColor = UINT32_MAX;

                    for (std::uint32_t i = 0; i < angularConstraints.numConstraints; ++i)
                    {
                        if (angularConstraints.objIdx[i] == UINT32_MAX)
                            break;

                        if (currentObjIdx != angularConstraints.objIdx[i])
                        {
                            currentObjIdx = angularConstraints.objIdx[i];
                            angularConstraintsGroup.push_back(i);

                            currentColor = angularConstraints.colorGraph[i];
                            angularConstraintsColorGroup.push_back(i);
                        }
                        else if (currentColor != angularConstraints.colorGraph[i])
                        {
                            currentColor = angularConstraints.colorGraph[i];
                            angularConstraintsColorGroup.push_back(i);
                        }
                    }
                    if (angularConstraints.numConstraints > 0)
                    {
                        angularConstraintsGroup.push_back(angularConstraints.numConstraints);
                        angularConstraintsColorGroup.push_back(angularConstraints.numConstraints);
                    }
                }
            );
            convexHullCache.clear();
            manifoldCache.clear();
            groundCache.clear();
            objectHashesSmall.clear();
            objectHashesLarge.clear();
        });
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
        physicsBones.limitPositive[bi] = ToVector(bone.limitPositive);
        physicsBones.limitNegative[bi] = ToVector(bone.limitNegative);
        physicsBones.angularLimitPositive[bi] = ToVector(bone.angularLimitPositive);
        physicsBones.angularLimitNegative[bi] = DirectX::XMVectorNegate(ToVector(bone.angularLimitNegative));

        ClampZeroToInfinity(physicsBones.limitPositive[bi]);
        ClampZeroToInfinity(physicsBones.limitNegative[bi]);
        ClampZeroToInfinityRot(physicsBones.angularLimitPositive[bi], false);
        ClampZeroToInfinityRot(physicsBones.angularLimitNegative[bi], true);

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
        physicsBones.inertiaCorrectionPositive[bi] = ToVector(bone.inertiaCorrectionPositive);
        physicsBones.inertiaCorrectionNegative[bi] = ToVector(bone.inertiaCorrectionNegative);
        physicsBones.angularBlendFactor[bi] = bone.angularBlendFactor;
        physicsBones.gravity[bi] = DirectX::XMVectorMultiply(GetSkyrimGravity(bone.gravity), vPhysicsScale);
        physicsBones.windFactor[bi] = ToVector(bone.windFactor);
        physicsBones.physicsScale[bi] = physicsScale;

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

    void XPBDWorld::SetConstraint(const std::uint32_t ai, const PhysicsInput::Constraint::AnchorData& anchData, const float physicsScale)
    {
        constraints.anchData[ai].complianceSquish = anchData.complianceSquish;
        constraints.anchData[ai].complianceStretch = anchData.complianceStretch;
        constraints.anchData[ai].squishMargin = anchData.squishMargin;
        constraints.anchData[ai].stretchMargin = anchData.stretchMargin;
        constraints.anchData[ai].squishDamping = anchData.squishDamping;
        constraints.anchData[ai].stretchDamping = anchData.stretchDamping;
    }

    void XPBDWorld::SetAngularConstraint(const std::uint32_t ai, const PhysicsInput::AngularConstraint::AnchorData& anchData, const float physicsScale)
    {
        angularConstraints.anchData[ai].compliancePositive = ToVector(anchData.compliancePositive);
        angularConstraints.anchData[ai].complianceNegative = ToVector(anchData.complianceNegative);
        angularConstraints.anchData[ai].marginPositive = ToVector(anchData.marginPositive);
        angularConstraints.anchData[ai].marginNegative = ToVector(anchData.marginNegative);
        angularConstraints.anchData[ai].dampingPositive = ToVector(anchData.dampingPositive);
        angularConstraints.anchData[ai].dampingNegative = ToVector(anchData.dampingNegative);
    }

    void XPBDWorld::SetDeformConstraint(const std::uint32_t ai, const PhysicsInput::DeformConstraint::AnchorData& anchData, const float physicsScale)
    {
        deformConstraints.anchData[ai].squishWeight = ToVector(anchData.squishWeight);
        deformConstraints.anchData[ai].stretchWeight = ToVector(anchData.stretchWeight);
        deformConstraints.anchData[ai].bulgeWeight = ToVector(anchData.bulgeWeight);
    }

    void XPBDWorld::UpdateChildTreeData(RE::NiNode* node) const
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
            RE::NiUpdateData ctx = {0.0f, RE::NiUpdateData ::Flag::kDirty};
            childNode->UpdateWorldData(&ctx);
            UpdateChildTreeData(childNode);
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
