#include "MXPBD/PhysicsWorld.h"

#define TIMELOG_START                                               \
    static double nsSum = 0.0;                                      \
    static std::uint32_t timeCount = 0;                             \
    const auto timeStart = std::chrono::high_resolution_clock::now();   

#define TIMELOG_END                                                                                                                 \
    const auto timeEnd = std::chrono::high_resolution_clock::now();                                                                 \
    nsSum += std::chrono::duration_cast<std::chrono::nanoseconds>(timeEnd - timeStart).count();                                     \
    timeCount++;                                                                                                                    \
    if (timeCount >= 1000)                                                                                                          \
    {                                                                                                                               \
        const auto ms = (nsSum * 0.001f) * ns2ms;                                                                                \
        logger::debug("{} time: {:.3f}ms ({} bones / {} constrants / {} angularConstrants / {} colliders)", __func__, ms,           \
                     physicsBones.numBones, constraints.numConstraints, angularConstraints.numConstraints, colliders.numColliders); \
        nsSum = 0;                                                                                                                  \
        timeCount = 0;                                                                                                              \
    }

namespace MXPBD
{
    XPBDWorld::XPBDWorld()
    {
        std::uint32_t pCoreCount = 0;
        std::uint64_t pCoreMask = 0;
        GetPCore(pCoreCount, pCoreMask);
        threadPool = std::make_unique<TBB_ThreadPool>(pCoreCount, pCoreMask);
    };

    void XPBDWorld::AddPhysics(RE::TESObjectREFR* object, RE::NiNode* rootNode, const RootType rootType, const PhysicsInput& input)
    {
        if (!object || !rootNode)
            return;
        if (input.bones.empty() && input.constraints.empty() && input.convexHullColliders.colliders.empty())
            return;
        if (rootType == XPBDWorld::RootType::kNone)
            return;

        const ObjectDatas::Root newRoot = {.type = rootType, .bipedSlot = input.bipedSlot};

        logger::info("{:x} : adding physics for {} bones, {} constraints, {} colliders", object->formID, input.bones.size(), input.constraints.size(), input.convexHullColliders.colliders.size());

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
            ReserveAngularConstraint(input.constraints.size());

            // add bone data
            Mus::nif::VisitObjects(rootNode, [&](RE::NiAVObject* obj, std::uint32_t depth) {
                if (!obj || obj->name.empty())
                    return true;

                std::uint32_t bi = 0;
                if (auto boneIdxIt = boneNameToIdx.find(obj->name.c_str()); boneIdxIt != boneNameToIdx.end())
                    bi = boneIdxIt->second;
                else
                {
                    auto found = input.bones.find(obj->name.c_str());
                    if (found == input.bones.end())
                        return true;

                    logger::debug("{:x} : add physics bone {}", object->formID, found->first);
                    bi = AllocateBone();
                    const Vector offset = DirectX::XMVector3Rotate(DirectX::XMVectorScale(ToVector(found->second.offset), obj->world.scale), ToQuaternion(obj->world.rotate));
                    physicsBones.pos[bi] = DirectX::XMVectorAdd(ToVector(obj->world.translate), offset);
                    physicsBones.pred[bi] = physicsBones.pos[bi];
                    physicsBones.vel[bi] = vZero;

                    physicsBones.rot[bi] = ToQuaternion(obj->world.rotate);
                    physicsBones.predRot[bi] = physicsBones.rot[bi];
                    physicsBones.angVel[bi] = vZero;

                    physicsBones.damping[bi] = found->second.damping;
                    physicsBones.inertiaScale[bi] = found->second.inertiaScale;
                    physicsBones.restitution[bi] = found->second.restitution;
                    physicsBones.rotationBlendFactor[bi] = found->second.rotationBlendFactor;
                    physicsBones.gravity[bi] = ToVector(GetSkyrimGravity(found->second.gravity));
                    physicsBones.offset[bi] = ToVector(found->second.offset);
                    physicsBones.invMass[bi] = (found->second.mass > FloatPrecision) ? reciprocal(found->second.mass) : 0.0f;
                    physicsBones.windFactor[bi] = found->second.windFactor;

                    physicsBones.linearRotTorque[bi] = NiPoin3x3ToXMMATRIX(found->second.linearRotTorque);

                    physicsBones.restPoseLimit[bi] = found->second.restPoseLimit;
                    physicsBones.restPoseCompliance[bi] = found->second.restPoseCompliance;

                    physicsBones.restPoseAngularLimit[bi] = found->second.restPoseAngularLimit;
                    physicsBones.restPoseAngularCompliance[bi] = found->second.restPoseAngularCompliance;

                    if (found->second.mass > FloatPrecision && found->second.inertiaScale > FloatPrecision)
                        physicsBones.invInertia[bi] = reciprocal(found->second.mass * found->second.inertiaScale);
                    else
                        physicsBones.invInertia[bi] = 0.0f;

                    physicsBones.collisionMargin[bi] = (found->second.collisionMargin < COL_MARGIN_MIN ? COL_MARGIN_MIN : found->second.collisionMargin);
                    physicsBones.collisionShrink[bi] = (found->second.collisionMargin < COL_MARGIN_MIN ? COL_MARGIN_MIN - found->second.collisionMargin : 0.0f);
                    physicsBones.collisionFriction[bi] = found->second.collisionFriction;
                    physicsBones.collisionRotationBias[bi] = found->second.collisionRotationBias;
                    physicsBones.layerGroup[bi] = found->second.collisionLayerGroup;
                    physicsBones.collideLayer[bi] = found->second.collisionCollideLayer;

                    physicsBones.node[bi] = RE::NiPointer(obj);
                    physicsBones.isParticle[bi] = 0;
                    physicsBones.parentBoneIdx[bi] = UINT32_MAX;
                    if (obj->parent && !obj->parent->name.empty())
                    {
                        auto pit = boneNameToIdx.find(obj->parent->name.c_str());
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

                    physicsBones.orgWorldScale[bi] = obj->world.scale;
                    physicsBones.orgLocalPos[bi] = ToVector(obj->local.translate);
                    physicsBones.orgLocalRot[bi] = ToQuaternion(obj->local.rotate);

                    boneNameToIdx[found->first] = bi;
                }

                if (auto pptn = particleParentToName.find(obj->name.c_str()); pptn != particleParentToName.end())
                {
                    auto func = [this, &object, &input, objIdx, rootIdx, depth, &boneNameToIdx, &particleParentToName](auto&& func, const std::string& parentName, const std::vector<std::string>& pptnList, const std::uint32_t parentIdx, std::uint32_t particleDepth) -> void {
                        for (const auto& particleName : pptnList)
                        {
                            const auto pit = input.bones.find(particleName);
                            if (pit == input.bones.end())
                                continue;
                            if (boneNameToIdx.find(particleName) != boneNameToIdx.end())
                                continue;

                            logger::debug("{:x} : add physics particle({}) bone {} for {}", object->formID, particleDepth, particleName, parentName);
                            const std::uint32_t particleBi = AllocateBone();
                            const Vector offset = DirectX::XMVector3Rotate(DirectX::XMVectorScale(ToVector(pit->second.offset), physicsBones.orgWorldScale[parentIdx]), physicsBones.rot[parentIdx]);
                            physicsBones.pos[particleBi] = DirectX::XMVectorAdd(physicsBones.pos[parentIdx], offset);
                            physicsBones.pred[particleBi] = physicsBones.pos[particleBi];
                            physicsBones.vel[particleBi] = vZero;

                            physicsBones.rot[particleBi] = physicsBones.rot[parentIdx];
                            physicsBones.predRot[particleBi] = physicsBones.rot[particleBi];
                            physicsBones.angVel[particleBi] = vZero;

                            physicsBones.damping[particleBi] = pit->second.damping;
                            physicsBones.inertiaScale[particleBi] = pit->second.inertiaScale;
                            physicsBones.restitution[particleBi] = pit->second.restitution;
                            physicsBones.rotationBlendFactor[particleBi] = pit->second.rotationBlendFactor;
                            physicsBones.gravity[particleBi] = ToVector(GetSkyrimGravity(pit->second.gravity));
                            physicsBones.offset[particleBi] = ToVector(pit->second.offset);
                            physicsBones.invMass[particleBi] = (pit->second.mass > FloatPrecision) ? reciprocal(pit->second.mass) : 0.0f;
                            physicsBones.windFactor[particleBi] = pit->second.windFactor;

                            if (pit->second.mass > FloatPrecision && pit->second.inertiaScale > FloatPrecision)
                                physicsBones.invInertia[particleBi] = reciprocal(pit->second.mass * pit->second.inertiaScale);
                            else
                                physicsBones.invInertia[particleBi] = 0.0f;
                            
                            physicsBones.linearRotTorque[particleBi] = NiPoin3x3ToXMMATRIX(pit->second.linearRotTorque);

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
                            physicsBones.orgLocalPos[particleBi] = ToVector(pit->second.offset);
                            physicsBones.orgLocalRot[particleBi] = vZero;

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

                std::uint32_t validAnchorCount = 0;
                const std::uint32_t aiBase = static_cast<std::uint32_t>(ci) * ANCHOR_MAX;
                for (std::uint32_t a = 0; a < constraint.second.anchorBoneNames.size(); ++a)
                {
                    const std::string& anchorName = constraint.second.anchorBoneNames[a];
                    if (boneNameToIdx.find(anchorName) == boneNameToIdx.end())
                    {
                        logger::error("{:x} : Unable to get anchor node {} for {}", object->formID, anchorName, constraint.first);
                        continue;
                    }
                    if (validAnchorCount >= ANCHOR_MAX)
                    {
                        logger::error("{:x} : Unable to add anchor node {} for {} due to reached maximum anchor count", object->formID, anchorName, constraint.first);
                        continue;
                    }
                    logger::debug("{:x} : add constraint {}({}) on {}", object->formID, constraint.first, validAnchorCount, anchorName);

                    const std::uint32_t ai = aiBase + validAnchorCount;
                    const std::uint32_t anchBi = boneNameToIdx[anchorName];
                    constraints.anchData[ai].anchIdx = anchBi;
                    const Vector tPos = physicsBones.pos[bit->second];
                    const Vector aPos = physicsBones.pos[anchBi];
                    constraints.anchData[ai].restLen = DirectX::XMVectorGetX(DirectX::XMVector3Length(DirectX::XMVectorSubtract(tPos, aPos)));
                    constraints.anchData[ai].complianceSquish = constraint.second.complianceSquish[a];
                    constraints.anchData[ai].complianceStretch = constraint.second.complianceStretch[a];
                    constraints.anchData[ai].squishLimit = constraint.second.squishLimit[a];
                    constraints.anchData[ai].stretchLimit = constraint.second.stretchLimit[a];
                    constraints.anchData[ai].angularLimit = constraint.second.angularLimit[a];
                    Vector dirWorld = DirectX::XMVectorSubtract(tPos, aPos);
                    if (DirectX::XMVector3Less(vFloatPrecision, DirectX::XMVector3LengthSq(dirWorld)))
                    {
                        dirWorld = DirectX::XMVector3Normalize(dirWorld);
                        const Quaternion anchRotInv = DirectX::XMQuaternionConjugate(physicsBones.rot[anchBi]);
                        constraints.anchData[ai].restDirLocal = DirectX::XMVector3Rotate(dirWorld, anchRotInv);
                    }
                    else
                    {
                        constraints.anchData[ai].restDirLocal = DirectX::XMVectorSet(0.0f, -1.0f, 0.0f, 0.0f);
                    }
                    constraints.anchData[ai].squishDamping = constraint.second.squishDamping[a];
                    constraints.anchData[ai].stretchDamping = constraint.second.stretchDamping[a];
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

                std::uint32_t validAnchorCount = 0;
                const std::uint32_t aciBase = static_cast<std::uint32_t>(aci) * ANCHOR_MAX;
                for (std::uint32_t a = 0; a < angularConstraint.second.anchorBoneNames.size(); ++a)
                {
                    auto abit = boneNameToIdx.find(angularConstraint.second.anchorBoneNames[a]);
                    if (abit == boneNameToIdx.end())
                        continue;
                    if (validAnchorCount >= ANCHOR_MAX)
                        break;
                    logger::debug("{:x} : add angular constraint {}({}) on {}", object->formID, angularConstraint.first, validAnchorCount, angularConstraint.second.anchorBoneNames[a]);

                    const std::uint32_t aai = aciBase + validAnchorCount;
                    angularConstraints.anchData[aai].anchIdx = abit->second;

                    physicsBones.advancedRotation[abit->second] = 1;

                    const Quaternion childRot = physicsBones.rot[bit->second];
                    const Quaternion anchorRot = physicsBones.rot[abit->second];
                    const Quaternion anchorRotInv = DirectX::XMQuaternionInverse(anchorRot);
                    const Quaternion restRot = DirectX::XMQuaternionNormalize(DirectX::XMQuaternionMultiply(childRot, anchorRotInv));
                    angularConstraints.anchData[aai].restRot = restRot;
                    angularConstraints.anchData[aai].compliance = angularConstraint.second.compliance[a];
                    angularConstraints.anchData[aai].limit = angularConstraint.second.limit[a];
                    angularConstraints.anchData[aai].damping = angularConstraint.second.damping[a];
                    validAnchorCount++;
                }
                angularConstraints.numAnchors[aci] = validAnchorCount;
            }
        }
        else if (rootType == XPBDWorld::RootType::kCollider)
        {
            if (!input.convexHullColliders.colliders.empty())
            {
                ReserveCollider(input.convexHullColliders.colliders.size());
                RemoveCollider(object, newRoot);

                if (colliders.numColliders == 0)
                    convexHullCache.reserve(input.convexHullColliders.colliders.size() * 8);

                std::uint32_t addedColCount = 0;
                for (auto& collider : input.convexHullColliders.colliders)
                {
                    const auto& boneName = collider.first;
                    auto bit = boneNameToIdx.find(boneName);
                    if (bit == boneNameToIdx.end())
                        continue;

                    const std::uint32_t ci = AllocateCollider();
                    colliders.boneIdx[ci] = bit->second;
                    colliders.objIdx[ci] = objIdx;
                    colliders.rootIdx[ci] = rootIdx;

                    colliders.convexHullData[ci] = collider.second;

                    logger::debug("{:x} => add collider {} / colGroup {:x} / colLayer {:x}", object->formID, boneName, physicsBones.layerGroup[bit->second], physicsBones.collideLayer[bit->second]);
                    addedColCount++;

                    AABB aabb = AABB();
                    for (std::uint32_t v = 0; v < COL_VERTEX_MAX; ++v)
                    {
                        const Vector p = DirectX::XMVectorSet(colliders.convexHullData[ci].vX[v], colliders.convexHullData[ci].vY[v], colliders.convexHullData[ci].vZ[v], 0.0f);
                        const AABB vAABB(p, p);
                        aabb = aabb.Merge(vAABB);
                    }
                    colliders.boundingAABB[ci] = aabb;

                    const Vector center = DirectX::XMVectorMultiply(DirectX::XMVectorAdd(aabb.min, aabb.max), vHalf);
                    colliders.boundingSphereCenter[ci] = center;

                    Vector maxRadiusSq = vZero;
                    for (std::uint32_t v = 0; v < COL_VERTEX_MAX; ++v)
                    {
                        const Vector p = DirectX::XMVectorSet(colliders.convexHullData[ci].vX[v], colliders.convexHullData[ci].vY[v], colliders.convexHullData[ci].vZ[v], 0.0f);
                        const Vector dist = DirectX::XMVectorSubtract(p, center);
                        const Vector distSq = DirectX::XMVector3LengthSq(dist);
                        if (DirectX::XMVector3Less(maxRadiusSq, distSq))
                            maxRadiusSq = distSq;
                    }
                    colliders.boundingSphere[ci] = DirectX::XMVectorGetX(DirectX::XMVectorSqrt(maxRadiusSq));

                    if (const float colShrink = physicsBones.collisionShrink[bit->second]; colShrink > FloatPrecision)
                    {
                        const float maxShrink = colliders.boundingSphere[ci] * 0.9f;
                        const float finalShrink = std::min(physicsBones.collisionShrink[bit->second], maxShrink);
                        const float shrinkRatio = 1.0f - (finalShrink * reciprocal(colliders.boundingSphere[ci]));

                        AABB shrinkedAABB = AABB();
                        DirectX::XMFLOAT3 center_f3;
                        DirectX::XMStoreFloat3(&center_f3, colliders.boundingSphereCenter[ci]);
                        for (std::uint32_t v = 0; v < COL_VERTEX_MAX; ++v)
                        {
                            colliders.convexHullData[ci].vX[v] = center_f3.x + (colliders.convexHullData[ci].vX[v] - center_f3.x) * shrinkRatio;
                            colliders.convexHullData[ci].vY[v] = center_f3.y + (colliders.convexHullData[ci].vY[v] - center_f3.y) * shrinkRatio;
                            colliders.convexHullData[ci].vZ[v] = center_f3.z + (colliders.convexHullData[ci].vZ[v] - center_f3.z) * shrinkRatio;
                            const Vector p = DirectX::XMVectorSet(colliders.convexHullData[ci].vX[v], colliders.convexHullData[ci].vY[v], colliders.convexHullData[ci].vZ[v], 0.0f);
                            shrinkedAABB = shrinkedAABB.Merge(AABB(p, p));
                        }
                        colliders.boundingSphere[ci] *= shrinkRatio;
                        colliders.boundingAABB[ci] = shrinkedAABB;
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
                                colliders.noCollideBoneIdx[static_cast<std::uint32_t>(ci) * NOCOLLIDE_MAX + ncCount] = ncBit->second;
                                logger::debug("{:x} : {} => add no collide {}", object->formID, boneName, ncName);
                                ncCount++;
                            }
                        }
                    }

                    auto ncIt = input.convexHullColliders.noCollideBones.find(boneName);
                    if (ncIt != input.convexHullColliders.noCollideBones.end())
                    {
                        for (const auto& ncName : ncIt->second)
                        {
                            if (boneName == ncName)
                                continue;
                            auto ncBit = boneNameToIdx.find(ncName);
                            if (ncBit == boneNameToIdx.end())
                                continue;
                            const std::uint32_t ncbi = ncBit->second;
                            auto begin = colliders.noCollideBoneIdx.begin() + static_cast<std::uint32_t>(ci) * NOCOLLIDE_MAX;
                            auto end = begin + ncCount;
                            auto it = std::find(begin, end, ncbi);
                            if (it != end)
                                continue;
                            colliders.noCollideBoneIdx[static_cast<std::uint32_t>(ci) * NOCOLLIDE_MAX + ncCount] = ncbi;
                            logger::debug("{:x} : {} => add no collide {}", object->formID, boneName, ncName);
                            ncCount++;
                        }
                    }
                    colliders.noCollideCount[ci] = ncCount;
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

        if (reset)
            Reset(object);

        logger::info("{:x} : Replacing physics setting...", object->formID);
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
            if (RE::TESObjectREFR* object = GetREFR(objectDatas.objectID[oi]); object && object->loadedData && object->loadedData->data3D)
                objectDatas.prevWorldPos[oi] = ToVector(object->loadedData->data3D->world.translate);
            else
                objectDatas.prevWorldPos[oi] = ToVector(object->GetPosition());
            objectDatas.acceleration[oi] = vZero;
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

                        logger::debug("{:x} : Found bone ({}) for physics", object->formID, node->name.c_str());

                        physicsBones.offset[bi] = ToVector(found->second.offset);
                        const Vector offset = DirectX::XMVector3Rotate(DirectX::XMVectorScale(physicsBones.offset[bi], physicsBones.orgWorldScale[bi]), ToQuaternion(node->world.rotate));
                        
                        physicsBones.pos[bi] = DirectX::XMVectorAdd(ToVector(node->world.translate), offset);
                        physicsBones.pred[bi] = physicsBones.pos[bi];
                        physicsBones.vel[bi] = vZero;

                        physicsBones.rot[bi] = ToQuaternion(node->world.rotate);
                        physicsBones.predRot[bi] = physicsBones.rot[bi];
                        physicsBones.angVel[bi] = vZero;

                        physicsBones.damping[bi] = found->second.damping;
                        physicsBones.inertiaScale[bi] = found->second.inertiaScale;
                        physicsBones.restitution[bi] = found->second.restitution;
                        physicsBones.rotationBlendFactor[bi] = found->second.rotationBlendFactor;
                        physicsBones.gravity[bi] = ToVector(GetSkyrimGravity(found->second.gravity));
                        physicsBones.invMass[bi] = (found->second.mass > FloatPrecision) ? reciprocal(found->second.mass) : 0.0f;
                        physicsBones.windFactor[bi] = found->second.windFactor;

                        physicsBones.restPoseLimit[bi] = found->second.restPoseLimit;
                        physicsBones.restPoseCompliance[bi] = found->second.restPoseCompliance;

                        physicsBones.restPoseAngularLimit[bi] = found->second.restPoseAngularLimit;
                        physicsBones.restPoseAngularCompliance[bi] = found->second.restPoseAngularCompliance;

                        if (found->second.mass > FloatPrecision && found->second.inertiaScale > FloatPrecision)
                            physicsBones.invInertia[bi] = reciprocal(found->second.mass * found->second.inertiaScale);
                        else
                            physicsBones.invInertia[bi] = 0.0f;

                        physicsBones.linearRotTorque[bi] = NiPoin3x3ToXMMATRIX(found->second.linearRotTorque);

                        physicsBones.collisionMargin[bi] = (found->second.collisionMargin < 0.0f ? 0.0f : found->second.collisionMargin);
                        physicsBones.collisionShrink[bi] = (found->second.collisionMargin < COL_MARGIN_MIN ? COL_MARGIN_MIN - found->second.collisionMargin : 0.0f);
                        physicsBones.collisionFriction[bi] = found->second.collisionFriction;
                        physicsBones.collisionRotationBias[bi] = found->second.collisionRotationBias;
                        physicsBones.collisionCompliance[bi] = found->second.collisionCompliance;
                        physicsBones.layerGroup[bi] = found->second.collisionLayerGroup;
                        physicsBones.collideLayer[bi] = found->second.collisionCollideLayer;

                        physicsBones.orgWorldScale[bi] = node->world.scale;
                    }
                    else if (!physicsBones.particleName[bi].empty())
                    {
                        const std::uint32_t pbi = physicsBones.parentBoneIdx[bi];
                        if (pbi == UINT32_MAX)
                            continue;
                        boneNameToIdx[physicsBones.particleName[bi]] = bi;

                        const auto found = input.bones.find(physicsBones.particleName[bi]);
                        if (found == input.bones.end())
                            continue;

                        logger::debug("{:x} : Found particle ({}) for physics", object->formID, physicsBones.particleName[bi].c_str());

                        physicsBones.offset[bi] = ToVector(found->second.offset);
                        const Vector offset = DirectX::XMVector3Rotate(DirectX::XMVectorScale(physicsBones.offset[bi], physicsBones.orgWorldScale[pbi]), physicsBones.rot[pbi]);
                        
                        physicsBones.pos[bi] = DirectX::XMVectorAdd(physicsBones.pos[pbi], offset);
                        physicsBones.pred[bi] = physicsBones.pos[bi];
                        physicsBones.vel[bi] = vZero;

                        physicsBones.rot[bi] = physicsBones.rot[pbi];
                        physicsBones.predRot[bi] = physicsBones.rot[bi];
                        physicsBones.angVel[bi] = vZero;

                        physicsBones.damping[bi] = found->second.damping;
                        physicsBones.inertiaScale[bi] = found->second.inertiaScale;
                        physicsBones.restitution[bi] = found->second.restitution;
                        physicsBones.rotationBlendFactor[bi] = found->second.rotationBlendFactor;
                        physicsBones.gravity[bi] = ToVector(GetSkyrimGravity(found->second.gravity));
                        physicsBones.invMass[bi] = (found->second.mass > FloatPrecision) ? reciprocal(found->second.mass) : 0.0f;
                        physicsBones.windFactor[bi] = found->second.windFactor;

                        physicsBones.restPoseLimit[bi] = found->second.restPoseLimit;
                        physicsBones.restPoseCompliance[bi] = found->second.restPoseCompliance;

                        physicsBones.restPoseAngularLimit[bi] = found->second.restPoseAngularLimit;
                        physicsBones.restPoseAngularCompliance[bi] = found->second.restPoseAngularCompliance;

                        if (found->second.mass > FloatPrecision && found->second.inertiaScale > FloatPrecision)
                            physicsBones.invInertia[bi] = (found->second.mass * found->second.inertiaScale);
                        else
                            physicsBones.invInertia[bi] = 0.0f;

                        physicsBones.linearRotTorque[bi] = NiPoin3x3ToXMMATRIX(found->second.linearRotTorque);

                        physicsBones.orgWorldScale[bi] = physicsBones.orgWorldScale[pbi];
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
                    if (constraints.boneIdx[ci] == UINT32_MAX)
                        continue;
                    const std::uint32_t bi = constraints.boneIdx[ci];
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

                    logger::debug("{:x} : Found bone ({}) for constraints", object->formID, nodeName);

                    std::uint32_t validAnchorCount = 0;
                    const std::uint32_t aiBase = ci * ANCHOR_MAX;
                    for (std::uint32_t a = 0; a < found->second.anchorBoneNames.size(); ++a)
                    {
                        if (boneNameToIdx.find(found->second.anchorBoneNames[a]) == boneNameToIdx.end())
                            continue;
                        if (validAnchorCount >= ANCHOR_MAX)
                            break;
                        logger::debug("{:x} : add constraint {}({}) on {}", object->formID, nodeName, validAnchorCount, found->second.anchorBoneNames[a]);
                        const std::uint32_t ai = aiBase + validAnchorCount;
                        const std::uint32_t anchGlobIdx = boneNameToIdx[found->second.anchorBoneNames[a]];
                        constraints.anchData[ai].anchIdx = anchGlobIdx;
                        const Vector tPos = physicsBones.pos[bi];
                        const Vector aPos = physicsBones.pos[anchGlobIdx];
                        constraints.anchData[ai].restLen = DirectX::XMVectorGetX(DirectX::XMVector3Length(DirectX::XMVectorSubtract(tPos, aPos)));
                        constraints.anchData[ai].complianceSquish = found->second.complianceSquish[a];
                        constraints.anchData[ai].complianceStretch = found->second.complianceStretch[a];
                        constraints.anchData[ai].squishLimit = found->second.squishLimit[a];
                        constraints.anchData[ai].stretchLimit = found->second.stretchLimit[a];
                        constraints.anchData[ai].angularLimit = found->second.angularLimit[a];
                        Vector dirWorld = DirectX::XMVectorSubtract(tPos, aPos);
                        if (DirectX::XMVector3Less(vFloatPrecision, DirectX::XMVector3LengthSq(dirWorld)))
                        {
                            dirWorld = DirectX::XMVector3Normalize(dirWorld);
                            const Quaternion anchRotInv = DirectX::XMQuaternionConjugate(physicsBones.rot[anchGlobIdx]);
                            constraints.anchData[ai].restDirLocal = DirectX::XMVector3Rotate(dirWorld, anchRotInv);
                        }
                        else
                        {
                            constraints.anchData[ai].restDirLocal = DirectX::XMVectorSet(0.0f, -1.0f, 0.0f, 0.0f);
                        }
                        constraints.anchData[ai].squishDamping = found->second.squishDamping[a];
                        constraints.anchData[ai].stretchDamping = found->second.stretchDamping[a];
                        validAnchorCount++;
                    }
                    constraints.numAnchors[ci] = validAnchorCount;
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
                    if (angularConstraints.boneIdx[aci] == UINT32_MAX)
                        continue;
                    const std::uint32_t bi = angularConstraints.boneIdx[aci];
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

                    physicsBones.advancedRotation[bi] = 1;

                    logger::debug("{:x} : Found bone ({}) for angular constraints", object->formID, nodeName);

                    std::uint32_t validAnchorCount = 0;
                    const std::uint32_t aiBase = aci * ANCHOR_MAX;
                    for (std::uint32_t a = 0; a < found->second.anchorBoneNames.size(); ++a)
                    {
                        if (boneNameToIdx.find(found->second.anchorBoneNames[a]) == boneNameToIdx.end())
                            continue;
                        if (validAnchorCount >= ANCHOR_MAX)
                            break;
                        logger::debug("{:x} : add angular constraint {}({}) on {}", object->formID, nodeName, validAnchorCount, found->second.anchorBoneNames[a]);
                        const std::uint32_t ai = aiBase + validAnchorCount;
                        const std::uint32_t anchBi = boneNameToIdx[found->second.anchorBoneNames[a]];
                        angularConstraints.anchData[ai].anchIdx = anchBi;

                        const auto anchBoneIt = input.bones.find(found->second.anchorBoneNames[a]);
                        if (anchBoneIt == input.bones.end())
                            continue;

                        physicsBones.advancedRotation[bi] = 1;

                        const Quaternion childRot = physicsBones.rot[bi];
                        const Quaternion anchorRot = physicsBones.rot[anchBi];
                        const Quaternion anchorRotInv = DirectX::XMQuaternionInverse(anchorRot);
                        const Quaternion restRot = DirectX::XMQuaternionNormalize(DirectX::XMQuaternionMultiply(childRot, anchorRotInv));
                        angularConstraints.anchData[ai].restRot = restRot;
                        angularConstraints.anchData[ai].compliance = found->second.compliance[a];
                        angularConstraints.anchData[ai].limit = found->second.limit[a];
                        angularConstraints.anchData[ai].damping = found->second.damping[a];
                        validAnchorCount++;
                    }
                    angularConstraints.numAnchors[aci] = validAnchorCount;
                }
            }
        }
    }

    void XPBDWorld::ResetAll()
    {
        std::lock_guard lg(lock);

        for (std::uint32_t oi = 0; oi < objectDatas.objectID.size(); ++oi)
        {
            if (RE::TESObjectREFR* object = GetREFR(objectDatas.objectID[oi]); object)
            {
                if (object && object->loadedData && object->loadedData->data3D)
                    objectDatas.prevWorldPos[oi] = ToVector(object->loadedData->data3D->world.translate);
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

        std::lock_guard lg(lock);

        // find object
        std::uint32_t oi = UINT32_MAX;
        for (std::uint32_t i = 0; i < objectDatas.objectID.size(); ++i)
        {
            if (objectDatas.objectID[i] == object->formID)
            {
                oi = i;
                objectDatas.prevWorldPos[i] = ToVector(object->GetPosition());
                objectDatas.acceleration[i] = vZero;
                break;
            }
        }
        if (oi == UINT32_MAX)
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
                if (physicsBones.objIdx[begin] != oi)
                    continue;
                for (std::uint32_t bi = begin; bi < end; ++bi)
                {
                    ResetBone(bi);
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
            const RE::NiMatrix3 oldLocalRot = ToMatrix(physicsBones.orgLocalRot[bi]);
            memcpy(&node->local.translate, &oldLocalPos, sizeof(oldLocalPos));
            memcpy(&node->local.rotate, &oldLocalRot, sizeof(oldLocalRot));

            if (node->parent)
            {
                const RE::NiPoint3 parentWorldPos = node->parent->world.translate;
                const RE::NiMatrix3 parentWorldRot = node->parent->world.rotate;
                const RE::NiPoint3 worldPos = parentWorldPos + (parentWorldRot * oldLocalPos);
                const RE::NiMatrix3 worldRot = parentWorldRot * ToMatrix(physicsBones.orgLocalRot[bi]);
                memcpy(&node->world.translate, &worldPos, sizeof(worldPos));
                memcpy(&node->world.rotate, &worldRot, sizeof(worldRot));
            }

            const Vector offset = DirectX::XMVector3Rotate(DirectX::XMVectorScale(physicsBones.offset[bi], physicsBones.orgWorldScale[bi]), ToQuaternion(node->world.rotate));
            physicsBones.pos[bi] = DirectX::XMVectorAdd(ToVector(node->world.translate), offset);
            physicsBones.prevPos[bi] = physicsBones.pos[bi];
            physicsBones.pred[bi] = physicsBones.pos[bi];
            physicsBones.vel[bi] = vZero;

            physicsBones.rot[bi] = ToQuaternion(node->world.rotate);
            physicsBones.prevRot[bi] = physicsBones.rot[bi];
            physicsBones.predRot[bi] = physicsBones.rot[bi];
            physicsBones.backupRot[bi] = physicsBones.rot[bi];
            physicsBones.angVel[bi] = vZero;
        }
        else if (!physicsBones.particleName[bi].empty() && physicsBones.parentBoneIdx[bi] != UINT32_MAX)
        {
            const std::uint32_t pbi = physicsBones.parentBoneIdx[bi];
            const Vector offset = DirectX::XMVector3Rotate(DirectX::XMVectorScale(physicsBones.offset[bi], physicsBones.orgWorldScale[pbi]), physicsBones.rot[pbi]);
            physicsBones.pos[bi] = DirectX::XMVectorAdd(physicsBones.pos[pbi], offset);
            physicsBones.prevPos[bi] = physicsBones.pos[bi];
            physicsBones.pred[bi] = physicsBones.pos[bi];
            physicsBones.vel[bi] = vZero;

            physicsBones.rot[bi] = physicsBones.rot[pbi];
            physicsBones.prevRot[bi] = physicsBones.rot[bi];
            physicsBones.predRot[bi] = physicsBones.rot[bi];
            physicsBones.backupRot[bi] = physicsBones.rot[bi];
            physicsBones.angVel[bi] = vZero;
        }
    }

    void XPBDWorld::RemovePhysics(const RE::FormID objectID)
    {
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
        std::lock_guard lg(lock);
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

        TIMELOG_START;
        threadPool->Execute([&] {
            std::uint32_t stepCount = 0;
            objectAccelerationTime += deltaTime;
            const float prevTimeAccumulator = timeAccumulator;
            timeAccumulator += std::min(DeltaTime60 * 6, deltaTime); // low 10 fps
            while (timeAccumulator >= DeltaTime60)
            {
                const float subStepTime = (stepCount + 1) * DeltaTime60;
                const float alpha = std::clamp((subStepTime - prevTimeAccumulator) * reciprocal(deltaTime), 0.0f, 1.0f);

                if (stepCount == 0)
                {
                    UpdateObjectData(objectAccelerationTime);
                    objectAccelerationTime = 0.0f;
                }
                ClampObjectRotation();
                PrefetchBoneDatas(alpha, stepCount == 0);
                UpdateGlobalAABBTree();
                ObjectCulling();
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
                groundCache.resize(physicsBones.numBones);

                PredictBones(DeltaTime60);
                tbb::parallel_invoke(
                    [&] {
                        CreateLocalSpatialHash();
                        GenerateCollisionManifolds();
                    },
                    [&] {
                        GenerateGroundCache();
                    }
                );
                for (std::uint32_t i = 0; i < ITERATION_MAX; ++i)
                {
                    SolveCachedCollisions(DeltaTime60);
                    SolveCachedGroundCollisions(DeltaTime60);
                    SolveConstraints(DeltaTime60, i == 0);
                    SolveRestPoseConstraints(DeltaTime60, i == 0);
                }
                UpdateBoneVelocity(DeltaTime60);

                timeAccumulator -= DeltaTime60;
                stepCount++;
                currentFrame++;
            }
            ApplyToSkyrim();
        });
        TIMELOG_END;
    }

    void XPBDWorld::UpdateObjectData(const float deltaTime)
    {
        if (physicsBonesGroup.empty())
            return;
        // logger::info("{}", __func__);
        TIMELOG_START;
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
                    const Vector currentVel = DirectX::XMVectorMultiply(DirectX::XMVectorSubtract(currentWorldPos, objectDatas.prevWorldPos[oi]), invDt);
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
        TIMELOG_END;
    }

    void XPBDWorld::ClampObjectRotation()
    {
        if (physicsBonesGroup.empty())
            return;
        TIMELOG_START;
        const std::uint32_t groups = physicsBonesGroup.size() - 1;
        for (std::uint32_t g = 0; g < groups; ++g)
        {
            const std::uint32_t begin = physicsBonesGroup[g];
            const std::uint32_t oi = physicsBones.objIdx[begin];
            if (IsDisable(oi))
                continue;
            if (objectDatas.objectID[oi] != 0x14)
                continue;
            RE::Actor* player = GetActor(objectDatas.objectID[oi]);
            if (!player || !player->loadedData || !player->loadedData->data3D)
                continue;
            RE::NiNode* npcNode = objectDatas.npcNode[oi];
            if (!npcNode || !npcNode->parent)
                continue;
            if (RE::ActorState* state = player->AsActorState(); state && state->IsWeaponDrawn())
                continue;
            if (RE::PlayerCamera* playerCamera = RE::PlayerCamera::GetSingleton(); playerCamera)
            {
                if (playerCamera->IsInFirstPerson() || playerCamera->IsInFreeCameraMode())
                    continue;
            }

            const Quaternion q_prev = objectDatas.prevNPCWorldRot[oi];
            const Quaternion q_inv = DirectX::XMQuaternionInverse(q_prev);
            const Quaternion q_curr = ToQuaternion(objectDatas.npcNode[oi]->world.rotate);
            const Quaternion q_diff = DirectX::XMQuaternionMultiply(q_inv, q_curr);
            const Quaternion q_target = DirectX::XMQuaternionNormalize(DirectX::XMQuaternionMultiply(q_diff, objectDatas.targetNPCWorldRot[oi]));
            objectDatas.targetNPCWorldRot[oi] = q_target;

            Quaternion q_delta = DirectX::XMQuaternionMultiply(q_inv, q_target);
            if (DirectX::XMVectorGetW(q_delta) < 0.0f)
                q_delta = DirectX::XMVectorNegate(q_delta);

            const float cosHalfAngle = std::clamp(DirectX::XMVectorGetW(q_delta), -1.0f, 1.0f);
            const float angle = 2.0f * std::acos(cosHalfAngle);
            const float t = (angle > ROTATION_CLAMP) ? (ROTATION_CLAMP * reciprocal(angle)) : 1.0f;
            const Quaternion q_final = DirectX::XMQuaternionNormalize(DirectX::XMQuaternionSlerp(q_prev, q_target, t));

            const Quaternion q_parent_world = ToQuaternion(npcNode->parent->world.rotate);
            const Quaternion q_parent_inv = DirectX::XMQuaternionInverse(q_parent_world);
            const Quaternion q_npc_local = DirectX::XMQuaternionNormalize(DirectX::XMQuaternionMultiply(q_final, q_parent_inv));

            const RE::NiMatrix3 npc_local = ToMatrix(q_npc_local);
            std::memcpy(&npcNode->local.rotate, &npc_local, sizeof(npc_local));

            objectDatas.prevNPCWorldRot[oi] = q_final;
            const RE::NiMatrix3 npc_world = ToMatrix(q_final);
            std::memcpy(&npcNode->world.rotate, &npc_world, sizeof(npc_world));

            RE::NiUpdateData ctx = {0.0f, RE::NiUpdateData ::Flag::kDirty};
            npcNode->UpdateWorldData(&ctx);
            UpdateChildTreeData(npcNode);
        }
        TIMELOG_END;
    }

    void XPBDWorld::PrefetchBoneDatas(const float alpha, const bool isFirstStep)
    {
        // logger::info("{}", __func__);
        TIMELOG_START;
        tbb::parallel_for(
            tbb::blocked_range<std::uint32_t>(0, physicsBones.numBones, 32),
            [&](const tbb::blocked_range<std::uint32_t>& r) {
                for (std::uint32_t bi = r.begin(); bi != r.end(); ++bi)
                {
                    if (!physicsBones.isParticle[bi])
                    {
                        auto& node = physicsBones.node[bi];
                        if (isFirstStep)
                        {
                            physicsBones.prevNodeWorldPos[bi] = physicsBones.targetNodeWorldPos[bi];
                            physicsBones.prevNodeWorldRot[bi] = physicsBones.targetNodeWorldRot[bi];
                        }
                        physicsBones.targetNodeWorldPos[bi] = ToVector(node->world.translate);
                        physicsBones.targetNodeWorldRot[bi] = ToQuaternion(node->world.rotate);

                        const Vector interPos = DirectX::XMVectorLerp(physicsBones.prevNodeWorldPos[bi], physicsBones.targetNodeWorldPos[bi], alpha);
                        const Quaternion interRot = DirectX::XMQuaternionSlerp(physicsBones.prevNodeWorldRot[bi], physicsBones.targetNodeWorldRot[bi], alpha);

                        if (physicsBones.invMass[bi] <= FloatPrecision)
                        {
                            const Vector offset = DirectX::XMVector3Rotate(DirectX::XMVectorScale(physicsBones.offset[bi], physicsBones.orgWorldScale[bi]), interRot);
                            physicsBones.pos[bi] = DirectX::XMVectorAdd(interPos, offset);
                            physicsBones.prevPos[bi] = physicsBones.pos[bi];
                            physicsBones.pred[bi] = physicsBones.pos[bi];
                            physicsBones.backupRot[bi] = physicsBones.rot[bi];
                            physicsBones.rot[bi] = interRot;
                            physicsBones.prevRot[bi] = physicsBones.rot[bi];
                            physicsBones.predRot[bi] = physicsBones.rot[bi];
                        }
                        physicsBones.orgWorldScale[bi] = node->world.scale;
                    }
                    else
                    {
                        if (physicsBones.parentBoneIdx[bi] != UINT32_MAX)
                        {
                            const std::uint32_t pbi = physicsBones.parentBoneIdx[bi];
                            if (physicsBones.invMass[bi] <= FloatPrecision)
                            {
                                const Vector offset = DirectX::XMVector3Rotate(DirectX::XMVectorScale(physicsBones.offset[bi], physicsBones.orgWorldScale[pbi]), physicsBones.rot[pbi]);
                                physicsBones.pos[bi] = DirectX::XMVectorAdd(physicsBones.pos[pbi], offset);
                                physicsBones.prevPos[bi] = physicsBones.pos[bi];
                                physicsBones.pred[bi] = physicsBones.pos[bi];
                                physicsBones.backupRot[bi] = physicsBones.rot[bi];
                                physicsBones.rot[bi] = physicsBones.rot[pbi];
                                physicsBones.prevRot[bi] = physicsBones.rot[bi];
                                physicsBones.predRot[bi] = physicsBones.rot[bi];
                            }
                            physicsBones.rot[bi] = physicsBones.rot[pbi];
                            physicsBones.orgWorldScale[bi] = physicsBones.orgWorldScale[pbi];
                        }
                    }
                }
            },
            tbb::static_partitioner()
        );
        TIMELOG_END;
    }

    void XPBDWorld::UpdateGlobalAABBTree()
    {
        // logger::info("{}", __func__);
        TIMELOG_START;
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
        TIMELOG_END;
    }

    void XPBDWorld::ObjectCulling()
    {
        RE::NiCamera* niCam = RE::Main::WorldRootCamera();
        if (!niCam)
            return;

        TIMELOG_START;
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
        TIMELOG_END;
    }

    void XPBDWorld::UpdateWindStrength()
    {
        if (physicsBonesGroup.empty() || windSpeed <= FloatPrecision)
            return;

        TIMELOG_START;
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
            const std::uint32_t oi = colliders.objIdx[begin];
            if (IsDisable(oi))
                continue;
            RE::bhkWorld* bhkWorld = objectDatas.bhkWorld[oi];
            if (!bhkWorld)
                continue;

            const AABB& worldAABB = objectDatas.boundingAABB[oi];
            const Vector worldCenter = worldAABB.GetCenter();
            const Vector extents = worldAABB.GetExtents();

            const float rx = rand_PCG32_Float(objectDatas.randState[oi]);
            const float ry = rand_PCG32_Float(objectDatas.randState[oi]);
            const float rz = rand_PCG32_Float(objectDatas.randState[oi]);
            const Vector randomOffset = DirectX::XMVectorMultiply(DirectX::XMVectorSet(rx, ry, rz, 0.0f), extents);

            const Vector from = DirectX::XMVectorAdd(worldCenter, randomOffset);
            const Vector havokFrom = DirectX::XMVectorScale(from, Scale_havokWorld);
            const Vector windRayTo = DirectX::XMVectorScale(windVector, -WIND_DETECT_RANGE);
            const Vector to = DirectX::XMVectorAdd(from, windRayTo);
            const Vector havokTo = DirectX::XMVectorScale(to, Scale_havokWorld);
            
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
                if (FloatPrecision < normalDot)
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
        TIMELOG_END;
    }

    void XPBDWorld::PredictBones(const float deltaTime)
    {
        // logger::info("{}", __func__);
        TIMELOG_START;
        const Vector dt = DirectX::XMVectorReplicate(deltaTime);
        const float flutterNoise = 1.0f + 0.5f * sin(currentFrame * 0.1f);
        const float currentWindSpeed = windSpeed * flutterNoise;
        const float windX = sin(windAngle) * windSpeed;
        const float windY = std::cos(windAngle) * windSpeed;
        const float windTimeFreq = currentFrame * 0.05f;
        tbb::parallel_for(
            tbb::blocked_range<std::uint32_t>(0, physicsBones.numBones, 128),
            [&](const tbb::blocked_range<std::uint32_t>& r) {
                for (std::uint32_t bi = r.begin(); bi != r.end(); ++bi)
                {
                    const std::uint32_t oi = physicsBones.objIdx[bi];
                    if (IsDisable(oi))
                        continue;
                    if (physicsBones.invMass[bi] <= FloatPrecision)
                        continue;

                    // apply linear velocity
                    {
                        const Vector fictAcc = DirectX::XMVectorScale(objectDatas.acceleration[oi], -physicsBones.inertiaScale[bi]);
                        const Vector totalAcc = DirectX::XMVectorScale(DirectX::XMVectorAdd(physicsBones.gravity[bi], fictAcc), deltaTime);
                        physicsBones.vel[bi] = DirectX::XMVectorAdd(physicsBones.vel[bi], totalAcc);
                    }

                    // apply wind
                    if (FloatPrecision < objectDatas.windMultiplier[oi])
                    {
                        const float posNoise = sin(windTimeFreq + (DirectX::XMVectorGetX(physicsBones.pos[bi]) * 0.02f));
                        const float windZ = currentWindSpeed * (0.3f + posNoise * 0.4f);
                        const float scaledWind = objectDatas.windMultiplier[oi] * physicsBones.windFactor[bi] * physicsBones.invMass[bi];
                        const Vector windForce = DirectX::XMVectorScale(DirectX::XMVectorSet(windX, windY, windZ, 0.0f), scaledWind * deltaTime);
                        physicsBones.vel[bi] = DirectX::XMVectorAdd(physicsBones.vel[bi], windForce);
                    }

                    // predic rotation
                    {
                        std::uint32_t pbi = physicsBones.parentBoneIdx[bi];
                        while (pbi != UINT32_MAX && physicsBones.invMass[pbi] > FloatPrecision)
                        {
                            pbi = physicsBones.parentBoneIdx[pbi];
                        }
                        if (pbi != UINT32_MAX)
                        {
                            const Quaternion delta = DirectX::XMQuaternionMultiply(DirectX::XMQuaternionConjugate(physicsBones.backupRot[pbi]), physicsBones.rot[pbi]);
                            const Vector magSq = DirectX::XMVector3LengthSq(DirectX::XMVectorSetW(delta, 0.0f));
                            if (DirectX::XMVector3Less(vFloatPrecision, magSq))
                            {
                                const Vector pivot = physicsBones.pred[pbi];
                                const Vector pos = physicsBones.pred[bi];
                                const Vector localPos = DirectX::XMVectorSubtract(pos, pivot);
                                const Vector rotatedPos = DirectX::XMVector3Rotate(localPos, delta);
                                physicsBones.pred[bi] = DirectX::XMVectorAdd(pivot, rotatedPos);
                                physicsBones.rot[bi] = DirectX::XMQuaternionNormalize(DirectX::XMQuaternionMultiply(physicsBones.rot[bi], delta));
                            }

                            const Quaternion q = physicsBones.rot[bi];
                            const Quaternion w = DirectX::XMVectorMultiply(physicsBones.angVel[bi], DirectX::XMVectorMultiply(dt, vHalf));
                            physicsBones.predRot[bi] = DirectX::XMQuaternionNormalize(DirectX::XMVectorAdd(q, DirectX::XMQuaternionMultiply(q, w)));
                        }
                    }

                    // predic linear
                    {
                        physicsBones.pred[bi] = DirectX::XMVectorMultiplyAdd(physicsBones.vel[bi], DirectX::XMVectorReplicate(deltaTime), physicsBones.pos[bi]);
                    }

                    // init
                    groundCache[bi].isCast = false;
                }
            },
            tbb::static_partitioner()
        );
        TIMELOG_END;
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

        TIMELOG_START;

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
                        const Vector worldCenter = DirectX::XMVectorAdd(physicsBones.pred[bi], DirectX::XMVector3Rotate(DirectX::XMVectorScale(colliders.boundingSphereCenter[ci], physicsBones.orgWorldScale[bi]), physicsBones.predRot[bi]));
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
                        const Vector worldCenter = DirectX::XMVectorAdd(physicsBones.pred[bi], DirectX::XMVector3Rotate(DirectX::XMVectorScale(colliders.boundingSphereCenter[ci], physicsBones.orgWorldScale[bi]), physicsBones.predRot[bi]));
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
        TIMELOG_END;
    }

    void XPBDWorld::GenerateCollisionManifolds()
    {
        if (collidersGroup.empty())
            return;
        // logger::info("{}", __func__);
        TIMELOG_START;

        auto AddManifold = [&](const std::uint32_t coiA, const std::uint32_t coiB) {
            ContactManifold manifold;
            if (!ConvexHullvsConvexHull(coiA, coiB, manifold))
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
                        const Vector worldCenterA = DirectX::XMVectorAdd(physicsBones.pred[biA], DirectX::XMVector3Rotate(DirectX::XMVectorScale(colliders.boundingSphereCenter[coiA], physicsBones.orgWorldScale[biA]), physicsBones.predRot[biA]));
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
                                if (physicsBones.invMass[biA] + physicsBones.invMass[biB] <= FloatPrecision)
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
                                    const Vector worldCenterA = DirectX::XMVectorAdd(physicsBones.pred[biA], DirectX::XMVector3Rotate(DirectX::XMVectorScale(colliders.boundingSphereCenter[coiA], physicsBones.orgWorldScale[biA]), physicsBones.predRot[biA]));
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
                                            if (physicsBones.invMass[biA] <= FloatPrecision && physicsBones.invMass[biB] <= FloatPrecision)
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
        TIMELOG_END;

        
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

    void XPBDWorld::GenerateGroundCache()
    {
        if (collidersLeafs.empty() || GROUND_DETECT_RANGE <= FloatPrecision)
            return;
        // logger::info("{}", __func__);

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

        TIMELOG_START;
        const std::uint32_t totalLeafs = collidersLeafs.size();
        const std::uint32_t quality = std::max(1u, std::min(GROUND_DETECT_QUALITY, totalLeafs));
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
            AABB worldAABB = colliders.boundingAABB[ci].GetWorldAABB(physicsBones.pred[bi], physicsBones.predRot[bi], physicsBones.orgWorldScale[bi]);
            worldAABB.Fatten(physicsBones.collisionMargin[bi]);
            const Vector worldBottomCenter = DirectX::XMVectorSubtract(worldAABB.GetCenter(), DirectX::XMVectorSet(0.0f, 0.0f, DirectX::XMVectorGetZ(worldAABB.min), 0.0f));

            const Vector from = DirectX::XMVectorAdd(worldBottomCenter, groundRayFrom);
            const Vector havokFrom = DirectX::XMVectorScale(from, Scale_havokWorld);
            const Vector to = DirectX::XMVectorAdd(worldBottomCenter, groundRayTo);
            const Vector havokTo = DirectX::XMVectorScale(to, Scale_havokWorld);

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
        TIMELOG_END;
    }

    void XPBDWorld::SolveCachedCollisions(const float deltaTime)
    {
        const std::uint32_t validCollisions = std::min(manifoldCacheCount, expectedCollisionCount);
        if (validCollisions == 0)
            return;
        TIMELOG_START;
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
            const float invInertiaA = physicsBones.invInertia[biA] * physicsBones.collisionRotationBias[biA] * biasA;
            const float invInertiaB = physicsBones.invInertia[biB] * physicsBones.collisionRotationBias[biB] * biasB;
            if (wA + wB + invInertiaA + invInertiaB <= FloatPrecision)
                return;

            tbb::spin_mutex::scoped_lock lock1, lock2;
            if (biA < biB)
            {
                if (wA > 0.0f || invInertiaA > 0.0f)
                    lock1.acquire(physicsBonesLock[biA]);
                if (wB > 0.0f || invInertiaB > 0.0f)
                    lock2.acquire(physicsBonesLock[biB]);
            }
            else
            {
                if (wB > 0.0f || invInertiaB > 0.0f)
                    lock1.acquire(physicsBonesLock[biB]);
                if (wA > 0.0f || invInertiaA > 0.0f)
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
                const Vector pA = physicsBones.pred[biA];
                const Quaternion qA = physicsBones.predRot[biA];
                const Vector rA = DirectX::XMVector3Rotate(DirectX::XMVectorScale(cp.localPointA, physicsBones.orgWorldScale[biA]), qA);
                const Vector wPtA = DirectX::XMVectorAdd(pA, rA);

                const Vector pB = physicsBones.pred[biB];
                const Quaternion qB = physicsBones.predRot[biB];
                const Vector rB = DirectX::XMVector3Rotate(DirectX::XMVectorScale(cp.localPointB, physicsBones.orgWorldScale[biB]), qB);
                const Vector wPtB = DirectX::XMVectorAdd(pB, rB);

                const Vector penetration = DirectX::XMVectorSubtract(wPtA, wPtB);
                const float currentDepth = -DirectX::XMVectorGetX(DirectX::XMVector3Dot(penetration, normal));
                if (currentDepth <= FloatPrecision)
                    continue;

                const Vector rAxN = DirectX::XMVector3Cross(rA, normal);
                const Vector rBxN = DirectX::XMVector3Cross(rB, normal);

                const float wRotA = invInertiaA * DirectX::XMVectorGetX(DirectX::XMVector3LengthSq(rAxN));
                const float wRotB = invInertiaB * DirectX::XMVectorGetX(DirectX::XMVector3LengthSq(rBxN));
                const float wNormSum = wA + wB + wRotA + wRotB;
                if (wNormSum <= FloatPrecision)
                    continue;

                const float lambdaN = currentDepth * reciprocal(wNormSum + alphaProxy);
                const float relaxedLambda = lambdaN * COL_CONVERGENCE;
                const Vector pCorrN = DirectX::XMVectorScale(normal, relaxedLambda);

                if (wA > 0.0f)
                    physicsBones.pred[biA] = DirectX::XMVectorAdd(physicsBones.pred[biA], DirectX::XMVectorScale(pCorrN, wA));
                if (invInertiaA > 0.0f)
                {
                    const Vector dThetaA = DirectX::XMVectorScale(rAxN, relaxedLambda * invInertiaA * rotConfidence);
                    const Vector dqA = DirectX::XMVectorMultiply(DirectX::XMQuaternionMultiply(physicsBones.predRot[biA], DirectX::XMVectorSetW(dThetaA, 0.0f)), vHalf);
                    physicsBones.predRot[biA] = DirectX::XMQuaternionNormalize(DirectX::XMVectorAdd(physicsBones.predRot[biA], dqA));
                }

                if (wB > 0.0f)
                    physicsBones.pred[biB] = DirectX::XMVectorSubtract(physicsBones.pred[biB], DirectX::XMVectorScale(pCorrN, wB));
                if (invInertiaB > 0.0f)
                {
                    const Vector dThetaB = DirectX::XMVectorScale(rBxN, -relaxedLambda * invInertiaB * rotConfidence);
                    const Vector dqB = DirectX::XMVectorMultiply(DirectX::XMQuaternionMultiply(physicsBones.predRot[biB], DirectX::XMVectorSetW(dThetaB, 0.0f)), vHalf);
                    physicsBones.predRot[biB] = DirectX::XMQuaternionNormalize(DirectX::XMVectorAdd(physicsBones.predRot[biB], dqB));
                }

                if (wA > 0.0f || invInertiaA > 0.0f)
                {
                    physicsBones.frictionCache[biA].n = DirectX::XMVectorAdd(physicsBones.frictionCache[biA].n, normal);
                    physicsBones.frictionCache[biA].depth = std::max(physicsBones.frictionCache[biA].depth, currentDepth);
                }
                if (wB > 0.0f || invInertiaB > 0.0f)
                {
                    physicsBones.frictionCache[biB].n = DirectX::XMVectorAdd(physicsBones.frictionCache[biB].n, DirectX::XMVectorNegate(normal));
                    physicsBones.frictionCache[biB].depth = std::max(physicsBones.frictionCache[biB].depth, currentDepth);
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
        TIMELOG_END;
    }

    void XPBDWorld::SolveCachedGroundCollisions(const float deltaTime)
    {
        if (collidersLeafs.empty() || GROUND_DETECT_RANGE <= FloatPrecision)
            return;
        // logger::info("{}", __func__);
        TIMELOG_START;
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
                    if (w <= FloatPrecision)
                        continue;

                    const Vector pos = physicsBones.pred[bi];

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
                    if (currentDepth <= FloatPrecision)
                        continue;

                    const Vector contactPoint = DirectX::XMVectorSubtract(worldCenter, DirectX::XMVectorScale(normal, projRadius));
                    const Vector rVector = DirectX::XMVectorSubtract(contactPoint, pos);

                    const Vector rXN = DirectX::XMVector3Cross(rVector, normal);
                    const float invInertia = physicsBones.invInertia[bi] * physicsBones.collisionRotationBias[bi];
                    const float wRot = DirectX::XMVectorGetX(DirectX::XMVector3LengthSq(rXN)) * invInertia;
                    const float wSum = w + wRot;

                    const float alphaProxy = physicsBones.collisionCompliance[bi] * invDtSq;
                    const float lambda = currentDepth * reciprocal(wSum + alphaProxy);
                    const float relaxedLambda = lambda * COL_CONVERGENCE;
                    const Vector correction = DirectX::XMVectorScale(normal, relaxedLambda * w);
                    physicsBones.pred[bi] = DirectX::XMVectorAdd(physicsBones.pred[bi], correction);

                    if (FloatPrecision < invInertia)
                    {
                        const Vector dTheta = DirectX::XMVectorScale(rXN, relaxedLambda * invInertia);
                        const Vector theta = DirectX::XMVectorSetW(dTheta, 0.0f);
                        const Vector dq = DirectX::XMVectorMultiply(DirectX::XMQuaternionMultiply(physicsBones.predRot[bi], theta), vHalf);
                        physicsBones.predRot[bi] = DirectX::XMQuaternionNormalize(DirectX::XMVectorAdd(physicsBones.predRot[bi], dq));
                    }

                    physicsBones.frictionCache[bi].n = DirectX::XMVectorAdd(physicsBones.frictionCache[bi].n, DirectX::XMVectorNegate(normal));
                    physicsBones.frictionCache[bi].depth = std::max(physicsBones.frictionCache[bi].depth, currentDepth);
                }
            },
            tbb::auto_partitioner()
        );
        TIMELOG_END;
    }

    void XPBDWorld::SolveConstraints(const float deltaTime, const bool initLambda)
    {
        if (constraintsGroup.empty() && angularConstraintsGroup.empty())
            return;
        // logger::info("{}", __func__);
        TIMELOG_START;
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
                                    if (invMass <= FloatPrecision)
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
                                        if (wSum <= FloatPrecision)
                                            continue;

                                        const Vector dir = DirectX::XMVectorSubtract(physicsBones.pred[bi], physicsBones.pred[abi]);
                                        const float distSq = DirectX::XMVectorGetX(DirectX::XMVector3LengthSq(dir));
                                        if (distSq < FloatPrecision)
                                            continue;

                                        const float invDist = rsqrt(distSq);
                                        const Vector normal = DirectX::XMVectorScale(dir, invDist);
                                        const float dist = distSq * invDist;

                                        const float C = dist - anchData.restLen;
                                        const bool isLessThanZero = C < 0.0f;
                                        const float currentComp = isLessThanZero ? anchData.complianceSquish : anchData.complianceStretch;
                                        const float currentLimit = isLessThanZero ? anchData.squishLimit : anchData.stretchLimit;
                                        const float currentDamping = isLessThanZero ? anchData.squishDamping : anchData.stretchDamping;
                                        float currentFactor = 1.0f;
                                        if (FloatPrecision < currentLimit)
                                        {
                                            const float ratio = std::min(std::abs(C) * reciprocal(currentLimit), 1.0f);
                                            const float ratioCubic = ratio * ratio * ratio;
                                            currentFactor = std::max(1.0f - ratioCubic, FloatPrecision);
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
                                        physicsBones.pred[bi] = DirectX::XMVectorAdd(physicsBones.pred[bi], DirectX::XMVectorScale(normal, correctionMagBi));
                                        if (FloatPrecision < anchInvMass)
                                        {
                                            const float correctionMagAbi = deltaLambda * anchInvMass;
                                            physicsBones.pred[abi] = DirectX::XMVectorSubtract(physicsBones.pred[abi], DirectX::XMVectorScale(normal, correctionMagAbi));
                                        }

                                        const float invWSum = reciprocal(wSum);
                                        if (FloatPrecision < currentDamping)
                                        {
                                            const Vector deltaBi = DirectX::XMVectorSubtract(physicsBones.pred[bi], physicsBones.pos[bi]);
                                            const Vector deltaAbi = DirectX::XMVectorSubtract(physicsBones.pred[abi], physicsBones.pos[abi]);

                                            const Vector relDelta = DirectX::XMVectorSubtract(deltaBi, deltaAbi);

                                            const float projDelta = DirectX::XMVectorGetX(DirectX::XMVector3Dot(relDelta, normal));
                                            const float dampLambda = (-projDelta * currentDamping) * invWSum;
                                            const Vector dampCorrection = DirectX::XMVectorScale(normal, dampLambda);

                                            physicsBones.pred[bi] = DirectX::XMVectorMultiplyAdd(dampCorrection, DirectX::XMVectorReplicate(invMass), physicsBones.pred[bi]);
                                            if (FloatPrecision < anchInvMass)
                                            {
                                                const Vector correctionA = DirectX::XMVectorScale(dampCorrection, anchInvMass);
                                                physicsBones.pred[abi] = DirectX::XMVectorSubtract(physicsBones.pred[abi], correctionA);
                                            }
                                        }

                                        const float limitAngle = anchData.angularLimit;
                                        if (FloatPrecision < limitAngle)
                                        {
                                            const Vector vLimitAngle = DirectX::XMVectorReplicate(limitAngle);
                                            Vector currentDir = DirectX::XMVectorSubtract(physicsBones.pred[bi], physicsBones.pred[abi]);
                                            if (DirectX::XMVector3LessOrEqual(DirectX::XMVector3Length(currentDir), vFloatPrecision))
                                                continue;

                                            const Vector currentDist = DirectX::XMVector3Length(currentDir);
                                            const Vector invCurrentDist = DirectX::XMVectorReciprocal(currentDist);
                                            currentDir = DirectX::XMVectorMultiply(currentDir, invCurrentDist);

                                            Vector restDirWorld = DirectX::XMVector3Rotate(anchData.restDirLocal, physicsBones.predRot[abi]);
                                            restDirWorld = DirectX::XMVector3Normalize(restDirWorld);

                                            Vector dot = DirectX::XMVector3Dot(currentDir, restDirWorld);
                                            dot = DirectX::XMVectorMin(dot, vOne);
                                            dot = DirectX::XMVectorMax(dot, vNegOne);
                                            const Vector currentAngle = DirectX::XMVectorACos(dot);
                                            if (DirectX::XMVector3LessOrEqual(currentAngle, vLimitAngle))
                                                continue;

                                            Vector axis = DirectX::XMVector3Cross(restDirWorld, currentDir);
                                            if (DirectX::XMVector3Less(vFloatPrecision, DirectX::XMVector3LengthSq(axis)))
                                            {
                                                axis = DirectX::XMVector3Normalize(axis);
                                                const Quaternion limitRot = DirectX::XMQuaternionRotationAxis(axis, limitAngle);
                                                const Vector maxAllowedDir = DirectX::XMVector3Rotate(restDirWorld, limitRot);
                                                const Vector targetPos = DirectX::XMVectorAdd(physicsBones.pred[abi], DirectX::XMVectorMultiply(maxAllowedDir, currentDist));
                                                const Vector angCorrection = DirectX::XMVectorSubtract(targetPos, physicsBones.pred[bi]);
                                                physicsBones.pred[bi] = DirectX::XMVectorAdd(physicsBones.pred[bi], DirectX::XMVectorScale(angCorrection, invMass * invWSum));
                                                if (FloatPrecision < anchInvMass)
                                                {
                                                    physicsBones.pred[abi] = DirectX::XMVectorSubtract(physicsBones.pred[abi], DirectX::XMVectorScale(angCorrection, anchInvMass * invWSum));
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
                                    const float invInertia = physicsBones.invInertia[bi];
                                    if (invInertia <= FloatPrecision)
                                        continue;
                                    const float invMass = physicsBones.invMass[bi];
                                    if (invMass <= FloatPrecision)
                                        continue;

                                    const std::uint32_t numAnchors = angularConstraints.numAnchors[ci];
                                    const std::uint32_t aciBase = ci * ANCHOR_MAX;
                                    for (std::uint32_t anchor = 0; anchor < numAnchors; ++anchor)
                                    {
                                        const std::uint32_t ai = aciBase + anchor;
                                        auto& anchData = angularConstraints.anchData[ai];
                                        const std::uint32_t& abi = anchData.anchIdx;
                                        if (abi == UINT32_MAX)
                                            continue;

                                        const float invInertiaA = physicsBones.invInertia[abi];
                                        const float wSum = invInertiaA + invInertia;
                                        if (wSum <= FloatPrecision)
                                            continue;

                                        const Quaternion target = DirectX::XMQuaternionMultiply(anchData.restRot, physicsBones.predRot[abi]);
                                        const Quaternion targetInv = DirectX::XMQuaternionConjugate(target);
                                        const Quaternion diff = DirectX::XMQuaternionMultiply(targetInv, physicsBones.predRot[bi]);
                                        Vector omega = DirectX::XMVectorGetW(diff) < 0.0f ? DirectX::XMVectorNegate(diff) : diff;
                                        omega = DirectX::XMVectorSetW(omega, 0.0f);
                                        const float CSq = DirectX::XMVectorGetX(DirectX::XMVector3LengthSq(omega));
                                        if (CSq <= FloatPrecision)
                                            continue;

                                        const float C = std::sqrt(CSq);
                                        const float currentComp = anchData.compliance;
                                        const float currentLimit = anchData.limit;
                                        float currentFactor = 1.0f;
                                        if (FloatPrecision < currentLimit)
                                        {
                                            const float ratio = std::min(std::abs(C) * reciprocal(currentLimit), 1.0f);
                                            const float ratioCubic = ratio * ratio * ratio;
                                            currentFactor = std::max(1.0f - ratioCubic, FloatPrecision);
                                        }
                                        const float alphaProxy = (currentComp * currentFactor) * invDtSq;
                                        const float denom = wSum + alphaProxy;
                                        float deltaLambda = 1.0f;
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

                                        const Vector correctionDir = DirectX::XMVectorMultiply(omega, DirectX::XMVectorReciprocal(DirectX::XMVectorReplicate(C)));
                                        {
                                            const Vector correction = DirectX::XMVectorScale(correctionDir, deltaLambda);
                                            if (invInertiaA > FloatPrecision)
                                            {
                                                const Vector thetaA = DirectX::XMVectorSetW(DirectX::XMVectorScale(correction, -invInertiaA), 0.0f);
                                                const Vector dqA = DirectX::XMVectorMultiply(DirectX::XMQuaternionMultiply(physicsBones.predRot[abi], thetaA), vHalf);
                                                physicsBones.predRot[abi] = DirectX::XMQuaternionNormalize(DirectX::XMVectorAdd(physicsBones.predRot[abi], dqA));
                                            }
                                            const Vector theta = DirectX::XMVectorSetW(DirectX::XMVectorScale(correction, invInertia), 0.0f);
                                            const Vector dq = DirectX::XMVectorMultiply(DirectX::XMQuaternionMultiply(physicsBones.predRot[bi], theta), vHalf);
                                            physicsBones.predRot[bi] = DirectX::XMQuaternionNormalize(DirectX::XMVectorAdd(physicsBones.predRot[bi], dq));
                                        }

                                        const float damping = anchData.damping;
                                        if (FloatPrecision < damping)
                                        {
                                            Quaternion dqBi = DirectX::XMQuaternionMultiply(physicsBones.predRot[bi], DirectX::XMQuaternionConjugate(physicsBones.rot[bi]));
                                            if (DirectX::XMVectorGetW(dqBi) < 0.0f)
                                                dqBi = DirectX::XMVectorNegate(dqBi);
                                            const Vector angVelBi = DirectX::XMVectorSetW(DirectX::XMVectorScale(dqBi, 2.0f), 0.0f);

                                            Quaternion dqAbi = DirectX::XMQuaternionMultiply(physicsBones.predRot[abi], DirectX::XMQuaternionConjugate(physicsBones.rot[abi]));
                                            if (DirectX::XMVectorGetW(dqAbi) < 0.0f)
                                                dqAbi = DirectX::XMVectorNegate(dqAbi);
                                            const Vector angVelAbi = DirectX::XMVectorSetW(DirectX::XMVectorScale(dqAbi, 2.0f), 0.0f);

                                            const Vector relAngVel = DirectX::XMVectorSubtract(angVelBi, angVelAbi);
                                            const float relAngVelDot = DirectX::XMVectorGetX(DirectX::XMVector3Dot(relAngVel, correctionDir));
                                            const float angDampLambda = (-relAngVelDot * damping) * reciprocal(wSum);
                                            const Vector dampCorrection = DirectX::XMVectorScale(correctionDir, angDampLambda);

                                            if (FloatPrecision < invInertiaA)
                                            {
                                                const Vector thetaA = DirectX::XMVectorSetW(DirectX::XMVectorScale(dampCorrection, -invInertiaA), 0.0f);
                                                const Vector dqA = DirectX::XMVectorMultiply(DirectX::XMQuaternionMultiply(physicsBones.predRot[abi], thetaA), vHalf);
                                                physicsBones.predRot[abi] = DirectX::XMQuaternionNormalize(DirectX::XMVectorAdd(physicsBones.predRot[abi], dqA));
                                            }
                                            const Vector theta = DirectX::XMVectorSetW(DirectX::XMVectorScale(dampCorrection, invInertia), 0.0f);
                                            const Vector dq = DirectX::XMVectorMultiply(DirectX::XMQuaternionMultiply(physicsBones.predRot[bi], theta), vHalf);
                                            physicsBones.predRot[bi] = DirectX::XMQuaternionNormalize(DirectX::XMVectorAdd(physicsBones.predRot[bi], dq));
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
        TIMELOG_END;
    }

    void XPBDWorld::SolveRestPoseConstraints(const float deltaTime, const bool initLambda)
    {
        if (physicsBonesGroup.empty())
            return;
        TIMELOG_START;
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

                        const float wBi = physicsBones.invMass[bi];
                        const float wPbi = physicsBones.invMass[pbi];
                        const float invInertiaBi = physicsBones.invInertia[bi];
                        const float invInertiaPbi = physicsBones.invInertia[pbi];
                        if (wBi + wPbi + invInertiaBi + invInertiaPbi <= FloatPrecision)
                            continue;

                        if (const float limit = physicsBones.restPoseLimit[bi]; FloatPrecision < limit && FloatPrecision < wBi + wPbi)
                        {
                            const float wSum = wBi + wPbi;

                            const Vector parentPos = physicsBones.pred[pbi];
                            const Quaternion parentRot = physicsBones.predRot[pbi];
                            const float parentScale = physicsBones.orgWorldScale[pbi];

                            const Vector localPos = physicsBones.orgLocalPos[bi];
                            const Vector scaledLocalPos = DirectX::XMVectorScale(localPos, parentScale);
                            const Vector rotatedLocalPos = DirectX::XMVector3Rotate(scaledLocalPos, parentRot);

                            const Vector targetRestPos = DirectX::XMVectorAdd(parentPos, rotatedLocalPos);
                            const Vector dir = DirectX::XMVectorSubtract(physicsBones.pred[bi], targetRestPos);
                            const Vector distSqVec = DirectX::XMVector3LengthSq(dir);
                            const float distSq = DirectX::XMVectorGetX(distSqVec);
                            if (FloatPrecision < distSq)
                            {
                                const float currentDist = std::sqrt(distSq);
                                const Vector normal = DirectX::XMVectorMultiply(dir, DirectX::XMVectorReciprocal(DirectX::XMVectorReplicate(currentDist)));
                                const float ratio = std::min(currentDist * reciprocal(limit), 1.0f);
                                const float ratioSq = ratio * ratio;
                                const float ratioQuartic = ratioSq * ratioSq;
                                const float ratioQuintic = ratioQuartic * ratioQuartic;
                                const float currentFactor = std::max(1.0f - ratioQuintic, FloatPrecision);

                                const float compliance = physicsBones.restPoseCompliance[bi];
                                const float alphaProxy = (compliance * currentFactor) * invDtSq;
                                const float denom = wSum + alphaProxy;

                                float deltaLambda = 0.0f;
                                if (initLambda)
                                {
                                    deltaLambda = -currentDist * reciprocal(denom);
                                    physicsBones.restPoseLambda[bi] = deltaLambda;
                                }
                                else
                                {
                                    deltaLambda = (-currentDist - (alphaProxy * physicsBones.restPoseLambda[bi])) * reciprocal(denom);
                                    physicsBones.restPoseLambda[bi] += deltaLambda;
                                }

                                const float correctionMagBi = deltaLambda * wBi;
                                physicsBones.pred[bi] = DirectX::XMVectorAdd(physicsBones.pred[bi], DirectX::XMVectorScale(normal, correctionMagBi));
                                if (FloatPrecision < wPbi)
                                {
                                    const float correctionMagPbi = deltaLambda * wPbi;
                                    physicsBones.pred[pbi] = DirectX::XMVectorSubtract(physicsBones.pred[pbi], DirectX::XMVectorScale(normal, correctionMagPbi));
                                }
                            }
                        }

                        if (const float limit = physicsBones.restPoseAngularLimit[bi]; 
                            FloatPrecision < limit && FloatPrecision < invInertiaBi + invInertiaPbi)
                        {
                            const float wSum = invInertiaBi + invInertiaPbi;

                            const Quaternion parentRot = physicsBones.predRot[pbi];
                            const Quaternion localRestRot = physicsBones.orgLocalRot[bi];
                            const Quaternion targetRestRot = DirectX::XMQuaternionMultiply(localRestRot, parentRot);

                            const Quaternion targetInv = DirectX::XMQuaternionConjugate(targetRestRot);
                            const Quaternion diff = DirectX::XMQuaternionMultiply(targetInv, physicsBones.predRot[bi]);

                            Vector omega = DirectX::XMVectorGetW(diff) < 0.0f ? DirectX::XMVectorNegate(diff) : diff;
                            omega = DirectX::XMVectorSetW(omega, 0.0f);
                            const float CSq = DirectX::XMVectorGetX(DirectX::XMVector3LengthSq(omega));
                            if (FloatPrecision < CSq)
                            {
                                const float C = std::sqrt(CSq);
                                const float ratio = std::min(C * reciprocal(limit), 1.0f);
                                const float ratioSq = ratio * ratio;
                                const float ratioQuartic = ratioSq * ratioSq;
                                const float ratioQuintic = ratioQuartic * ratioQuartic;
                                const float currentFactor = std::max(1.0f - ratioQuintic, 0.001f);

                                const float compliance = physicsBones.restPoseAngularCompliance[bi];
                                const float alphaProxy = (compliance * currentFactor) * invDtSq;
                                const float denom = wSum + alphaProxy;

                                float deltaLambda = 0.0f;
                                if (initLambda)
                                {
                                    deltaLambda = -C * reciprocal(denom);
                                    physicsBones.restPoseAngularLambda[bi] = deltaLambda;
                                }
                                else
                                {
                                    deltaLambda = (-C - (alphaProxy * physicsBones.restPoseAngularLambda[bi])) * reciprocal(denom);
                                    physicsBones.restPoseAngularLambda[bi] += deltaLambda;
                                }

                                const Vector correctionDir = DirectX::XMVectorMultiply(omega, DirectX::XMVectorReciprocal(DirectX::XMVectorReplicate(C)));
                                const Vector correction = DirectX::XMVectorScale(correctionDir, deltaLambda);

                                if (invInertiaBi > FloatPrecision)
                                {
                                    const Vector theta = DirectX::XMVectorSetW(DirectX::XMVectorScale(correction, invInertiaBi), 0.0f);
                                    const Vector dq = DirectX::XMVectorMultiply(DirectX::XMQuaternionMultiply(physicsBones.predRot[bi], theta), vHalf);
                                    physicsBones.predRot[bi] = DirectX::XMQuaternionNormalize(DirectX::XMVectorAdd(physicsBones.predRot[bi], dq));
                                }

                                if (invInertiaPbi > FloatPrecision)
                                {
                                    const Vector thetaP = DirectX::XMVectorSetW(DirectX::XMVectorScale(correction, -invInertiaPbi), 0.0f);
                                    const Vector dqP = DirectX::XMVectorMultiply(DirectX::XMQuaternionMultiply(physicsBones.predRot[pbi], thetaP), vHalf);
                                    physicsBones.predRot[pbi] = DirectX::XMQuaternionNormalize(DirectX::XMVectorAdd(physicsBones.predRot[pbi], dqP));
                                }
                            }
                        }
                    }
                }
            },
            tbb::auto_partitioner());
        TIMELOG_END;
    }

    void XPBDWorld::UpdateBoneVelocity(const float deltaTime)
    {
        // logger::info("{}", __func__);
        TIMELOG_START;
        const Vector invDt = DirectX::XMVectorReciprocal(DirectX::XMVectorReplicate(deltaTime));
        const Vector dbInvDt = DirectX::XMVectorScale(invDt, 2.0f);
        const float gravity = DirectX::XMVectorGetX(DirectX::XMVector3Length(SkyrimGravity));
        const Vector bounceThreshold = DirectX::XMVectorReplicate(-gravity * deltaTime * 2.0f);
        tbb::parallel_for(
            tbb::blocked_range<std::uint32_t>(0, physicsBones.numBones, 128),
            [&](const tbb::blocked_range<std::uint32_t>& r) {
                for (std::uint32_t bi = r.begin(); bi != r.end(); ++bi)
                {
                    const std::uint32_t oi = physicsBones.objIdx[bi];
                    if (IsDisable(oi))
                        continue;
                    if (physicsBones.invMass[bi] <= FloatPrecision)
                        continue;

                    physicsBones.prevPos[bi] = physicsBones.pos[bi];
                    physicsBones.prevRot[bi] = physicsBones.rot[bi];
                    const Vector dampingFactor = DirectX::XMVectorReplicate(1.0f - physicsBones.damping[bi]);
                    const Vector deltaPos = DirectX::XMVectorSubtract(physicsBones.pred[bi], physicsBones.pos[bi]);
                    Vector vel = DirectX::XMVectorMultiply(deltaPos, DirectX::XMVectorMultiply(invDt, dampingFactor));

                    const auto& frictionCache = physicsBones.frictionCache[bi];
                    const float depth = frictionCache.depth;
                    const Vector normalImpulse = DirectX::XMVectorScale(invDt, depth);
                    const Vector friction = DirectX::XMVectorReplicate(physicsBones.collisionFriction[bi]);
                    const bool hasCollision = FloatPrecision < depth;
                    if (hasCollision)
                    {
                        const Vector n = DirectX::XMVector3Normalize(frictionCache.n);
                        const Vector v_n_mag = DirectX::XMVector3Dot(vel, n);
                        const Vector v_n = DirectX::XMVectorMultiply(n, v_n_mag);
                        const Vector v_t = DirectX::XMVectorSubtract(vel, v_n);

                        const Vector preVel = physicsBones.vel[bi];
                        const Vector v_pre_n_mag = DirectX::XMVector3Dot(preVel, n);

                        const Vector restitution = DirectX::XMVectorReplicate(physicsBones.restitution[bi]);
                        Vector bounceMag = DirectX::XMVectorMultiply(DirectX::XMVectorNegate(v_pre_n_mag), restitution);

                        const Vector bounceMask = DirectX::XMVectorLess(v_pre_n_mag, bounceThreshold);
                        bounceMag = DirectX::XMVectorSelect(vZero, bounceMag, bounceMask);

                        const Vector v_n_clipped = DirectX::XMVectorMax(v_n_mag, bounceMag);
                        const Vector v_n_final = DirectX::XMVectorMultiply(n, v_n_clipped);
                        const Vector v_t_lenSq = DirectX::XMVector3LengthSq(v_t);
                        if (DirectX::XMVector3LessOrEqual(v_t_lenSq, vFloatPrecision))
                            vel = v_n_final;
                        else
                        {
                            const Vector frictionDrop = DirectX::XMVectorMultiply(friction, normalImpulse);
                            const Vector vt_mag = DirectX::XMVectorSqrt(v_t_lenSq);
                            const Vector scale = DirectX::XMVectorMax(vZero, DirectX::XMVectorMultiply(DirectX::XMVectorSubtract(vt_mag, frictionDrop), DirectX::XMVectorReciprocal(vt_mag)));
                            const Vector v_t_new = DirectX::XMVectorMultiply(v_t, scale);
                            vel = DirectX::XMVectorAdd(v_t_new, v_n_final);
                        }
                    }
                    physicsBones.vel[bi] = vel;
                    physicsBones.frictionCache[bi] = {};

                    physicsBones.pos[bi] = physicsBones.pred[bi];
                    if (physicsBones.advancedRotation[bi])
                    {
                        Vector dq = DirectX::XMQuaternionMultiply(DirectX::XMQuaternionConjugate(physicsBones.rot[bi]), physicsBones.predRot[bi]);
                        if (DirectX::XMVectorGetW(dq) < 0.0f)
                            dq = DirectX::XMVectorNegate(dq);

                        const Vector vTwo = DirectX::XMVectorReplicate(2.0f);
                        const Vector angVelMultiplier = DirectX::XMVectorMultiply(vTwo, DirectX::XMVectorMultiply(invDt, dampingFactor));
                        physicsBones.angVel[bi] = DirectX::XMVectorSetW(DirectX::XMVectorMultiply(dq, angVelMultiplier), 0.0f);
                        physicsBones.rot[bi] = physicsBones.predRot[bi];

                        if (hasCollision && FloatPrecision <= physicsBones.collisionRotationBias[bi])
                        {
                            const Vector v_angVel_lenSq = DirectX::XMVector3LengthSq(physicsBones.angVel[bi]);
                            if (DirectX::XMVector3LessOrEqual(v_angVel_lenSq, vFloatPrecision))
                                physicsBones.angVel[bi] = vZero;
                            else
                            {
                                const Vector angVel_mag = DirectX::XMVectorSqrt(v_angVel_lenSq);
                                const Vector rotBias = DirectX::XMVectorReplicate(physicsBones.collisionRotationBias[bi]);
                                const Vector rotFrictionDrop = DirectX::XMVectorMultiply(DirectX::XMVectorMultiply(friction, normalImpulse), rotBias);
                                const Vector rotScale = DirectX::XMVectorMax(vZero, DirectX::XMVectorMultiply(DirectX::XMVectorSubtract(angVel_mag, rotFrictionDrop), DirectX::XMVectorReciprocal(angVel_mag)));
                                physicsBones.angVel[bi] = DirectX::XMVectorMultiply(physicsBones.angVel[bi], rotScale);
                            }
                        }

                        const Quaternion invRot = DirectX::XMQuaternionConjugate(physicsBones.predRot[bi]);
                        const Vector localVel = DirectX::XMVector3Rotate(physicsBones.vel[bi], invRot);
                        const Vector localAcc = DirectX::XMVector3Rotate(objectDatas.acceleration[oi], invRot);

                        const Vector localTorque = DirectX::XMVector3TransformNormal(localVel, physicsBones.linearRotTorque[bi]);
                        const Vector torqueLenSq = DirectX::XMVector3LengthSq(localTorque);
                        if (DirectX::XMVector3Less(vFloatPrecision, torqueLenSq))
                        {
                            const Vector worldTorque = DirectX::XMVector3Rotate(localTorque, physicsBones.predRot[bi]);
                            physicsBones.angVel[bi] = DirectX::XMVectorAdd(physicsBones.angVel[bi], worldTorque);
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
        TIMELOG_END;
    }

    void XPBDWorld::ApplyToSkyrim()
    {
        if (physicsBonesGroup.empty())
            return;
        // logger::info("{}", __func__);
        TIMELOG_START;
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
                    for (std::uint32_t bi = begin; bi < end; ++bi)
                    {
                        if (physicsBones.invMass[bi] <= FloatPrecision)
                            continue;

                        auto& node = physicsBones.node[bi];
                        if (!node || !node->parent)
                            continue;

                        const RE::NiPoint3 parentWorldPos = node->parent->world.translate;
                        const RE::NiMatrix3 parentWorldRot = node->parent->world.rotate;
                        const Quaternion qParentWorldRot = ToQuaternion(parentWorldRot);
                        const float parentWorldScale = node->parent->world.scale;

                        const Vector renderPos = DirectX::XMVectorLerp(physicsBones.prevPos[bi], physicsBones.pos[bi], alpha);
                        const RE::NiPoint3 physicsWorldPos = ToPoint3(renderPos);

                        const Quaternion origWorldRot = DirectX::XMQuaternionMultiply(physicsBones.orgLocalRot[bi], qParentWorldRot);
                        RE::NiMatrix3 finalWorldRot;
                        if (physicsBones.advancedRotation[bi])
                        {
                            const Quaternion renderRot = DirectX::XMQuaternionNormalize(DirectX::XMQuaternionSlerp(physicsBones.prevRot[bi], physicsBones.rot[bi], alpha));
                            finalWorldRot = ToMatrix(renderRot);
                        }
                        else
                        {
                            Vector restDir = DirectX::XMVector3Rotate(physicsBones.orgLocalPos[bi], qParentWorldRot);
                            if (DirectX::XMVector3Less(DirectX::XMVector3LengthSq(restDir), vFloatPrecision))
                                restDir = DirectX::XMQuaternionMultiply(DirectX::XMVector3Rotate(vYone, physicsBones.orgLocalRot[bi]), qParentWorldRot);
                            restDir = DirectX::XMVector3Normalize(restDir);

                            Vector currentDir = DirectX::XMVectorSubtract(ToVector(physicsWorldPos), ToVector(parentWorldPos));
                            if (DirectX::XMVector3Less(DirectX::XMVector3LengthSq(currentDir), vFloatPrecision))
                                currentDir = restDir;
                            else
                                currentDir = DirectX::XMVector3Normalize(currentDir);

                            const float dot = DirectX::XMVectorGetX(DirectX::XMVector3Dot(restDir, currentDir));
                            Vector target;
                            if (dot < -0.9999f)
                            {
                                Vector vUp = vYone;
                                const float dotUp = DirectX::XMVectorGetX(DirectX::XMVector3Dot(restDir, vUp));
                                if (dotUp < -0.99f)
                                    vUp = vXone;

                                const Vector axis = DirectX::XMVector3Normalize(DirectX::XMVector3Cross(vUp, restDir));
                                target = DirectX::XMQuaternionMultiply(origWorldRot, DirectX::XMQuaternionRotationAxis(axis, DirectX::XM_PI));
                            }
                            else
                            {
                                const Vector rotationAxis = DirectX::XMVector3Cross(restDir, currentDir);
                                const Vector lookAt = DirectX::XMQuaternionNormalize(DirectX::XMVectorSetW(rotationAxis, dot + 1.0f));
                                target = DirectX::XMQuaternionNormalize(DirectX::XMQuaternionMultiply(origWorldRot, lookAt));
                            }
                            const Vector final = DirectX::XMQuaternionNormalize(DirectX::XMQuaternionSlerp(origWorldRot, target, physicsBones.rotationBlendFactor[bi]));

                            finalWorldRot = ToMatrix(final);
                            physicsBones.rot[bi] = final;
                        }

                        const RE::NiPoint3 diff = physicsWorldPos - parentWorldPos;
                        const RE::NiPoint3 localPos = (parentWorldRot.Transpose() * diff) * reciprocal(parentWorldScale);
                        const RE::NiMatrix3 localRot = parentWorldRot.Transpose() * finalWorldRot;

                        // prevents memory write skips by compiler optimization
                        std::memcpy(&node->local.translate, &localPos, sizeof(localPos));
                        std::memcpy(&node->local.rotate, &localRot, sizeof(localRot));

                        std::memcpy(&node->world.translate, &physicsWorldPos, sizeof(physicsWorldPos));
                        std::memcpy(&node->world.rotate, &finalWorldRot, sizeof(finalWorldRot));
                    }
                }
            },
            tbb::auto_partitioner()
        );
        TIMELOG_END;
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

                    AABB worldAABB = colliders.boundingAABB[ci].GetWorldAABB(physicsBones.pred[bi], physicsBones.predRot[bi], physicsBones.orgWorldScale[bi]);
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
                    AABB boneAABB(physicsBones.pred[bi], physicsBones.pred[bi]);
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

        const Vector posA = physicsBones.pred[biA];
        const Quaternion rotA = physicsBones.predRot[biA];
        const float scaleA = physicsBones.orgWorldScale[biA];

        const Vector posB = physicsBones.pred[biB];
        const Quaternion rotB = physicsBones.predRot[biB];
        const float scaleB = physicsBones.orgWorldScale[biB];

        const float marginA = physicsBones.collisionMargin[biA];
        const float marginB = physicsBones.collisionMargin[biB];
        const float sumMargin = marginA + marginB;

        const float rA = colliders.boundingSphere[coiA] * scaleA + marginA;
        const float rB = colliders.boundingSphere[coiB] * scaleB + marginB;
        const float sumR = rA + rB;

        AABB aWorldAABB = colliders.boundingAABB[coiA].GetWorldAABB(posA, rotA, scaleA);
        AABB bWorldAABB = colliders.boundingAABB[coiB].GetWorldAABB(posB, rotB, scaleB);
        aWorldAABB.Fatten(marginA);
        bWorldAABB.Fatten(marginB);
        if (!aWorldAABB.Overlaps(bWorldAABB))
            return false;

        const Vector centerA = aWorldAABB.GetCenter();
        const Vector centerB = bWorldAABB.GetCenter();
        const Vector centerToCenter = DirectX::XMVectorSubtract(centerB, centerA);

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
            if (DirectX::XMVector3Less(lenSqV, vFloatPrecision))
                return true;

            const Vector axis = DirectX::XMVector3Normalize(inAxis);
            for (std::uint32_t i = 0; i < histCount; ++i)
            {
                const Vector dot = DirectX::XMVectorAbs(DirectX::XMVector3Dot(axis, hist[i]));
                if (DirectX::XMVector3Less(vAxisSimilarityLimit, dot))
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
            if (DirectX::XMVector3LessOrEqual(DirectX::XMVector3Dot(nA, centerToCenter), vFloatPrecision))
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
            if (DirectX::XMVector3LessOrEqual(DirectX::XMVector3LengthSq(wEdgeB[i]), vFloatPrecision))
                continue;
            wEdgeBValid[i] = true;
        }

        const std::uint32_t eCountA = hullA.edgeCount;
        for (std::uint32_t eiA = 0; eiA < eCountA; ++eiA)
        {
            const Vector eA = DirectX::XMVectorSet(hullA.eX[eiA], hullA.eY[eiA], hullA.eZ[eiA], 0);
            const Vector wA = DirectX::XMVector3Rotate(eA, rotA);

            if (DirectX::XMVector3LessOrEqual(DirectX::XMVector3LengthSq(wA), vFloatPrecision))
                continue;

            for (std::uint32_t eiB = 0; eiB < eCountB; ++eiB)
            {
                if (!wEdgeBValid[eiB])
                    continue;
                const Vector crossAxis = DirectX::XMVector3Cross(wA, wEdgeB[eiB]);
                if (DirectX::XMVector3LessOrEqual(DirectX::XMVector3LengthSq(crossAxis), vFloatPrecision))
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
            if (DirectX::XMVector3Less(vBreakThresholdSq, driftSq))
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

        const DirectX::XMMATRIX matA = DirectX::XMMatrixRotationQuaternion(rotA);
        const DirectX::XMMATRIX matB = DirectX::XMMatrixRotationQuaternion(rotB);
        const DirectX::XMMATRIX matInvA = DirectX::XMMatrixRotationQuaternion(invRotA);
        const DirectX::XMMATRIX matInvB = DirectX::XMMatrixRotationQuaternion(invRotB);

        alignas(16) DirectX::XMFLOAT4X4A fMatA, fMatB, fMatInvA, fMatInvB;
        DirectX::XMStoreFloat4x4A(&fMatA, matA);
        DirectX::XMStoreFloat4x4A(&fMatB, matB);
        DirectX::XMStoreFloat4x4A(&fMatInvA, matInvA);
        DirectX::XMStoreFloat4x4A(&fMatInvB, matInvB);

        const float invScaleA_f = reciprocal(scaleA);
        const float invScaleB_f = reciprocal(scaleB);

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
            const Vector lB = DirectX::XMVectorScale(DirectX::XMVector3TransformNormal(diffB, matInvB), invScaleB_f);

            AddTempPoint(lA, lB, bestPenA);
        }
        if (bestIdxB != -1 && bestPenB > 0.0f)
        {
            const Vector lB = DirectX::XMVectorSet(hullB.vX[bestIdxB], hullB.vY[bestIdxB], hullB.vZ[bestIdxB], 0.0f);
            const Vector rot_lB = DirectX::XMVector3TransformNormal(lB, matB);
            const Vector worldB_i = DirectX::XMVectorAdd(posB, DirectX::XMVectorScale(rot_lB, scaleB));

            const Vector wA = DirectX::XMVectorSubtract(worldB_i, DirectX::XMVectorScale(normal, bestPenB));
            const Vector diffA = DirectX::XMVectorSubtract(wA, posA);
            const Vector lA = DirectX::XMVectorScale(DirectX::XMVector3TransformNormal(diffA, matInvA), invScaleA_f);

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
            const Vector lB = DirectX::XMVectorScale(DirectX::XMVector3TransformNormal(diffB, matInvB), invScaleB_f);
            AddTempPoint(lA, lB, bestPenA);
        }
        if (bestIdxB != -1 && bestPenB > 0.0f)
        {
            const Vector lB = DirectX::XMVectorSet(hullB.vX[bestIdxB], hullB.vY[bestIdxB], hullB.vZ[bestIdxB], 0.0f);
            const Vector rot_lB = DirectX::XMVector3TransformNormal(lB, matB);
            const Vector worldB_i = DirectX::XMVectorAdd(posB, DirectX::XMVectorScale(rot_lB, scaleB));
            const Vector wA = DirectX::XMVectorSubtract(worldB_i, DirectX::XMVectorScale(normal, bestPenB));
            const Vector diffA = DirectX::XMVectorSubtract(wA, posA);
            const Vector lA = DirectX::XMVectorScale(DirectX::XMVector3TransformNormal(diffA, matInvA), invScaleA_f);
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
            const Vector lB = DirectX::XMVectorScale(DirectX::XMVector3TransformNormal(diffB, matInvB), invScaleB_f);
            AddTempPoint(lA, lB, bestPenA);
        }
        if (bestIdxB != -1 && bestPenB > 0.0f)
        {
            const Vector lB = DirectX::XMVectorSet(hullB.vX[bestIdxB], hullB.vY[bestIdxB], hullB.vZ[bestIdxB], 0.0f);
            const Vector rot_lB = DirectX::XMVector3TransformNormal(lB, matB);
            const Vector worldB_i = DirectX::XMVectorAdd(posB, DirectX::XMVectorScale(rot_lB, scaleB));
            const Vector wA = DirectX::XMVectorSubtract(worldB_i, DirectX::XMVectorScale(normal, bestPenB));
            const Vector diffA = DirectX::XMVectorSubtract(wA, posA);
            const Vector lA = DirectX::XMVectorScale(DirectX::XMVector3TransformNormal(diffA, matInvA), invScaleA_f);
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
            Vector3 maxDistSq = vNegOne;
            Vector3 maxTriAreaSq = vNegOne;
            Vector3 maxQuadAreaSq = vNegOne;

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
                    RE::NiAVObject* npcObj = object->loadedData->data3D->GetObjectByName("NPC");
                    objectDatas.npcNode[objIdx] = npcObj ? npcObj->AsNode() : nullptr;
                    objectDatas.prevNPCWorldRot[objIdx] = objectDatas.npcNode[objIdx] ? ToQuaternion(objectDatas.npcNode[objIdx]->world.rotate) : ToQuaternion(RE::NiMatrix3());
                    objectDatas.targetNPCWorldRot[objIdx] = objectDatas.prevNPCWorldRot[objIdx];
                }
                else
                {
                    objectDatas.prevWorldPos[objIdx] = ToVector(object->GetPosition());
                    objectDatas.npcNode[objIdx] = nullptr;
                    objectDatas.prevNPCWorldRot[objIdx] = ToQuaternion(RE::NiMatrix3());
                    objectDatas.targetNPCWorldRot[objIdx] = objectDatas.prevNPCWorldRot[objIdx];
                }
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
                        objectDatas.npcNode.push_back(npcObj->AsNode());
                        objectDatas.prevNPCWorldRot.push_back(ToQuaternion(npcObj->world.rotate));
                        objectDatas.targetNPCWorldRot.push_back(ToQuaternion(npcObj->world.rotate));
                        hasNPCNode = true;
                    }
                }
                if (!hasNPCNode)
                {
                    objectDatas.prevWorldPos.push_back(ToVector(object->GetPosition()));
                    objectDatas.npcNode.push_back(nullptr);
                    objectDatas.prevNPCWorldRot.push_back(ToQuaternion(RE::NiMatrix3()));
                    objectDatas.targetNPCWorldRot.push_back(ToQuaternion(RE::NiMatrix3()));
                }
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
        physicsBones.pred.push_back(vZero);
        physicsBones.vel.push_back(vZero);
        
        physicsBones.advancedRotation.push_back(0);
        physicsBones.rot.push_back(vZero);
        physicsBones.prevRot.push_back(vZero);
        physicsBones.predRot.push_back(vZero);
        physicsBones.backupRot.push_back(vZero);
        physicsBones.angVel.push_back(vZero);
        physicsBones.invInertia.push_back(0);

        physicsBones.damping.push_back(0);
        physicsBones.inertiaScale.push_back(0);
        physicsBones.restitution.push_back(0);
        physicsBones.rotationBlendFactor.push_back(0);
        physicsBones.gravity.push_back(ToVector(GetSkyrimGravity(1.0f)));
        physicsBones.offset.push_back(vZero);
        physicsBones.invMass.push_back(0);
        physicsBones.windFactor.push_back(0);

        physicsBones.restPoseLimit.push_back(0);
        physicsBones.restPoseCompliance.push_back(0);
        physicsBones.restPoseLambda.push_back(0);
        physicsBones.restPoseAngularLimit.push_back(0);
        physicsBones.restPoseAngularCompliance.push_back(0);
        physicsBones.restPoseAngularLambda.push_back(0);

        physicsBones.linearRotTorque.push_back(vmZero);

        physicsBones.collisionMargin.push_back(0);
        physicsBones.collisionShrink.push_back(0);
        physicsBones.collisionFriction.push_back(0);
        physicsBones.collisionRotationBias.push_back(0);
        physicsBones.collisionCompliance.push_back(0);

        physicsBones.layerGroup.push_back(0);
        physicsBones.collideLayer.push_back(0);

        physicsBones.collisionCache.push_back(PhysicsBones::CollisionCache());
        physicsBones.frictionCache.push_back(PhysicsBones::FrictionCache());

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
        physicsBones.prevNodeWorldRot.push_back(vZero);
        physicsBones.targetNodeWorldRot.push_back(vZero);

        physicsBones.orgWorldScale.push_back(1);
        physicsBones.orgLocalPos.push_back(vZero);
        physicsBones.orgLocalRot.push_back(vZero);
        return newIdx;
    }
    void XPBDWorld::ReserveBone(std::uint32_t n)
    {
        if (n == 0)
            return;
        n += physicsBones.numBones;
        physicsBones.pos.reserve(n);
        physicsBones.prevPos.reserve(n);
        physicsBones.pred.reserve(n);
        physicsBones.vel.reserve(n);

        physicsBones.advancedRotation.reserve(n);
        physicsBones.rot.reserve(n);
        physicsBones.prevRot.reserve(n);
        physicsBones.predRot.reserve(n);
        physicsBones.backupRot.reserve(n);
        physicsBones.angVel.reserve(n);
        physicsBones.invInertia.reserve(n);

        physicsBones.damping.reserve(n);
        physicsBones.inertiaScale.reserve(n);
        physicsBones.restitution.reserve(n);
        physicsBones.rotationBlendFactor.reserve(n);
        physicsBones.gravity.reserve(n);
        physicsBones.offset.reserve(n);
        physicsBones.invMass.reserve(n);
        physicsBones.windFactor.reserve(n);

        physicsBones.restPoseLimit.reserve(n);
        physicsBones.restPoseCompliance.reserve(n);
        physicsBones.restPoseLambda.reserve(n);
        physicsBones.restPoseAngularLimit.reserve(n);
        physicsBones.restPoseAngularCompliance.reserve(n);
        physicsBones.restPoseAngularLambda.reserve(n);

        physicsBones.linearRotTorque.reserve(n);

        physicsBones.collisionMargin.reserve(n);
        physicsBones.collisionShrink.reserve(n);
        physicsBones.collisionFriction.reserve(n);
        physicsBones.collisionRotationBias.reserve(n);
        physicsBones.collisionCompliance.reserve(n);

        physicsBones.layerGroup.reserve(n);
        physicsBones.collideLayer.reserve(n);

        physicsBones.collisionCache.reserve(n);
        physicsBones.frictionCache.reserve(n);

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

        colliders.convexHullData.push_back(ConvexHullDataBatch());
        colliders.boundingAABB.push_back(AABB());

        colliders.boundingSphere.push_back(0);
        colliders.boundingSphereCenter.push_back(vZero);
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

        colliders.convexHullData.reserve(n);
        colliders.boundingAABB.reserve(n);

        colliders.boundingSphere.reserve(n);
        colliders.boundingSphereCenter.reserve(n);
    }

    void XPBDWorld::ReorderMaps()
    {
        logger::info("Reorder maps...");

        threadPool->Execute([&] {
            { // PhysicsBones
                std::vector<std::uint32_t> oldToNewBoneIdx(physicsBones.numBones, UINT32_MAX);
                physicsBonesOrder.resize(physicsBones.numBones);
                std::iota(physicsBonesOrder.begin(), physicsBonesOrder.end(), 0);
                std::ranges::sort(physicsBonesOrder, [&](std::uint32_t a, std::uint32_t b) {
                    if ((physicsBones.node[a] == nullptr && physicsBones.particleName[a].empty()) != (physicsBones.node[b] == nullptr && physicsBones.particleName[b].empty()))
                    {
                        return (physicsBones.node[a] != nullptr || !physicsBones.particleName[a].empty()) > (physicsBones.node[b] != nullptr || !physicsBones.particleName[b].empty());
                    }
                    const std::uint32_t objIdxA = physicsBones.objIdx[a];
                    const std::uint32_t objIdxB = physicsBones.objIdx[b];
                    if (objIdxA != objIdxB)
                        return objIdxA < objIdxB;
                    if (physicsBones.rootIdx[a] != physicsBones.rootIdx[b])
                        return physicsBones.rootIdx[a] < physicsBones.rootIdx[b];
                    if (physicsBones.depth[a] != physicsBones.depth[b])
                        return physicsBones.depth[a] < physicsBones.depth[b];
                    const bool isParticleA = !physicsBones.particleName[a].empty();
                    const bool isParticleB = !physicsBones.particleName[b].empty();
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
                    const auto& oi = physicsBonesOrder[bi];
                    if ((physicsBones.node[oi] == nullptr && physicsBones.particleName[oi].empty()) ||
                        physicsBones.objIdx[oi] == UINT32_MAX ||
                        physicsBones.rootIdx[oi] == UINT32_MAX)
                        break;
                    if (currentObjIdx != physicsBones.objIdx[oi])
                    {
                        currentObjIdx = physicsBones.objIdx[oi];
                        physicsBonesGroup.push_back(bi);
                    }
                    if (currentRootIdx != physicsBones.rootIdx[oi])
                    {
                        currentRootIdx = physicsBones.rootIdx[oi];
                        physicsBonesRoots.push_back(bi);
                    }
                    oldToNewBoneIdx[oi] = bi;
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
                                physicsBones.pred[i] = tmpPhysicsBones.pred[srcIdx];
                                physicsBones.vel[i] = tmpPhysicsBones.vel[srcIdx];

                                physicsBones.advancedRotation[i] = tmpPhysicsBones.advancedRotation[srcIdx];
                                physicsBones.rot[i] = tmpPhysicsBones.rot[srcIdx];
                                physicsBones.predRot[i] = tmpPhysicsBones.predRot[srcIdx];
                                physicsBones.backupRot[i] = tmpPhysicsBones.backupRot[srcIdx];
                                physicsBones.angVel[i] = tmpPhysicsBones.angVel[srcIdx];
                                physicsBones.invInertia[i] = tmpPhysicsBones.invInertia[srcIdx];

                                physicsBones.damping[i] = tmpPhysicsBones.damping[srcIdx];
                                physicsBones.inertiaScale[i] = tmpPhysicsBones.inertiaScale[srcIdx];
                                physicsBones.restitution[i] = tmpPhysicsBones.restitution[srcIdx];
                                physicsBones.rotationBlendFactor[i] = tmpPhysicsBones.rotationBlendFactor[srcIdx];
                                physicsBones.gravity[i] = tmpPhysicsBones.gravity[srcIdx];
                                physicsBones.offset[i] = tmpPhysicsBones.offset[srcIdx];
                                physicsBones.invMass[i] = tmpPhysicsBones.invMass[srcIdx];
                                physicsBones.windFactor[i] = tmpPhysicsBones.windFactor[srcIdx];

                                physicsBones.restPoseLimit[i] = tmpPhysicsBones.restPoseLimit[srcIdx];
                                physicsBones.restPoseCompliance[i] = tmpPhysicsBones.restPoseCompliance[srcIdx];
                                physicsBones.restPoseLambda[i] = tmpPhysicsBones.restPoseLambda[srcIdx];
                                physicsBones.restPoseAngularLimit[i] = tmpPhysicsBones.restPoseAngularLimit[srcIdx];
                                physicsBones.restPoseAngularCompliance[i] = tmpPhysicsBones.restPoseAngularCompliance[srcIdx];
                                physicsBones.restPoseAngularLambda[i] = tmpPhysicsBones.restPoseAngularLambda[srcIdx];

                                physicsBones.linearRotTorque[i] = tmpPhysicsBones.linearRotTorque[srcIdx];

                                physicsBones.collisionMargin[i] = tmpPhysicsBones.collisionMargin[srcIdx];
                                physicsBones.collisionShrink[i] = tmpPhysicsBones.collisionShrink[srcIdx];
                                physicsBones.collisionFriction[i] = tmpPhysicsBones.collisionFriction[srcIdx];
                                physicsBones.collisionRotationBias[i] = tmpPhysicsBones.collisionRotationBias[srcIdx];
                                physicsBones.collisionCompliance[i] = tmpPhysicsBones.collisionCompliance[srcIdx];

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
                    physicsBones.pred.resize(validCount);
                    physicsBones.vel.resize(validCount);

                    physicsBones.advancedRotation.resize(validCount);
                    physicsBones.rot.resize(validCount);
                    physicsBones.predRot.resize(validCount);
                    physicsBones.angVel.resize(validCount);
                    physicsBones.invInertia.resize(validCount);

                    physicsBones.damping.resize(validCount);
                    physicsBones.inertiaScale.resize(validCount);
                    physicsBones.restitution.resize(validCount);
                    physicsBones.rotationBlendFactor.resize(validCount);
                    physicsBones.gravity.resize(validCount);
                    physicsBones.offset.resize(validCount);
                    physicsBones.invMass.resize(validCount);
                    physicsBones.windFactor.resize(validCount);

                    physicsBones.restPoseLimit.resize(validCount);
                    physicsBones.restPoseCompliance.resize(validCount);
                    physicsBones.restPoseLambda.resize(validCount);
                    physicsBones.restPoseAngularLimit.resize(validCount);
                    physicsBones.restPoseAngularCompliance.resize(validCount);
                    physicsBones.restPoseAngularLambda.resize(validCount);

                    physicsBones.linearRotTorque.resize(validCount);

                    physicsBones.collisionMargin.resize(validCount);
                    physicsBones.collisionShrink.resize(validCount);
                    physicsBones.collisionFriction.resize(validCount);
                    physicsBones.collisionRotationBias.resize(validCount);
                    physicsBones.collisionCompliance.resize(validCount);

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

                                    colliders.convexHullData[i] = tmpColliders.convexHullData[srcIdx];
                                    colliders.boundingAABB[i] = tmpColliders.boundingAABB[srcIdx];

                                    colliders.boundingSphere[i] = tmpColliders.boundingSphere[srcIdx];
                                    colliders.boundingSphereCenter[i] = tmpColliders.boundingSphereCenter[srcIdx];
                                }
                            },
                            tbb::static_partitioner());

                        colliders.boneIdx.resize(validCount);
                        colliders.objIdx.resize(validCount);
                        colliders.rootIdx.resize(validCount);

                        colliders.noCollideCount.resize(validCount);
                        const std::uint32_t validNoCollideCount = validCount * NOCOLLIDE_MAX;
                        colliders.noCollideBoneIdx.resize(validNoCollideCount);

                        colliders.convexHullData.resize(validCount);
                        colliders.boundingAABB.resize(validCount);

                        colliders.boundingSphere.resize(validCount);
                        colliders.boundingSphereCenter.resize(validCount);
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
                        if (physicsBones.invMass[bi] <= FloatPrecision)
                            continue;
                        if (isParent[bi] != 0)
                            continue;
                        collidersLeafs.push_back(ci);
                    }
                }
            );
            logger::info("Reorder maps done");
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
        });
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
            childNode->world.translate = worldPos + (worldRot * (childNode->local.translate * worldScale));
            childNode->world.rotate = worldRot * childNode->local.rotate;
        }
    }
} // namespace MXPBD
