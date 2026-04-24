#include "MXPBD/PhysicsWorld.h"

#define TIMELOG_START                                               \
    static double nsSum = 0.0;                                      \
    static std::uint32_t timeCount = 0;                             \
    const auto timeStart = std::chrono::high_resolution_clock::now();   

#define TIMELOG_END                                                                                                                 \
    const auto timeEnd = std::chrono::high_resolution_clock::now();                                                                     \
    nsSum += std::chrono::duration_cast<std::chrono::nanoseconds>(timeEnd - timeStart).count();                                             \
    timeCount++;                                                                                                                    \
    if (timeCount >= 1000)                                                                                                          \
    {                                                                                                                               \
        const auto ms = (nsSum / timeCount) * ns2ms;                                                                                \
        logger::debug("{} time: {:.3f}ms ({} bones / {} constrants / {} angularConstrants / {} colliders)", __func__, ms,            \
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
        if (rootType == XPBDWorld::RootType::none)
            return;

        const ObjectDatas::Root newRoot = {.type = rootType, .bipedSlot = input.bipedSlot};

        logger::info("{:x} : adding physics for {} bones, {} constraints, {} colliders", object->formID, input.bones.size(), input.constraints.size(), input.convexHullColliders.colliders.size());

        std::lock_guard lg(lock);
        if (orderDirty)
        {
            ReorderMaps();
            orderDirty = false;
        }

        // find object
        std::uint32_t currentObjIdx = UINT32_MAX;
        {
            bool found = false;
            for (std::uint32_t oi = 0; oi < objectDatas.objectID.size(); ++oi)
            {
                if (objectDatas.objectID[oi] != object->formID)
                    continue;
                currentObjIdx = oi;
                found = true;
                logger::debug("{:x} : found object {}", object->formID, currentObjIdx);
                break;
            }

            if (!found)
            {
                // find empty slot
                for (std::uint32_t oi = 0; oi < objectDatas.objectID.size(); ++oi)
                {
                    if (objectDatas.objectID[oi] != 0)
                        continue;
                    objectDatas.objectID[oi] = object->formID;
                    objectDatas.roots[oi].clear();
                    objectDatas.prevWorldPos[oi] = ToVector(object->GetPosition());
                    if (object->loadedData && object->loadedData->data3D)
                    {
                        RE::NiAVObject* npcObj = object->loadedData->data3D->GetObjectByName("NPC");
                        objectDatas.npcNode[oi] = npcObj ? npcObj->AsNode() : nullptr;
                        objectDatas.prevNPCWorldRot[oi] = objectDatas.npcNode[oi] ? ToQuaternion(objectDatas.npcNode[oi]->world.rotate) : ToQuaternion(RE::NiMatrix3());
                        objectDatas.targetNPCWorldRot[oi] = objectDatas.prevNPCWorldRot[oi];
                    }
                    else
                    {
                        objectDatas.npcNode[oi] = nullptr;
                        objectDatas.prevNPCWorldRot[oi] = ToQuaternion(RE::NiMatrix3());
                        objectDatas.targetNPCWorldRot[oi] = objectDatas.prevNPCWorldRot[oi];
                    }
                    if (RE::TESObjectCELL* cell = object->GetParentCell(); cell)
                        objectDatas.bhkWorld[oi] = cell->GetbhkWorld();
                    else
                        objectDatas.bhkWorld[oi] = nullptr;
                    objectDatas.velocity[oi] = vZero;
                    objectDatas.acceleration[oi] = vZero;
                    objectDatas.boundingAABB[oi] = AABB();
                    objectDatas.isDisable[oi] = 0;
                    currentObjIdx = oi;
                    found = true;
                    logger::debug("{:x} : add new object {}", object->formID, currentObjIdx);
                    break;
                }
                if (!found)
                {
                    currentObjIdx = objectDatas.objectID.size();
                    objectDatas.objectID.push_back(object->formID);
                    objectDatas.roots.push_back({});
                    objectDatas.prevWorldPos.push_back(ToVector(object->GetPosition()));
                    bool hasNPCNode = false;
                    if (object->loadedData && object->loadedData->data3D)
                    {
                        RE::NiAVObject* npcObj = object->loadedData->data3D->GetObjectByName("NPC");
                        if (npcObj && npcObj->parent)
                        {
                            objectDatas.npcNode.push_back(npcObj->AsNode());
                            objectDatas.prevNPCWorldRot.push_back(ToQuaternion(npcObj->world.rotate));
                            objectDatas.targetNPCWorldRot.push_back(ToQuaternion(npcObj->world.rotate));
                            hasNPCNode = true;
                        }
                    }
                    if (!hasNPCNode)
                    {
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
                    objectDatas.isDisable.push_back(0);
                    logger::debug("{:x} : add new object {}", object->formID, currentObjIdx);
                }
            }
        }
        if (currentObjIdx == UINT32_MAX)
            return;
        objectDatas.isDisable[currentObjIdx] = false;

        // add root
        std::uint32_t currentRootIdx = UINT32_MAX;
        auto& root = objectDatas.roots[currentObjIdx];
        for (std::uint32_t ri = 0; ri < root.size(); ++ri)
        {
            if (root[ri] == newRoot)
            {
                currentRootIdx = ri;
                logger::debug("{:x} : found root {}", object->formID, currentRootIdx);
                break;
            }
        }
        if (currentRootIdx == UINT32_MAX)
        {
            currentRootIdx = static_cast<std::uint32_t>(root.size());
            root.push_back(newRoot);
            logger::debug("{:x} : add new root {}", object->formID, currentRootIdx);
        }

        // caching bones
        std::unordered_map<std::string, std::uint32_t> boneNameToIdx;
        if (!physicsBonesGroup.empty())
        {
            const std::uint32_t groups = physicsBonesGroup.size() - 1;
            for (std::uint32_t g = 0; g < groups; ++g)
            {
                const std::uint32_t begin = physicsBonesGroup[g];
                const std::uint32_t end = physicsBonesGroup[g + 1];
                if (physicsBones.objIdx[begin] != currentObjIdx)
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

        if (rootType == XPBDWorld::RootType::skeleton || rootType == XPBDWorld::RootType::facegen || rootType == XPBDWorld::RootType::cloth || rootType == XPBDWorld::RootType::weapon)
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

                std::string_view nodeName = obj->name.c_str();
                if (boneNameToIdx.find(nodeName.data()) != boneNameToIdx.end())
                    return true;
                auto found = input.bones.find(nodeName.data());
                if (found == input.bones.end())
                    return true;

                logger::debug("{:x} : add physics bone {}", object->formID, found->first);
                const std::uint32_t bi = AllocateBone();
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
                physicsBones.rotationRatio[bi] = found->second.rotationRatio;
                physicsBones.gravity[bi] = ToVector(GetSkyrimGravity(found->second.gravity));
                physicsBones.offset[bi] = ToVector(found->second.offset);
                physicsBones.invMass[bi] = (found->second.mass > FloatPrecision) ? (1.0f / found->second.mass) : 0.0f;

                if (found->second.mass > FloatPrecision && found->second.inertiaScale > FloatPrecision)
                    physicsBones.invInertia[bi] = 1.0f / (found->second.mass * found->second.inertiaScale);
                else
                    physicsBones.invInertia[bi] = 0.0f;

                physicsBones.collisionMargin[bi] = (found->second.collisionMargin < COL_MARGIN_MIN ? COL_MARGIN_MIN : found->second.collisionMargin);
                physicsBones.colShrink[bi] = (found->second.collisionMargin < COL_MARGIN_MIN ? COL_MARGIN_MIN - found->second.collisionMargin : 0.0f);
                physicsBones.collisionFriction[bi] = found->second.collisionFriction;
                physicsBones.collisionRotationBias[bi] = found->second.collisionRotationBias;
                physicsBones.collisionCompliance[bi] = found->second.collisionCompliance;

                physicsBones.node[bi] = RE::NiPointer(obj);
                physicsBones.isParticle[bi] = 0;
                physicsBones.parentBoneIdx[bi] = UINT32_MAX;
                if (obj->parent && !obj->parent->name.empty())
                {
                    auto pit = boneNameToIdx.find(obj->parent->name.c_str());
                    if (pit != boneNameToIdx.end())
                        physicsBones.parentBoneIdx[bi] = pit->second;
                }
                physicsBones.objIdx[bi] = currentObjIdx;
                physicsBones.rootIdx[bi] = currentRootIdx;
                physicsBones.depth[bi] = depth;

                physicsBones.worldScale[bi] = obj->world.scale;
                physicsBones.worldRot[bi] = obj->world.rotate;
                physicsBones.orgLocalPos[bi] = obj->local.translate;
                physicsBones.orgLocalRot[bi] = obj->local.rotate;

                boneNameToIdx[found->first] = bi;

                if (auto pptn = particleParentToName.find(nodeName.data()); pptn != particleParentToName.end())
                {
                    auto func = [this, &object, &input, currentObjIdx, currentRootIdx, depth, &boneNameToIdx, &particleParentToName](auto&& func, const std::string& parentName, const std::vector<std::string>& pptnList, const std::uint32_t parentIdx, std::uint32_t particleDepth) -> void {
                        for (const auto& particleName : pptnList)
                        {
                            const auto pit = input.bones.find(particleName);
                            if (pit == input.bones.end())
                                continue;

                            logger::debug("{:x} : add physics particle({}) bone {} for {}", object->formID, particleDepth, particleName, parentName);
                            const std::uint32_t particleBi = AllocateBone();
                            const Vector offset = DirectX::XMVector3Rotate(DirectX::XMVectorScale(ToVector(pit->second.offset), physicsBones.worldScale[parentIdx]), ToQuaternion(physicsBones.worldRot[parentIdx]));
                            physicsBones.pos[particleBi] = DirectX::XMVectorAdd(physicsBones.pos[parentIdx], offset);
                            physicsBones.pred[particleBi] = physicsBones.pos[particleBi];
                            physicsBones.vel[particleBi] = vZero;

                            physicsBones.rot[particleBi] = ToQuaternion(physicsBones.worldRot[parentIdx]);
                            physicsBones.predRot[particleBi] = physicsBones.rot[particleBi];
                            physicsBones.angVel[particleBi] = vZero;

                            physicsBones.damping[particleBi] = pit->second.damping;
                            physicsBones.inertiaScale[particleBi] = pit->second.inertiaScale;
                            physicsBones.restitution[particleBi] = pit->second.restitution;
                            physicsBones.rotationRatio[particleBi] = pit->second.rotationRatio;
                            physicsBones.gravity[particleBi] = ToVector(GetSkyrimGravity(pit->second.gravity));
                            physicsBones.offset[particleBi] = ToVector(pit->second.offset);
                            physicsBones.invMass[particleBi] = (pit->second.mass > FloatPrecision) ? (1.0f / pit->second.mass) : 0.0f;

                            if (pit->second.mass > FloatPrecision && pit->second.inertiaScale > FloatPrecision)
                                physicsBones.invInertia[particleBi] = 1.0f / (pit->second.mass * pit->second.inertiaScale);
                            else
                                physicsBones.invInertia[particleBi] = 0.0f;
                            
                            physicsBones.linearRotTorque[particleBi] = pit->second.linearRotTorque;

                            physicsBones.node[particleBi] = nullptr;
                            physicsBones.particleName[particleBi] = pit->first;
                            physicsBones.isParticle[particleBi] = 1;
                            physicsBones.particleDepth[particleBi] = particleDepth;
                            physicsBones.parentBoneIdx[particleBi] = parentIdx;
                            physicsBones.objIdx[particleBi] = currentObjIdx;
                            physicsBones.rootIdx[particleBi] = currentRootIdx;
                            physicsBones.depth[particleBi] = depth;

                            physicsBones.worldScale[particleBi] = physicsBones.worldScale[parentIdx];
                            physicsBones.worldRot[particleBi] = physicsBones.worldRot[parentIdx];
                            physicsBones.orgLocalPos[particleBi] = RE::NiPoint3();
                            physicsBones.orgLocalRot[particleBi] = RE::NiMatrix3();

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
                constraints.objIdx[ci] = currentObjIdx;
                constraints.rootIdx[ci] = currentRootIdx;

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
                    constraints.anchIdx[ai] = anchBi;
                    const Vector tPos = physicsBones.pos[bit->second];
                    const Vector aPos = physicsBones.pos[anchBi];
                    constraints.restLen[ai] = DirectX::XMVectorGetX(DirectX::XMVector3Length(DirectX::XMVectorSubtract(tPos, aPos)));
                    constraints.compSquish[ai] = constraint.second.complianceSquish[a];
                    constraints.compStretch[ai] = constraint.second.complianceStretch[a];
                    constraints.squishLimit[ai] = constraint.second.squishLimit[a];
                    constraints.stretchLimit[ai] = constraint.second.stretchLimit[a];
                    constraints.angularLimit[ai] = constraint.second.angularLimit[a];
                    Vector dirWorld = DirectX::XMVectorSubtract(tPos, aPos);
                    if (DirectX::XMVector3Less(vFloatPrecision, DirectX::XMVector3LengthSq(dirWorld)))
                    {
                        dirWorld = DirectX::XMVector3Normalize(dirWorld);
                        const Quaternion anchRotInv = DirectX::XMQuaternionConjugate(ToQuaternion(physicsBones.worldRot[anchBi]));
                        constraints.restDirLocal[ai] = DirectX::XMVector3Rotate(dirWorld, anchRotInv);
                    }
                    else
                    {
                        constraints.restDirLocal[ai] = DirectX::XMVectorSet(0.0f, -1.0f, 0.0f, 0.0f);
                    }
                    constraints.squishDamping[ai] = constraint.second.squishDamping[a];
                    constraints.stretchDamping[ai] = constraint.second.stretchDamping[a];
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
                angularConstraints.objIdx[aci] = currentObjIdx;
                angularConstraints.rootIdx[aci] = currentRootIdx;

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
                    angularConstraints.anchIdx[aai] = abit->second;

                    physicsBones.advancedRotation[abit->second] = 1;

                    const Quaternion childRot = ToQuaternion(physicsBones.worldRot[bit->second]);
                    const Quaternion anchorRot = ToQuaternion(physicsBones.worldRot[abit->second]);
                    const Quaternion anchorRotInv = DirectX::XMQuaternionInverse(anchorRot);
                    const Quaternion restRot = DirectX::XMQuaternionNormalize(DirectX::XMQuaternionMultiply(childRot, anchorRotInv));
                    angularConstraints.restRot[aai] = restRot;
                    angularConstraints.comp[aai] = angularConstraint.second.compliance[a];
                    angularConstraints.limit[aai] = angularConstraint.second.limit[a];
                    angularConstraints.damping[aai] = angularConstraint.second.damping[a];
                    validAnchorCount++;
                }
                angularConstraints.numAnchors[aci] = validAnchorCount;
            }
        }
        else if (rootType == XPBDWorld::RootType::collider)
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
                    colliders.objIdx[ci] = currentObjIdx;
                    colliders.rootIdx[ci] = currentRootIdx;

                    colliders.convexHullData[ci] = collider.second;

                    logger::debug("{:x} => add collider {}", object->formID, boneName);
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

                    if (const float colShrink = physicsBones.colShrink[bit->second]; colShrink > FloatPrecision)
                    {
                        const float maxShrink = colliders.boundingSphere[ci] * 0.9f;
                        const float finalShrink = std::min(physicsBones.colShrink[bit->second], maxShrink);
                        const float shrinkRatio = 1.0f - (finalShrink / colliders.boundingSphere[ci]);

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
                            if (ncBit == boneNameToIdx.end() || ncCount >= NOCOLLIDE_MAX)
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
        for (std::uint32_t i = 0; i < objectDatas.objectID.size(); ++i)
        {
            if (objectDatas.objectID[i] != object->formID)
                continue;
            objectDatas.prevWorldPos[i] = ToVector(object->GetPosition());
            objectDatas.acceleration[i] = vZero;
            currentObjIdx = i;
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
                if (physicsBones.objIdx[begin] != currentObjIdx)
                    continue;
                logger::debug("{:x} : Found bones group", object->formID);
                for (std::uint32_t bi = begin; bi < end; ++bi)
                {
                    if (auto& node = physicsBones.node[bi]; node && !node->name.empty())
                    {
                        const std::string nodeName = node->name.c_str();
                        boneNameToIdx[nodeName] = bi;

                        const auto found = input.bones.find(nodeName);
                        if (found == input.bones.end())
                            continue;

                        logger::debug("{:x} : Found bone ({}) for physics", object->formID, nodeName);

                        physicsBones.offset[bi] = ToVector(found->second.offset);
                        const Vector offset = DirectX::XMVector3Rotate(DirectX::XMVectorScale(physicsBones.offset[bi], physicsBones.worldScale[bi]), ToQuaternion(node->world.rotate));
                        
                        physicsBones.pos[bi] = DirectX::XMVectorAdd(ToVector(node->world.translate), offset);
                        physicsBones.pred[bi] = physicsBones.pos[bi];
                        physicsBones.vel[bi] = vZero;

                        physicsBones.rot[bi] = ToQuaternion(node->world.rotate);
                        physicsBones.predRot[bi] = physicsBones.rot[bi];
                        physicsBones.angVel[bi] = vZero;

                        physicsBones.damping[bi] = found->second.damping;
                        physicsBones.inertiaScale[bi] = found->second.inertiaScale;
                        physicsBones.restitution[bi] = found->second.restitution;
                        physicsBones.rotationRatio[bi] = found->second.rotationRatio;
                        physicsBones.gravity[bi] = ToVector(GetSkyrimGravity(found->second.gravity));
                        physicsBones.invMass[bi] = (found->second.mass > FloatPrecision) ? (1.0f / found->second.mass) : 0.0f;

                        if (found->second.mass > FloatPrecision && found->second.inertiaScale > FloatPrecision)
                            physicsBones.invInertia[bi] = 1.0f / (found->second.mass * found->second.inertiaScale);
                        else
                            physicsBones.invInertia[bi] = 0.0f;

                        physicsBones.linearRotTorque[bi] = found->second.linearRotTorque;

                        physicsBones.collisionMargin[bi] = (found->second.collisionMargin < 0.0f ? 0.0f : found->second.collisionMargin);
                        physicsBones.colShrink[bi] = (found->second.collisionMargin < COL_MARGIN_MIN ? COL_MARGIN_MIN - found->second.collisionMargin : 0.0f);
                        physicsBones.collisionFriction[bi] = found->second.collisionFriction;
                        physicsBones.collisionRotationBias[bi] = found->second.collisionRotationBias;
                        physicsBones.collisionCompliance[bi] = found->second.collisionCompliance;

                        physicsBones.worldScale[bi] = node->world.scale;
                        physicsBones.worldRot[bi] = node->world.rotate;
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
                        const Vector offset = DirectX::XMVector3Rotate(DirectX::XMVectorScale(physicsBones.offset[bi], physicsBones.worldScale[pbi]), ToQuaternion(physicsBones.worldRot[pbi]));
                        
                        physicsBones.pos[bi] = DirectX::XMVectorAdd(physicsBones.pos[pbi], offset);
                        physicsBones.pred[bi] = physicsBones.pos[bi];
                        physicsBones.vel[bi] = vZero;

                        physicsBones.rot[bi] = ToQuaternion(physicsBones.worldRot[pbi]);
                        physicsBones.predRot[bi] = physicsBones.rot[bi];
                        physicsBones.angVel[bi] = vZero;

                        physicsBones.damping[bi] = found->second.damping;
                        physicsBones.inertiaScale[bi] = found->second.inertiaScale;
                        physicsBones.restitution[bi] = found->second.restitution;
                        physicsBones.rotationRatio[bi] = found->second.rotationRatio;
                        physicsBones.gravity[bi] = ToVector(GetSkyrimGravity(found->second.gravity));
                        physicsBones.invMass[bi] = (found->second.mass > FloatPrecision) ? (1.0f / found->second.mass) : 0.0f;

                        if (found->second.mass > FloatPrecision && found->second.inertiaScale > FloatPrecision)
                            physicsBones.invInertia[bi] = (found->second.mass * found->second.inertiaScale);
                        else
                            physicsBones.invInertia[bi] = 0.0f;
                        physicsBones.linearRotTorque[bi] = found->second.linearRotTorque;

                        physicsBones.worldScale[bi] = physicsBones.worldScale[pbi];
                        physicsBones.worldRot[bi] = physicsBones.worldRot[pbi];
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
                        constraints.anchIdx[ai] = anchGlobIdx;
                        const Vector tPos = physicsBones.pos[bi];
                        const Vector aPos = physicsBones.pos[anchGlobIdx];
                        constraints.restLen[ai] = DirectX::XMVectorGetX(DirectX::XMVector3Length(DirectX::XMVectorSubtract(tPos, aPos)));
                        constraints.compSquish[ai] = found->second.complianceSquish[a];
                        constraints.compStretch[ai] = found->second.complianceStretch[a];
                        constraints.squishLimit[ai] = found->second.squishLimit[a];
                        constraints.stretchLimit[ai] = found->second.stretchLimit[a];
                        constraints.angularLimit[ai] = found->second.angularLimit[a];
                        Vector dirWorld = DirectX::XMVectorSubtract(tPos, aPos);
                        if (DirectX::XMVector3Less(vFloatPrecision, DirectX::XMVector3LengthSq(dirWorld)))
                        {
                            dirWorld = DirectX::XMVector3Normalize(dirWorld);
                            const Quaternion anchRotInv = DirectX::XMQuaternionConjugate(ToQuaternion(physicsBones.worldRot[anchGlobIdx]));
                            constraints.restDirLocal[ai] = DirectX::XMVector3Rotate(dirWorld, anchRotInv);
                        }
                        else
                        {
                            constraints.restDirLocal[ai] = DirectX::XMVectorSet(0.0f, -1.0f, 0.0f, 0.0f);
                        }
                        constraints.squishDamping[ai] = found->second.squishDamping[a];
                        constraints.stretchDamping[ai] = found->second.stretchDamping[a];
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
                        angularConstraints.anchIdx[ai] = anchBi;

                        const auto anchBoneIt = input.bones.find(found->second.anchorBoneNames[a]);
                        if (anchBoneIt == input.bones.end())
                            continue;

                        physicsBones.advancedRotation[bi] = 1;

                        const Quaternion childRot = ToQuaternion(physicsBones.worldRot[bi]);
                        const Quaternion anchorRot = ToQuaternion(physicsBones.worldRot[anchBi]);
                        const Quaternion anchorRotInv = DirectX::XMQuaternionInverse(anchorRot);
                        const Quaternion restRot = DirectX::XMQuaternionNormalize(DirectX::XMQuaternionMultiply(childRot, anchorRotInv));
                        angularConstraints.restRot[ai] = restRot;
                        angularConstraints.comp[ai] = found->second.compliance[a];
                        angularConstraints.limit[ai] = found->second.limit[a];
                        angularConstraints.damping[ai] = found->second.damping[a];
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

        for (std::uint32_t i = 0; i < objectDatas.objectID.size(); ++i)
        {
            RE::TESObjectREFR* object = GetREFR(objectDatas.objectID[i]);
            if (!object)
                continue;
            objectDatas.prevWorldPos[i] = ToVector(object->GetPosition());
            objectDatas.acceleration[i] = vZero;
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
            memcpy(&node->local.translate, &physicsBones.orgLocalPos[bi], sizeof(physicsBones.orgLocalPos[bi]));
            memcpy(&node->local.rotate, &physicsBones.orgLocalRot[bi], sizeof(physicsBones.orgLocalRot[bi]));

            if (node->parent)
            {
                const RE::NiPoint3 parentWorldPos = node->parent->world.translate;
                const RE::NiMatrix3 parentWorldRot = node->parent->world.rotate;
                const RE::NiPoint3 worldPos = parentWorldPos + (parentWorldRot * physicsBones.orgLocalPos[bi]);
                const RE::NiMatrix3 worldRot = parentWorldRot * physicsBones.orgLocalRot[bi];
                memcpy(&node->world.translate, &worldPos, sizeof(worldPos));
                memcpy(&node->world.rotate, &worldRot, sizeof(worldRot));
            }

            const Vector offset = DirectX::XMVector3Rotate(DirectX::XMVectorScale(physicsBones.offset[bi], physicsBones.worldScale[bi]), ToQuaternion(node->world.rotate));
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
            const Vector offset = DirectX::XMVector3Rotate(DirectX::XMVectorScale(physicsBones.offset[bi], physicsBones.worldScale[pbi]), ToQuaternion(physicsBones.worldRot[pbi]));
            physicsBones.pos[bi] = DirectX::XMVectorAdd(physicsBones.pos[pbi], offset);
            physicsBones.prevPos[bi] = physicsBones.pos[bi];
            physicsBones.pred[bi] = physicsBones.pos[bi];
            physicsBones.vel[bi] = vZero;

            physicsBones.rot[bi] = ToQuaternion(physicsBones.worldRot[pbi]);
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
        std::unordered_set<CleanObject, CleanObjectHash> cleanList;
        for (std::uint32_t i = 0; i < objectDatas.objectID.size(); ++i)
        {
            if (objectDatas.objectID[i] != objectID)
                continue;
            objectDatas.objectID[i] = 0;
            objectDatas.isDisable[i] = true;
            for (std::uint32_t ri = 0; ri < objectDatas.roots[i].size(); ++ri)
            {
                cleanList.insert(CleanObject(i, ri));
            }
            objectDatas.roots[i].clear();
            CleanPhysics(cleanList);
            break;
        }
    }

    void XPBDWorld::RemovePhysics(RE::TESObjectREFR* object, const RootType rootType, const std::uint32_t bipedSlot)
    {
        if (!object)
            return;
        Reset(object); 
        const ObjectDatas::Root targetRoot = {.type = rootType, .bipedSlot = bipedSlot};
        std::lock_guard lg(lock);
        std::unordered_set<CleanObject, CleanObjectHash> cleanList;
        for (std::uint32_t i = 0; i < objectDatas.objectID.size(); ++i)
        {
            if (objectDatas.objectID[i] != object->formID)
                continue;
            auto& root = objectDatas.roots[i];
            for (std::uint32_t ri = 0; ri < root.size(); ++ri)
            {
                if (root[ri] != targetRoot)
                    continue;
                cleanList.insert(CleanObject(i, ri));
                root[ri].type = RootType::none;
                break;
            }
            CleanPhysics(cleanList);
            break;
        }
    }

    void XPBDWorld::TogglePhysics(const RE::FormID objectID, bool isDisable)
    {
        std::lock_guard lg(lock);
        for (std::uint32_t i = 0; i < objectDatas.objectID.size(); ++i)
        {
            if (objectDatas.objectID[i] != objectID)
                continue;
            if (objectDatas.isDisable[i] != isDisable)
            {
                objectDatas.isDisable[i] = isDisable;
                orderDirty = true;
            }
            break;
        }
    }

    void XPBDWorld::CleanPhysics(const std::unordered_set<CleanObject, CleanObjectHash>& cleanList)
    {
        if (cleanList.empty())
            return;

        // clean nodes
        for (std::uint32_t i = 0; i < physicsBones.node.size(); ++i)
        {
            if (cleanList.count(CleanObject(physicsBones.objIdx[i], physicsBones.rootIdx[i])) > 0)
            {
                physicsBones.node[i] = nullptr;
                physicsBones.objIdx[i] = UINT32_MAX;
                physicsBones.rootIdx[i] = UINT32_MAX;
                physicsBones.parentBoneIdx[i] = UINT32_MAX;
                physicsBones.particleName[i].clear();
                physicsBones.particleDepth[i] = UINT32_MAX;
            }
        }

        // clean constraints
        for (std::uint32_t i = 0; i < constraints.numConstraints; ++i)
        {
            if (constraints.boneIdx[i] != UINT32_MAX && (!physicsBones.node[constraints.boneIdx[i]] && physicsBones.particleName[constraints.boneIdx[i]].empty()))
            {
                constraints.boneIdx[i] = UINT32_MAX;
                constraints.objIdx[i] = UINT32_MAX;
                constraints.rootIdx[i] = UINT32_MAX;
            }
        }

        // clean angular constraints
        for (std::uint32_t i = 0; i < angularConstraints.numConstraints; ++i)
        {
            if (angularConstraints.boneIdx[i] != UINT32_MAX && (!physicsBones.node[angularConstraints.boneIdx[i]] && physicsBones.particleName[angularConstraints.boneIdx[i]].empty()))
            {
                angularConstraints.boneIdx[i] = UINT32_MAX;
                angularConstraints.objIdx[i] = UINT32_MAX;
                angularConstraints.rootIdx[i] = UINT32_MAX;
            }
        }

        // clean colliders
        for (std::uint32_t i = 0; i < colliders.numColliders; ++i)
        {
            if ((colliders.boneIdx[i] != UINT32_MAX && !physicsBones.node[colliders.boneIdx[i]]))
            {
                colliders.boneIdx[i] = UINT32_MAX;
                colliders.objIdx[i] = UINT32_MAX;
                colliders.rootIdx[i] = UINT32_MAX;
            }
        }
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
            timeAccumulator += std::min(DeltaTime60 * 6, deltaTime); // low 10 fps
            while (timeAccumulator >= DeltaTime60)
            {
                if (stepCount == 0)
                {
                    UpdateObjectData(objectAccelerationTime);
                    objectAccelerationTime = 0.0f;
                }
                ClampObjectRotation();
                PrefetchBoneDatas();
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
        // logger::info("{}", __func__);
        TIMELOG_START;
        const Vector invDt = DirectX::XMVectorReplicate(1.0f / deltaTime);
        tbb::parallel_for(
            tbb::blocked_range<std::uint32_t>(0, objectDatas.objectID.size()),
            [&](const tbb::blocked_range<std::uint32_t>& r) {
                for (std::uint32_t oi = r.begin(); oi != r.end(); ++oi)
                {
                    RE::TESObjectREFR* object = GetREFR(objectDatas.objectID[oi]);
                    if (!object || objectDatas.isDisable[oi])
                        continue;
                    const Vector currentWorldPos = ToVector(object->GetPosition());
                    const Vector currentVel = DirectX::XMVectorMultiply(DirectX::XMVectorSubtract(currentWorldPos, objectDatas.prevWorldPos[oi]), invDt);
                    objectDatas.acceleration[oi] = DirectX::XMVectorMultiply(DirectX::XMVectorSubtract(currentVel, objectDatas.velocity[oi]), invDt);
                    if (RE::TESObjectCELL* cell = object->GetParentCell(); cell)
                        objectDatas.bhkWorld[oi] = cell->GetbhkWorld();
                    else
                        objectDatas.bhkWorld[oi] = nullptr;
                    objectDatas.velocity[oi] = currentVel;
                    objectDatas.prevWorldPos[oi] = currentWorldPos;
                }
            },
            tbb::static_partitioner()
        );
        TIMELOG_END;
    }

    void XPBDWorld::ClampObjectRotation()
    {
        TIMELOG_START;
        for (std::uint32_t oi = 0; oi != objectDatas.objectID.size(); ++oi)
        {
            if (objectDatas.isDisable[oi])
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
            const float t = (angle > ROTATION_CLAMP) ? (ROTATION_CLAMP / angle) : 1.0f;
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

    void XPBDWorld::PrefetchBoneDatas()
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
                        if (physicsBones.invMass[bi] <= FloatPrecision)
                        {
                            const Vector offset = DirectX::XMVector3Rotate(DirectX::XMVectorScale(physicsBones.offset[bi], physicsBones.worldScale[bi]), ToQuaternion(node->world.rotate));
                            physicsBones.pos[bi] = DirectX::XMVectorAdd(ToVector(node->world.translate), offset);
                            physicsBones.prevPos[bi] = physicsBones.pos[bi];
                            physicsBones.pred[bi] = physicsBones.pos[bi];
                            physicsBones.backupRot[bi] = physicsBones.rot[bi];
                            physicsBones.rot[bi] = ToQuaternion(node->world.rotate);
                            physicsBones.prevRot[bi] = physicsBones.rot[bi];
                            physicsBones.predRot[bi] = physicsBones.rot[bi];
                        }
                        physicsBones.worldRot[bi] = node->world.rotate;
                        physicsBones.worldScale[bi] = node->world.scale;
                    }
                    else
                    {
                        if (physicsBones.parentBoneIdx[bi] != UINT32_MAX)
                        {
                            const std::uint32_t pbi = physicsBones.parentBoneIdx[bi];
                            if (physicsBones.invMass[bi] <= FloatPrecision)
                            {
                                const Vector offset = DirectX::XMVector3Rotate(DirectX::XMVectorScale(physicsBones.offset[bi], physicsBones.worldScale[pbi]), ToQuaternion(physicsBones.worldRot[pbi]));
                                physicsBones.pos[bi] = DirectX::XMVectorAdd(physicsBones.pos[pbi], offset);
                                physicsBones.prevPos[bi] = physicsBones.pos[bi];
                                physicsBones.pred[bi] = physicsBones.pos[bi];
                                physicsBones.backupRot[bi] = physicsBones.rot[bi];
                                physicsBones.rot[bi] = physicsBones.rot[pbi];
                                physicsBones.prevRot[bi] = physicsBones.rot[bi];
                                physicsBones.predRot[bi] = physicsBones.rot[bi];
                            }
                            physicsBones.worldRot[bi] = physicsBones.worldRot[pbi];
                            physicsBones.worldScale[bi] = physicsBones.worldScale[pbi];
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

        for (std::uint32_t i = 0; i < objectDatas.objectID.size(); ++i)
        {
            if (objectDatas.isDisable[i] || objectDatas.objectID[i] == 0)
            {
                if (objIdxToTreeNodeIdx[i] != UINT32_MAX)
                {
                    globalAABBTree.RemoveLeaf(objIdxToTreeNodeIdx[i]);
                    objIdxToTreeNodeIdx[i] = UINT32_MAX;
                }
                objectDatas.boundingAABB[i] = AABB();
                continue;
            }
            objectDatas.boundingAABB[i] = GetObjectAABB(i);
            if (objectDatas.boundingAABB[i].IsZero())
                continue;

            if (objIdxToTreeNodeIdx[i] == UINT32_MAX)
            {
                AABB fatAABB = objectDatas.boundingAABB[i];
                fatAABB.Fatten();
                objIdxToTreeNodeIdx[i] = globalAABBTree.InsertLeaf(i, fatAABB);
            }
            else
            {
                objIdxToTreeNodeIdx[i] = globalAABBTree.UpdateLeaf(objIdxToTreeNodeIdx[i], objectDatas.boundingAABB[i]);
            }
        }
        TIMELOG_END;
    }

    void XPBDWorld::PredictBones(const float deltaTime)
    {
        // logger::info("{}", __func__);
        TIMELOG_START;
        const Vector dt = DirectX::XMVectorReplicate(deltaTime);
        tbb::parallel_for(
            tbb::blocked_range<std::uint32_t>(0, physicsBones.numBones, 128),
            [&](const tbb::blocked_range<std::uint32_t>& r) {
                for (std::uint32_t bi = r.begin(); bi != r.end(); ++bi)
                {
                    const std::uint32_t oi = physicsBones.objIdx[bi];
                    if (oi == UINT32_MAX || objectDatas.isDisable[oi])
                        continue;
                    if (physicsBones.invMass[bi] <= FloatPrecision)
                        continue;

                    // apply linear velocity
                    {
                        const Vector fictAcc = DirectX::XMVectorScale(objectDatas.acceleration[oi], -physicsBones.inertiaScale[bi]);
                        const Vector totalAcc = DirectX::XMVectorScale(DirectX::XMVectorAdd(physicsBones.gravity[bi], fictAcc), deltaTime);
                        physicsBones.vel[bi] = DirectX::XMVectorAdd(physicsBones.vel[bi], totalAcc);
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

    void XPBDWorld::SolveConstraints(const float deltaTime, const bool initLambda)
    {
        if (constraintsGroup.empty() && angularConstraintsGroup.empty())
            return;
        // logger::info("{}", __func__);
        TIMELOG_START;
        const float invDtSq = 1.0f / (deltaTime * deltaTime);
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
                            if (oi == UINT32_MAX || objectDatas.isDisable[oi])
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
                                        const std::uint32_t& abi = constraints.anchIdx[ai];
                                        if (abi == UINT32_MAX)
                                            continue;

                                        const float anchInvMass = physicsBones.invMass[abi];
                                        const float wSum = invMass + anchInvMass;
                                        if (wSum <= FloatPrecision)
                                            continue;

                                        const Vector dir = DirectX::XMVectorSubtract(physicsBones.pred[bi], physicsBones.pred[abi]);
                                        const Vector distSq = DirectX::XMVector3LengthSq(dir);
                                        if (DirectX::XMVector3LessOrEqual(distSq, vFloatPrecision))
                                            continue;

                                        const Vector invDist = DirectX::XMVectorReciprocalSqrt(distSq);
                                        const Vector normal = DirectX::XMVectorMultiply(dir, invDist);
                                        const Vector dist = DirectX::XMVectorMultiply(distSq, invDist);

                                        const Vector restLen = DirectX::XMVectorReplicate(constraints.restLen[ai]);
                                        const Vector vC = DirectX::XMVectorSubtract(dist, restLen);
                                        const float C = DirectX::XMVectorGetX(vC);
                                        const float currentComp = C < 0.0f ? constraints.compSquish[ai] : constraints.compStretch[ai];
                                        const float currentLimit = C < 0.0f ? constraints.squishLimit[ai] : constraints.stretchLimit[ai];
                                        const float currentDamping = C < 0.0f ? constraints.squishDamping[ai] : constraints.stretchDamping[ai];
                                        float currentFactor = 1.0f;
                                        if (FloatPrecision < currentLimit)
                                        {
                                            const float ratio = std::min(std::abs(C) / currentLimit, 1.0f);
                                            const float smoothFalloff = (1.0f - ratio) * (1.0f - ratio);
                                            currentFactor = std::max(0.1f, smoothFalloff);
                                        }
                                        const Vector alphaProxy = DirectX::XMVectorReplicate((currentComp * currentFactor) * invDtSq);
                                        const Vector vWSum = DirectX::XMVectorReplicate(wSum);
                                        const Vector vDenom = DirectX::XMVectorAdd(vWSum, alphaProxy);
                                        const Vector vInvDenom = DirectX::XMVectorReciprocal(vDenom);
                                        Vector deltaLambda = vZero;
                                        if (initLambda)
                                        {
                                            deltaLambda = DirectX::XMVectorMultiply(DirectX::XMVectorNegate(vC), vInvDenom);
                                            constraints.lambda[ai] = DirectX::XMVectorGetX(deltaLambda);
                                        }
                                        else
                                        {
                                            deltaLambda = DirectX::XMVectorMultiply(DirectX::XMVectorSubtract(DirectX::XMVectorNegate(vC), DirectX::XMVectorScale(alphaProxy, constraints.lambda[ai])), vInvDenom);
                                            constraints.lambda[ai] += DirectX::XMVectorGetX(deltaLambda);
                                        }

                                        const Vector correction = DirectX::XMVectorMultiply(normal, deltaLambda);
                                        physicsBones.pred[bi] = DirectX::XMVectorMultiplyAdd(correction, DirectX::XMVectorReplicate(invMass), physicsBones.pred[bi]);

                                        if (FloatPrecision < anchInvMass)
                                        {
                                            const Vector correctionA = DirectX::XMVectorScale(correction, anchInvMass);
                                            physicsBones.pred[abi] = DirectX::XMVectorSubtract(physicsBones.pred[abi], correctionA);
                                        }

                                        if (FloatPrecision < currentDamping)
                                        {
                                            const Vector deltaBi = DirectX::XMVectorSubtract(physicsBones.pred[bi], physicsBones.pos[bi]);
                                            const Vector deltaAbi = DirectX::XMVectorSubtract(physicsBones.pred[abi], physicsBones.pos[abi]);

                                            const Vector relDelta = DirectX::XMVectorSubtract(deltaBi, deltaAbi);

                                            const Vector projDelta = DirectX::XMVector3Dot(relDelta, normal);

                                            const Vector dampingCorrectionMag = DirectX::XMVectorScale(DirectX::XMVectorNegate(projDelta), currentDamping);
                                            const Vector dampCorrection = DirectX::XMVectorMultiply(normal, DirectX::XMVectorScale(dampingCorrectionMag, 1.0f / wSum));

                                            physicsBones.pred[bi] = DirectX::XMVectorMultiplyAdd(dampCorrection, DirectX::XMVectorReplicate(invMass), physicsBones.pred[bi]);
                                            if (FloatPrecision < anchInvMass)
                                            {
                                                const Vector correctionA = DirectX::XMVectorScale(dampCorrection, anchInvMass);
                                                physicsBones.pred[abi] = DirectX::XMVectorSubtract(physicsBones.pred[abi], correctionA);
                                            }
                                        }

                                        const float limitAngle = constraints.angularLimit[ai];
                                        if (FloatPrecision < limitAngle)
                                        {
                                            const Vector vLimitAngle = DirectX::XMVectorReplicate(limitAngle);
                                            Vector currentDir = DirectX::XMVectorSubtract(physicsBones.pred[bi], physicsBones.pred[abi]);
                                            if (DirectX::XMVector3LessOrEqual(DirectX::XMVector3Length(currentDir), vFloatPrecision))
                                                continue;

                                            const Vector currentDist = DirectX::XMVector3Length(currentDir);
                                            const Vector invCurrentDist = DirectX::XMVectorReciprocal(currentDist);
                                            currentDir = DirectX::XMVectorMultiply(currentDir, invCurrentDist);

                                            Vector restDirWorld = DirectX::XMVector3Rotate(constraints.restDirLocal[ai], physicsBones.predRot[abi]);
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
                                                physicsBones.pred[bi] = DirectX::XMVectorAdd(physicsBones.pred[bi], DirectX::XMVectorScale(angCorrection, invMass / wSum));
                                                if (FloatPrecision < anchInvMass)
                                                {
                                                    physicsBones.pred[abi] = DirectX::XMVectorSubtract(physicsBones.pred[abi], DirectX::XMVectorScale(angCorrection, anchInvMass / wSum));
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
            } 
        }, [&] {
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
                            if (oi == UINT32_MAX || objectDatas.isDisable[oi])
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
                                        const std::uint32_t& abi = angularConstraints.anchIdx[ai];
                                        if (abi == UINT32_MAX)
                                            continue;

                                        const float invInertiaA = physicsBones.invInertia[abi];
                                        const float wSum = invInertiaA + invInertia;
                                        if (wSum <= FloatPrecision)
                                            continue;

                                        const Quaternion target = DirectX::XMQuaternionMultiply(angularConstraints.restRot[ai], physicsBones.predRot[abi]);
                                        const Quaternion targetInv = DirectX::XMQuaternionConjugate(target);
                                        const Quaternion diff = DirectX::XMQuaternionMultiply(targetInv, physicsBones.predRot[bi]);
                                        Vector omega = DirectX::XMVectorGetW(diff) < 0.0f ? DirectX::XMVectorNegate(diff) : diff;
                                        omega = DirectX::XMVectorSetW(omega, 0.0f);
                                        const Vector CSq = DirectX::XMVector3LengthSq(omega);
                                        if (DirectX::XMVector3LessOrEqual(CSq, vFloatPrecision))
                                            continue;

                                        const Vector vC = DirectX::XMVectorSqrt(CSq);
                                        const float C = DirectX::XMVectorGetX(vC);
                                        const float currentLimit = angularConstraints.limit[ai];
                                        float currentFactor = 1.0f;
                                        if (currentLimit > FloatPrecision)
                                        {
                                            const float ratio = std::min(C / currentLimit, 1.0f);
                                            const float smoothFalloff = (1.0f - ratio) * (1.0f - ratio);
                                            currentFactor = std::max(0.05f, smoothFalloff);
                                        }
                                        const Vector alphaProxy = DirectX::XMVectorReplicate((angularConstraints.comp[ai] * currentFactor) * invDtSq);
                                        const Vector vWSum = DirectX::XMVectorReplicate(wSum);
                                        const Vector vDenom = DirectX::XMVectorAdd(vWSum, alphaProxy);
                                        const Vector vInvDenom = DirectX::XMVectorReciprocal(vDenom);
                                        Vector deltaLambda = vZero;
                                        if (initLambda)
                                        {
                                            deltaLambda = DirectX::XMVectorMultiply(DirectX::XMVectorNegate(vC), vInvDenom);
                                            angularConstraints.lambda[ai] = DirectX::XMVectorGetX(deltaLambda);
                                        }
                                        else
                                        {
                                            deltaLambda = DirectX::XMVectorMultiply(DirectX::XMVectorSubtract(DirectX::XMVectorNegate(vC), DirectX::XMVectorScale(alphaProxy, angularConstraints.lambda[ai])), vInvDenom);
                                            angularConstraints.lambda[ai] += DirectX::XMVectorGetX(deltaLambda);
                                        }

                                        const Vector correctionDir = DirectX::XMVectorMultiply(omega, DirectX::XMVectorReciprocal(vC));
                                        {
                                            const Vector correction = DirectX::XMVectorMultiply(correctionDir, deltaLambda);
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

                                        const float damping = angularConstraints.damping[ai];
                                        if (damping > FloatPrecision)
                                        {
                                            auto CalcAngVel = [](const Quaternion& predQ, const Quaternion& prevQ) -> Vector {
                                                Quaternion dq = DirectX::XMQuaternionMultiply(predQ, DirectX::XMQuaternionConjugate(prevQ));
                                                if (DirectX::XMVectorGetW(dq) < 0.0f)
                                                    dq = DirectX::XMVectorNegate(dq);
                                                return DirectX::XMVectorSetW(DirectX::XMVectorScale(dq, 2.0f), 0.0f);
                                            };

                                            const Vector angVelBi = CalcAngVel(physicsBones.predRot[bi], physicsBones.rot[bi]);
                                            const Vector angVelAbi = CalcAngVel(physicsBones.predRot[abi], physicsBones.rot[abi]);

                                            const Vector relAngVel = DirectX::XMVectorSubtract(angVelBi, angVelAbi);

                                            const Vector dampingCorrectionMag = DirectX::XMVectorScale(DirectX::XMVectorNegate(DirectX::XMVector3Dot(relAngVel, correctionDir)), damping);
                                            const Vector dampCorrection = DirectX::XMVectorMultiply(correctionDir, DirectX::XMVectorScale(dampingCorrectionMag, 1.0f / wSum));

                                            if (invInertiaA > FloatPrecision)
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
            } 
        });
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
                    if (begin >= end || end - begin == 0)
                        continue;
                    const std::uint32_t oi = colliders.objIdx[begin];
                    if (oi == UINT32_MAX || objectDatas.isDisable[oi])
                        continue;
                    auto& localHashSmall = objectHashesSmall[oi];
                    auto& localHashLarge = objectHashesLarge[oi];
                    localHashSmall.Init(end - begin, SMALL_GRID_SIZE);
                    localHashLarge.Init(end - begin, LARGE_GRID_SIZE);
                    for (std::uint32_t ci = begin; ci < end; ++ci)
                    {
                        const std::uint32_t bi = colliders.boneIdx[ci];
                        if (bi == UINT32_MAX)
                            continue;
                        const float radius = colliders.boundingSphere[ci] * physicsBones.worldScale[bi] + physicsBones.collisionMargin[bi];
                        const Vector worldCenter = DirectX::XMVectorAdd(physicsBones.pred[bi], DirectX::XMVector3Rotate(DirectX::XMVectorScale(colliders.boundingSphereCenter[ci], physicsBones.worldScale[bi]), physicsBones.predRot[bi]));
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
                    localHashSmall.cellStart[0] = 0;
                    localHashLarge.cellStart[0] = 0;
                    for (std::uint32_t i = 0; i < HASH_TABLE_SIZE; ++i)
                    {
                        localHashSmall.cellStart[i + 1] = localHashSmall.cellStart[i] + localHashSmall.cellCount[i];
                        localHashSmall.cellCount[i] = 0;
                        localHashLarge.cellStart[i + 1] = localHashLarge.cellStart[i] + localHashLarge.cellCount[i];
                        localHashLarge.cellCount[i] = 0;
                    }
                    for (std::uint32_t ci = begin; ci < end; ++ci)
                    {
                        const std::uint32_t bi = colliders.boneIdx[ci];
                        if (bi == UINT32_MAX)
                            continue;
                        const float radius = colliders.boundingSphere[ci] * physicsBones.worldScale[bi] + physicsBones.collisionMargin[bi];
                        const Vector worldCenter = DirectX::XMVectorAdd(physicsBones.pred[bi], DirectX::XMVector3Rotate(DirectX::XMVectorScale(colliders.boundingSphereCenter[ci], physicsBones.worldScale[bi]), physicsBones.predRot[bi]));
                        if (radius <= SMALL_GRID_SIZE * 0.5f)
                        {
                            const std::uint32_t hashHigh = localHashSmall.HashWorldCoordsHigh(worldCenter);
                            const std::uint32_t offsetHigh = localHashSmall.cellStart[hashHigh] + localHashSmall.cellCount[hashHigh]++;
                            localHashSmall.entries[offsetHigh] = ci;
                            const std::uint32_t hashLow = localHashSmall.HashWorldCoordsLow(worldCenter);
                            const std::uint32_t offsetLow = localHashSmall.cellStart[hashLow] + localHashSmall.cellCount[hashLow]++;
                            localHashSmall.entries[offsetLow] = ci;
                        }
                        else
                        {
                            const std::uint32_t hashHigh = localHashLarge.HashWorldCoordsHigh(worldCenter);
                            const std::uint32_t offsetHigh = localHashLarge.cellStart[hashHigh] + localHashLarge.cellCount[hashHigh]++;
                            localHashLarge.entries[offsetHigh] = ci;
                            const std::uint32_t hashLow = localHashLarge.HashWorldCoordsLow(worldCenter);
                            const std::uint32_t offsetLow = localHashLarge.cellStart[hashLow] + localHashLarge.cellCount[hashLow]++;
                            localHashLarge.entries[offsetLow] = ci;
                        }
                    }
                }
            },
            tbb::auto_partitioner());
        TIMELOG_END;
    }

    void XPBDWorld::GenerateCollisionManifolds()
    {
        if (collidersGroup.empty())
            return;
        // logger::info("{}", __func__);
        TIMELOG_START;

        auto ResolveCollision = [&](const std::uint32_t coiA, const std::uint32_t coiB) {
            ContactManifold manifold;
            if (!ConvexHullvsConvexHull(coiA, coiB, manifold))
                return;
            const std::uint32_t idx = std::atomic_ref<std::uint32_t>(manifoldCacheCount).fetch_add(1, std::memory_order_relaxed);
            if (idx < expectedCollisionCount)
            {
                manifoldCache[idx] = {coiA, coiB, manifold};
            }
        };

        const std::uint32_t groups = collidersGroup.size() - 1;
        std::vector<std::vector<AABBPair>> pairs(groups);
        tbb::parallel_for(
            tbb::blocked_range<std::uint32_t>(0, groups),
            [&](const tbb::blocked_range<std::uint32_t>& r) {
                for (std::uint32_t g = r.begin(); g != r.end(); ++g)
                {
                    const std::uint32_t beginA = collidersGroup[g];
                    const std::uint32_t endA = collidersGroup[g + 1];
                    if (beginA >= endA || endA - beginA == 0)
                        continue;
                    const std::uint32_t oi = colliders.objIdx[beginA];
                    if (oi == UINT32_MAX || objectDatas.isDisable[oi])
                        continue;
                    pairs[g] = GetAABBPairs(oi);
                }
            },
            tbb::static_partitioner()
        );

        tbb::parallel_invoke([&] {
            tbb::parallel_for(
                tbb::blocked_range<std::uint32_t>(0, groups),
                [&](const tbb::blocked_range<std::uint32_t>& r) {
                    for (std::uint32_t g = r.begin(); g != r.end(); ++g)
                    {
                        const std::uint32_t beginA = collidersGroup[g];
                        const std::uint32_t endA = collidersGroup[g + 1];
                        if (beginA >= endA || endA - beginA == 0)
                            continue;
                        const std::uint32_t oi = colliders.objIdx[beginA];
                        if (oi == UINT32_MAX || objectDatas.isDisable[oi])
                            continue;
                        const auto& ownHashSmall = objectHashesSmall[oi];
                        const auto& ownHashLarge = objectHashesLarge[oi];
                        tbb::parallel_for(
                            tbb::blocked_range<std::uint32_t>(beginA, endA),
                            [&](const tbb::blocked_range<std::uint32_t>& cr) {
                                std::vector<std::uint32_t> checkedB;
                                checkedB.reserve(128);
                                for (std::uint32_t ciA = cr.begin(); ciA != cr.end(); ++ciA)
                                {
                                    const std::uint32_t biA = colliders.boneIdx[ciA];
                                    const Vector worldCenterA = DirectX::XMVectorAdd(physicsBones.pred[biA], DirectX::XMVector3Rotate(DirectX::XMVectorScale(colliders.boundingSphereCenter[ciA], physicsBones.worldScale[biA]), physicsBones.predRot[biA]));
                                    checkedB.clear();
                                    auto CheckCell = [&](const std::uint32_t hash, const LocalSpatialHash& ownHash) {
                                        const std::uint32_t beginHash = ownHash.cellStart[hash];
                                        const std::uint32_t endHash = ownHash.cellStart[hash + 1];
                                        for (std::uint32_t ei = beginHash; ei < endHash; ++ei)
                                        {
                                            const std::uint32_t ciB = ownHash.entries[ei];
                                            if (ciA >= ciB)
                                                continue;
                                            const std::uint32_t biB = colliders.boneIdx[ciB];
                                            if (biA == biB)
                                                continue;
                                            if (physicsBones.invMass[biA] + physicsBones.invMass[biB] <= FloatPrecision)
                                                continue;
                                            if (std::find(checkedB.begin(), checkedB.end(), ciB) != checkedB.end())
                                                continue;
                                            checkedB.push_back(ciB);

                                            const std::uint32_t noColCountA = colliders.noCollideCount[ciA];
                                            auto beginA = colliders.noCollideBoneIdx.begin() + static_cast<std::uint32_t>(ciA) * NOCOLLIDE_MAX;
                                            auto endA = beginA + noColCountA;
                                            const std::uint32_t noColCountB = colliders.noCollideCount[ciB];
                                            auto beginB = colliders.noCollideBoneIdx.begin() + static_cast<std::uint32_t>(ciB) * NOCOLLIDE_MAX;
                                            auto endB = beginB + noColCountB;
                                            if (std::find(beginA, endA, biB) != endA || std::find(beginB, endB, biA) != endB)
                                                continue;

                                            ResolveCollision(ciA, ciB);
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
                    }
                },
                tbb::auto_partitioner()
            ); }, [&] {
            tbb::parallel_for(
                tbb::blocked_range<std::uint32_t>(0, groups),
                [&](const tbb::blocked_range<std::uint32_t>& r) {
                    for (std::uint32_t g = r.begin(); g != r.end(); ++g)
                    {
                        const std::uint32_t beginA = collidersGroup[g];
                        const std::uint32_t endA = collidersGroup[g + 1];
                        const std::uint32_t pGroups = pairs[g].size();
                        if (pGroups == 0)
                            continue;
                        tbb::parallel_for(
                            tbb::blocked_range<std::uint32_t>(0, pGroups),
                            [&](const tbb::blocked_range<std::uint32_t>& pr) {
                                for (std::uint32_t pg = pr.begin(); pg != pr.end(); ++pg)
                                {
                                    const auto& pair = pairs[g][pg];
                                    const auto& anotherHashSmall = objectHashesSmall[pair.objIdxB];
                                    const auto& anotherHashLarge = objectHashesLarge[pair.objIdxB];

                                    tbb::parallel_for(
                                        tbb::blocked_range<std::uint32_t>(beginA, endA),
                                        [&](const tbb::blocked_range<std::uint32_t>& cr) {
                                            std::vector<std::uint32_t> checkedB;
                                            checkedB.reserve(128);

                                            for (std::uint32_t ciA = cr.begin(); ciA != cr.end(); ++ciA)
                                            {
                                                const std::uint32_t biA = colliders.boneIdx[ciA];
                                                const Vector worldCenterA = DirectX::XMVectorAdd(physicsBones.pred[biA], DirectX::XMVector3Rotate(DirectX::XMVectorScale(colliders.boundingSphereCenter[ciA], physicsBones.worldScale[biA]), physicsBones.predRot[biA]));
                                                checkedB.clear();
                                                auto CheckCell = [&](const std::uint32_t hash, const LocalSpatialHash& anotherHash) {
                                                    const std::uint32_t beginHash = anotherHash.cellStart[hash];
                                                    const std::uint32_t endHash = anotherHash.cellStart[hash + 1];
                                                    for (std::uint32_t ei = beginHash; ei < endHash; ++ei)
                                                    {
                                                        const std::uint32_t ciB = anotherHash.entries[ei];
                                                        if (ciA >= ciB)
                                                            continue;
                                                        const std::uint32_t biB = colliders.boneIdx[ciB];
                                                        if (physicsBones.invMass[biA] <= FloatPrecision && physicsBones.invMass[biB] <= FloatPrecision)
                                                            continue;
                                                        if (std::find(checkedB.begin(), checkedB.end(), ciB) != checkedB.end())
                                                            continue;
                                                        checkedB.push_back(ciB);
                                                        ResolveCollision(ciA, ciB);
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
                    }
                },
                tbb::auto_partitioner()
            );
        });
        TIMELOG_END;
    }

    void XPBDWorld::GenerateGroundCache()
    {
        if (collidersLeafs.empty() || GROUND_DETECT_RANGE <= FloatPrecision)
            return;
        // logger::info("{}", __func__);
        TIMELOG_START;
        const std::uint32_t totalLeafs = collidersLeafs.size();
        const std::uint32_t quality = std::max(1u, std::min(GROUND_DETECT_QUALITY, totalLeafs));
        const std::uint32_t chunkSize = (totalLeafs + quality - 1) / quality;
        const std::uint32_t begin = (currentFrame % quality) * chunkSize;
        const std::uint32_t end = std::min(begin + chunkSize, totalLeafs);
        if (begin >= totalLeafs)
            return;
        for (std::uint32_t ci = begin; ci != end; ++ci)
        {
            const std::uint32_t oi = physicsBones.objIdx[begin];
            if (oi == UINT32_MAX || objectDatas.isDisable[oi])
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
            const Vector pos = physicsBones.pred[bi];
            const Quaternion rot = physicsBones.predRot[bi];
            const float scale = physicsBones.worldScale[bi];

            const Vector localHalfExtents = DirectX::XMVectorMultiply(DirectX::XMVectorSubtract(localAABB.max, localAABB.min), vHalf);
            const Vector scaledHalfExtents = DirectX::XMVectorScale(localHalfExtents, scale);
            const Vector localCenter = colliders.boundingSphereCenter[ci];
            const Vector worldCenter = DirectX::XMVectorAdd(pos, DirectX::XMVector3Rotate(DirectX::XMVectorScale(localCenter, scale), rot));

            const Quaternion invRot = DirectX::XMQuaternionConjugate(rot);
            const Vector worldDown = DirectX::XMVectorSet(0.0f, 0.0f, -1.0f, 0.0f);
            const Vector localDown = DirectX::XMVector3Rotate(worldDown, invRot);
            const Vector absLocalDown = DirectX::XMVectorAbs(localDown);
            const Vector projRadius = DirectX::XMVectorScale(DirectX::XMVector3Dot(scaledHalfExtents, absLocalDown), scale);

            const Vector worldBottomCenter = DirectX::XMVectorSubtract(worldCenter, DirectX::XMVectorSet(0.0f, 0.0f, DirectX::XMVectorGetX(projRadius), 0.0f));

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
        const float InvDtSq = 1.0f / (deltaTime * deltaTime);
        auto StackManifold = [&](const std::uint32_t coiA, const std::uint32_t coiB, const ContactManifold& manifold) {
            const std::uint32_t biA = colliders.boneIdx[coiA];
            const std::uint32_t biB = colliders.boneIdx[coiB];

            const float wA = physicsBones.invMass[biA];
            const float wB = physicsBones.invMass[biB];

            if (wA + wB <= FloatPrecision)
                return;

            Vector sumNormal = vZero;
            Vector sumRA = vZero;
            Vector sumRB = vZero;
            Vector totalDepth = vZero;
            Vector maxDepth = vZero;
            for (std::uint32_t i = 0; i < manifold.pointCount; ++i)
            {
                const auto& cp = manifold.points[i];
                const Vector rA = DirectX::XMVector3Rotate(DirectX::XMVectorScale(cp.localPointA, physicsBones.worldScale[biA]), physicsBones.predRot[biA]);
                const Vector rB = DirectX::XMVector3Rotate(DirectX::XMVectorScale(cp.localPointB, physicsBones.worldScale[biB]), physicsBones.predRot[biB]);

                const Vector currWA = DirectX::XMVectorAdd(physicsBones.pred[biA], rA);
                const Vector currWB = DirectX::XMVectorAdd(physicsBones.pred[biB], rB);

                const Vector penetration = DirectX::XMVectorSubtract(currWA, currWB);
                const Vector currentDepth = DirectX::XMVectorNegate(DirectX::XMVector3Dot(penetration, manifold.normal));
                if (DirectX::XMVector3Less(vFloatPrecision, currentDepth))
                {
                    sumNormal = DirectX::XMVectorAdd(sumNormal, DirectX::XMVectorMultiply(manifold.normal, currentDepth));
                    sumRA = DirectX::XMVectorAdd(sumRA, DirectX::XMVectorMultiply(rA, currentDepth));
                    sumRB = DirectX::XMVectorAdd(sumRB, DirectX::XMVectorMultiply(rB, currentDepth));
                    totalDepth = DirectX::XMVectorAdd(totalDepth, currentDepth);
                    maxDepth = DirectX::XMVectorMax(maxDepth, currentDepth);
                }
            }

            if (DirectX::XMVector3LessOrEqual(totalDepth, vFloatPrecision))
                return;

            if (wA > FloatPrecision)
            {
                tbb::spin_mutex::scoped_lock sl(physicsBonesLock[biA]);
                physicsBones.collisionCache[biA].n = DirectX::XMVectorAdd(physicsBones.collisionCache[biA].n, sumNormal);
                physicsBones.collisionCache[biA].p = DirectX::XMVectorAdd(physicsBones.collisionCache[biA].p, sumRA);
                physicsBones.collisionCache[biA].totalDepth += DirectX::XMVectorGetX(totalDepth);
                physicsBones.collisionCache[biA].maxDepth = std::max(physicsBones.collisionCache[biA].maxDepth, DirectX::XMVectorGetX(maxDepth));
            }
            if (wB > FloatPrecision)
            {
                tbb::spin_mutex::scoped_lock sl(physicsBonesLock[biB]);
                physicsBones.collisionCache[biB].n = DirectX::XMVectorAdd(physicsBones.collisionCache[biB].n, DirectX::XMVectorNegate(sumNormal));
                physicsBones.collisionCache[biB].p = DirectX::XMVectorAdd(physicsBones.collisionCache[biB].p, sumRB);
                physicsBones.collisionCache[biB].totalDepth += DirectX::XMVectorGetX(totalDepth);
                physicsBones.collisionCache[biB].maxDepth = std::max(physicsBones.collisionCache[biB].maxDepth, DirectX::XMVectorGetX(maxDepth));
            }
        };

        tbb::parallel_for(
            tbb::blocked_range<std::uint32_t>(0, validCollisions),
            [&](const tbb::blocked_range<std::uint32_t>& r) {
                for (std::uint32_t i = r.begin(); i != r.end(); ++i)
                {
                    const auto& cached = manifoldCache[i];
                    StackManifold(cached.coiA, cached.coiB, cached.manifold);
                }
            },
            tbb::auto_partitioner()
        );
        tbb::parallel_for(
            tbb::blocked_range<std::uint32_t>(0, physicsBones.numBones, 128),
            [&](const tbb::blocked_range<std::uint32_t>& r) {
                for (std::uint32_t bi = r.begin(); bi != r.end(); ++bi)
                {
                    const float sumDepth = physicsBones.collisionCache[bi].totalDepth;
                    if (sumDepth <= FloatPrecision)
                        continue;

                    const float w = physicsBones.invMass[bi];
                    if (w <= FloatPrecision)
                        continue;

                    const Vector blendedNormal = DirectX::XMVector3Normalize(physicsBones.collisionCache[bi].n);
                    const Vector blendedPoint = DirectX::XMVectorScale(physicsBones.collisionCache[bi].p, 1.0f / sumDepth);

                    const float maxDepth = physicsBones.collisionCache[bi].maxDepth;

                    const Vector rXN = DirectX::XMVector3Cross(blendedPoint, blendedNormal);
                    const float invInertia = physicsBones.invInertia[bi] * physicsBones.collisionRotationBias[bi];
                    const float wRot = invInertia * DirectX::XMVectorGetX(DirectX::XMVector3LengthSq(rXN));
                    const float wSum = w + wRot;

                    const float alphaProxy = physicsBones.collisionCompliance[bi] * InvDtSq;
                    const float lambda = maxDepth / (wSum + alphaProxy);
                    const float relaxedLambda = lambda * COL_CONVERGENCE;
                    physicsBones.pred[bi] = DirectX::XMVectorAdd(physicsBones.pred[bi], DirectX::XMVectorScale(blendedNormal, relaxedLambda * w));

                    if (invInertia > FloatPrecision)
                    {
                        const Vector dTheta = DirectX::XMVectorScale(rXN, relaxedLambda * invInertia);
                        const Vector theta = DirectX::XMVectorSetW(dTheta, 0.0f);
                        const Vector dq = DirectX::XMVectorMultiply(DirectX::XMQuaternionMultiply(physicsBones.predRot[bi], theta), vHalf);
                        physicsBones.predRot[bi] = DirectX::XMQuaternionNormalize(DirectX::XMVectorAdd(physicsBones.predRot[bi], dq));
                    }

                    physicsBones.frictionCache[bi].n = DirectX::XMVector3Normalize(DirectX::XMVectorAdd(physicsBones.frictionCache[bi].n, DirectX::XMVectorNegate(blendedNormal)));
                    physicsBones.frictionCache[bi].depth = std::max(physicsBones.frictionCache[bi].depth, maxDepth);

                    physicsBones.collisionCache[bi] = {};
                }
            },
            tbb::static_partitioner()
        );
        TIMELOG_END;
    }

    void XPBDWorld::SolveCachedGroundCollisions(const float deltaTime)
    {
        if (collidersLeafs.empty() || GROUND_DETECT_RANGE <= FloatPrecision)
            return;
        // logger::info("{}", __func__);
        TIMELOG_START;
        const float invDtSq = 1.0f / (deltaTime * deltaTime);
        tbb::parallel_for(
            tbb::blocked_range<std::uint32_t>(0, colliders.numColliders, 32),
            [&](const tbb::blocked_range<std::uint32_t>& r) {
                for (std::uint32_t ci = r.begin(); ci != r.end(); ++ci)
                {
                    const std::uint32_t bi = colliders.boneIdx[ci];
                    if (bi == UINT32_MAX)
                        continue;
                    if (!groundCache[bi].hasHit)
                        continue;
                    const Vector w = DirectX::XMVectorReplicate(physicsBones.invMass[bi]);
                    if (DirectX::XMVector3LessOrEqual(w, vFloatPrecision))
                        continue;

                    const Vector pos = physicsBones.pred[bi];

                    const float groundHeight = groundCache[bi].height;
                    const Vector normal = groundCache[bi].normal;
                    const float scale = physicsBones.worldScale[bi];
                    const Quaternion rot = physicsBones.predRot[bi];

                    const AABB& localAABB = colliders.boundingAABB[ci];
                    const Vector localHalfExtents = DirectX::XMVectorMultiply(DirectX::XMVectorSubtract(localAABB.max, localAABB.min), vHalf);
                    const Vector scaledHalfExtents = DirectX::XMVectorScale(localHalfExtents, scale);
                    const Vector localCenter = colliders.boundingSphereCenter[ci];
                    const Vector worldCenter = DirectX::XMVectorAdd(pos, DirectX::XMVector3Rotate(DirectX::XMVectorScale(localCenter, scale), rot));

                    const Quaternion invRot = DirectX::XMQuaternionConjugate(rot);
                    const Vector localNormal = DirectX::XMVector3Rotate(normal, invRot);
                    const Vector absLocalNormal = DirectX::XMVectorAbs(localNormal);
                    const Vector projRadius = DirectX::XMVectorAdd(DirectX::XMVector3Dot(scaledHalfExtents, absLocalNormal), DirectX::XMVectorReplicate(physicsBones.collisionMargin[bi]));

                    const Vector pointOnPlane = DirectX::XMVectorSetZ(pos, groundHeight);
                    const Vector distanceToPlane = DirectX::XMVector3Dot(normal, DirectX::XMVectorSubtract(worldCenter, pointOnPlane));
                    const Vector currentDepth = DirectX::XMVectorSubtract(projRadius, distanceToPlane);
                    if (DirectX::XMVector3LessOrEqual(currentDepth, vFloatPrecision))
                        continue;

                    // linear
                    const Vector alphaProxy = DirectX::XMVectorReplicate(physicsBones.collisionCompliance[bi] * invDtSq);
                    const Vector lambda = DirectX::XMVectorMultiply(currentDepth, DirectX::XMVectorReciprocal(DirectX::XMVectorAdd(w, alphaProxy)));
                    const Vector relaxedLambda = DirectX::XMVectorScale(lambda, COL_CONVERGENCE);
                    const Vector correction = DirectX::XMVectorMultiply(normal, DirectX::XMVectorMultiply(relaxedLambda, w));
                    physicsBones.pred[bi] = DirectX::XMVectorAdd(physicsBones.pred[bi], correction);

                    // rotate
                    const Vector contactPoint = DirectX::XMVectorSubtract(worldCenter, DirectX::XMVectorMultiply(normal, projRadius));
                    const Vector rVector = DirectX::XMVectorSubtract(contactPoint, pos);

                    const Vector rXN = DirectX::XMVector3Cross(rVector, normal);
                    const float invInertia = physicsBones.invInertia[bi] * physicsBones.collisionRotationBias[bi];
                    const Vector wRot = DirectX::XMVectorScale(DirectX::XMVector3LengthSq(rXN), invInertia);
                    const Vector wSum = DirectX::XMVectorAdd(wRot, w);
                    if (FloatPrecision < invInertia)
                    {
                        const Vector dTheta = DirectX::XMVectorMultiply(rXN, DirectX::XMVectorScale(relaxedLambda, invInertia));
                        const Vector theta = DirectX::XMVectorSetW(dTheta, 0.0f);
                        const Vector dq = DirectX::XMVectorMultiply(DirectX::XMQuaternionMultiply(physicsBones.predRot[bi], theta), vHalf);
                        physicsBones.predRot[bi] = DirectX::XMQuaternionNormalize(DirectX::XMVectorAdd(physicsBones.predRot[bi], dq));
                    }

                    physicsBones.frictionCache[bi].n = DirectX::XMVectorAdd(physicsBones.frictionCache[bi].n, DirectX::XMVectorNegate(normal));
                    physicsBones.frictionCache[bi].depth = std::max(physicsBones.frictionCache[bi].depth, DirectX::XMVectorGetX(currentDepth));
                }
            },
            tbb::auto_partitioner()
        );
        TIMELOG_END;
    }

    void XPBDWorld::UpdateBoneVelocity(const float deltaTime)
    {
        // logger::info("{}", __func__);
        TIMELOG_START;
        const Vector invDt = DirectX::XMVectorReplicate(1.0f / deltaTime);
        const Vector dbInvDt = DirectX::XMVectorScale(invDt, 2.0f);
        const float gravity = DirectX::XMVectorGetX(DirectX::XMVector3Length(SkyrimGravity));
        const Vector bounceThreshold = DirectX::XMVectorReplicate(-gravity * deltaTime * 2.0f);
        tbb::parallel_for(
            tbb::blocked_range<std::uint32_t>(0, physicsBones.numBones, 128),
            [&](const tbb::blocked_range<std::uint32_t>& r) {
                for (std::uint32_t bi = r.begin(); bi != r.end(); ++bi)
                {
                    const std::uint32_t oi = physicsBones.objIdx[bi];
                    if (oi == UINT32_MAX || objectDatas.isDisable[oi])
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
                    const bool hasCollision = depth > FloatPrecision;
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
                        else if (FloatPrecision < physicsBones.linearRotTorque[bi])
                        {
                            const Vector boneDir = DirectX::XMVector3Rotate(vYone, physicsBones.predRot[bi]);
                            const Vector fakeTorque = DirectX::XMVector3Cross(boneDir, physicsBones.vel[bi]);
                            physicsBones.angVel[bi] = DirectX::XMVectorAdd(physicsBones.angVel[bi], DirectX::XMVectorScale(fakeTorque, physicsBones.linearRotTorque[bi]));
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
        const float alpha = timeAccumulator / DeltaTime60;
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
                    if (oi == UINT32_MAX || objectDatas.isDisable[oi])
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
                        const float parentWorldScale = node->parent->world.scale;

                        const Vector renderPos = DirectX::XMVectorLerp(physicsBones.prevPos[bi], physicsBones.pos[bi], alpha);
                        const RE::NiPoint3 physicsWorldPos = ToPoint3(renderPos);

                        const RE::NiMatrix3 origWorldRotMat = parentWorldRot * physicsBones.orgLocalRot[bi];
                        const Quaternion qOriginal = ToQuaternion(origWorldRotMat);
                        RE::NiMatrix3 finalWorldRot;
                        if (physicsBones.advancedRotation[bi])
                        {
                            const Quaternion renderRot = DirectX::XMQuaternionNormalize(DirectX::XMQuaternionSlerp(physicsBones.prevRot[bi], physicsBones.rot[bi], alpha));
                            finalWorldRot = ToMatrix(renderRot);
                        }
                        else
                        {
                            Vector restDir = ToVector(parentWorldRot * physicsBones.orgLocalPos[bi]);
                            if (DirectX::XMVector3Less(DirectX::XMVector3LengthSq(restDir), vFloatPrecision))
                                restDir = ToVector(parentWorldRot * (physicsBones.orgLocalRot[bi] * RE::NiPoint3(0.0f, 1.0f, 0.0f)));
                            restDir = DirectX::XMVector3Normalize(restDir);

                            Vector currentDir = DirectX::XMVectorSubtract(ToVector(physicsWorldPos), ToVector(parentWorldPos));
                            if (DirectX::XMVector3Less(DirectX::XMVector3LengthSq(currentDir), vFloatPrecision))
                                currentDir = restDir;
                            else
                                currentDir = DirectX::XMVector3Normalize(currentDir);

                            const Vector dot = DirectX::XMVector3Dot(restDir, currentDir);
                            Vector target;
                            if (DirectX::XMVector3Less(dot, vDotOppositeThreshold))
                            {
                                Vector vUp = vYone;
                                const Vector dotUp = DirectX::XMVector3Dot(restDir, vUp);
                                if (DirectX::XMVector3Less(DirectX::XMVectorAbs(dotUp), vAxisOverlapLimit))
                                    vUp = vXone;

                                const Vector axis = DirectX::XMVector3Normalize(DirectX::XMVector3Cross(vUp, restDir));
                                target = DirectX::XMQuaternionMultiply(qOriginal, DirectX::XMQuaternionRotationAxis(axis, DirectX::XM_PI));
                            }
                            else
                            {
                                const Vector rotationAxis = DirectX::XMVector3Cross(restDir, currentDir);
                                const Vector lookAt = DirectX::XMQuaternionNormalize(DirectX::XMVectorSetW(rotationAxis, DirectX::XMVectorGetX(dot) + 1.0f));
                                target = DirectX::XMQuaternionNormalize(DirectX::XMQuaternionMultiply(qOriginal, lookAt));
                            }
                            const Vector final = DirectX::XMQuaternionNormalize(DirectX::XMQuaternionSlerp(qOriginal, target, physicsBones.rotationRatio[bi]));

                            finalWorldRot = ToMatrix(final);
                            physicsBones.rot[bi] = final;
                        }

                        const RE::NiPoint3 diff = physicsWorldPos - parentWorldPos;
                        const RE::NiPoint3 localPos = (parentWorldRot.Transpose() * diff) / parentWorldScale;
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
        bool hasColliders = false;

        if (collidersGroup.empty())
            return bounds;

        const std::uint32_t groups = collidersGroup.size() - 1ull;
        for (std::uint32_t g = 0; g < groups; ++g)
        {
            std::uint32_t begin = collidersGroup[g];
            std::uint32_t end = collidersGroup[g + 1];
            if (colliders.objIdx[begin] != objIdx)
                continue;
            if (begin >= end || end - begin == 0)
                break;
            for (std::uint32_t ci = begin; ci < end; ++ci)
            {
                const std::uint32_t bi = colliders.boneIdx[ci];
                if (bi == UINT32_MAX)
                    continue;

                const Vector worldCenter = DirectX::XMVectorAdd(physicsBones.pred[bi], DirectX::XMVector3Rotate(DirectX::XMVectorScale(colliders.boundingSphereCenter[ci], physicsBones.worldScale[bi]), physicsBones.predRot[bi]));
                const float scaledRadius = colliders.boundingSphere[ci] * physicsBones.worldScale[bi] + physicsBones.collisionMargin[bi];
                const Vector vScaledRadius = DirectX::XMVectorReplicate(scaledRadius);
                AABB scaledAABB(DirectX::XMVectorSubtract(worldCenter, vScaledRadius), DirectX::XMVectorAdd(worldCenter, vScaledRadius));
                if (!hasColliders)
                {
                    bounds = scaledAABB;
                    hasColliders = true;
                }
                else
                    bounds = bounds.Merge(scaledAABB);
            }
            break;
        }
        if (!hasColliders)
            return AABB();
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
        const float scaleA = physicsBones.worldScale[biA];

        const Vector posB = physicsBones.pred[biB];
        const Quaternion rotB = physicsBones.predRot[biB];
        const float scaleB = physicsBones.worldScale[biB];

        const float marginA = physicsBones.collisionMargin[biA];
        const float marginB = physicsBones.collisionMargin[biB];
        const float sumMargin = marginA + marginB;

        const float rA = colliders.boundingSphere[coiA] * scaleA + marginA;
        const float rB = colliders.boundingSphere[coiB] * scaleB + marginB;
        const float sumR = rA + rB;

        const Vector sphereCenterA = DirectX::XMVectorAdd(posA, DirectX::XMVector3Rotate(DirectX::XMVectorScale(colliders.boundingSphereCenter[coiA], scaleA), rotA));
        const Vector sphereCenterB = DirectX::XMVectorAdd(posB, DirectX::XMVector3Rotate(DirectX::XMVectorScale(colliders.boundingSphereCenter[coiB], scaleB), rotB));
        const Vector centerToCenter = DirectX::XMVectorSubtract(sphereCenterB, sphereCenterA);
        const Vector distSq = DirectX::XMVector3LengthSq(centerToCenter);
        if (DirectX::XMVector3Less(DirectX::XMVectorReplicate(sumR * sumR), distSq))
            return false;

        DirectX::XMFLOAT3 cA_f3, cB_f3;
        DirectX::XMStoreFloat3(&cA_f3, sphereCenterA);
        DirectX::XMStoreFloat3(&cB_f3, sphereCenterB);

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

        const Vector dirAtoB = DirectX::XMVectorSubtract(posB, posA);
        const std::uint32_t fCount = std::max(hullA.faceCount, hullB.faceCount);
        for (std::uint32_t i = 0; i < fCount; ++i)
        {
            const Vector nA = DirectX::XMVector3Rotate(DirectX::XMVectorSet(hullA.fX[i], hullA.fY[i], hullA.fZ[i], 0), rotA);
            if (DirectX::XMVector3LessOrEqual(DirectX::XMVector3Dot(nA, dirAtoB), vFloatPrecision))
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
        ContactManifold::ContactPoint tempPoints[16];
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

        auto AddTempPoint = [&](const Vector& lA, const Vector& lB, const float depth) {
            for (std::int32_t i = 0; i < tempCount; ++i)
            {
                const Vector distSq = DirectX::XMVector3LengthSq(DirectX::XMVectorSubtract(tempPoints[i].localPointA, lA));
                if (DirectX::XMVector3Less(distSq, vContactMergeThresholdSq))
                    return;
            }
            if (tempCount < 16)
            {
                tempPoints[tempCount++] = {lA, lB, depth};
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

        const float invScaleA_f = 1.0f / scaleA;
        const float invScaleB_f = 1.0f / scaleB;

        DirectX::XMFLOAT3 n_p3, pA_p3, pB_p3;
        DirectX::XMStoreFloat3(&n_p3, normal);
        DirectX::XMStoreFloat3(&pA_p3, posA);
        DirectX::XMStoreFloat3(&pB_p3, posB);

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

        const __m512 v_normX = _mm512_set1_ps(n_p3.x);
        const __m512 v_normY = _mm512_set1_ps(n_p3.y);
        const __m512 v_normZ = _mm512_set1_ps(n_p3.z);
        const __m512 v_posAx = _mm512_set1_ps(pA_p3.x);
        const __m512 v_posAy = _mm512_set1_ps(pA_p3.y);
        const __m512 v_posAz = _mm512_set1_ps(pA_p3.z);
        const __m512 v_posBx = _mm512_set1_ps(pB_p3.x);
        const __m512 v_posBy = _mm512_set1_ps(pB_p3.y);
        const __m512 v_posBz = _mm512_set1_ps(pB_p3.z);
        const __m512 v_invScaleB = _mm512_set1_ps(invScaleB_f);

        const __m512i v_seq_base = _mm512_setr_epi32(0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15);
        const __m512 v_tolA = _mm512_set1_ps(minDotA + tolerance);
        for (std::uint32_t i = 0; i < COL_VERTEX_MAX; i += 16)
        {
            const __m512 vx = _mm512_load_ps(&hullA.vX[i]);
            const __m512 vy = _mm512_load_ps(&hullA.vY[i]);
            const __m512 vz = _mm512_load_ps(&hullA.vZ[i]);
            __m512 dot = _mm512_fmadd_ps(vz, v_lnAz, _mm512_fmadd_ps(vy, v_lnAy, _mm512_mul_ps(vx, v_lnAx)));
            dot = _mm512_fmadd_ps(dot, v_scaleA, v_posDotA);

            __mmask16 mask = _mm512_cmp_ps_mask(dot, v_tolA, _CMP_LE_OQ);
            if (!mask)
                continue;

            const __m512 v_pen = _mm512_sub_ps(_mm512_set1_ps(maxDotB), dot);
            mask &= _mm512_cmp_ps_mask(v_pen, _mm512_setzero_ps(), _CMP_GT_OQ);
            if (!mask)
                continue;

            const __m512 rAx = _mm512_fmadd_ps(vz, _mm512_set1_ps(fMatA._31), _mm512_fmadd_ps(vy, _mm512_set1_ps(fMatA._21), _mm512_mul_ps(vx, _mm512_set1_ps(fMatA._11))));
            const __m512 rAy = _mm512_fmadd_ps(vz, _mm512_set1_ps(fMatA._32), _mm512_fmadd_ps(vy, _mm512_set1_ps(fMatA._22), _mm512_mul_ps(vx, _mm512_set1_ps(fMatA._12))));
            const __m512 rAz = _mm512_fmadd_ps(vz, _mm512_set1_ps(fMatA._33), _mm512_fmadd_ps(vy, _mm512_set1_ps(fMatA._23), _mm512_mul_ps(vx, _mm512_set1_ps(fMatA._13))));

            const __m512 wAx = _mm512_fmadd_ps(rAx, v_scaleA, v_posAx);
            const __m512 wAy = _mm512_fmadd_ps(rAy, v_scaleA, v_posAy);
            const __m512 wAz = _mm512_fmadd_ps(rAz, v_scaleA, v_posAz);

            const __m512 wBx = _mm512_fmadd_ps(v_normX, v_pen, wAx);
            const __m512 wBy = _mm512_fmadd_ps(v_normY, v_pen, wAy);
            const __m512 wBz = _mm512_fmadd_ps(v_normZ, v_pen, wAz);

            const __m512 diffBx = _mm512_sub_ps(wBx, v_posBx);
            const __m512 diffBy = _mm512_sub_ps(wBy, v_posBy);
            const __m512 diffBz = _mm512_sub_ps(wBz, v_posBz);

            const __m512 rBx = _mm512_fmadd_ps(diffBz, _mm512_set1_ps(fMatInvB._31), _mm512_fmadd_ps(diffBy, _mm512_set1_ps(fMatInvB._21), _mm512_mul_ps(diffBx, _mm512_set1_ps(fMatInvB._11))));
            const __m512 rBy = _mm512_fmadd_ps(diffBz, _mm512_set1_ps(fMatInvB._32), _mm512_fmadd_ps(diffBy, _mm512_set1_ps(fMatInvB._22), _mm512_mul_ps(diffBx, _mm512_set1_ps(fMatInvB._12))));
            const __m512 rBz = _mm512_fmadd_ps(diffBz, _mm512_set1_ps(fMatInvB._33), _mm512_fmadd_ps(diffBy, _mm512_set1_ps(fMatInvB._23), _mm512_mul_ps(diffBx, _mm512_set1_ps(fMatInvB._13))));

            const __m512 lBx = _mm512_mul_ps(rBx, v_invScaleB);
            const __m512 lBy = _mm512_mul_ps(rBy, v_invScaleB);
            const __m512 lBz = _mm512_mul_ps(rBz, v_invScaleB);

            alignas(64) float p_lAx[16], p_lAy[16], p_lAz[16], p_lBx[16], p_lBy[16], p_lBz[16], p_pen[16];
            alignas(64) uint32_t p_idx[16];
            _mm512_mask_compressstoreu_ps(p_lAx, mask, vx);
            _mm512_mask_compressstoreu_ps(p_lAy, mask, vy);
            _mm512_mask_compressstoreu_ps(p_lAz, mask, vz);
            _mm512_mask_compressstoreu_ps(p_lBx, mask, lBx);
            _mm512_mask_compressstoreu_ps(p_lBy, mask, lBy);
            _mm512_mask_compressstoreu_ps(p_lBz, mask, lBz);
            _mm512_mask_compressstoreu_ps(p_pen, mask, v_pen);
            const __m512i v_idx = _mm512_add_epi32(v_seq_base, _mm512_set1_epi32(i));
            _mm512_mask_compressstoreu_epi32(p_idx, mask, v_idx);

            int count = _mm_popcnt_u32(mask);
            for (int j = 0; j < count; ++j)
            {
                if (p_idx[j] >= vCountA)
                    continue;
                AddTempPoint(DirectX::XMVectorSet(p_lAx[j], p_lAy[j], p_lAz[j], 0), DirectX::XMVectorSet(p_lBx[j], p_lBy[j], p_lBz[j], 0), p_pen[j]);
            }
        }

        const __m512 v_invScaleA = _mm512_set1_ps(invScaleA_f);
        const __m512 v_tolB = _mm512_set1_ps(maxDotB - tolerance);
        for (std::uint32_t i = 0; i < COL_VERTEX_MAX; i += 16)
        {
            const __m512 vx = _mm512_load_ps(&hullB.vX[i]);
            const __m512 vy = _mm512_load_ps(&hullB.vY[i]);
            const __m512 vz = _mm512_load_ps(&hullB.vZ[i]);
            __m512 dot = _mm512_fmadd_ps(vz, v_lnBz, _mm512_fmadd_ps(vy, v_lnBy, _mm512_mul_ps(vx, v_lnBx)));
            dot = _mm512_fmadd_ps(dot, v_scaleB, v_posDotB);

            __mmask16 mask = _mm512_cmp_ps_mask(dot, v_tolB, _CMP_GE_OQ);
            if (!mask)
                continue;

            const __m512 v_pen = _mm512_sub_ps(dot, _mm512_set1_ps(minDotA));
            mask &= _mm512_cmp_ps_mask(v_pen, _mm512_setzero_ps(), _CMP_GT_OQ);
            if (!mask)
                continue;

            const __m512 rBx = _mm512_fmadd_ps(vz, _mm512_set1_ps(fMatB._31), _mm512_fmadd_ps(vy, _mm512_set1_ps(fMatB._21), _mm512_mul_ps(vx, _mm512_set1_ps(fMatB._11))));
            const __m512 rBy = _mm512_fmadd_ps(vz, _mm512_set1_ps(fMatB._32), _mm512_fmadd_ps(vy, _mm512_set1_ps(fMatB._22), _mm512_mul_ps(vx, _mm512_set1_ps(fMatB._12))));
            const __m512 rBz = _mm512_fmadd_ps(vz, _mm512_set1_ps(fMatB._33), _mm512_fmadd_ps(vy, _mm512_set1_ps(fMatB._23), _mm512_mul_ps(vx, _mm512_set1_ps(fMatB._13))));

            const __m512 wBx = _mm512_fmadd_ps(rBx, v_scaleB, v_posBx);
            const __m512 wBy = _mm512_fmadd_ps(rBy, v_scaleB, v_posBy);
            const __m512 wBz = _mm512_fmadd_ps(rBz, v_scaleB, v_posBz);

            const __m512 wAx = _mm512_fnmadd_ps(v_normX, v_pen, wBx);
            const __m512 wAy = _mm512_fnmadd_ps(v_normY, v_pen, wBy);
            const __m512 wAz = _mm512_fnmadd_ps(v_normZ, v_pen, wBz);

            const __m512 diffAx = _mm512_sub_ps(wAx, v_posAx);
            const __m512 diffAy = _mm512_sub_ps(wAy, v_posAy);
            const __m512 diffAz = _mm512_sub_ps(wAz, v_posAz);

            const __m512 rAx = _mm512_fmadd_ps(diffAz, _mm512_set1_ps(fMatInvA._31), _mm512_fmadd_ps(diffAy, _mm512_set1_ps(fMatInvA._21), _mm512_mul_ps(diffAx, _mm512_set1_ps(fMatInvA._11))));
            const __m512 rAy = _mm512_fmadd_ps(diffAz, _mm512_set1_ps(fMatInvA._32), _mm512_fmadd_ps(diffAy, _mm512_set1_ps(fMatInvA._22), _mm512_mul_ps(diffAx, _mm512_set1_ps(fMatInvA._12))));
            const __m512 rAz = _mm512_fmadd_ps(diffAz, _mm512_set1_ps(fMatInvA._33), _mm512_fmadd_ps(diffAy, _mm512_set1_ps(fMatInvA._23), _mm512_mul_ps(diffAx, _mm512_set1_ps(fMatInvA._13))));

            const __m512 lAx = _mm512_mul_ps(rAx, v_invScaleA);
            const __m512 lAy = _mm512_mul_ps(rAy, v_invScaleA);
            const __m512 lAz = _mm512_mul_ps(rAz, v_invScaleA);

            alignas(64) float p_lAx[16], p_lAy[16], p_lAz[16], p_lBx[16], p_lBy[16], p_lBz[16], p_pen[16];
            alignas(64) uint32_t p_idx[16];
            _mm512_mask_compressstoreu_ps(p_lAx, mask, lAx);
            _mm512_mask_compressstoreu_ps(p_lAy, mask, lAy);
            _mm512_mask_compressstoreu_ps(p_lAz, mask, lAz);
            _mm512_mask_compressstoreu_ps(p_lBx, mask, vx);
            _mm512_mask_compressstoreu_ps(p_lBy, mask, vy);
            _mm512_mask_compressstoreu_ps(p_lBz, mask, vz);
            _mm512_mask_compressstoreu_ps(p_pen, mask, v_pen);
            const __m512i v_idx = _mm512_add_epi32(v_seq_base, _mm512_set1_epi32(i));
            _mm512_mask_compressstoreu_epi32(p_idx, mask, v_idx);

            int count = _mm_popcnt_u32(mask);
            for (int j = 0; j < count; ++j)
            {
                if (p_idx[j] >= vCountB)
                    continue;
                AddTempPoint(DirectX::XMVectorSet(p_lAx[j], p_lAy[j], p_lAz[j], 0), DirectX::XMVectorSet(p_lBx[j], p_lBy[j], p_lBz[j], 0), p_pen[j]);
            }
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

        const __m256 v_normX = _mm256_set1_ps(n_p3.x);
        const __m256 v_normY = _mm256_set1_ps(n_p3.y);
        const __m256 v_normZ = _mm256_set1_ps(n_p3.z);
        const __m256 v_posAx = _mm256_set1_ps(pA_p3.x);
        const __m256 v_posAy = _mm256_set1_ps(pA_p3.y);
        const __m256 v_posAz = _mm256_set1_ps(pA_p3.z);
        const __m256 v_posBx = _mm256_set1_ps(pB_p3.x);
        const __m256 v_posBy = _mm256_set1_ps(pB_p3.y);
        const __m256 v_posBz = _mm256_set1_ps(pB_p3.z);
        const __m256 v_invScaleB = _mm256_set1_ps(invScaleB_f);
        const __m256 v_tolA = _mm256_set1_ps(minDotA + tolerance);

        for (std::uint32_t i = 0; i < COL_VERTEX_MAX; i += 8)
        {
            const __m256 vx = _mm256_load_ps(&hullA.vX[i]);
            const __m256 vy = _mm256_load_ps(&hullA.vY[i]);
            const __m256 vz = _mm256_load_ps(&hullA.vZ[i]);
            __m256 dot = _mm256_fmadd_ps(vz, v_lnAz, _mm256_fmadd_ps(vy, v_lnAy, _mm256_mul_ps(vx, v_lnAx)));
            dot = _mm256_fmadd_ps(dot, v_scaleA, v_posDotA);

            const __m256 cmp = _mm256_cmp_ps(dot, v_tolA, _CMP_LE_OQ);
            int mask = _mm256_movemask_ps(cmp);
            if (!mask)
                continue;

            const __m256 v_pen = _mm256_sub_ps(_mm256_set1_ps(maxDotB), dot);
            mask &= _mm256_movemask_ps(_mm256_cmp_ps(v_pen, _mm256_setzero_ps(), _CMP_GT_OQ));
            if (!mask)
                continue;

            const __m256 rAx = _mm256_fmadd_ps(vz, _mm256_set1_ps(fMatA._31), _mm256_fmadd_ps(vy, _mm256_set1_ps(fMatA._21), _mm256_mul_ps(vx, _mm256_set1_ps(fMatA._11))));
            const __m256 rAy = _mm256_fmadd_ps(vz, _mm256_set1_ps(fMatA._32), _mm256_fmadd_ps(vy, _mm256_set1_ps(fMatA._22), _mm256_mul_ps(vx, _mm256_set1_ps(fMatA._12))));
            const __m256 rAz = _mm256_fmadd_ps(vz, _mm256_set1_ps(fMatA._33), _mm256_fmadd_ps(vy, _mm256_set1_ps(fMatA._23), _mm256_mul_ps(vx, _mm256_set1_ps(fMatA._13))));

            const __m256 wAx = _mm256_fmadd_ps(rAx, v_scaleA, v_posAx);
            const __m256 wAy = _mm256_fmadd_ps(rAy, v_scaleA, v_posAy);
            const __m256 wAz = _mm256_fmadd_ps(rAz, v_scaleA, v_posAz);

            const __m256 wBx = _mm256_fmadd_ps(v_normX, v_pen, wAx);
            const __m256 wBy = _mm256_fmadd_ps(v_normY, v_pen, wAy);
            const __m256 wBz = _mm256_fmadd_ps(v_normZ, v_pen, wAz);

            const __m256 diffBx = _mm256_sub_ps(wBx, v_posBx);
            const __m256 diffBy = _mm256_sub_ps(wBy, v_posBy);
            const __m256 diffBz = _mm256_sub_ps(wBz, v_posBz);

            const __m256 rBx = _mm256_fmadd_ps(diffBz, _mm256_set1_ps(fMatInvB._31), _mm256_fmadd_ps(diffBy, _mm256_set1_ps(fMatInvB._21), _mm256_mul_ps(diffBx, _mm256_set1_ps(fMatInvB._11))));
            const __m256 rBy = _mm256_fmadd_ps(diffBz, _mm256_set1_ps(fMatInvB._32), _mm256_fmadd_ps(diffBy, _mm256_set1_ps(fMatInvB._22), _mm256_mul_ps(diffBx, _mm256_set1_ps(fMatInvB._12))));
            const __m256 rBz = _mm256_fmadd_ps(diffBz, _mm256_set1_ps(fMatInvB._33), _mm256_fmadd_ps(diffBy, _mm256_set1_ps(fMatInvB._23), _mm256_mul_ps(diffBx, _mm256_set1_ps(fMatInvB._13))));

            const __m256 lBx = _mm256_mul_ps(rBx, v_invScaleB);
            const __m256 lBy = _mm256_mul_ps(rBy, v_invScaleB);
            const __m256 lBz = _mm256_mul_ps(rBz, v_invScaleB);

            alignas(32) float p_lBx[8], p_lBy[8], p_lBz[8], p_pen[8];
            _mm256_store_ps(p_lBx, lBx);
            _mm256_store_ps(p_lBy, lBy);
            _mm256_store_ps(p_lBz, lBz);
            _mm256_store_ps(p_pen, v_pen);

            while (mask)
            {
                std::uint32_t k = _tzcnt_u32(mask);
                mask &= mask - 1;
                const std::uint32_t idx = i + k;
                if (idx >= vCountA)
                    continue;

                AddTempPoint(DirectX::XMVectorSet(hullA.vX[idx], hullA.vY[idx], hullA.vZ[idx], 0.0f),
                             DirectX::XMVectorSet(p_lBx[k], p_lBy[k], p_lBz[k], 0.0f), p_pen[k]);
            }
        }

        const __m256 v_invScaleA = _mm256_set1_ps(invScaleA_f);
        const __m256 v_tolB = _mm256_set1_ps(maxDotB - tolerance);

        for (std::uint32_t i = 0; i < COL_VERTEX_MAX; i += 8)
        {
            const __m256 vx = _mm256_load_ps(&hullB.vX[i]);
            const __m256 vy = _mm256_load_ps(&hullB.vY[i]);
            const __m256 vz = _mm256_load_ps(&hullB.vZ[i]);
            __m256 dot = _mm256_fmadd_ps(vz, v_lnBz, _mm256_fmadd_ps(vy, v_lnBy, _mm256_mul_ps(vx, v_lnBx)));
            dot = _mm256_fmadd_ps(dot, v_scaleB, v_posDotB);

            const __m256 cmp = _mm256_cmp_ps(dot, v_tolB, _CMP_GE_OQ);
            int mask = _mm256_movemask_ps(cmp);
            if (!mask)
                continue;

            const __m256 v_pen = _mm256_sub_ps(dot, _mm256_set1_ps(minDotA));
            mask &= _mm256_movemask_ps(_mm256_cmp_ps(v_pen, _mm256_setzero_ps(), _CMP_GT_OQ));
            if (!mask)
                continue;

            const __m256 rBx = _mm256_fmadd_ps(vz, _mm256_set1_ps(fMatB._31), _mm256_fmadd_ps(vy, _mm256_set1_ps(fMatB._21), _mm256_mul_ps(vx, _mm256_set1_ps(fMatB._11))));
            const __m256 rBy = _mm256_fmadd_ps(vz, _mm256_set1_ps(fMatB._32), _mm256_fmadd_ps(vy, _mm256_set1_ps(fMatB._22), _mm256_mul_ps(vx, _mm256_set1_ps(fMatB._12))));
            const __m256 rBz = _mm256_fmadd_ps(vz, _mm256_set1_ps(fMatB._33), _mm256_fmadd_ps(vy, _mm256_set1_ps(fMatB._23), _mm256_mul_ps(vx, _mm256_set1_ps(fMatB._13))));

            const __m256 wBx = _mm256_fmadd_ps(rBx, v_scaleB, v_posBx);
            const __m256 wBy = _mm256_fmadd_ps(rBy, v_scaleB, v_posBy);
            const __m256 wBz = _mm256_fmadd_ps(rBz, v_scaleB, v_posBz);

            const __m256 wAx = _mm256_fnmadd_ps(v_normX, v_pen, wBx);
            const __m256 wAy = _mm256_fnmadd_ps(v_normY, v_pen, wBy);
            const __m256 wAz = _mm256_fnmadd_ps(v_normZ, v_pen, wBz);

            const __m256 diffAx = _mm256_sub_ps(wAx, v_posAx);
            const __m256 diffAy = _mm256_sub_ps(wAy, v_posAy);
            const __m256 diffAz = _mm256_sub_ps(wAz, v_posAz);

            const __m256 rAx = _mm256_fmadd_ps(diffAz, _mm256_set1_ps(fMatInvA._31), _mm256_fmadd_ps(diffAy, _mm256_set1_ps(fMatInvA._21), _mm256_mul_ps(diffAx, _mm256_set1_ps(fMatInvA._11))));
            const __m256 rAy = _mm256_fmadd_ps(diffAz, _mm256_set1_ps(fMatInvA._32), _mm256_fmadd_ps(diffAy, _mm256_set1_ps(fMatInvA._22), _mm256_mul_ps(diffAx, _mm256_set1_ps(fMatInvA._12))));
            const __m256 rAz = _mm256_fmadd_ps(diffAz, _mm256_set1_ps(fMatInvA._33), _mm256_fmadd_ps(diffAy, _mm256_set1_ps(fMatInvA._23), _mm256_mul_ps(diffAx, _mm256_set1_ps(fMatInvA._13))));

            const __m256 lAx = _mm256_mul_ps(rAx, v_invScaleA);
            const __m256 lAy = _mm256_mul_ps(rAy, v_invScaleA);
            const __m256 lAz = _mm256_mul_ps(rAz, v_invScaleA);

            alignas(32) float p_lAx[8], p_lAy[8], p_lAz[8], p_pen[8];
            _mm256_store_ps(p_lAx, lAx);
            _mm256_store_ps(p_lAy, lAy);
            _mm256_store_ps(p_lAz, lAz);
            _mm256_store_ps(p_pen, v_pen);

            while (mask)
            {
                std::uint32_t k = _tzcnt_u32(mask);
                mask &= mask - 1;
                const std::uint32_t idx = i + k;
                if (idx >= vCountB)
                    continue;

                AddTempPoint(DirectX::XMVectorSet(p_lAx[k], p_lAy[k], p_lAz[k], 0.0f),
                             DirectX::XMVectorSet(hullB.vX[idx], hullB.vY[idx], hullB.vZ[idx], 0.0f), p_pen[k]);
            }
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

        const __m256 v_normX = _mm256_set1_ps(n_p3.x);
        const __m256 v_normY = _mm256_set1_ps(n_p3.y);
        const __m256 v_normZ = _mm256_set1_ps(n_p3.z);
        const __m256 v_posAx = _mm256_set1_ps(pA_p3.x);
        const __m256 v_posAy = _mm256_set1_ps(pA_p3.y);
        const __m256 v_posAz = _mm256_set1_ps(pA_p3.z);
        const __m256 v_posBx = _mm256_set1_ps(pB_p3.x);
        const __m256 v_posBy = _mm256_set1_ps(pB_p3.y);
        const __m256 v_posBz = _mm256_set1_ps(pB_p3.z);
        const __m256 v_invScaleB = _mm256_set1_ps(invScaleB_f);
        const __m256 v_tolA = _mm256_set1_ps(minDotA + tolerance);
        for (std::uint32_t i = 0; i < COL_VERTEX_MAX; i += 8)
        {
            const __m256 vx = _mm256_load_ps(&hullA.vX[i]);
            const __m256 vy = _mm256_load_ps(&hullA.vY[i]);
            const __m256 vz = _mm256_load_ps(&hullA.vZ[i]);
            __m256 dot = _mm256_add_ps(_mm256_add_ps(_mm256_mul_ps(vx, v_lnAx), _mm256_mul_ps(vy, v_lnAy)), _mm256_mul_ps(vz, v_lnAz));
            dot = _mm256_add_ps(_mm256_mul_ps(dot, v_scaleA), v_posDotA);

            const __m256 cmp = _mm256_cmp_ps(dot, v_tolA, _CMP_LE_OQ);
            int mask = _mm256_movemask_ps(cmp);
            if (!mask)
                continue;

            const __m256 v_pen = _mm256_sub_ps(_mm256_set1_ps(maxDotB), dot);
            mask &= _mm256_movemask_ps(_mm256_cmp_ps(v_pen, _mm256_setzero_ps(), _CMP_GT_OQ));
            if (!mask)
                continue;

            const __m256 rAx = _mm256_add_ps(_mm256_add_ps(_mm256_mul_ps(vx, _mm256_set1_ps(fMatA._11)), _mm256_mul_ps(vy, _mm256_set1_ps(fMatA._21))), _mm256_mul_ps(vz, _mm256_set1_ps(fMatA._31)));
            const __m256 rAy = _mm256_add_ps(_mm256_add_ps(_mm256_mul_ps(vx, _mm256_set1_ps(fMatA._12)), _mm256_mul_ps(vy, _mm256_set1_ps(fMatA._22))), _mm256_mul_ps(vz, _mm256_set1_ps(fMatA._32)));
            const __m256 rAz = _mm256_add_ps(_mm256_add_ps(_mm256_mul_ps(vx, _mm256_set1_ps(fMatA._13)), _mm256_mul_ps(vy, _mm256_set1_ps(fMatA._23))), _mm256_mul_ps(vz, _mm256_set1_ps(fMatA._33)));

            const __m256 wAx = _mm256_add_ps(_mm256_mul_ps(rAx, v_scaleA), v_posAx);
            const __m256 wAy = _mm256_add_ps(_mm256_mul_ps(rAy, v_scaleA), v_posAy);
            const __m256 wAz = _mm256_add_ps(_mm256_mul_ps(rAz, v_scaleA), v_posAz);

            const __m256 wBx = _mm256_add_ps(_mm256_mul_ps(v_normX, v_pen), wAx);
            const __m256 wBy = _mm256_add_ps(_mm256_mul_ps(v_normY, v_pen), wAy);
            const __m256 wBz = _mm256_add_ps(_mm256_mul_ps(v_normZ, v_pen), wAz);

            const __m256 diffBx = _mm256_sub_ps(wBx, v_posBx);
            const __m256 diffBy = _mm256_sub_ps(wBy, v_posBy);
            const __m256 diffBz = _mm256_sub_ps(wBz, v_posBz);

            const __m256 rBx = _mm256_add_ps(_mm256_add_ps(_mm256_mul_ps(diffBx, _mm256_set1_ps(fMatInvB._11)), _mm256_mul_ps(diffBy, _mm256_set1_ps(fMatInvB._21))), _mm256_mul_ps(diffBz, _mm256_set1_ps(fMatInvB._31)));
            const __m256 rBy = _mm256_add_ps(_mm256_add_ps(_mm256_mul_ps(diffBx, _mm256_set1_ps(fMatInvB._12)), _mm256_mul_ps(diffBy, _mm256_set1_ps(fMatInvB._22))), _mm256_mul_ps(diffBz, _mm256_set1_ps(fMatInvB._32)));
            const __m256 rBz = _mm256_add_ps(_mm256_add_ps(_mm256_mul_ps(diffBx, _mm256_set1_ps(fMatInvB._13)), _mm256_mul_ps(diffBy, _mm256_set1_ps(fMatInvB._23))), _mm256_mul_ps(diffBz, _mm256_set1_ps(fMatInvB._33)));

            const __m256 lBx = _mm256_mul_ps(rBx, v_invScaleB);
            const __m256 lBy = _mm256_mul_ps(rBy, v_invScaleB);
            const __m256 lBz = _mm256_mul_ps(rBz, v_invScaleB);

            alignas(32) float p_lBx[8], p_lBy[8], p_lBz[8], p_pen[8];
            _mm256_store_ps(p_lBx, lBx);
            _mm256_store_ps(p_lBy, lBy);
            _mm256_store_ps(p_lBz, lBz);
            _mm256_store_ps(p_pen, v_pen);

            while (mask)
            {
                std::uint32_t k = _tzcnt_u32(mask);
                mask &= mask - 1;
                const std::uint32_t idx = i + k;
                if (idx >= vCountA)
                    continue;

                AddTempPoint(DirectX::XMVectorSet(hullA.vX[idx], hullA.vY[idx], hullA.vZ[idx], 0.0f),
                             DirectX::XMVectorSet(p_lBx[k], p_lBy[k], p_lBz[k], 0.0f), p_pen[k]);
            }
        }

        const __m256 v_invScaleA = _mm256_set1_ps(invScaleA_f);
        const __m256 v_tolB = _mm256_set1_ps(maxDotB - tolerance);
        for (std::uint32_t i = 0; i < COL_VERTEX_MAX; i += 8)
        {
            const __m256 vx = _mm256_load_ps(&hullB.vX[i]);
            const __m256 vy = _mm256_load_ps(&hullB.vY[i]);
            const __m256 vz = _mm256_load_ps(&hullB.vZ[i]);
            __m256 dot = _mm256_add_ps(_mm256_add_ps(_mm256_mul_ps(vx, v_lnBx), _mm256_mul_ps(vy, v_lnBy)), _mm256_mul_ps(vz, v_lnBz));
            dot = _mm256_add_ps(_mm256_mul_ps(dot, v_scaleB), v_posDotB);

            const __m256 cmp = _mm256_cmp_ps(dot, v_tolB, _CMP_GE_OQ);
            int mask = _mm256_movemask_ps(cmp);
            if (!mask)
                continue;

            const __m256 v_pen = _mm256_sub_ps(dot, _mm256_set1_ps(minDotA));
            mask &= _mm256_movemask_ps(_mm256_cmp_ps(v_pen, _mm256_setzero_ps(), _CMP_GT_OQ));
            if (!mask)
                continue;

            const __m256 rBx = _mm256_add_ps(_mm256_add_ps(_mm256_mul_ps(vx, _mm256_set1_ps(fMatB._11)), _mm256_mul_ps(vy, _mm256_set1_ps(fMatB._21))), _mm256_mul_ps(vz, _mm256_set1_ps(fMatB._31)));
            const __m256 rBy = _mm256_add_ps(_mm256_add_ps(_mm256_mul_ps(vx, _mm256_set1_ps(fMatB._12)), _mm256_mul_ps(vy, _mm256_set1_ps(fMatB._22))), _mm256_mul_ps(vz, _mm256_set1_ps(fMatB._32)));
            const __m256 rBz = _mm256_add_ps(_mm256_add_ps(_mm256_mul_ps(vx, _mm256_set1_ps(fMatB._13)), _mm256_mul_ps(vy, _mm256_set1_ps(fMatB._23))), _mm256_mul_ps(vz, _mm256_set1_ps(fMatB._33)));

            const __m256 wBx = _mm256_add_ps(_mm256_mul_ps(rBx, v_scaleB), v_posBx);
            const __m256 wBy = _mm256_add_ps(_mm256_mul_ps(rBy, v_scaleB), v_posBy);
            const __m256 wBz = _mm256_add_ps(_mm256_mul_ps(rBz, v_scaleB), v_posBz);

            const __m256 wAx = _mm256_sub_ps(wBx, _mm256_mul_ps(v_normX, v_pen));
            const __m256 wAy = _mm256_sub_ps(wBy, _mm256_mul_ps(v_normY, v_pen));
            const __m256 wAz = _mm256_sub_ps(wBz, _mm256_mul_ps(v_normZ, v_pen));

            const __m256 diffAx = _mm256_sub_ps(wAx, v_posAx);
            const __m256 diffAy = _mm256_sub_ps(wAy, v_posAy);
            const __m256 diffAz = _mm256_sub_ps(wAz, v_posAz);

            const __m256 rAx = _mm256_add_ps(_mm256_add_ps(_mm256_mul_ps(diffAx, _mm256_set1_ps(fMatInvA._11)), _mm256_mul_ps(diffAy, _mm256_set1_ps(fMatInvA._21))), _mm256_mul_ps(diffAz, _mm256_set1_ps(fMatInvA._31)));
            const __m256 rAy = _mm256_add_ps(_mm256_add_ps(_mm256_mul_ps(diffAx, _mm256_set1_ps(fMatInvA._12)), _mm256_mul_ps(diffAy, _mm256_set1_ps(fMatInvA._22))), _mm256_mul_ps(diffAz, _mm256_set1_ps(fMatInvA._32)));
            const __m256 rAz = _mm256_add_ps(_mm256_add_ps(_mm256_mul_ps(diffAx, _mm256_set1_ps(fMatInvA._13)), _mm256_mul_ps(diffAy, _mm256_set1_ps(fMatInvA._23))), _mm256_mul_ps(diffAz, _mm256_set1_ps(fMatInvA._33)));

            const __m256 lAx = _mm256_mul_ps(rAx, v_invScaleA);
            const __m256 lAy = _mm256_mul_ps(rAy, v_invScaleA);
            const __m256 lAz = _mm256_mul_ps(rAz, v_invScaleA);

            alignas(32) float p_lAx[8], p_lAy[8], p_lAz[8], p_pen[8];
            _mm256_store_ps(p_lAx, lAx);
            _mm256_store_ps(p_lAy, lAy);
            _mm256_store_ps(p_lAz, lAz);
            _mm256_store_ps(p_pen, v_pen);

            while (mask)
            {
                std::uint32_t k = _tzcnt_u32(mask);
                mask &= mask - 1;
                const std::uint32_t idx = i + k;
                if (idx >= vCountB)
                    continue;

                AddTempPoint(DirectX::XMVectorSet(p_lAx[k], p_lAy[k], p_lAz[k], 0.0f),
                             DirectX::XMVectorSet(hullB.vX[idx], hullB.vY[idx], hullB.vZ[idx], 0.0f), p_pen[k]);
            }
        }
#endif

        cache.persistentManifold.normal = normal;
        if (tempCount <= 4)
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
            cache.persistentManifold.points[mfCount++] = tempPoints[p1];
            if (p2 != -1)
                cache.persistentManifold.points[mfCount++] = tempPoints[p2];
            if (p3 != -1)
                cache.persistentManifold.points[mfCount++] = tempPoints[p3];
            if (p4 != -1)
                cache.persistentManifold.points[mfCount++] = tempPoints[p4];
            cache.persistentManifold.pointCount = mfCount;
        }

        outManifold = cache.persistentManifold;
        return true;
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
        physicsBones.rotationRatio.reserve(n);
        physicsBones.gravity.reserve(n);
        physicsBones.offset.reserve(n);
        physicsBones.invMass.reserve(n);

        physicsBones.linearRotTorque.reserve(n);

        physicsBones.collisionMargin.reserve(n);
        physicsBones.colShrink.reserve(n);
        physicsBones.collisionFriction.reserve(n);
        physicsBones.collisionRotationBias.reserve(n);
        physicsBones.collisionCompliance.reserve(n);

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

        physicsBones.worldScale.reserve(n);
        physicsBones.worldRot.reserve(n);
        physicsBones.orgLocalPos.reserve(n);
        physicsBones.orgLocalRot.reserve(n);
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
        physicsBones.rotationRatio.push_back(0);
        physicsBones.gravity.push_back(ToVector(GetSkyrimGravity(1.0f)));
        physicsBones.offset.push_back({0, 0, 0});
        physicsBones.invMass.push_back(0);

        physicsBones.linearRotTorque.push_back(0);

        physicsBones.collisionMargin.push_back(0);
        physicsBones.colShrink.push_back(0);
        physicsBones.collisionFriction.push_back(0);
        physicsBones.collisionRotationBias.push_back(0);
        physicsBones.collisionCompliance.push_back(0);

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

        physicsBones.worldScale.push_back(1);
        physicsBones.worldRot.push_back(RE::NiMatrix3());
        physicsBones.orgLocalPos.push_back(RE::NiPoint3());
        physicsBones.orgLocalRot.push_back(RE::NiMatrix3());
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
        constraints.anchIdx.reserve(an);
        constraints.restLen.reserve(an);
        constraints.compSquish.reserve(an);
        constraints.compStretch.reserve(an);
        constraints.squishLimit.reserve(an);
        constraints.stretchLimit.reserve(an);
        constraints.angularLimit.reserve(an);
        constraints.restDirLocal.reserve(an);
        constraints.squishDamping.reserve(an);
        constraints.stretchDamping.reserve(an);
        constraints.lambda.reserve(an);
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
            constraints.anchIdx.push_back(UINT32_MAX);
            constraints.restLen.push_back(0.0f);
            constraints.compSquish.push_back(0.0f);
            constraints.compStretch.push_back(0.0f);
            constraints.squishLimit.push_back(0.0f);
            constraints.stretchLimit.push_back(0.0f);
            constraints.angularLimit.push_back(0.0f);
            constraints.restDirLocal.push_back(vZero);
            constraints.squishDamping.push_back(0.0f);
            constraints.stretchDamping.push_back(0.0f);
            constraints.lambda.push_back(0.0f);
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
        angularConstraints.anchIdx.reserve(an);
        angularConstraints.restRot.reserve(an);
        angularConstraints.comp.reserve(an);
        angularConstraints.limit.reserve(an);
        angularConstraints.damping.reserve(an);
        angularConstraints.lambda.reserve(an);
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
            angularConstraints.anchIdx.push_back(UINT32_MAX);
            angularConstraints.restRot.push_back(vZero);
            angularConstraints.comp.push_back(0.0f);
            angularConstraints.limit.push_back(0.0f);
            angularConstraints.damping.push_back(0.0f);
            angularConstraints.lambda.push_back(0.0f);
        }
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
                    if (objIdxA != UINT32_MAX && objIdxB != UINT32_MAX && objectDatas.isDisable[objIdxA] != objectDatas.isDisable[objIdxB])
                        return objectDatas.isDisable[objIdxA] < objectDatas.isDisable[objIdxB];
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
                                physicsBones.rotationRatio[i] = tmpPhysicsBones.rotationRatio[srcIdx];
                                physicsBones.gravity[i] = tmpPhysicsBones.gravity[srcIdx];
                                physicsBones.offset[i] = tmpPhysicsBones.offset[srcIdx];
                                physicsBones.invMass[i] = tmpPhysicsBones.invMass[srcIdx];

                                physicsBones.linearRotTorque[i] = tmpPhysicsBones.linearRotTorque[srcIdx];

                                physicsBones.collisionMargin[i] = tmpPhysicsBones.collisionMargin[srcIdx];
                                physicsBones.colShrink[i] = tmpPhysicsBones.colShrink[srcIdx];
                                physicsBones.collisionFriction[i] = tmpPhysicsBones.collisionFriction[srcIdx];
                                physicsBones.collisionRotationBias[i] = tmpPhysicsBones.collisionRotationBias[srcIdx];
                                physicsBones.collisionCompliance[i] = tmpPhysicsBones.collisionCompliance[srcIdx];

                                physicsBones.node[i] = tmpPhysicsBones.node[srcIdx];
                                physicsBones.particleName[i] = tmpPhysicsBones.particleName[srcIdx];
                                physicsBones.isParticle[i] = tmpPhysicsBones.isParticle[srcIdx];
                                physicsBones.particleDepth[i] = tmpPhysicsBones.particleDepth[srcIdx];
                                physicsBones.parentBoneIdx[i] = tmpPhysicsBones.parentBoneIdx[srcIdx];
                                physicsBones.objIdx[i] = tmpPhysicsBones.objIdx[srcIdx];
                                physicsBones.rootIdx[i] = tmpPhysicsBones.rootIdx[srcIdx];
                                physicsBones.depth[i] = tmpPhysicsBones.depth[srcIdx];

                                physicsBones.worldScale[i] = tmpPhysicsBones.worldScale[srcIdx];
                                physicsBones.worldRot[i] = tmpPhysicsBones.worldRot[srcIdx];
                                physicsBones.orgLocalPos[i] = tmpPhysicsBones.orgLocalPos[srcIdx];
                                physicsBones.orgLocalRot[i] = tmpPhysicsBones.orgLocalRot[srcIdx];
                            }
                        },
                        tbb::static_partitioner());

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
                    physicsBones.rotationRatio.resize(validCount);
                    physicsBones.gravity.resize(validCount);
                    physicsBones.offset.resize(validCount);
                    physicsBones.invMass.resize(validCount);

                    physicsBones.linearRotTorque.resize(validCount);

                    physicsBones.collisionMargin.resize(validCount);
                    physicsBones.colShrink.resize(validCount);
                    physicsBones.collisionFriction.resize(validCount);
                    physicsBones.collisionRotationBias.resize(validCount);
                    physicsBones.collisionCompliance.resize(validCount);

                    physicsBones.node.resize(validCount);
                    physicsBones.particleName.resize(validCount);
                    physicsBones.isParticle.resize(validCount);
                    physicsBones.particleDepth.resize(validCount);
                    physicsBones.parentBoneIdx.resize(validCount);
                    physicsBones.objIdx.resize(validCount);
                    physicsBones.rootIdx.resize(validCount);
                    physicsBones.depth.resize(validCount);

                    physicsBones.worldScale.resize(validCount);
                    physicsBones.worldRot.resize(validCount);
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
                                if (constraints.anchIdx[ai] != UINT32_MAX)
                                    constraints.anchIdx[ai] = oldToNewBoneIdx[constraints.anchIdx[ai]];
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
                                if (angularConstraints.anchIdx[ai] != UINT32_MAX)
                                    angularConstraints.anchIdx[ai] = oldToNewBoneIdx[angularConstraints.anchIdx[ai]];
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
                        if (objIdxA != UINT32_MAX && objIdxB != UINT32_MAX && objectDatas.isDisable[objIdxA] != objectDatas.isDisable[objIdxB])
                            return objectDatas.isDisable[objIdxA] < objectDatas.isDisable[objIdxB];
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

                                            constraints.anchIdx[dstA] = tmpConstraints.anchIdx[srcA];
                                            constraints.restLen[dstA] = tmpConstraints.restLen[srcA];
                                            constraints.compSquish[dstA] = tmpConstraints.compSquish[srcA];
                                            constraints.compStretch[dstA] = tmpConstraints.compStretch[srcA];
                                            constraints.squishLimit[dstA] = tmpConstraints.squishLimit[srcA];
                                            constraints.stretchLimit[dstA] = tmpConstraints.stretchLimit[srcA];
                                            constraints.angularLimit[dstA] = tmpConstraints.angularLimit[srcA];
                                            constraints.restDirLocal[dstA] = tmpConstraints.restDirLocal[srcA];
                                            constraints.squishDamping[dstA] = tmpConstraints.squishDamping[srcA];
                                            constraints.stretchDamping[dstA] = tmpConstraints.stretchDamping[srcA];
                                            constraints.lambda[dstA] = tmpConstraints.lambda[srcA];
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
                    constraints.anchIdx.resize(validStrideCount);
                    constraints.restLen.resize(validStrideCount);
                    constraints.compSquish.resize(validStrideCount);
                    constraints.compStretch.resize(validStrideCount);
                    constraints.squishLimit.resize(validStrideCount);
                    constraints.stretchLimit.resize(validStrideCount);
                    constraints.angularLimit.resize(validStrideCount);
                    constraints.restDirLocal.resize(validStrideCount);
                    constraints.squishDamping.resize(validStrideCount);
                    constraints.stretchDamping.resize(validStrideCount);
                    constraints.lambda.resize(validStrideCount);

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
                        if (objIdxA != UINT32_MAX && objIdxB != UINT32_MAX && objectDatas.isDisable[objIdxA] != objectDatas.isDisable[objIdxB])
                            return objectDatas.isDisable[objIdxA] < objectDatas.isDisable[objIdxB];
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
                                        angularConstraints.anchIdx[dstA] = tmpAngCons.anchIdx[srcA];
                                        angularConstraints.restRot[dstA] = tmpAngCons.restRot[srcA];
                                        angularConstraints.comp[dstA] = tmpAngCons.comp[srcA];
                                        angularConstraints.limit[dstA] = tmpAngCons.limit[srcA];
                                        angularConstraints.damping[dstA] = tmpAngCons.damping[srcA];
                                        angularConstraints.lambda[dstA] = tmpAngCons.lambda[srcA];
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
                        angularConstraints.anchIdx.resize(validStrideCount);
                        angularConstraints.restRot.resize(validStrideCount);
                        angularConstraints.comp.resize(validStrideCount);
                        angularConstraints.limit.resize(validStrideCount);
                        angularConstraints.damping.resize(validStrideCount);
                        angularConstraints.lambda.resize(validStrideCount);
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
                        if (objIdxA != UINT32_MAX && objIdxB != UINT32_MAX && objectDatas.isDisable[objIdxA] != objectDatas.isDisable[objIdxB])
                            return objectDatas.isDisable[objIdxA] < objectDatas.isDisable[objIdxB];
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
                            std::uint32_t anchIdx = constraints.anchIdx[aiBase + a];
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
                            markUsed(constraints.anchIdx[aiBase + a]);
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

                                    constraints.anchIdx[dstA] = tmpConstraints.anchIdx[srcA];
                                    constraints.restLen[dstA] = tmpConstraints.restLen[srcA];
                                    constraints.compSquish[dstA] = tmpConstraints.compSquish[srcA];
                                    constraints.compStretch[dstA] = tmpConstraints.compStretch[srcA];
                                    constraints.squishLimit[dstA] = tmpConstraints.squishLimit[srcA];
                                    constraints.stretchLimit[dstA] = tmpConstraints.stretchLimit[srcA];
                                    constraints.angularLimit[dstA] = tmpConstraints.angularLimit[srcA];
                                    constraints.restDirLocal[dstA] = tmpConstraints.restDirLocal[srcA];
                                    constraints.squishDamping[dstA] = tmpConstraints.squishDamping[srcA];
                                    constraints.stretchDamping[dstA] = tmpConstraints.stretchDamping[srcA];
                                    constraints.lambda[dstA] = tmpConstraints.lambda[srcA];
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
                            const std::uint32_t abi = angularConstraints.anchIdx[aiBase + a];
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
                            markUsed(angularConstraints.anchIdx[aiBase + a]);
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

                                    angularConstraints.anchIdx[dstA] = tmpAngCons.anchIdx[srcA];
                                    angularConstraints.restRot[dstA] = tmpAngCons.restRot[srcA];
                                    angularConstraints.comp[dstA] = tmpAngCons.comp[srcA];
                                    angularConstraints.limit[dstA] = tmpAngCons.limit[srcA];
                                    angularConstraints.damping[dstA] = tmpAngCons.damping[srcA];
                                    angularConstraints.lambda[dstA] = tmpAngCons.lambda[srcA];
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
