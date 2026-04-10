#include "PhysicsWorld.h"

#define TIMELOG_START                                               \
    static double nsSum = 0.0;                                      \
    static std::uint32_t timeCount = 0;                             \
    const auto start = std::chrono::high_resolution_clock::now();   

#define TIMELOG_END                                                                                                                 \
    const auto end = std::chrono::high_resolution_clock::now();                                                                     \
    nsSum += std::chrono::duration_cast<std::chrono::nanoseconds>(end - start).count();                                             \
    timeCount++;                                                                                                                    \
    if (timeCount >= 1000)                                                                                                          \
    {                                                                                                                               \
        const auto ms = (nsSum / timeCount) * ns2ms;                                                                                \
        logger::debug("{} time: {:.3f}ms ({} bones / {} constrants / {} angularConstrants / {} colliders)", __func__, ms,            \
                     physicsBones.numBones, constraints.numConstraints, angularConstraints.numConstraints, colliders.numColliders); \
        nsSum = 0;                                                                                                                  \
        timeCount = 0;                                                                                                              \
    }

//#define COLLISION_ROTATE

namespace MXPBD
{
    XPBDWorld::XPBDWorld(std::uint8_t iteration, float gridSize)
        : ITERATION_MAX(iteration), GRID_SIZE(gridSize)
    {
        std::uint64_t pCoreMask = 0;
        std::uint32_t pCoreCount = 0;
        DWORD len = 0;
        GetLogicalProcessorInformationEx(RelationProcessorCore, nullptr, &len);
        std::vector<BYTE> buffer(len);
        if (GetLogicalProcessorInformationEx(RelationProcessorCore, reinterpret_cast<PSYSTEM_LOGICAL_PROCESSOR_INFORMATION_EX>(buffer.data()), &len))
        {
            BYTE* ptr = buffer.data();
            BYTE maxEfficiency = 0;
            while (ptr < buffer.data() + len)
            {
                auto info = reinterpret_cast<PSYSTEM_LOGICAL_PROCESSOR_INFORMATION_EX>(ptr);
                if (info->Relationship == RelationProcessorCore)
                {
                    if (info->Processor.EfficiencyClass > maxEfficiency)
                    {
                        maxEfficiency = info->Processor.EfficiencyClass;
                    }
                }
                ptr += info->Size;
            }
            ptr = buffer.data();
            while (ptr < buffer.data() + len)
            {
                auto info = reinterpret_cast<PSYSTEM_LOGICAL_PROCESSOR_INFORMATION_EX>(ptr);
                if (info->Relationship == RelationProcessorCore)
                {
                    if (info->Processor.EfficiencyClass == maxEfficiency)
                    {
                        pCoreMask |= info->Processor.GroupMask[0].Mask;
                    }
                }
                ptr += info->Size;
            }
        }
        pCoreCount = std::popcount(pCoreMask);
        threadPool = std::make_unique<TBB_ThreadPool>(pCoreCount, pCoreMask);
    };

    void XPBDWorld::AddPhysics(RE::TESObjectREFR* object, RE::NiNode* rootNode, const RootType rootType, const PhysicsInput& input)
    {
        if (!object || !rootNode)
            return;
        if (input.bones.empty() && input.constraints.empty() && input.convexHullColliders.colliders.empty())
            return;

        const ObjectDatas::Root newRoot = {.type = rootType, .bipedSlot = input.bipedSlot};

        logger::info("{:x} : adding physics for {} bones, {} constraints, {} colliders", object->formID, input.bones.size(), input.constraints.size(), input.convexHullColliders.colliders.size());

        std::lock_guard lg(lock);
        if (orderDirty)
        {
            ReorderMaps();
            orderDirty = false;
        }

        ReserveBone(input.bones.size());
        ReserveConstraint(input.constraints.size());
        ReserveAngularConstraint(input.constraints.size());
        ReserveCollider(input.convexHullColliders.colliders.size());

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
                    objectDatas.beforeWorldPos[oi] = ToVector(object->GetPosition());
                    objectDatas.velocity[oi] = EmptyVector;
                    objectDatas.acceleration[oi] = EmptyVector;
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
                    objectDatas.beforeWorldPos.push_back(ToVector(object->GetPosition()));
                    objectDatas.velocity.push_back(EmptyVector);
                    objectDatas.acceleration.push_back(EmptyVector);
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
            const std::size_t groups = physicsBonesGroup.size() - 1;
            for (std::size_t g = 0; g < groups; ++g)
            {
                const std::size_t begin = physicsBonesGroup[g];
                const std::size_t end = physicsBonesGroup[g + 1];
                if (physicsBones.objIdx[begin] != currentObjIdx)
                    continue;
                for (std::size_t bi = begin; bi < end; ++bi)
                {
                    if (auto& node = physicsBones.node[bi]; node && !node->name.empty())
                    {
                        boneNameToIdx[node->name.c_str()] = bi;
                    }
                    else if (!physicsBones.particleName[bi].empty())
                    {
                        boneNameToIdx[physicsBones.particleName[bi]] = bi;
                    }
                }
            }
        }
        const auto existsBones = boneNameToIdx;
        std::unordered_map<std::string, std::vector<std::string>> particleParentToName;
        for (const auto& bone : input.bones)
        {
            if (!bone.second.isParticle || existsBones.find(bone.first) != existsBones.end())
                continue;
            particleParentToName[bone.second.parentBoneName].push_back(bone.first);
        }

        // add bone data
        Mus::nif::VisitObjects(rootNode, [&](RE::NiAVObject* obj, std::uint32_t depth) {
            if (!obj || obj->name.empty())
                return true;

            std::string nodeName = obj->name.c_str();
            const auto found = input.bones.find(nodeName);
            if (found == input.bones.end())
                return true;
            if (boneNameToIdx.find(nodeName) != boneNameToIdx.end())
                return true;

            logger::debug("{:x} : add physics bone {}", object->formID, nodeName);
            const std::uint32_t bi = AllocateBone();
            const Vector offset = DirectX::XMVector3Rotate(DirectX::XMVectorScale(ToVector(found->second.offset), obj->world.scale), ToQuaternion(obj->world.rotate));
            physicsBones.pos[bi] = DirectX::XMVectorAdd(ToVector(obj->world.translate), offset);
            physicsBones.pred[bi] = physicsBones.pos[bi];
            physicsBones.vel[bi] = EmptyVector;

            physicsBones.advancedRotation[bi] = found->second.advancedRotation;
            physicsBones.rot[bi] = ToQuaternion(obj->world.rotate);
            physicsBones.predRot[bi] = physicsBones.rot[bi];
            physicsBones.angVel[bi] = EmptyVector;
            if (found->second.advancedRotation && found->second.mass > 0.0f && found->second.inertiaScale > 0.0f)
                physicsBones.invInertia[bi] = 1.0f / (found->second.mass * found->second.inertiaScale);
            else
                physicsBones.invInertia[bi] = 0.0f;

            physicsBones.damping[bi] = found->second.damping;
            physicsBones.inertiaScale[bi] = found->second.inertiaScale;
            physicsBones.rotRatio[bi] = found->second.rotRatio;
            physicsBones.gravity[bi] = ToVector(GetSkyrimGravity(found->second.gravity));
            physicsBones.offset[bi] = ToVector(found->second.offset);
            physicsBones.invMass[bi] = (found->second.mass > 0.0f) ? (1.0f / found->second.mass) : 0.0f;

            physicsBones.colMargin[bi] = found->second.colMargin;
            physicsBones.colFriction[bi] = found->second.colFriction;
            physicsBones.colComp[bi] = found->second.colComp;

            physicsBones.node[bi] = RE::NiPointer(obj);
            physicsBones.isParticle[bi] = 0;
            physicsBones.parentBoneIdx[bi] = UINT32_MAX;
            if (obj->parent && !obj->parent->name.empty())
            {
                if (auto pit = boneNameToIdx.find(obj->parent->name.c_str()); pit != boneNameToIdx.end())
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

            if (const auto pptn = particleParentToName.find(obj->name.c_str()); pptn != particleParentToName.end())
            {
                auto func = [this, &object, &input, currentObjIdx, currentRootIdx, depth, &boneNameToIdx, &particleParentToName]
                (auto&& func, const std::string& parentName, const std::vector<std::string>& pptnList, const std::uint32_t parentIdx, std::uint32_t particleDepth) -> void {
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
                        physicsBones.vel[particleBi] = EmptyVector;

                        physicsBones.advancedRotation[particleBi] = pit->second.advancedRotation;
                        physicsBones.rot[particleBi] = ToQuaternion(physicsBones.worldRot[parentIdx]);
                        physicsBones.predRot[particleBi] = physicsBones.rot[particleBi];
                        physicsBones.angVel[particleBi] = EmptyVector;
                        if (pit->second.advancedRotation && pit->second.mass > 0.0f && pit->second.inertiaScale > 0.0f)
                            physicsBones.invInertia[particleBi] = 1.0f / (pit->second.mass * pit->second.inertiaScale);
                        else
                            physicsBones.invInertia[particleBi] = 0.0f;

                        physicsBones.damping[particleBi] = pit->second.damping;
                        physicsBones.inertiaScale[particleBi] = pit->second.inertiaScale;
                        physicsBones.rotRatio[particleBi] = pit->second.rotRatio;
                        physicsBones.gravity[particleBi] = ToVector(GetSkyrimGravity(pit->second.gravity));
                        physicsBones.offset[particleBi] = ToVector(pit->second.offset);
                        physicsBones.invMass[particleBi] = (pit->second.mass > 0.0f) ? (1.0f / pit->second.mass) : 0.0f;

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
                continue;

            const std::uint32_t ci = AllocateConstraint();
            constraints.boneIdx[ci] = bit->second;
            constraints.objIdx[ci] = currentObjIdx;
            constraints.rootIdx[ci] = currentRootIdx;

            std::uint8_t validAnchorCount = 0;
            const std::size_t aiBase = static_cast<std::size_t>(ci) * ANCHOR_MAX;
            for (std::uint8_t a = 0; a < constraint.second.anchorBoneNames.size(); ++a)
            {
                if (boneNameToIdx.find(constraint.second.anchorBoneNames[a]) == boneNameToIdx.end())
                    continue;
                if (validAnchorCount >= ANCHOR_MAX)
                    break;
                logger::debug("{:x} : add constraint {}({}) on {}", object->formID, constraint.first, validAnchorCount, constraint.second.anchorBoneNames[a]);

                const std::size_t ai = aiBase + validAnchorCount;
                const std::uint32_t anchBi = boneNameToIdx[constraint.second.anchorBoneNames[a]];
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
                const float distSq = DirectX::XMVectorGetX(DirectX::XMVector3LengthSq(dirWorld));
                if (distSq > FloatPrecision)
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

            std::uint8_t validAnchorCount = 0;
            const std::size_t aciBase = static_cast<std::size_t>(aci) * ANCHOR_MAX;
            for (std::uint8_t a = 0; a < angularConstraint.second.anchorBoneNames.size(); ++a)
            {
                if (boneNameToIdx.find(angularConstraint.second.anchorBoneNames[a]) == boneNameToIdx.end())
                    continue;
                if (validAnchorCount >= ANCHOR_MAX)
                    break;
                logger::debug("{:x} : add angular constraint {}({}) on {}", object->formID, angularConstraint.first, validAnchorCount, angularConstraint.second.anchorBoneNames[a]);

                const std::size_t aai = aciBase + validAnchorCount;
                const std::uint32_t anchBi = boneNameToIdx[angularConstraint.second.anchorBoneNames[a]];
                angularConstraints.anchIdx[aai] = anchBi;
                const Quaternion childRot = ToQuaternion(physicsBones.worldRot[bit->second]);
                const Quaternion anchorRot = ToQuaternion(physicsBones.worldRot[anchBi]);
                const Quaternion anchorRotInv = DirectX::XMQuaternionInverse(anchorRot);
                const Quaternion restRot = DirectX::XMQuaternionMultiply(childRot, anchorRotInv);
                angularConstraints.restRot[aai] = restRot;
                angularConstraints.comp[aai] = angularConstraint.second.compliance[a];
                angularConstraints.limit[aai] = angularConstraint.second.limit[a];
                angularConstraints.damping[aai] = angularConstraint.second.damping[a];
                validAnchorCount++;
            }
            angularConstraints.numAnchors[aci] = validAnchorCount;
        }

        if (!input.convexHullColliders.colliders.empty())
        {
            if (colliders.numColliders == 0)
                colliders.convexHullCache.reserve(input.convexHullColliders.colliders.size() * 8);

            RemoveCollider(object, newRoot);
            for (auto& collider : input.convexHullColliders.colliders)
            {
                auto bit = boneNameToIdx.find(collider.boneName);
                if (bit == boneNameToIdx.end())
                    continue;

                if (collider.vertices.empty())
                    continue;

                const std::uint32_t ci = AllocateCollider();
                colliders.boneIdx[ci] = bit->second;
                colliders.objIdx[ci] = currentObjIdx;
                colliders.rootIdx[ci] = currentRootIdx;

                const std::uint8_t vCount = static_cast<std::uint8_t>(std::min(collider.vertices.size(), static_cast<std::size_t>(COL_VERTEX_MAX)));
                colliders.vertexCount[ci] = vCount;

                AABB aabb = {FLT_MAX, FLT_MAX, FLT_MAX, -FLT_MAX, -FLT_MAX, -FLT_MAX};
                float maxRadiusSq = 0.0f;
                for (std::uint8_t v = 0; v < vCount; ++v)
                {
                    const auto p = collider.vertices[v];

                    colliders.convexHullData[ci].vX[v] = p.x;
                    colliders.convexHullData[ci].vY[v] = p.y;
                    colliders.convexHullData[ci].vZ[v] = p.z;

                    aabb.minX = std::min(aabb.minX, p.x);
                    aabb.minY = std::min(aabb.minY, p.y);
                    aabb.minZ = std::min(aabb.minZ, p.z);
                    aabb.maxX = std::max(aabb.maxX, p.x);
                    aabb.maxY = std::max(aabb.maxY, p.y);
                    aabb.maxZ = std::max(aabb.maxZ, p.z);

                    const float distSq = p.SqrLength();
                    if (distSq > maxRadiusSq)
                    {
                        maxRadiusSq = distSq;
                    }
                }
                colliders.boundingAABB[ci] = aabb;
                colliders.boundingSphere[ci] = std::sqrt(maxRadiusSq);

                {
                    const float lastVX = colliders.convexHullData[ci].vX[vCount - 1]; 
                    const float lastVY = colliders.convexHullData[ci].vY[vCount - 1]; 
                    const float lastVZ = colliders.convexHullData[ci].vZ[vCount - 1]; 
                    for (std::uint8_t v = vCount; v < COL_VERTEX_MAX; ++v)
                    {
                        colliders.convexHullData[ci].vX[v] = lastVX;
                        colliders.convexHullData[ci].vY[v] = lastVY;
                        colliders.convexHullData[ci].vZ[v] = lastVZ;
                    }
                }

                {
                    struct Edge
                    {
                        RE::NiPoint3 v;
                        float length;
                    };
                    std::vector<Edge> tempEdges;
                    tempEdges.reserve(collider.indices.size());

                    for (std::size_t i = 0; i + 2 < collider.indices.size(); i += 3)
                    {
                        const std::uint32_t i0 = collider.indices[i + 0];
                        const std::uint32_t i1 = collider.indices[i + 1];
                        const std::uint32_t i2 = collider.indices[i + 2];

                        auto addTempEdge = [&](std::uint32_t a, std::uint32_t b) {
                            if (a >= vCount || b >= vCount)
                                return;
                            RE::NiPoint3 d = collider.vertices[b] - collider.vertices[a];
                            const float lenSq = d.SqrLength();

                            if (lenSq > FloatPrecision)
                            {
                                const float len = std::sqrt(lenSq);
                                tempEdges.push_back({d / len, len});
                            }
                        };

                        addTempEdge(i0, i1);
                        addTempEdge(i1, i2);
                        addTempEdge(i2, i0);
                    }
                    std::sort(tempEdges.begin(), tempEdges.end(), [](const Edge& a, const Edge& b) {
                        return a.length > b.length;
                    });

                    std::vector<RE::NiPoint3> finalEdges;
                    finalEdges.reserve(COL_EDGE_MAX);

                    for (const auto& te : tempEdges)
                    {
                        bool merged = false;
                        for (auto& fe : finalEdges)
                        {
                            const float dot = te.v.Dot(fe);
                            if (std::abs(dot) > 0.939f) // 20
                            {
                                const float sign = (dot > 0.0f) ? 1.0f : -1.0f;
                                fe.x += te.v.x * sign;
                                fe.y += te.v.y * sign;
                                fe.z += te.v.z * sign;

                                const float newLen = fe.Length();
                                fe.x /= newLen;
                                fe.y /= newLen;
                                fe.z /= newLen;

                                merged = true;
                                break;
                            }
                        }

                        if (!merged && finalEdges.size() < COL_EDGE_MAX)
                        {
                            finalEdges.push_back(te.v);
                        }

                        if (finalEdges.empty())
                        {
                            finalEdges.push_back({1.0f, 0.0f, 0.0f});
                        }

                        for (std::uint8_t e = 0; e < COL_EDGE_MAX; ++e)
                        {
                            const std::uint8_t fe = (e < finalEdges.size()) ? e : 0;
                            colliders.convexHullData[ci].eX[e] = finalEdges[fe].x;
                            colliders.convexHullData[ci].eY[e] = finalEdges[fe].y;
                            colliders.convexHullData[ci].eZ[e] = finalEdges[fe].z;
                        }
                    }
                }

                {
                    std::vector<RE::NiPoint3> finalFaces;
                    finalFaces.reserve(COL_FACE_MAX);
                    for (std::size_t i = 0; i + 2 < collider.indices.size(); i += 3)
                    {
                        const std::uint32_t i0 = collider.indices[i + 0];
                        const std::uint32_t i1 = collider.indices[i + 1];
                        const std::uint32_t i2 = collider.indices[i + 2];

                        if (i0 >= vCount || i1 >= vCount || i2 >= vCount)
                            continue;

                        const auto& v0 = collider.vertices[i0];
                        const auto& v1 = collider.vertices[i1];
                        const auto& v2 = collider.vertices[i2];

                        const float e1x = v1.x - v0.x;
                        const float e1y = v1.y - v0.y;
                        const float e1z = v1.z - v0.z;
                        const float e2x = v2.x - v0.x;
                        const float e2y = v2.y - v0.y;
                        const float e2z = v2.z - v0.z;

                        RE::NiPoint3 n = {(e1y * e2z) - (e1z * e2y),
                                          (e1z * e2x) - (e1x * e2z),
                                          (e1x * e2y) - (e1y * e2x)};

                        const float lenSq = n.SqrLength();
                        if (lenSq < FloatPrecision)
                            continue;

                        const float invLen = rsqrt(lenSq);
                        n *= invLen;

                        bool isDuplicate = false;
                        for (const auto& ff : finalFaces)
                        {
                            const float dot = n.Dot(ff);
                            if (dot > 0.999f)
                            {
                                isDuplicate = true;
                                break;
                            }
                        }

                        if (!isDuplicate && finalFaces.size() < COL_FACE_MAX)
                        {
                            finalFaces.push_back(n);
                        }
                    }

                    if (finalFaces.empty())
                    {
                        finalFaces.push_back({1.0f, 0.0f, 0.0f});
                    }

                    for (std::uint8_t f = 0; f < COL_FACE_MAX; ++f)
                    {
                        const std::uint8_t ff = (f < finalFaces.size()) ? f : 0;
                        colliders.convexHullData[ci].fX[f] = finalFaces[ff].x;
                        colliders.convexHullData[ci].fY[f] = finalFaces[ff].y;
                        colliders.convexHullData[ci].fZ[f] = finalFaces[ff].z;
                    }
                }

                std::uint8_t ncCount = 0;
                {
                    auto& node = physicsBones.node[bit->second];
                    if (!node || !node->parent || node->parent->name.empty())
                        continue;
                    const std::string ncName = node->parent->name.c_str();
                    auto ncBit = boneNameToIdx.find(ncName);
                    if (ncBit == boneNameToIdx.end() || ncCount >= NOCOLLIDE_MAX)
                        continue;
                    colliders.noCollideBoneIdx[static_cast<std::size_t>(ci) * NOCOLLIDE_MAX + ncCount] = ncBit->second;
                    logger::debug("{:x} : {} => add no collide {}", object->formID, collider.boneName, ncName);
                    ncCount++;
                }

                auto ncIt = input.convexHullColliders.noCollideBones.find(collider.boneName);
                if (ncIt != input.convexHullColliders.noCollideBones.end())
                {
                    for (const auto& ncName : ncIt->second)
                    {
                        if (collider.boneName == ncName)
                            continue;
                        auto ncBit = boneNameToIdx.find(ncName);
                        if (ncBit == boneNameToIdx.end() || ncCount >= NOCOLLIDE_MAX)
                            continue;
                        const std::uint32_t ncbi = ncBit->second;
                        auto begin = colliders.noCollideBoneIdx.begin() + static_cast<std::size_t>(ci) * NOCOLLIDE_MAX;
                        auto end = begin + ncCount;
                        auto it = std::find(begin, end, ncbi);
                        if (it != end)
                            continue;
                        colliders.noCollideBoneIdx[static_cast<std::size_t>(ci) * NOCOLLIDE_MAX + ncCount] = ncbi;
                        logger::debug("{:x} : {} => add no collide {}", object->formID, collider.boneName, ncName);
                        ncCount++;
                    }
                }
                colliders.noCollideCount[ci] = ncCount;
                logger::debug("{:x} => add collider {}", object->formID, collider.boneName);
            }
            logger::debug("{:x} : add colliders {}", object->formID, input.convexHullColliders.colliders.size());
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
        bool found = false;
        for (std::uint32_t i = 0; i < objectDatas.objectID.size(); ++i)
        {
            if (objectDatas.objectID[i] != object->formID)
                continue;
            objectDatas.beforeWorldPos[i] = ToVector(object->GetPosition());
            objectDatas.acceleration[i] = EmptyVector;
            currentObjIdx = i;
            found = true;
            logger::debug("{:x} : Found objIdx", object->formID);
            break;
        }
        if (!found)
            return;

        if (physicsBonesGroup.empty())
            return;

        std::unordered_map<std::string, std::uint32_t> boneNameToIdx;
        {
            const std::size_t groups = physicsBonesGroup.size() - 1;
            for (std::size_t g = 0; g < groups; ++g)
            {
                const std::size_t begin = physicsBonesGroup[g];
                const std::size_t end = physicsBonesGroup[g + 1];
                if (physicsBones.objIdx[begin] != currentObjIdx)
                    continue;
                logger::debug("{:x} : Found bones group", object->formID);
                for (std::size_t bi = begin; bi < end; ++bi)
                {
                    if (auto& node = physicsBones.node[bi]; node && !node->name.empty())
                    {
                        const std::string nodeName = node->name.c_str();
                        boneNameToIdx[nodeName] = bi;

                        const auto found = input.bones.find(nodeName);
                        if (found == input.bones.end())
                            continue;

                        logger::debug("{:x} : Found bone ({}) for physics", object->formID, node->name.c_str());

                        physicsBones.offset[bi] = ToVector(found->second.offset);
                        const Vector offset = DirectX::XMVector3Rotate(DirectX::XMVectorScale(physicsBones.offset[bi], physicsBones.worldScale[bi]), ToQuaternion(node->world.rotate));
                        
                        physicsBones.pos[bi] = DirectX::XMVectorAdd(ToVector(node->world.translate), offset);
                        physicsBones.pred[bi] = physicsBones.pos[bi];
                        physicsBones.vel[bi] = EmptyVector;

                        physicsBones.advancedRotation[bi] = found->second.advancedRotation;
                        physicsBones.rot[bi] = ToQuaternion(node->world.rotate);
                        physicsBones.predRot[bi] = physicsBones.rot[bi];
                        physicsBones.angVel[bi] = EmptyVector;
                        if (found->second.advancedRotation && found->second.mass > 0.0f && found->second.inertiaScale > 0.0f)
                            physicsBones.invInertia[bi] = 1.0f / (found->second.mass * found->second.inertiaScale);
                        else
                            physicsBones.invInertia[bi] = 0.0f;

                        physicsBones.damping[bi] = found->second.damping;
                        physicsBones.inertiaScale[bi] = found->second.inertiaScale;
                        physicsBones.rotRatio[bi] = found->second.rotRatio;
                        physicsBones.gravity[bi] = ToVector(GetSkyrimGravity(found->second.gravity));
                        physicsBones.invMass[bi] = (found->second.mass > 0.0f) ? (1.0f / found->second.mass) : 0.0f;

                        physicsBones.linearRotTorque[bi] = found->second.linearRotTorque;

                        physicsBones.colMargin[bi] = found->second.colMargin;
                        physicsBones.colFriction[bi] = found->second.colFriction;
                        physicsBones.colComp[bi] = found->second.colComp;

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
                        physicsBones.vel[bi] = EmptyVector;

                        physicsBones.advancedRotation[bi] = found->second.advancedRotation;
                        physicsBones.rot[bi] = ToQuaternion(physicsBones.worldRot[pbi]);
                        physicsBones.predRot[bi] = physicsBones.rot[bi];
                        physicsBones.angVel[bi] = EmptyVector;
                        if (found->second.advancedRotation && found->second.mass > 0.0f && found->second.inertiaScale > 0.0f)
                            physicsBones.invInertia[bi] = 1.0f / (found->second.mass * found->second.inertiaScale);
                        else
                            physicsBones.invInertia[bi] = 0.0f;

                        physicsBones.damping[bi] = found->second.damping;
                        physicsBones.inertiaScale[bi] = found->second.inertiaScale;
                        physicsBones.rotRatio[bi] = found->second.rotRatio;
                        physicsBones.gravity[bi] = ToVector(GetSkyrimGravity(found->second.gravity));
                        physicsBones.invMass[bi] = (found->second.mass > 0.0f) ? (1.0f / found->second.mass) : 0.0f;

                        physicsBones.linearRotTorque[bi] = found->second.linearRotTorque;

                        physicsBones.worldScale[bi] = physicsBones.worldScale[pbi];
                        physicsBones.worldRot[bi] = physicsBones.worldRot[pbi];
                    }
                }
            }
        }
        if (!constraintsGroup.empty())
        {
            const std::size_t groups = constraintsGroup.size() - 1;
            for (std::size_t g = 0; g < groups; ++g)
            {
                const std::size_t begin = constraintsGroup[g];
                const std::size_t end = constraintsGroup[g + 1];
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

                    std::uint8_t validAnchorCount = 0;
                    const std::size_t aiBase = ci * ANCHOR_MAX;
                    for (std::uint8_t a = 0; a < found->second.anchorBoneNames.size(); ++a)
                    {
                        if (boneNameToIdx.find(found->second.anchorBoneNames[a]) == boneNameToIdx.end())
                            continue;
                        if (validAnchorCount >= ANCHOR_MAX)
                            break;
                        logger::debug("{:x} : add constraint {}({}) on {}", object->formID, nodeName, validAnchorCount, found->second.anchorBoneNames[a]);
                        const std::size_t ai = aiBase + validAnchorCount;
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
                        const float distSq = DirectX::XMVectorGetX(DirectX::XMVector3LengthSq(dirWorld));
                        if (distSq > FloatPrecision)
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
            const std::size_t groups = angularConstraintsGroup.size() - 1;
            for (std::size_t g = 0; g < groups; ++g)
            {
                const std::size_t begin = angularConstraintsGroup[g];
                const std::size_t end = angularConstraintsGroup[g + 1];
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

                    logger::debug("{:x} : Found bone ({}) for angular constraints", object->formID, nodeName);

                    std::uint8_t validAnchorCount = 0;
                    const std::size_t aiBase = aci * ANCHOR_MAX;
                    for (std::uint8_t a = 0; a < found->second.anchorBoneNames.size(); ++a)
                    {
                        if (boneNameToIdx.find(found->second.anchorBoneNames[a]) == boneNameToIdx.end())
                            continue;
                        if (validAnchorCount >= ANCHOR_MAX)
                            break;
                        logger::debug("{:x} : add angular constraint {}({}) on {}", object->formID, nodeName, validAnchorCount, found->second.anchorBoneNames[a]);
                        const std::size_t ai = aiBase + validAnchorCount;
                        const std::uint32_t anchGlobIdx = boneNameToIdx[found->second.anchorBoneNames[a]];
                        angularConstraints.anchIdx[ai] = anchGlobIdx;
                        const Quaternion childRot = ToQuaternion(physicsBones.worldRot[bi]);
                        const Quaternion anchorRot = ToQuaternion(physicsBones.worldRot[anchGlobIdx]);
                        const Quaternion anchorRotInv = DirectX::XMQuaternionInverse(anchorRot);
                        const Quaternion restRot = DirectX::XMQuaternionMultiply(childRot, anchorRotInv);
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

    void XPBDWorld::Reset()
    {
        std::lock_guard lg(lock);

        for (std::uint32_t i = 0; i < objectDatas.objectID.size(); ++i)
        {
            RE::TESObjectREFR* object = GetREFR(objectDatas.objectID[i]);
            if (!object)
                continue;
            objectDatas.beforeWorldPos[i] = ToVector(object->GetPosition());
            objectDatas.acceleration[i] = EmptyVector;
        }

        // reset bone data
        if (!physicsBonesGroup.empty())
        {
            const std::size_t groups = physicsBonesGroup.size() - 1;
            threadPool->Execute([&] {
                tbb::parallel_for(
                    tbb::blocked_range<std::size_t>(0, groups),
                    [&](const tbb::blocked_range<std::size_t>& r) {
                        for (std::size_t g = r.begin(); g != r.end(); ++g)
                        {
                            const std::size_t begin = physicsBonesGroup[g];
                            const std::size_t end = physicsBonesGroup[g + 1];
                            for (std::size_t bi = begin; bi < end; ++bi)
                            {
                                if (auto& node = physicsBones.node[bi]; node)
                                {
                                    node->local.translate = physicsBones.orgLocalPos[bi];
                                    node->local.rotate = physicsBones.orgLocalRot[bi];

                                    if (node->parent)
                                    {
                                        const RE::NiPoint3 parentWorldPos = node->parent->world.translate;
                                        const RE::NiMatrix3 parentWorldRot = node->parent->world.rotate;
                                        node->world.translate = parentWorldPos + (parentWorldRot * (physicsBones.orgLocalPos[bi] * node->parent->world.scale));
                                        node->world.rotate = parentWorldRot * physicsBones.orgLocalRot[bi];
                                    }

                                    const Vector offset = DirectX::XMVector3Rotate(DirectX::XMVectorScale(physicsBones.offset[bi], physicsBones.worldScale[bi]), ToQuaternion(node->world.rotate));
                                    physicsBones.pos[bi] = DirectX::XMVectorAdd(ToVector(node->world.translate), offset);
                                    physicsBones.pred[bi] = physicsBones.pos[bi];
                                    physicsBones.vel[bi] = EmptyVector;

                                    physicsBones.rot[bi] = ToQuaternion(node->world.rotate);
                                    physicsBones.predRot[bi] = physicsBones.rot[bi];
                                    physicsBones.angVel[bi] = EmptyVector;

                                    RE::NiUpdateData ctx = {.time = 0.0f, .flags = RE::NiUpdateData::Flag::kNone};
                                    node->UpdateWorldData(&ctx);
                                }
                                else if (!physicsBones.particleName[bi].empty() && physicsBones.parentBoneIdx[bi] != UINT32_MAX)
                                {
                                    const std::uint32_t pbi = physicsBones.parentBoneIdx[bi];

                                    const Vector offset = DirectX::XMVector3Rotate(DirectX::XMVectorScale(physicsBones.offset[bi], physicsBones.worldScale[pbi]), ToQuaternion(physicsBones.worldRot[pbi]));
                                    physicsBones.pos[bi] = DirectX::XMVectorAdd(physicsBones.pos[pbi], offset);
                                    physicsBones.pred[bi] = physicsBones.pos[bi];
                                    physicsBones.vel[bi] = EmptyVector;

                                    physicsBones.rot[bi] = ToQuaternion(physicsBones.worldRot[pbi]);
                                    physicsBones.predRot[bi] = physicsBones.rot[bi];
                                    physicsBones.angVel[bi] = EmptyVector;
                                }
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
                objectDatas.beforeWorldPos[i] = ToVector(object->GetPosition());
                objectDatas.acceleration[i] = EmptyVector;
                break;
            }
        }
        if (oi == UINT32_MAX)
            return;

        bool isUpdateWorldData = resetObjects.insert(object->formID).second;
        // reset bone data
        if (!physicsBonesGroup.empty())
        {
            const std::size_t groups = physicsBonesGroup.size() - 1;
            for (std::size_t g = 0; g < groups; ++g)
            {
                const std::size_t begin = physicsBonesGroup[g];
                const std::size_t end = physicsBonesGroup[g + 1];
                if (physicsBones.objIdx[begin] != oi)
                    continue;
                for (std::size_t bi = begin; bi < end; ++bi)
                {
                    if (auto& node = physicsBones.node[bi]; node)
                    {
                        node->local.translate = physicsBones.orgLocalPos[bi];
                        node->local.rotate = physicsBones.orgLocalRot[bi];

                        if (node->parent)
                        {
                            const RE::NiPoint3 parentWorldPos = node->parent->world.translate;
                            const RE::NiMatrix3 parentWorldRot = node->parent->world.rotate;
                            node->world.translate = parentWorldPos + (parentWorldRot * physicsBones.orgLocalPos[bi]);
                            node->world.rotate = parentWorldRot * physicsBones.orgLocalRot[bi];
                        }

                        const Vector offset = DirectX::XMVector3Rotate(DirectX::XMVectorScale(physicsBones.offset[bi], physicsBones.worldScale[bi]), ToQuaternion(node->world.rotate));
                        physicsBones.pos[bi] = DirectX::XMVectorAdd(ToVector(node->world.translate), offset);
                        physicsBones.pred[bi] = physicsBones.pos[bi];
                        physicsBones.vel[bi] = EmptyVector;

                        physicsBones.rot[bi] = ToQuaternion(node->world.rotate);
                        physicsBones.predRot[bi] = physicsBones.rot[bi];
                        physicsBones.angVel[bi] = EmptyVector;

                        //RE::NiUpdateData ctx = {.time = 0.0f, .flags = RE::NiUpdateData::Flag::kNone};
                        //node->UpdateWorldData(&ctx);
                    }
                    else if (!physicsBones.particleName[bi].empty() && physicsBones.parentBoneIdx[bi] != UINT32_MAX)
                    {
                        const std::uint32_t pbi = physicsBones.parentBoneIdx[bi];
                        const Vector offset = DirectX::XMVector3Rotate(DirectX::XMVectorScale(physicsBones.offset[bi], physicsBones.worldScale[pbi]), ToQuaternion(physicsBones.worldRot[pbi]));
                        physicsBones.pos[bi] = DirectX::XMVectorAdd(physicsBones.pos[pbi], offset);
                        physicsBones.pred[bi] = physicsBones.pos[bi];
                        physicsBones.vel[bi] = EmptyVector;

                        physicsBones.rot[bi] = ToQuaternion(physicsBones.worldRot[pbi]);
                        physicsBones.predRot[bi] = physicsBones.rot[bi];
                        physicsBones.angVel[bi] = EmptyVector;
                    }
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

    void XPBDWorld::RemovePhysics(const RE::FormID objectID)
    {
        Reset(objectID);
        std::lock_guard lg(lock);
        std::unordered_set<CleanObject, CleanObjectHash> cleanList;
        for (std::size_t i = 0; i < objectDatas.objectID.size(); ++i)
        {
            if (objectDatas.objectID[i] != objectID)
                continue;
            objectDatas.objectID[i] = 0;
            objectDatas.isDisable[i] = true;
            for (std::size_t ri = 0; ri < objectDatas.roots[i].size(); ++ri)
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
        for (std::size_t i = 0; i < physicsBones.node.size(); ++i)
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
            const std::size_t groups = collidersGroup.size() - 1;
            for (std::size_t g = 0; g < groups; ++g)
            {
                const std::size_t begin = collidersGroup[g];
                const std::size_t end = collidersGroup[g + 1];
                if (colliders.objIdx[begin] == currentObjIdx)
                {
                    for (std::uint32_t ci = begin; ci < end; ++ci)
                    {
                        if (colliders.rootIdx[ci] == currentRootIdx)
                        {
                            colliders.boneIdx[ci] = UINT32_MAX;
                            colliders.objIdx[ci] = UINT32_MAX;
                            colliders.rootIdx[ci] = UINT32_MAX;
                            colliders.vertexCount[ci] = 0;
                            colliders.edgeCount[ci] = 0;
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
        resetObjects.clear();
        if (orderDirty)
        {
            ReorderMaps();
            orderDirty = false;
        }

        TIMELOG_START;

        UpdateObjectAcceleration(deltaTime);
        PrefetchBoneDatas();
        UpdateGlobalAABBTree();

        if (colliders.convexHullCache.size() > colliders.collideMaxObserved)
            colliders.collideMaxObserved = colliders.convexHullCache.size();

        else
            colliders.collideMaxObserved = static_cast<std::size_t>(colliders.collideMaxObserved * 0.98f);
        colliders.convexHullCache.reserve(static_cast<std::size_t>(colliders.collideMaxObserved * 1.3f));

        const std::uint32_t steps = static_cast<std::uint32_t>(std::ceil(deltaTime / DeltaTime60));
        const float subDeltaTime = deltaTime / steps;
        for (std::uint32_t i = 0; i < steps; ++i)
        {
            PredictBones(subDeltaTime);
            CreateLocalSpatialHash();
            SolveConstraints(subDeltaTime);
            SolveCollisions(subDeltaTime);
            UpdateBoneVelocity(subDeltaTime);
        }
        ApplyToSkyrim(subDeltaTime);
        currentFrame++;

        TIMELOG_END;
    }

    void XPBDWorld::UpdateObjectAcceleration(const float deltaTime)
    {
        // logger::info("{}", __func__);
        TIMELOG_START;
        const float invDt = 1.0f / deltaTime;
        threadPool->Execute([&] {
            tbb::parallel_for(
                tbb::blocked_range<std::size_t>(0, objectDatas.objectID.size()),
                [&](const tbb::blocked_range<std::size_t>& r) {
                    for (std::size_t oi = r.begin(); oi != r.end(); ++oi)
                    {
                        RE::TESObjectREFR* object = GetREFR(objectDatas.objectID[oi]);
                        if (!object || objectDatas.isDisable[oi])
                            return;
                        const Vector currentWorldPos = ToVector(object->GetPosition());
                        const Vector currentVel = DirectX::XMVectorScale(DirectX::XMVectorSubtract(currentWorldPos, objectDatas.beforeWorldPos[oi]), invDt);
                        objectDatas.acceleration[oi] = DirectX::XMVectorScale(DirectX::XMVectorSubtract(currentVel, objectDatas.velocity[oi]), invDt);
                        objectDatas.velocity[oi] = currentVel;
                        objectDatas.beforeWorldPos[oi] = currentWorldPos;
                    }
                },
                tbb::auto_partitioner()
            );
        });
        TIMELOG_END;
    }

    void XPBDWorld::PrefetchBoneDatas()
    {
        if (physicsBonesGroup.empty())
            return;
        // logger::info("{}", __func__);
        TIMELOG_START;
        physicsBones.prevRot = physicsBones.rot;
        const std::size_t groups = physicsBonesGroup.size() - 1;
        threadPool->Execute([&] {
            tbb::parallel_for(
                tbb::blocked_range<std::size_t>(0, groups),
                [&](const tbb::blocked_range<std::size_t>& r) {
                    for (std::size_t g = r.begin(); g != r.end(); ++g)
                    {
                        const std::size_t begin = physicsBonesGroup[g];
                        const std::size_t end = physicsBonesGroup[g + 1];
                        if (begin >= end)
                            continue;
                        const std::uint32_t oi = physicsBones.objIdx[begin];
                        if (oi == UINT32_MAX || objectDatas.isDisable[oi])
                            continue;
                        for (std::size_t bi = begin; bi < end; ++bi)
                        {
                            if (!physicsBones.isParticle[bi])
                            {
                                auto& node = physicsBones.node[bi];
                                if (physicsBones.invMass[bi] == 0.0f)
                                {
                                    const Vector offset = DirectX::XMVector3Rotate(DirectX::XMVectorScale(physicsBones.offset[bi], physicsBones.worldScale[bi]), ToQuaternion(node->world.rotate));
                                    physicsBones.pos[bi] = DirectX::XMVectorAdd(ToVector(node->world.translate), offset);
                                    physicsBones.pred[bi] = physicsBones.pos[bi];
                                    physicsBones.rot[bi] = ToQuaternion(node->world.rotate);
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
                                    if (physicsBones.invMass[bi] == 0.0f)
                                    {
                                        const Vector offset = DirectX::XMVector3Rotate(DirectX::XMVectorScale(physicsBones.offset[bi], physicsBones.worldScale[pbi]), ToQuaternion(physicsBones.worldRot[pbi]));
                                        physicsBones.pos[bi] = DirectX::XMVectorAdd(physicsBones.pos[pbi], offset);
                                        physicsBones.pred[bi] = physicsBones.pos[bi];
                                        physicsBones.rot[bi] = physicsBones.rot[pbi];
                                        physicsBones.predRot[bi] = physicsBones.rot[bi];
                                    }
                                    physicsBones.worldRot[bi] = physicsBones.worldRot[pbi];
                                    physicsBones.worldScale[bi] = physicsBones.worldScale[pbi];
                                }
                            }
                        }
                    }
                },
                tbb::auto_partitioner()
            );
        });
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
            if (objectDatas.boundingAABB[i].minX == FLT_MAX ||
                (objectDatas.boundingAABB[i].minX == 0.0f && objectDatas.boundingAABB[i].maxX == 0.0f))
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
        threadPool->Execute([&] {
            tbb::parallel_for(
                tbb::blocked_range<std::size_t>(0, physicsBones.numBones),
                [&](const tbb::blocked_range<std::size_t>& r) {
                    for (std::size_t bi = r.begin(); bi != r.end(); ++bi)
                    {
                        const std::uint32_t oi = physicsBones.objIdx[bi];
                        if (oi == UINT32_MAX || objectDatas.isDisable[oi])
                            continue;
                        if (physicsBones.invMass[bi] == 0.0f)
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
                            while (pbi != UINT32_MAX && physicsBones.invMass[pbi] > 0.0f)
                            {
                                pbi = physicsBones.parentBoneIdx[pbi];
                            }
                            if (pbi != UINT32_MAX)
                            {
                                const Quaternion delta = DirectX::XMQuaternionMultiply(DirectX::XMQuaternionConjugate(physicsBones.prevRot[pbi]), physicsBones.rot[pbi]);
                                const float magSq = DirectX::XMVectorGetX(DirectX::XMVector3LengthSq(DirectX::XMVectorSetW(delta, 0.0f)));
                                if (magSq > FloatPrecision)
                                {
                                    const Vector pivot = physicsBones.pred[pbi];
                                    const Vector pos = physicsBones.pred[bi];
                                    const Vector localPos = DirectX::XMVectorSubtract(pos, pivot);
                                    const Vector rotatedPos = DirectX::XMVector3Rotate(localPos, delta);
                                    physicsBones.pred[bi] = DirectX::XMVectorAdd(pivot, rotatedPos);
                                    physicsBones.rot[bi] = DirectX::XMQuaternionNormalize(DirectX::XMQuaternionMultiply(physicsBones.rot[bi], delta));
                                }

                                const Quaternion q = physicsBones.rot[bi];
                                const Quaternion w = DirectX::XMVectorScale(physicsBones.angVel[bi], deltaTime * 0.5f);
                                physicsBones.predRot[bi] = DirectX::XMQuaternionNormalize(DirectX::XMVectorAdd(q, DirectX::XMQuaternionMultiply(q, w)));
                            }
                        }

                        // predic linear
                        {
                            physicsBones.pred[bi] = DirectX::XMVectorMultiplyAdd(physicsBones.vel[bi], DirectX::XMVectorReplicate(deltaTime), physicsBones.pos[bi]);
                        }
                    }
                },
                tbb::auto_partitioner()
            );
        });
        TIMELOG_END;
    }

    void XPBDWorld::CreateLocalSpatialHash()
    {
        if (collidersGroup.empty())
            return;
        // logger::info("{}", __func__);
        if (objectHashes.size() < objectDatas.objectID.size())
            objectHashes.resize(objectDatas.objectID.size());

        TIMELOG_START;

        const std::size_t groups = collidersGroup.size() - 1;
        threadPool->Execute([&] {
            tbb::parallel_for(
                tbb::blocked_range<std::size_t>(0, groups),
                [&](const tbb::blocked_range<std::size_t>& r) {
                    for (std::size_t g = r.begin(); g != r.end(); ++g)
                    {
                        const std::size_t begin = collidersGroup[g];
                        const std::size_t end = collidersGroup[g + 1];
                        if (begin >= end || end - begin == 0)
                            continue;
                        const std::uint32_t oi = colliders.objIdx[begin];
                        if (oi == UINT32_MAX || objectDatas.isDisable[oi])
                            continue;
                        const std::uint32_t objIdx = colliders.objIdx[begin];
                        auto& localHash = objectHashes[objIdx];
                        localHash.Init(end - begin, GRID_SIZE);
                        for (std::size_t ci = begin; ci < end; ++ci)
                        {
                            const std::uint32_t bi = colliders.boneIdx[ci];
                            const std::uint32_t hashHigh = localHash.HashWorldCoordsHigh(physicsBones.pred[bi]);
                            localHash.cellCount[hashHigh]++;
                            const std::uint32_t hashLow = localHash.HashWorldCoordsLow(physicsBones.pred[bi]);
                            localHash.cellCount[hashLow]++;
                        }
                        localHash.cellStart[0] = 0;
                        for (std::uint32_t i = 0; i < localHash.tableSize; ++i)
                        {
                            localHash.cellStart[i + 1] = localHash.cellStart[i] + localHash.cellCount[i];
                            localHash.cellCount[i] = 0;
                        }
                        for (std::uint32_t ci = begin; ci < end; ++ci)
                        {
                            const std::uint32_t bi = colliders.boneIdx[ci];
                            const std::uint32_t hashHigh = localHash.HashWorldCoordsHigh(physicsBones.pred[bi]);
                            const std::uint32_t offsetHigh = localHash.cellStart[hashHigh] + localHash.cellCount[hashHigh]++;
                            localHash.entries[offsetHigh] = ci;
                            const std::uint32_t hashLow = localHash.HashWorldCoordsLow(physicsBones.pred[bi]);
                            const std::uint32_t offsetLow = localHash.cellStart[hashLow] + localHash.cellCount[hashLow]++;
                            localHash.entries[offsetLow] = ci;
                        }
                    }
                },
                tbb::auto_partitioner()
            );
        });
        TIMELOG_END;
    }

    void XPBDWorld::SolveConstraints(const float deltaTime)
    {
        if (constraintsGroup.empty() && angularConstraintsGroup.empty())
            return;
        // logger::info("{}", __func__);
        TIMELOG_START;
        const float invDtSq = 1.0f / (deltaTime * deltaTime);
        for (std::uint8_t iter = 0; iter < ITERATION_MAX; ++iter)
        {
            threadPool->Execute([&] {
                tbb::parallel_invoke([&] {
                    if (!constraintsGroup.empty())
                    {
                        std::fill(constraints.lambda.begin(), constraints.lambda.end(), 0.0f);
                        const std::size_t groups = constraintsGroup.size() - 1;
                        tbb::parallel_for(
                            tbb::blocked_range<std::size_t>(0, groups),
                            [&](const tbb::blocked_range<std::size_t>& r) {
                                for (std::size_t g = r.begin(); g != r.end(); ++g)
                                {
                                    const std::size_t begin = constraintsGroup[g];
                                    const std::size_t end = constraintsGroup[g + 1];
                                    if (begin >= end)
                                        continue;
                                    const std::uint32_t oi = constraints.objIdx[begin];
                                    if (oi == UINT32_MAX || objectDatas.isDisable[oi])
                                        continue;

                                    auto ccgIt = std::lower_bound(constraintsColorGroup.begin(), constraintsColorGroup.end(), begin);
                                    std::size_t ccgi = std::distance(constraintsColorGroup.begin(), ccgIt);
                                    while (ccgi < constraintsColorGroup.size() - 1 && constraintsColorGroup[ccgi] < end)
                                    {
                                        std::size_t c_begin = constraintsColorGroup[ccgi];
                                        std::size_t c_end = std::min(static_cast<std::size_t>(constraintsColorGroup[ccgi + 1]), end);

                                        for (std::size_t ci = c_begin; ci < c_end; ++ci)
                                        {
                                            const std::uint32_t bi = constraints.boneIdx[ci];
                                            const float& invMass = physicsBones.invMass[bi];
                                            if (invMass == 0.0f)
                                                continue;

                                            const std::uint8_t numAnchors = constraints.numAnchors[ci];
                                            const std::size_t aiBase = ci * ANCHOR_MAX;
                                            for (std::uint8_t anchor = 0; anchor < numAnchors; ++anchor)
                                            {
                                                const std::size_t ai = aiBase + anchor;
                                                const std::uint32_t& abi = constraints.anchIdx[ai];
                                                if (abi == UINT32_MAX)
                                                    continue;

                                                const float anchInvMass = physicsBones.invMass[abi];
                                                const float wSum = invMass + anchInvMass;
                                                if (wSum == 0.0f)
                                                    continue;

                                                const Vector dir = DirectX::XMVectorSubtract(physicsBones.pred[bi], physicsBones.pred[abi]);
                                                const Vector distSq = DirectX::XMVector3LengthSq(dir);
                                                if (DirectX::XMVectorGetX(distSq) <= FloatPrecision)
                                                    continue;

                                                const Vector dist = DirectX::XMVectorSqrt(distSq);
                                                const Vector normal = DirectX::XMVectorDivide(dir, dist);

                                                const float restLen = constraints.restLen[ai];
                                                const float C = DirectX::XMVectorGetX(dist) - restLen;
                                                const float currentComp = C < 0.0f ? constraints.compSquish[ai] : constraints.compStretch[ai];
                                                const float currentLimit = C < 0.0f ? constraints.squishLimit[ai] : constraints.stretchLimit[ai];
                                                const float currentDamping = C < 0.0f ? constraints.squishDamping[ai] : constraints.stretchDamping[ai];
                                                float currentFactor = 1.0f;
                                                if (currentLimit > 0.0f)
                                                {
                                                    const float ratio = std::min(std::abs(C) / currentLimit, 1.0f);
                                                    const float smoothFalloff = (1.0f - ratio) * (1.0f - ratio);
                                                    currentFactor = std::max(0.1f, smoothFalloff);
                                                }
                                                const float alphaProxy = (currentComp * currentFactor) * invDtSq;
                                                float& currentLambda = constraints.lambda[ai];
                                                const float deltaLambda = (-C - alphaProxy * currentLambda) / (wSum + alphaProxy);
                                                currentLambda += deltaLambda;

                                                const Vector correction = DirectX::XMVectorScale(normal, deltaLambda);
                                                physicsBones.pred[bi] = DirectX::XMVectorMultiplyAdd(correction, DirectX::XMVectorReplicate(invMass), physicsBones.pred[bi]);

                                                if (anchInvMass > 0.0f)
                                                {
                                                    const Vector correctionA = DirectX::XMVectorScale(correction, anchInvMass);
                                                    physicsBones.pred[abi] = DirectX::XMVectorSubtract(physicsBones.pred[abi], correctionA);
                                                }

                                                if (currentDamping > 0.0f)
                                                {
                                                    const Vector deltaBi = DirectX::XMVectorSubtract(physicsBones.pred[bi], physicsBones.pos[bi]);
                                                    const Vector deltaAbi = DirectX::XMVectorSubtract(physicsBones.pred[abi], physicsBones.pos[abi]);

                                                    const Vector relDelta = DirectX::XMVectorSubtract(deltaBi, deltaAbi);

                                                    const Vector projDelta = DirectX::XMVector3Dot(relDelta, normal);
                                                    const float projectedSpeed = DirectX::XMVectorGetX(projDelta);

                                                    const float dampingCorrectionMag = -projectedSpeed * currentDamping;
                                                    const Vector dampCorrection = DirectX::XMVectorScale(normal, dampingCorrectionMag / wSum);

                                                    physicsBones.pred[bi] = DirectX::XMVectorMultiplyAdd(dampCorrection, DirectX::XMVectorReplicate(invMass), physicsBones.pred[bi]);
                                                    if (anchInvMass > 0.0f)
                                                    {
                                                        const Vector correctionA = DirectX::XMVectorScale(dampCorrection, anchInvMass);
                                                        physicsBones.pred[abi] = DirectX::XMVectorSubtract(physicsBones.pred[abi], correctionA);
                                                    }
                                                }

                                                const float limitAngle = constraints.angularLimit[ai];
                                                if (limitAngle > 0.0f)
                                                {
                                                    Vector currentDir = DirectX::XMVectorSubtract(physicsBones.pred[bi], physicsBones.pred[abi]);
                                                    const float currentDist = DirectX::XMVectorGetX(DirectX::XMVector3Length(currentDir));

                                                    if (currentDist > FloatPrecision)
                                                    {
                                                        currentDir = DirectX::XMVectorDivide(currentDir, DirectX::XMVectorReplicate(currentDist));

                                                        Vector restDirWorld = DirectX::XMVector3Rotate(constraints.restDirLocal[ai], physicsBones.predRot[abi]);
                                                        restDirWorld = DirectX::XMVector3Normalize(restDirWorld);

                                                        float dot = DirectX::XMVectorGetX(DirectX::XMVector3Dot(currentDir, restDirWorld));
                                                        dot = std::clamp(dot, -1.0f, 1.0f);
                                                        const float currentAngle = std::acos(dot);
                                                        if (currentAngle > limitAngle)
                                                        {
                                                            Vector axis = DirectX::XMVector3Cross(restDirWorld, currentDir);
                                                            const float axisLenSq = DirectX::XMVectorGetX(DirectX::XMVector3LengthSq(axis));
                                                            if (axisLenSq > FloatPrecision)
                                                            {
                                                                axis = DirectX::XMVector3Normalize(axis);
                                                                const Quaternion limitRot = DirectX::XMQuaternionRotationAxis(axis, limitAngle);
                                                                const Vector maxAllowedDir = DirectX::XMVector3Rotate(restDirWorld, limitRot);
                                                                const Vector targetPos = DirectX::XMVectorAdd(physicsBones.pred[abi], DirectX::XMVectorScale(maxAllowedDir, currentDist));
                                                                const Vector correction = DirectX::XMVectorSubtract(targetPos, physicsBones.pred[bi]);
                                                                physicsBones.pred[bi] = DirectX::XMVectorAdd(physicsBones.pred[bi], DirectX::XMVectorScale(correction, invMass / wSum));
                                                                if (anchInvMass > 0.0f)
                                                                {
                                                                    physicsBones.pred[abi] = DirectX::XMVectorSubtract(physicsBones.pred[abi], DirectX::XMVectorScale(correction, anchInvMass / wSum));
                                                                }
                                                            }
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
                    if (!angularConstraintsGroup.empty())
                    {
                        std::fill(angularConstraints.lambda.begin(), angularConstraints.lambda.end(), 0.0f);
                        const std::size_t groups = constraintsGroup.size() - 1;
                        tbb::parallel_for(
                            tbb::blocked_range<std::size_t>(0, groups),
                            [&](const tbb::blocked_range<std::size_t>& r) {
                                for (std::size_t g = r.begin(); g != r.end(); ++g)
                                {
                                    const std::size_t begin = angularConstraintsGroup[g];
                                    const std::size_t end = angularConstraintsGroup[g + 1];
                                    if (begin >= end)
                                        continue;
                                    const std::uint32_t oi = angularConstraints.objIdx[begin];
                                    if (oi == UINT32_MAX || objectDatas.isDisable[oi])
                                        continue;

                                    auto cgIt = std::lower_bound(angularConstraintsColorGroup.begin(), angularConstraintsColorGroup.end(), begin);
                                    std::size_t ccgi = std::distance(angularConstraintsColorGroup.begin(), cgIt);

                                    while (ccgi < angularConstraintsColorGroup.size() - 1 && angularConstraintsColorGroup[ccgi] < end)
                                    {
                                        std::size_t c_begin = angularConstraintsColorGroup[ccgi];
                                        std::size_t c_end = std::min(static_cast<std::size_t>(angularConstraintsColorGroup[ccgi + 1]), end);

                                        for (std::size_t ci = c_begin; ci < c_end; ++ci)
                                        {
                                            const std::uint32_t bi = angularConstraints.boneIdx[ci];
                                            const float invInertia = physicsBones.invInertia[bi];
                                            if (invInertia == 0.0f)
                                                continue;
                                            const float invMass = physicsBones.invMass[bi];
                                            if (invMass == 0.0f)
                                                continue;

                                            const std::uint8_t numAnchors = angularConstraints.numAnchors[ci];
                                            const std::size_t aciBase = ci * ANCHOR_MAX;
                                            for (std::uint8_t anchor = 0; anchor < numAnchors; ++anchor)
                                            {
                                                const std::size_t ai = aciBase + anchor;
                                                const std::uint32_t& abi = angularConstraints.anchIdx[ai];
                                                if (abi == UINT32_MAX)
                                                    continue;

                                                const float invInertiaA = physicsBones.invInertia[abi];
                                                const float wSum = invInertiaA + invInertia;
                                                if (wSum == 0.0f)
                                                    continue;

                                                const Quaternion target = DirectX::XMQuaternionMultiply(angularConstraints.restRot[ai], physicsBones.predRot[abi]);
                                                const Quaternion targetInv = DirectX::XMQuaternionConjugate(target);
                                                const Quaternion diff = DirectX::XMQuaternionMultiply(targetInv, physicsBones.predRot[bi]);
                                                Vector omega = DirectX::XMVectorGetW(diff) < 0.0f ? DirectX::XMVectorNegate(diff) : diff;
                                                omega = DirectX::XMVectorSetW(omega, 0.0f);
                                                const Vector CSq = DirectX::XMVector3LengthSq(omega);
                                                if (DirectX::XMVectorGetX(CSq) <= FloatPrecision)
                                                    continue;

                                                const Vector C = DirectX::XMVectorSqrt(CSq);
                                                const Vector correctionDir = DirectX::XMVectorDivide(omega, C);
                                                const float currentLimit = angularConstraints.limit[ai];
                                                float currentFactor = 1.0f;
                                                if (currentLimit > 0.0f)
                                                {
                                                    const float ratio = std::min(DirectX::XMVectorGetX(C) / currentLimit, 1.0f);
                                                    const float smoothFalloff = (1.0f - ratio) * (1.0f - ratio);
                                                    currentFactor = std::max(0.05f, smoothFalloff);
                                                }
                                                const float alphaProxy = (angularConstraints.comp[ai] * currentFactor) * invDtSq;

                                                float& currentLambda = angularConstraints.lambda[ai];
                                                const float deltaLambda = (-DirectX::XMVectorGetX(C) - alphaProxy * currentLambda) / (wSum + alphaProxy);
                                                currentLambda += deltaLambda;

                                                {
                                                    const Vector correction = DirectX::XMVectorScale(correctionDir, deltaLambda);
                                                    if (invInertiaA > 0.0f)
                                                    {
                                                        const Vector thetaA = DirectX::XMVectorSetW(DirectX::XMVectorScale(correction, -invInertiaA), 0.0f);
                                                        const Vector dqA = DirectX::XMVectorScale(DirectX::XMQuaternionMultiply(physicsBones.predRot[abi], thetaA), 0.5f);
                                                        physicsBones.predRot[abi] = DirectX::XMQuaternionNormalize(DirectX::XMVectorAdd(physicsBones.predRot[abi], dqA));
                                                    }
                                                    const Vector theta = DirectX::XMVectorSetW(DirectX::XMVectorScale(correction, invInertia), 0.0f);
                                                    const Vector dq = DirectX::XMVectorScale(DirectX::XMQuaternionMultiply(physicsBones.predRot[bi], theta), 0.5f);
                                                    physicsBones.predRot[bi] = DirectX::XMQuaternionNormalize(DirectX::XMVectorAdd(physicsBones.predRot[bi], dq));
                                                }

                                                const float damping = angularConstraints.damping[ai];
                                                if (damping > 0.0f)
                                                {
                                                    auto CalcAngVel = [](const Quaternion& predQ, const Quaternion& prevQ) -> Vector {
                                                        Quaternion dq = DirectX::XMQuaternionMultiply(predQ, DirectX::XMQuaternionConjugate(prevQ));
                                                        if (DirectX::XMVectorGetW(dq) < 0.0f)
                                                            dq = DirectX::XMVectorNegate(dq);
                                                        return DirectX::XMVectorSetW(DirectX::XMVectorScale(dq, 2.0f), 0.0f);
                                                    };

                                                    Vector angVelBi = CalcAngVel(physicsBones.predRot[bi], physicsBones.rot[bi]);
                                                    Vector angVelAbi = CalcAngVel(physicsBones.predRot[abi], physicsBones.rot[abi]);

                                                    Vector relAngVel = DirectX::XMVectorSubtract(angVelBi, angVelAbi);
                                                    float projAngSpeed = DirectX::XMVectorGetX(DirectX::XMVector3Dot(relAngVel, correctionDir));

                                                    float dampingCorrectionMag = -projAngSpeed * damping;
                                                    Vector dampCorrection = DirectX::XMVectorScale(correctionDir, dampingCorrectionMag / wSum);

                                                    if (invInertiaA > 0.0f)
                                                    {
                                                        const Vector thetaA = DirectX::XMVectorSetW(DirectX::XMVectorScale(dampCorrection, -invInertiaA), 0.0f);
                                                        const Vector dqA = DirectX::XMVectorScale(DirectX::XMQuaternionMultiply(physicsBones.predRot[abi], thetaA), 0.5f);
                                                        physicsBones.predRot[abi] = DirectX::XMQuaternionNormalize(DirectX::XMVectorAdd(physicsBones.predRot[abi], dqA));
                                                    }
                                                    const Vector theta = DirectX::XMVectorSetW(DirectX::XMVectorScale(dampCorrection, invInertia), 0.0f);
                                                    const Vector dq = DirectX::XMVectorScale(DirectX::XMQuaternionMultiply(physicsBones.predRot[bi], theta), 0.5f);
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
            });
        }
        TIMELOG_END;
    }

    void XPBDWorld::SolveCollisions(const float deltaTime)
    {
        if (collidersGroup.empty())
            return;
        // logger::info("{}", __func__);
        const float InvDtSq = 1.0f / (deltaTime * deltaTime);
        TIMELOG_START;

        auto ResolveCollision = [&](const std::uint32_t coiA, const std::uint32_t coiB) {
            ContactManifold manifold;
            if (!CheckCollisionConvexHullVSConvexHull(coiA, coiB, manifold))
                return;

            const std::uint32_t biA = colliders.boneIdx[coiA];
            const std::uint32_t biB = colliders.boneIdx[coiB];

            const float wA = physicsBones.invMass[biA];
            const float wB = physicsBones.invMass[biB];
#ifndef COLLISION_ROTATE
            const float wSum = wA + wB;
            if (wA + wB <= FloatPrecision)
                return;

            float maxDepth = 0.0f;
            for (std::uint8_t i = 0; i < manifold.pointCount; ++i)
            {
                const auto& cp = manifold.points[i];
                const Vector rA = DirectX::XMVector3Rotate(DirectX::XMVectorScale(cp.localPointA, physicsBones.worldScale[biA]), physicsBones.predRot[biA]);
                const Vector rB = DirectX::XMVector3Rotate(DirectX::XMVectorScale(cp.localPointB, physicsBones.worldScale[biB]), physicsBones.predRot[biB]);

                const Vector currWA = DirectX::XMVectorAdd(physicsBones.pred[biA], rA);
                const Vector currWB = DirectX::XMVectorAdd(physicsBones.pred[biB], rB);

                const Vector penetration = DirectX::XMVectorSubtract(currWA, currWB);
                const float currentDepth = -DirectX::XMVectorGetX(DirectX::XMVector3Dot(penetration, manifold.normal));

                if (currentDepth > maxDepth)
                {
                    maxDepth = currentDepth;
                }
            }

            if (maxDepth > 0.0f)
            {
                const float colComp = physicsBones.colComp[biA] + physicsBones.colComp[biB];
                const float alphaProxy = (colComp) * InvDtSq;
                const float lambda = maxDepth / (wSum + alphaProxy);

                if (wA > 0.0f)
                {
                    const float dispMag = lambda * wA;

                    DirectX::XMFLOAT3 n;
                    DirectX::XMStoreFloat3(&n, manifold.normal);
                    std::atomic_ref<float>(colliders.collisionCache[coiA].nx).fetch_add(n.x * maxDepth, std::memory_order_relaxed);
                    std::atomic_ref<float>(colliders.collisionCache[coiA].ny).fetch_add(n.y * maxDepth, std::memory_order_relaxed);
                    std::atomic_ref<float>(colliders.collisionCache[coiA].nz).fetch_add(n.z * maxDepth, std::memory_order_relaxed);
                    std::atomic_ref<std::uint32_t>(colliders.collisionCache[coiA].count).fetch_add(1, std::memory_order_relaxed);
                    AtomicMax(colliders.collisionCache[coiA].maxDisp, dispMag);

                    std::atomic_ref<float>(physicsBones.frictionCache[biA].nx).fetch_add(n.x, std::memory_order_relaxed);
                    std::atomic_ref<float>(physicsBones.frictionCache[biA].ny).fetch_add(n.y, std::memory_order_relaxed);
                    std::atomic_ref<float>(physicsBones.frictionCache[biA].nz).fetch_add(n.z, std::memory_order_relaxed);
                    std::atomic_ref<float>(physicsBones.frictionCache[biA].depth).fetch_add(maxDepth, std::memory_order_relaxed);
                }
                if (wB > 0.0f)
                {
                    const float dispMag = lambda * wB;

                    DirectX::XMFLOAT3 n;
                    DirectX::XMStoreFloat3(&n, DirectX::XMVectorNegate(manifold.normal));
                    std::atomic_ref<float>(colliders.collisionCache[coiB].nx).fetch_add(n.x * maxDepth, std::memory_order_relaxed);
                    std::atomic_ref<float>(colliders.collisionCache[coiB].ny).fetch_add(n.y * maxDepth, std::memory_order_relaxed);
                    std::atomic_ref<float>(colliders.collisionCache[coiB].nz).fetch_add(n.z * maxDepth, std::memory_order_relaxed);
                    std::atomic_ref<std::uint32_t>(colliders.collisionCache[coiB].count).fetch_add(1, std::memory_order_relaxed);
                    AtomicMax(colliders.collisionCache[coiB].maxDisp, dispMag);

                    std::atomic_ref<float>(physicsBones.frictionCache[biB].nx).fetch_add(n.x, std::memory_order_relaxed);
                    std::atomic_ref<float>(physicsBones.frictionCache[biB].ny).fetch_add(n.y, std::memory_order_relaxed);
                    std::atomic_ref<float>(physicsBones.frictionCache[biB].nz).fetch_add(n.z, std::memory_order_relaxed);
                    std::atomic_ref<float>(physicsBones.frictionCache[biB].depth).fetch_add(maxDepth, std::memory_order_relaxed);
                }
            }
#else
            if (wA + wB <= FloatPrecision)
                return;

            const float invIA = physicsBones.invInertia[biA];
            const float invIB = physicsBones.invInertia[biB];

            float maxDepth = 0.0f;
            float sumDepth = 0.0f;
            Vector weighted_rA = EmptyVector;
            Vector weighted_rB = EmptyVector;

            for (std::uint8_t i = 0; i < manifold.pointCount; ++i)
            {
                const auto& cp = manifold.points[i];
                const Vector rA = DirectX::XMVector3Rotate(DirectX::XMVectorScale(cp.localPointA, physicsBones.worldScale[biA]), physicsBones.predRot[biA]);
                const Vector rB = DirectX::XMVector3Rotate(DirectX::XMVectorScale(cp.localPointB, physicsBones.worldScale[biB]), physicsBones.predRot[biB]);

                const Vector currWA = DirectX::XMVectorAdd(physicsBones.pred[biA], rA);
                const Vector currWB = DirectX::XMVectorAdd(physicsBones.pred[biB], rB);

                const Vector penetration = DirectX::XMVectorSubtract(currWA, currWB);
                const float currentDepth = -DirectX::XMVectorGetX(DirectX::XMVector3Dot(penetration, manifold.normal));
                if (currentDepth > 0.0f)
                {
                    if (currentDepth > maxDepth)
                        maxDepth = currentDepth;
                    sumDepth += currentDepth;
                    weighted_rA = DirectX::XMVectorAdd(weighted_rA, DirectX::XMVectorScale(rA, currentDepth));
                    weighted_rB = DirectX::XMVectorAdd(weighted_rB, DirectX::XMVectorScale(rB, currentDepth));
                }
            }

            if (maxDepth > 0.0f && sumDepth > 0.0f)
            {
                const Vector virtual_rA = DirectX::XMVectorScale(weighted_rA, 1.0f / sumDepth);
                const Vector virtual_rB = DirectX::XMVectorScale(weighted_rB, 1.0f / sumDepth);

                const Vector rAxN = DirectX::XMVector3Cross(virtual_rA, manifold.normal);
                const Vector rBxN = DirectX::XMVector3Cross(virtual_rB, manifold.normal);

                const float wARot = invIA * DirectX::XMVectorGetX(DirectX::XMVector3LengthSq(rAxN));
                const float wBRot = invIB * DirectX::XMVectorGetX(DirectX::XMVector3LengthSq(rBxN));

                const float wSum = wA + wB + wARot + wBRot;

                const float colComp = physicsBones.colComp[biA] + physicsBones.colComp[biB];
                const float alphaProxy = colComp * InvDtSq;
                const float lambda = maxDepth / (wSum + alphaProxy);
                if (wA > 0.0f)
                {
                    const float dispMag = lambda * wA;
                    DirectX::XMFLOAT3 n;
                    DirectX::XMStoreFloat3(&n, manifold.normal);

                    std::atomic_ref<float>(colliders.collisionCache[coiA].nx).fetch_add(n.x * maxDepth, std::memory_order_relaxed);
                    std::atomic_ref<float>(colliders.collisionCache[coiA].ny).fetch_add(n.y * maxDepth, std::memory_order_relaxed);
                    std::atomic_ref<float>(colliders.collisionCache[coiA].nz).fetch_add(n.z * maxDepth, std::memory_order_relaxed);
                    std::atomic_ref<std::uint32_t>(colliders.collisionCache[coiA].count).fetch_add(1, std::memory_order_relaxed);
                    AtomicMax(colliders.collisionCache[coiA].maxDisp, dispMag);

                    if (invIA > 0.0f)
                    {
                        const Vector dThetaA = DirectX::XMVectorScale(rAxN, lambda * invIA);
                        DirectX::XMFLOAT3 dta;
                        DirectX::XMStoreFloat3(&dta, dThetaA);
                        std::atomic_ref<float>(colliders.collisionCache[coiA].dtx).fetch_add(dta.x, std::memory_order_relaxed);
                        std::atomic_ref<float>(colliders.collisionCache[coiA].dty).fetch_add(dta.y, std::memory_order_relaxed);
                        std::atomic_ref<float>(colliders.collisionCache[coiA].dtz).fetch_add(dta.z, std::memory_order_relaxed);
                    }

                    std::atomic_ref<float>(physicsBones.frictionCache[biA].nx).fetch_add(n.x, std::memory_order_relaxed);
                    std::atomic_ref<float>(physicsBones.frictionCache[biA].ny).fetch_add(n.y, std::memory_order_relaxed);
                    std::atomic_ref<float>(physicsBones.frictionCache[biA].nz).fetch_add(n.z, std::memory_order_relaxed);
                    std::atomic_ref<float>(physicsBones.frictionCache[biA].depth).fetch_add(maxDepth, std::memory_order_relaxed);
                }

                if (wB > 0.0f)
                {
                    const float dispMag = lambda * wB;
                    DirectX::XMFLOAT3 n;
                    DirectX::XMStoreFloat3(&n, DirectX::XMVectorNegate(manifold.normal));

                    std::atomic_ref<float>(colliders.collisionCache[coiB].nx).fetch_add(n.x * maxDepth, std::memory_order_relaxed);
                    std::atomic_ref<float>(colliders.collisionCache[coiB].ny).fetch_add(n.y * maxDepth, std::memory_order_relaxed);
                    std::atomic_ref<float>(colliders.collisionCache[coiB].nz).fetch_add(n.z * maxDepth, std::memory_order_relaxed);
                    std::atomic_ref<std::uint32_t>(colliders.collisionCache[coiB].count).fetch_add(1, std::memory_order_relaxed);
                    AtomicMax(colliders.collisionCache[coiB].maxDisp, dispMag);

                    if (invIB > 0.0f)
                    {
                        const Vector dThetaB = DirectX::XMVectorScale(rBxN, -lambda * invIB);
                        DirectX::XMFLOAT3 dtb;
                        DirectX::XMStoreFloat3(&dtb, dThetaB);
                        std::atomic_ref<float>(colliders.collisionCache[coiB].dtx).fetch_add(dtb.x, std::memory_order_relaxed);
                        std::atomic_ref<float>(colliders.collisionCache[coiB].dty).fetch_add(dtb.y, std::memory_order_relaxed);
                        std::atomic_ref<float>(colliders.collisionCache[coiB].dtz).fetch_add(dtb.z, std::memory_order_relaxed);
                    }

                    std::atomic_ref<float>(physicsBones.frictionCache[biB].nx).fetch_add(n.x, std::memory_order_relaxed);
                    std::atomic_ref<float>(physicsBones.frictionCache[biB].ny).fetch_add(n.y, std::memory_order_relaxed);
                    std::atomic_ref<float>(physicsBones.frictionCache[biB].nz).fetch_add(n.z, std::memory_order_relaxed);
                    std::atomic_ref<float>(physicsBones.frictionCache[biB].depth).fetch_add(maxDepth, std::memory_order_relaxed);
                }
            }
#endif // COLLISION_ROTATE
        };

        const std::size_t groups = collidersGroup.size() - 1;
        std::vector<std::vector<AABBPair>> pairs(groups);
        threadPool->Execute([&] {
            tbb::parallel_for(
                tbb::blocked_range<std::size_t>(0, groups),
                [&](const tbb::blocked_range<std::size_t>& r) {
                    for (std::size_t g = r.begin(); g != r.end(); ++g)
                    {
                        const std::size_t beginA = collidersGroup[g];
                        const std::size_t endA = collidersGroup[g + 1];
                        if (beginA >= endA || endA - beginA == 0)
                            continue;
                        const std::uint32_t oi = colliders.objIdx[beginA];
                        if (oi == UINT32_MAX || objectDatas.isDisable[oi])
                            continue;
                        const auto& ownHash = objectHashes[oi];
                        tbb::parallel_for(
                            tbb::blocked_range<std::uint32_t>(beginA, endA),
                            [&](const tbb::blocked_range<std::uint32_t>& cr) {
                                std::vector<std::uint32_t> checkedB;
                                checkedB.reserve(128);
                                for (std::uint32_t ciA = cr.begin(); ciA != cr.end(); ++ciA)
                                {
                                    const std::uint32_t biA = colliders.boneIdx[ciA];
                                    if (physicsBones.invMass[biA] == 0.0f)
                                        continue;

                                    const std::uint32_t hashHighA = ownHash.HashWorldCoordsHigh(physicsBones.pred[biA]);
                                    const std::uint32_t hashLowA = ownHash.HashWorldCoordsLow(physicsBones.pred[biA]);
                                    checkedB.clear();

                                    auto CheckCell = [&](const std::uint32_t hash) {
                                        const std::uint32_t beginHash = ownHash.cellStart[hash];
                                        const std::uint32_t endHash = ownHash.cellStart[hash + 1];
                                        for (std::uint32_t ei = beginHash; ei < endHash; ++ei)
                                        {
                                            const std::uint32_t ciB = ownHash.entries[ei];
                                            if (ciA >= ciB)
                                                continue;
                                            if (std::ranges::any_of(checkedB, [&](std::uint32_t prevB) { return prevB == ciB; }))
                                                continue;
                                            checkedB.push_back(ciB);

                                            const std::uint32_t biB = colliders.boneIdx[ciB];
                                            bool canCollide = true;
                                            const std::uint8_t noColCount = colliders.noCollideCount[ciA];
                                            const std::size_t noColBaseA = ciA * NOCOLLIDE_MAX;
                                            const std::size_t noColBaseB = ciB * NOCOLLIDE_MAX;
                                            for (std::uint8_t nci = 0; nci < noColCount; ++nci)
                                            {
                                                if (colliders.noCollideBoneIdx[noColBaseA + nci] == biB ||
                                                    colliders.noCollideBoneIdx[noColBaseB + nci] == biA)
                                                {
                                                    canCollide = false;
                                                    break;
                                                }
                                            }
                                            if (!canCollide)
                                                continue;
                                            ResolveCollision(ciA, ciB);
                                        }
                                    };

                                    CheckCell(hashHighA);
                                    CheckCell(hashLowA);
                                }
                            },
                            tbb::auto_partitioner()
                        );
                        pairs[g] = GetAABBPairs(oi);
                    }
                },
                tbb::auto_partitioner()
            );
        });
        threadPool->Execute([&] {
            tbb::parallel_for(
                tbb::blocked_range<std::size_t>(0, groups),
                [&](const tbb::blocked_range<std::size_t>& r) {
                    for (std::size_t g = r.begin(); g != r.end(); ++g)
                    {
                        const std::size_t beginA = collidersGroup[g];
                        const std::size_t endA = collidersGroup[g + 1];
                        const std::size_t pGroups = pairs[g].size();
                        if (pGroups == 0)
                            continue;
                        tbb::parallel_for(
                            tbb::blocked_range<std::size_t>(0, pGroups),
                            [&](const tbb::blocked_range<std::size_t>& pr) {
                                for (std::size_t pg = pr.begin(); pg != pr.end(); ++pg)
                                {
                                    const auto& pair = pairs[g][pg];
                                    const auto& anotherHash = objectHashes[pair.objIdxB];

                                    tbb::parallel_for(
                                        tbb::blocked_range<std::uint32_t>(beginA, endA),
                                        [&](const tbb::blocked_range<std::uint32_t>& cr) {
                                            std::vector<std::uint32_t> checkedB;
                                            checkedB.reserve(128);

                                            for (std::uint32_t ciA = cr.begin(); ciA != cr.end(); ++ciA)
                                            {
                                                const std::uint32_t biA = colliders.boneIdx[ciA];
                                                if (physicsBones.invMass[biA] == 0.0f)
                                                    continue;

                                                const std::uint32_t hashHighB = anotherHash.HashWorldCoordsHigh(physicsBones.pred[biA]);
                                                const std::uint32_t hashLowB = anotherHash.HashWorldCoordsLow(physicsBones.pred[biA]);

                                                checkedB.clear();

                                                auto CheckCell = [&](const std::uint32_t hash) {
                                                    const std::uint32_t beginHash = anotherHash.cellStart[hash];
                                                    const std::uint32_t endHash = anotherHash.cellStart[hash + 1];
                                                    for (std::uint32_t ei = beginHash; ei < endHash; ++ei)
                                                    {
                                                        const std::uint32_t ciB = anotherHash.entries[ei];
                                                        if (ciA >= ciB)
                                                            continue;
                                                        if (std::ranges::any_of(checkedB, [&](std::uint32_t prevB) { return prevB == ciB; }))
                                                            continue;
                                                        checkedB.push_back(ciB);
                                                        ResolveCollision(ciA, ciB);
                                                    }
                                                };

                                                CheckCell(hashHighB);
                                                CheckCell(hashLowB);
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

        threadPool->Execute([&] {
            tbb::parallel_for(
                tbb::blocked_range<std::uint32_t>(0, colliders.numColliders),
                [&](const tbb::blocked_range<std::uint32_t>& r) {
                    for (std::uint32_t ci = r.begin(); ci != r.end(); ++ci)
                    {
                        const auto& cache = colliders.collisionCache[ci];
                        if (cache.count == 0)
                            continue;
                        const Vector sumNormal = DirectX::XMVectorSet(cache.nx, cache.ny, cache.nz, 0.0f);
                        if (DirectX::XMVectorGetX(DirectX::XMVector3LengthSq(sumNormal)) <= FloatPrecision)
                            continue;
                        const Vector finalNormal = DirectX::XMVector3Normalize(sumNormal);
                        const Vector delta = DirectX::XMVectorScale(finalNormal, cache.maxDisp);
                        const std::uint32_t bi = colliders.boneIdx[ci];
                        physicsBones.pred[bi] = DirectX::XMVectorAdd(physicsBones.pred[bi], delta);
#ifdef COLLISION_ROTATE
                        if (physicsBones.invInertia[bi] > 0.0f)
                        {
                            const float invCount = 1.0f / cache.count;
                            const Vector dTheta = DirectX::XMVectorScale(DirectX::XMVectorSet(cache.dtx, cache.dty, cache.dtz, 0.0f), invCount);
                            const Vector theta = DirectX::XMVectorSetW(dTheta, 0.0f);
                            const Vector dq = DirectX::XMVectorScale(DirectX::XMQuaternionMultiply(physicsBones.predRot[bi], theta), 0.5f);
                            physicsBones.predRot[bi] = DirectX::XMQuaternionNormalize(DirectX::XMVectorAdd(physicsBones.predRot[bi], dq));
                        }
#endif // COLLISION_ROTATE
                        colliders.collisionCache[ci] = {};
                    }
                },
                tbb::auto_partitioner());
        });
        TIMELOG_END;
    }

    void XPBDWorld::UpdateBoneVelocity(const float deltaTime)
    {
        // logger::info("{}", __func__);
        TIMELOG_START;
        const float invDt = 1.0f / deltaTime;
        const std::vector<Quaternion> prevRots = physicsBones.rot;
        threadPool->Execute([&] {
            tbb::parallel_for(
                tbb::blocked_range<std::size_t>(0, physicsBones.numBones),
                [&](const tbb::blocked_range<std::size_t>& r) {
                    for (std::size_t bi = r.begin(); bi != r.end(); ++bi)
                    {
                        const std::uint32_t oi = physicsBones.objIdx[bi];
                        if (oi == UINT32_MAX || objectDatas.isDisable[oi])
                            continue;
                        if (physicsBones.invMass[bi] == 0.0f)
                            continue;
                        const float dampingFactor = 1.0f - physicsBones.damping[bi];
                        physicsBones.vel[bi] = DirectX::XMVectorScale(DirectX::XMVectorSubtract(physicsBones.pred[bi], physicsBones.pos[bi]), invDt * dampingFactor);

                        const auto& frictionCache = physicsBones.frictionCache[bi];
                        if (frictionCache.depth > 0.0f)
                        {
                            const Vector normal = DirectX::XMVector3Normalize(DirectX::XMVectorSet(frictionCache.nx, frictionCache.ny, frictionCache.nz, 0.0f));
                            const Vector relativeVel = physicsBones.vel[bi];
                            const Vector vn = DirectX::XMVectorScale(normal, DirectX::XMVectorGetX(DirectX::XMVector3Dot(relativeVel, normal)));

                            Vector vt = DirectX::XMVectorSubtract(relativeVel, vn);
                            const float vtLenSq = DirectX::XMVectorGetX(DirectX::XMVector3LengthSq(vt));

                            if (vtLenSq > FloatPrecision)
                            {
                                const float frictionLimit = physicsBones.colFriction[bi] * (frictionCache.depth * invDt);
                                const float vtLen = std::sqrt(vtLenSq);
                                if (vtLen <= frictionLimit)
                                    vt = EmptyVector;
                                else
                                    vt = DirectX::XMVectorScale(vt, 1.0f - (frictionLimit / vtLen));
                                physicsBones.vel[bi] = DirectX::XMVectorAdd(vn, vt);
                            }
                        }
                        physicsBones.frictionCache[bi] = {};

                        physicsBones.pos[bi] = physicsBones.pred[bi];
                        if (physicsBones.advancedRotation[bi])
                        {
                            Vector dq = DirectX::XMQuaternionMultiply(DirectX::XMQuaternionConjugate(physicsBones.rot[bi]), physicsBones.predRot[bi]);
                            if (DirectX::XMVectorGetW(dq) < 0.0f)
                                dq = DirectX::XMVectorNegate(dq);
                            const float scale = 2.0f * invDt * dampingFactor;
                            physicsBones.angVel[bi] = DirectX::XMVectorScale(dq, scale);
                            physicsBones.rot[bi] = physicsBones.predRot[bi];

                            if (physicsBones.linearRotTorque[bi] > 0.0f)
                            {
                                const Vector boneDir = DirectX::XMVector3Rotate(forwardDir, physicsBones.predRot[bi]);
                                const Vector fakeTorque = DirectX::XMVector3Cross(boneDir, physicsBones.vel[bi]);
                                physicsBones.angVel[bi] = DirectX::XMVectorAdd(physicsBones.angVel[bi], DirectX::XMVectorScale(fakeTorque, physicsBones.linearRotTorque[bi]));
                            }
                        }
                        else
                        {
                            physicsBones.angVel[bi] = EmptyVector;
                            physicsBones.predRot[bi] = physicsBones.rot[bi];
                        }
                    }
                },
                tbb::auto_partitioner()
            );
        });
        TIMELOG_END;
    }

    void XPBDWorld::ApplyToSkyrim(const float deltaTime)
    {
        if (physicsBonesGroup.empty())
            return;
        // logger::info("{}", __func__);
        TIMELOG_START;
        const std::size_t groups = physicsBonesGroup.size() - 1; 
        threadPool->Execute([&] {
            tbb::parallel_for(
                tbb::blocked_range<std::size_t>(0, groups),
                [&](const tbb::blocked_range<std::size_t>& r) {
                    for (std::size_t g = r.begin(); g != r.end(); ++g)
                    {
                        const std::size_t begin = physicsBonesGroup[g];
                        const std::size_t end = physicsBonesGroup[g + 1];
                        if (begin >= end)
                            continue;
                        const std::uint32_t oi = physicsBones.objIdx[begin];
                        if (oi == UINT32_MAX || objectDatas.isDisable[oi])
                            continue;
                        for (std::size_t bi = begin; bi < end; ++bi)
                        {
                            if (physicsBones.invMass[bi] == 0.0f)
                                continue;

                            auto& node = physicsBones.node[bi];
                            if (!node || !node->parent)
                                continue;

                            const RE::NiPoint3 parentWorldPos = node->parent->world.translate;
                            const RE::NiMatrix3 parentWorldRot = node->parent->world.rotate;
                            const float parentWorldScale = node->parent->world.scale;
                            const RE::NiPoint3 physicsWorldPos = ToPoint3(physicsBones.pred[bi]);

                            const RE::NiMatrix3 origWorldRotMat = parentWorldRot * physicsBones.orgLocalRot[bi];
                            const Quaternion qOriginal = ToQuaternion(origWorldRotMat);
                            RE::NiMatrix3 finalWorldRot;
                            if (physicsBones.advancedRotation[bi])
                            {
                                finalWorldRot = ToMatrix(physicsBones.predRot[bi]);
                            }
                            else
                            {
                                Vector restDir = ToVector(parentWorldRot * physicsBones.orgLocalPos[bi]);
                                if (DirectX::XMVectorGetX(DirectX::XMVector3LengthSq(restDir)) < FloatPrecision)
                                    restDir = ToVector(parentWorldRot * (physicsBones.orgLocalRot[bi] * RE::NiPoint3(0.0f, 1.0f, 0.0f)));
                                restDir = DirectX::XMVector3Normalize(restDir);

                                Vector currentDir = DirectX::XMVectorSubtract(ToVector(physicsWorldPos), ToVector(parentWorldPos));
                                if (DirectX::XMVectorGetX(DirectX::XMVector3LengthSq(currentDir)) < FloatPrecision)
                                    currentDir = restDir;
                                else
                                    currentDir = DirectX::XMVector3Normalize(currentDir);

                                const Vector dot = DirectX::XMVector3Dot(restDir, currentDir);
                                Vector target;
                                if (DirectX::XMVectorGetX(dot) < -0.9999f)
                                {
                                    Vector vUp = DirectX::XMVectorSet(0.0f, 1.0f, 0.0f, 0.0f);
                                    if (std::abs(DirectX::XMVectorGetX(restDir)) < 0.99f)
                                        vUp = DirectX::XMVectorSet(1.0f, 0.0f, 0.0f, 0.0f);

                                    const Vector axis = DirectX::XMVector3Normalize(DirectX::XMVector3Cross(vUp, restDir));
                                    target = DirectX::XMQuaternionMultiply(qOriginal, DirectX::XMQuaternionRotationAxis(axis, DirectX::XM_PI));
                                }
                                else
                                {
                                    const Vector rotationAxis = DirectX::XMVector3Cross(restDir, currentDir);
                                    const Vector lookAt = DirectX::XMQuaternionNormalize(DirectX::XMVectorSetW(rotationAxis, DirectX::XMVectorGetX(dot) + 1.0f));
                                    target = DirectX::XMQuaternionNormalize(DirectX::XMQuaternionMultiply(qOriginal, lookAt));
                                }
                                const Vector final = DirectX::XMQuaternionSlerp(qOriginal, target, physicsBones.rotRatio[bi]);

                                finalWorldRot = ToMatrix(final);
                                physicsBones.rot[bi] = final;
                            }

                            const RE::NiPoint3 diff = physicsWorldPos - parentWorldPos;
                            const RE::NiPoint3 localPos = (parentWorldRot.Transpose() * diff) / parentWorldScale;
                            const RE::NiMatrix3 localRot = parentWorldRot.Transpose() * finalWorldRot;

                            node->local.translate = localPos;
                            node->local.rotate = localRot;

                            node->world.translate = parentWorldPos + (parentWorldRot * (localPos * parentWorldScale));
                            node->world.rotate = parentWorldRot * localRot;

                            /*RE::NiUpdateData ctx = {.time = 0.0f, .flags = RE::NiUpdateData::Flag::kNone};
                            node->UpdateWorldData(&ctx);*/
                        }
                    }
                },
                tbb::auto_partitioner()
            );
        });
        TIMELOG_END;
    }

    AABB XPBDWorld::GetObjectAABB(const std::uint32_t objIdx) const
    {
        AABB bounds = {FLT_MAX, FLT_MAX, FLT_MAX, -FLT_MAX, -FLT_MAX, -FLT_MAX};
        bool hasColliders = false;

        if (collidersGroup.empty())
            return bounds;

        const std::size_t groups = collidersGroup.size() - 1;
        for (std::size_t g = 0; g < groups; ++g)
        {
            std::size_t begin = collidersGroup[g];
            std::size_t end = collidersGroup[g + 1];
            if (colliders.objIdx[begin] != objIdx)
                continue;
            if (begin >= end || end - begin == 0)
                break;
            for (std::size_t ci = begin; ci < end; ++ci)
            {
                const std::uint32_t bi = colliders.boneIdx[ci];
                if (bi == UINT32_MAX || physicsBones.invMass[bi] == 0.0f)
                    continue;

                const float scaledRadius = colliders.boundingSphere[ci] * physicsBones.worldScale[bi] + physicsBones.colMargin[bi];
                DirectX::XMFLOAT3 f3;
                DirectX::XMStoreFloat3(&f3, physicsBones.pred[bi]);
                bounds.minX = std::min(bounds.minX, f3.x - scaledRadius);
                bounds.minY = std::min(bounds.minY, f3.y - scaledRadius);
                bounds.minZ = std::min(bounds.minZ, f3.z - scaledRadius);
                bounds.maxX = std::max(bounds.maxX, f3.x + scaledRadius);
                bounds.maxY = std::max(bounds.maxY, f3.y + scaledRadius);
                bounds.maxZ = std::max(bounds.maxZ, f3.z + scaledRadius);
                hasColliders = true;
            }
            break;
        }
        if (!hasColliders)
            return {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
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

    bool XPBDWorld::CheckCollisionConvexHullVSConvexHull(const std::uint32_t coiA, const std::uint32_t coiB, ContactManifold& outManifold)
    {
        const std::uint32_t biA = colliders.boneIdx[coiA];
        const std::uint32_t biB = colliders.boneIdx[coiB];

        const Vector posA = physicsBones.pred[biA];
        const Quaternion rotA = physicsBones.predRot[biA];
        const float scaleA = physicsBones.worldScale[biA];

        const Vector posB = physicsBones.pred[biB];
        const Quaternion rotB = physicsBones.predRot[biB];
        const float scaleB = physicsBones.worldScale[biB];

        const float marginA = physicsBones.colMargin[biA];
        const float marginB = physicsBones.colMargin[biB];
        const float sumMargin = marginA + marginB;

        const auto aWorldAABB = colliders.boundingAABB[coiA].GetWorldAABB(posA, rotA, scaleA);
        const auto bWorldAABB = colliders.boundingAABB[coiB].GetWorldAABB(posB, rotB, scaleB);
        auto aWorldAABB_expanded = aWorldAABB;
        aWorldAABB_expanded.minX -= sumMargin;
        aWorldAABB_expanded.maxX += sumMargin;
        aWorldAABB_expanded.minY -= sumMargin;
        aWorldAABB_expanded.maxY += sumMargin;
        aWorldAABB_expanded.minZ -= sumMargin;
        aWorldAABB_expanded.maxZ += sumMargin;
        if (!aWorldAABB_expanded.Overlaps(bWorldAABB))
            return false;

        const float rA = colliders.boundingSphere[coiA] * scaleA + marginA;
        const float rB = colliders.boundingSphere[coiB] * scaleB + marginB;
        const float sumR = rA + rB;

        DirectX::XMFLOAT3 pA_f3, pB_f3;
        DirectX::XMStoreFloat3(&pA_f3, posA);
        DirectX::XMStoreFloat3(&pB_f3, posB);

        const float pAx = pA_f3.x;
        const float pAy = pA_f3.y;
        const float pAz = pA_f3.z;

        const float pBx = pB_f3.x;
        const float pBy = pB_f3.y;
        const float pBz = pB_f3.z;

        const Quaternion invRotA = DirectX::XMQuaternionConjugate(rotA);
        const Quaternion invRotB = DirectX::XMQuaternionConjugate(rotB);

        const auto& hullA = colliders.convexHullData[coiA];
        const auto& hullB = colliders.convexHullData[coiB];

        float minOverlap = FLT_MAX;
        Vector bestAxis = DirectX::XMVectorZero();
        bool flip = false;

        float histX[AXIS_HISTORY_MAX], histY[AXIS_HISTORY_MAX], histZ[AXIS_HISTORY_MAX];
        std::uint8_t histCount = 0;

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

        auto TestAxis = [&](float ax, float ay, float az) -> bool {
            const float lenSq = ax * ax + ay * ay + az * az;
            if (lenSq < FloatPrecision)
                return true;

            const float invLen = rsqrt(lenSq);
            ax *= invLen;
            ay *= invLen;
            az *= invLen;

            for (std::uint8_t i = 0; i < histCount; ++i)
            {
                const float dot = ax * histX[i] + ay * histY[i] + az * histZ[i];
                if (std::abs(dot) > 0.998f)
                    return true;
            }

            if (histCount < AXIS_HISTORY_MAX)
            {
                histX[histCount] = ax;
                histY[histCount] = ay;
                histZ[histCount] = az;
                histCount++;
            }

            const Vector axis = DirectX::XMVectorSet(ax, ay, az, 0);
            const float pAdotAx = pAx * ax + pAy * ay + pAz * az;
            const float pBdotAx = pBx * ax + pBy * ay + pBz * az;
            if (std::abs(pAdotAx - pBdotAx) > sumR)
                return false;

            const Vector localA = DirectX::XMVector3Rotate(axis, invRotA);
            DirectX::XMFLOAT3 localA_f3;
            DirectX::XMStoreFloat3(&localA_f3, localA);
            const __m256 v_axA = _mm256_set1_ps(localA_f3.x);
            const __m256 v_ayA = _mm256_set1_ps(localA_f3.y);
            const __m256 v_azA = _mm256_set1_ps(localA_f3.z);

            __m256 dotA0 = _mm256_add_ps(
                _mm256_add_ps(_mm256_mul_ps(_mm256_load_ps(&hullA.vX[0]), v_axA),
                              _mm256_mul_ps(_mm256_load_ps(&hullA.vY[0]), v_ayA)),
                _mm256_mul_ps(_mm256_load_ps(&hullA.vZ[0]), v_azA));

            __m256 dotA1 = _mm256_add_ps(
                _mm256_add_ps(_mm256_mul_ps(_mm256_load_ps(&hullA.vX[8]), v_axA),
                              _mm256_mul_ps(_mm256_load_ps(&hullA.vY[8]), v_ayA)),
                _mm256_mul_ps(_mm256_load_ps(&hullA.vZ[8]), v_azA));

            __m256 minA = _mm256_min_ps(dotA0, dotA1);
            __m256 maxA = _mm256_max_ps(dotA0, dotA1);

            const Vector localB = DirectX::XMVector3Rotate(axis, invRotB);
            DirectX::XMFLOAT3 localB_f3;
            DirectX::XMStoreFloat3(&localB_f3, localB);
            const __m256 v_axB = _mm256_set1_ps(localB_f3.x);
            const __m256 v_ayB = _mm256_set1_ps(localB_f3.y);
            const __m256 v_azB = _mm256_set1_ps(localB_f3.z);

            const __m256 dotB0 = _mm256_add_ps(
                _mm256_add_ps(_mm256_mul_ps(_mm256_load_ps(&hullB.vX[0]), v_axB),
                              _mm256_mul_ps(_mm256_load_ps(&hullB.vY[0]), v_ayB)),
                _mm256_mul_ps(_mm256_load_ps(&hullB.vZ[0]), v_azB));

            const __m256 dotB1 = _mm256_add_ps(
                _mm256_add_ps(_mm256_mul_ps(_mm256_load_ps(&hullB.vX[8]), v_axB),
                              _mm256_mul_ps(_mm256_load_ps(&hullB.vY[8]), v_ayB)),
                _mm256_mul_ps(_mm256_load_ps(&hullB.vZ[8]), v_azB));

            const __m256 minB = _mm256_min_ps(dotB0, dotB1);
            const __m256 maxB = _mm256_max_ps(dotB0, dotB1);

            const float minA_ = hmin256_ps(minA) * scaleA + pAdotAx - marginA;
            const float maxA_ = hmax256_ps(maxA) * scaleA + pAdotAx + marginA;
            const float minB_ = hmin256_ps(minB) * scaleB + pBdotAx - marginB;
            const float maxB_ = hmax256_ps(maxB) * scaleB + pBdotAx + marginB;
            if (maxA_ < minB_ || maxB_ < minA_)
                return false;

            const float overlap = std::min(maxA_ - minB_, maxB_ - minA_);
            if (overlap < minOverlap)
            {
                minOverlap = overlap;
                bestAxis = axis;
                flip = (pAdotAx < pBdotAx);
            }

            return true;
        };

        auto& cache = colliders.convexHullCache[GetCacheKey(coiA, coiB)];
        /*const auto cacheKey = GetCacheKey(coiA, coiB);
        const std::size_t shardIdx = cacheKey % SHARD_COUNT;
        Colliders::ConvexHullCache cache;
        {
            std::lock_guard<std::mutex> lock(sharedLocks[shardIdx]);
            cache = colliders.convexHullCaches[shardIdx][cacheKey];
        }*/
        if (cache.lastFrame == currentFrame - 1)
        {
            if (!TestAxis(cache.ax, cache.ay, cache.az))
                return false;
        }

        for (std::uint8_t i = 0; i < COL_FACE_MAX; ++i)
        {
            const Vector nA = DirectX::XMVector3Rotate(DirectX::XMVectorSet(hullA.fX[i], hullA.fY[i], hullA.fZ[i], 0), rotA);
            DirectX::XMFLOAT3 nA_f3;
            DirectX::XMStoreFloat3(&nA_f3, nA);
            if (!TestAxis(nA_f3.x, nA_f3.y, nA_f3.z))
                return false;

            const Vector nB = DirectX::XMVector3Rotate(DirectX::XMVectorSet(hullB.fX[i], hullB.fY[i], hullB.fZ[i], 0), rotB);
            DirectX::XMFLOAT3 nB_f3;
            DirectX::XMStoreFloat3(&nB_f3, nB);
            if (!TestAxis(nB_f3.x, nB_f3.y, nB_f3.z))
                return false;
        }

        Vector wEdgeB[COL_EDGE_MAX];
        bool wEdgeBValid[COL_EDGE_MAX] = {false};
        for (std::uint8_t i = 0; i < COL_EDGE_MAX; ++i)
        {
            const Vector eB = DirectX::XMVectorSet(hullB.eX[i], hullB.eY[i], hullB.eZ[i], 0);
            wEdgeB[i] = DirectX::XMVector3Rotate(eB, rotB);
            if (DirectX::XMVectorGetX(DirectX::XMVector3LengthSq(wEdgeB[i])) >= FloatPrecision)
                wEdgeBValid[i] = true;
        }

        for (std::uint8_t eiA = 0; eiA < COL_EDGE_MAX; ++eiA)
        {
            const Vector eA = DirectX::XMVectorSet(hullA.eX[eiA], hullA.eY[eiA], hullA.eZ[eiA], 0);
            const Vector wA = DirectX::XMVector3Rotate(eA, rotA);

            if (DirectX::XMVectorGetX(DirectX::XMVector3LengthSq(wA)) < FloatPrecision)
                continue;

            for (std::uint8_t eiB = 0; eiB < COL_EDGE_MAX; ++eiB)
            {
                if (!wEdgeBValid[eiB])
                    continue;
                const float edgeDot = DirectX::XMVectorGetX(DirectX::XMVector3Dot(wA, wEdgeB[eiB]));
                if (std::abs(edgeDot) > 0.99f)
                    continue;
                const Vector crossAxis = DirectX::XMVector3Cross(wA, wEdgeB[eiB]);
                DirectX::XMFLOAT3 cross_f3;
                DirectX::XMStoreFloat3(&cross_f3, crossAxis);
                if (!TestAxis(cross_f3.x, cross_f3.y, cross_f3.z))
                    return false;
            }
        }

        const Vector normal = flip ? DirectX::XMVectorNegate(bestAxis) : bestAxis;

        DirectX::XMFLOAT3 best_f3;
        DirectX::XMStoreFloat3(&best_f3, bestAxis);
        cache.ax = best_f3.x;
        cache.ay = best_f3.y;
        cache.az = best_f3.z;
        cache.lastFrame = currentFrame;

        // manifold
        ContactManifold::ContactPoint tempPoints[16];
        std::uint8_t tempCount = 0;
        const float breakThresholdSq = 0.04f;

        for (std::uint8_t i = 0; i < cache.persistentManifold.pointCount; ++i)
        {
            const auto& cp = cache.persistentManifold.points[i];
            const Vector wA = DirectX::XMVectorAdd(posA, DirectX::XMVector3Rotate(DirectX::XMVectorScale(cp.localPointA, scaleA), rotA));
            const Vector wB = DirectX::XMVectorAdd(posB, DirectX::XMVector3Rotate(DirectX::XMVectorScale(cp.localPointB, scaleB), rotB));

            Vector diff = DirectX::XMVectorSubtract(wA, wB);
            float currentDepth = -DirectX::XMVectorGetX(DirectX::XMVector3Dot(diff, normal));
            if (currentDepth < -sumMargin)
                continue;

            Vector projDiff = DirectX::XMVectorAdd(diff, DirectX::XMVectorScale(normal, currentDepth));
            float driftSq = DirectX::XMVectorGetX(DirectX::XMVector3LengthSq(projDiff));
            if (driftSq > breakThresholdSq)
                continue;

            tempPoints[tempCount] = cp;
            tempPoints[tempCount].depth = currentDepth;
            tempCount++;
        }

        const std::uint8_t vCountA = colliders.vertexCount[coiA];
        const std::uint8_t vCountB = colliders.vertexCount[coiB];

        alignas(32) float dotA[COL_VERTEX_MAX];
        alignas(32) float dotB[COL_VERTEX_MAX];

        const Vector localNormalA = DirectX::XMVector3Rotate(normal, invRotA);
        DirectX::XMFLOAT3 lnA_f3;
        DirectX::XMStoreFloat3(&lnA_f3, localNormalA);
        const float posDotA = DirectX::XMVectorGetX(DirectX::XMVector3Dot(posA, normal));

        const __m256 v_lnAx = _mm256_set1_ps(lnA_f3.x);
        const __m256 v_lnAy = _mm256_set1_ps(lnA_f3.y);
        const __m256 v_lnAz = _mm256_set1_ps(lnA_f3.z);
        const __m256 v_scaleA = _mm256_set1_ps(scaleA);
        const __m256 v_posDotA = _mm256_set1_ps(posDotA);
        __m256 v_minDotA = _mm256_set1_ps(FLT_MAX);

        for (std::uint8_t i = 0; i < COL_VERTEX_MAX; i += 8)
        {
            const __m256 vx = _mm256_load_ps(&hullA.vX[i]);
            const __m256 vy = _mm256_load_ps(&hullA.vY[i]);
            const __m256 vz = _mm256_load_ps(&hullA.vZ[i]);

            __m256 dot = _mm256_mul_ps(vx, v_lnAx);
            dot = _mm256_add_ps(dot, _mm256_mul_ps(vy, v_lnAy));
            dot = _mm256_add_ps(dot, _mm256_mul_ps(vz, v_lnAz));
            dot = _mm256_add_ps(v_posDotA, _mm256_mul_ps(dot, v_scaleA));
            _mm256_store_ps(&dotA[i], dot);
            v_minDotA = _mm256_min_ps(v_minDotA, dot);
        }
        const float minDotA = hmin256_ps(v_minDotA);

        const Vector localNormalB = DirectX::XMVector3Rotate(normal, invRotB);
        DirectX::XMFLOAT3 lnB_f3;
        DirectX::XMStoreFloat3(&lnB_f3, localNormalB);
        const float posDotB = DirectX::XMVectorGetX(DirectX::XMVector3Dot(posB, normal));

        const __m256 v_lnBx = _mm256_set1_ps(lnB_f3.x);
        const __m256 v_lnBy = _mm256_set1_ps(lnB_f3.y);
        const __m256 v_lnBz = _mm256_set1_ps(lnB_f3.z);
        const __m256 v_scaleB = _mm256_set1_ps(scaleB);
        const __m256 v_posDotB = _mm256_set1_ps(posDotB);
        __m256 v_maxDotB = _mm256_set1_ps(-FLT_MAX);

        for (std::uint8_t i = 0; i < COL_VERTEX_MAX; i += 8)
        {
            const __m256 vx = _mm256_load_ps(&hullB.vX[i]);
            const __m256 vy = _mm256_load_ps(&hullB.vY[i]);
            const __m256 vz = _mm256_load_ps(&hullB.vZ[i]);

            __m256 dot = _mm256_mul_ps(vx, v_lnBx);
            dot = _mm256_add_ps(dot, _mm256_mul_ps(vy, v_lnBy));
            dot = _mm256_add_ps(dot, _mm256_mul_ps(vz, v_lnBz));
            dot = _mm256_add_ps(v_posDotB, _mm256_mul_ps(dot, v_scaleB));
            _mm256_store_ps(&dotB[i], dot);
            v_maxDotB = _mm256_max_ps(v_maxDotB, dot);
        }
        const float maxDotB = hmax256_ps(v_maxDotB);
        const float tolerance = 0.02f + sumMargin;
        auto AddTempPoint = [&](const Vector& lA, const Vector& lB, const float depth) {
            for (std::uint8_t i = 0; i < tempCount; ++i)
            {
                const float distSq = DirectX::XMVectorGetX(DirectX::XMVector3LengthSq(DirectX::XMVectorSubtract(tempPoints[i].localPointA, lA)));
                if (distSq < 0.001f)
                    return;
            }
            if (tempCount < 16)
            {
                tempPoints[tempCount++] = {lA, lB, depth};
            }
        };

        const __m256 v_tolA = _mm256_set1_ps(minDotA + tolerance);
        for (std::uint8_t i = 0; i < COL_VERTEX_MAX; i += 8)
        {
            const __m256 v_dotA = _mm256_load_ps(&dotA[i]);
            const __m256 cmp = _mm256_cmp_ps(v_dotA, v_tolA, _CMP_LE_OQ);
            const int mask = _mm256_movemask_ps(cmp);

            if (mask == 0)
                continue;
            for (std::uint8_t k = 0; k < 8; ++k)
            {
                if (!((mask >> k) & 1))
                    continue;
                const std::uint8_t idx = i + k;
                if (idx >= vCountA)
                    continue;

                const float pen = maxDotB - dotA[idx];
                if (pen <= 0.0f)
                    continue;
                const Vector lA = DirectX::XMVectorSet(hullA.vX[idx], hullA.vY[idx], hullA.vZ[idx], 0);
                const Vector worldA_i = DirectX::XMVectorAdd(posA, DirectX::XMVector3Rotate(DirectX::XMVectorScale(lA, scaleA), rotA));

                const Vector wB = DirectX::XMVectorAdd(worldA_i, DirectX::XMVectorScale(normal, pen));
                const Vector lB = DirectX::XMVectorScale(DirectX::XMVector3Rotate(DirectX::XMVectorSubtract(wB, posB), invRotB), 1.0f / scaleB);
                AddTempPoint(lA, lB, pen);
            }
        }

        const __m256 v_tolB = _mm256_set1_ps(maxDotB - tolerance);
        for (std::uint8_t i = 0; i < COL_VERTEX_MAX; i += 8)
        {
            const __m256 v_dotB = _mm256_load_ps(&dotB[i]);
            const __m256 cmp = _mm256_cmp_ps(v_dotB, v_tolB, _CMP_GE_OQ);
            const int mask = _mm256_movemask_ps(cmp);

            if (mask == 0)
                continue;
            for (std::uint8_t k = 0; k < 8; ++k)
            {
                if (!((mask >> k) & 1))
                    continue;
                const std::uint8_t idx = i + k;
                if (idx >= vCountB)
                    continue;

                const float pen = dotB[idx] - minDotA;
                if (pen <= 0.0f)
                    continue;
                const Vector lB = DirectX::XMVectorSet(hullB.vX[idx], hullB.vY[idx], hullB.vZ[idx], 0);
                const Vector worldB_i = DirectX::XMVectorAdd(posB, DirectX::XMVector3Rotate(DirectX::XMVectorScale(lB, scaleB), rotB));

                const Vector wA = DirectX::XMVectorSubtract(worldB_i, DirectX::XMVectorScale(normal, pen));
                const Vector lA = DirectX::XMVectorScale(DirectX::XMVector3Rotate(DirectX::XMVectorSubtract(wA, posA), invRotA), 1.0f / scaleA);
                AddTempPoint(lA, lB, pen);
            }
        }
        cache.persistentManifold.normal = normal;
        if (tempCount <= 4)
        {
            cache.persistentManifold.pointCount = tempCount;
            for (std::uint8_t i = 0; i < tempCount; ++i)
            {
                cache.persistentManifold.points[i] = tempPoints[i];
            }
        }
        else
        {
            std::int32_t p1 = 0, p2 = -1, p3 = -1, p4 = -1;
            float maxDistSq = -1.0f, maxTriAreaSq = -1.0f, maxQuadAreaSq = -1.0f;

            for (std::uint8_t i = 1; i < tempCount; ++i)
            {
                if (tempPoints[i].depth > tempPoints[p1].depth)
                    p1 = i;
            }

            for (std::uint8_t i = 0; i < tempCount; ++i)
            {
                if (i == p1)
                    continue;
                const float dSq = DirectX::XMVectorGetX(DirectX::XMVector3LengthSq(DirectX::XMVectorSubtract(tempPoints[i].localPointA, tempPoints[p1].localPointA)));
                if (dSq > maxDistSq)
                {
                    maxDistSq = dSq;
                    p2 = i;
                }
            }

            if (p2 != -1)
            {
                for (std::uint8_t i = 0; i < tempCount; ++i)
                {
                    if (i == p1 || i == p2)
                        continue;
                    const Vector edge1 = DirectX::XMVectorSubtract(tempPoints[i].localPointA, tempPoints[p1].localPointA);
                    const Vector edge2 = DirectX::XMVectorSubtract(tempPoints[p2].localPointA, tempPoints[p1].localPointA);
                    const float areaSq = DirectX::XMVectorGetX(DirectX::XMVector3LengthSq(DirectX::XMVector3Cross(edge1, edge2)));
                    if (areaSq > maxTriAreaSq)
                    {
                        maxTriAreaSq = areaSq;
                        p3 = i;
                    }
                }
            }

            if (p3 != -1)
            {
                for (std::uint32_t i = 0; i < tempCount; ++i)
                {
                    if (i == p1 || i == p2 || i == p3)
                        continue;
                    const Vector e1 = DirectX::XMVector3Cross(DirectX::XMVectorSubtract(tempPoints[i].localPointA, tempPoints[p1].localPointA), DirectX::XMVectorSubtract(tempPoints[p2].localPointA, tempPoints[p1].localPointA));
                    const Vector e2 = DirectX::XMVector3Cross(DirectX::XMVectorSubtract(tempPoints[i].localPointA, tempPoints[p2].localPointA), DirectX::XMVectorSubtract(tempPoints[p3].localPointA, tempPoints[p2].localPointA));
                    const Vector e3 = DirectX::XMVector3Cross(DirectX::XMVectorSubtract(tempPoints[i].localPointA, tempPoints[p3].localPointA), DirectX::XMVectorSubtract(tempPoints[p1].localPointA, tempPoints[p3].localPointA));
                    const float areaSqSum = DirectX::XMVectorGetX(DirectX::XMVector3LengthSq(e1)) + DirectX::XMVectorGetX(DirectX::XMVector3LengthSq(e2)) + DirectX::XMVectorGetX(DirectX::XMVector3LengthSq(e3));
                    if (areaSqSum > maxQuadAreaSq)
                    {
                        maxQuadAreaSq = areaSqSum;
                        p4 = i;
                    }
                }
            }

            std::uint8_t fCount = 0;
            cache.persistentManifold.points[fCount++] = tempPoints[p1];
            if (p2 != -1)
                cache.persistentManifold.points[fCount++] = tempPoints[p2];
            if (p3 != -1)
                cache.persistentManifold.points[fCount++] = tempPoints[p3];
            if (p4 != -1)
                cache.persistentManifold.points[fCount++] = tempPoints[p4];
            cache.persistentManifold.pointCount = fCount;
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
        physicsBones.pred.reserve(n);
        physicsBones.vel.reserve(n);

        physicsBones.advancedRotation.reserve(n);
        physicsBones.rot.reserve(n);
        physicsBones.prevRot.reserve(n);
        physicsBones.predRot.reserve(n);
        physicsBones.angVel.reserve(n);
        physicsBones.invInertia.reserve(n);

        physicsBones.damping.reserve(n);
        physicsBones.inertiaScale.reserve(n);
        physicsBones.rotRatio.reserve(n);
        physicsBones.gravity.reserve(n);
        physicsBones.offset.reserve(n);
        physicsBones.invMass.reserve(n);

        physicsBones.linearRotTorque.reserve(n);

        physicsBones.colMargin.reserve(n);
        physicsBones.colFriction.reserve(n);
        physicsBones.colComp.reserve(n);

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
        physicsBones.pos.push_back(EmptyVector);
        physicsBones.pred.push_back(EmptyVector);
        physicsBones.vel.push_back(EmptyVector);

        physicsBones.advancedRotation.push_back(0);
        physicsBones.rot.push_back(EmptyVector);
        physicsBones.prevRot.push_back(EmptyVector);
        physicsBones.predRot.push_back(EmptyVector);
        physicsBones.angVel.push_back(EmptyVector);
        physicsBones.invInertia.push_back(0);

        physicsBones.damping.push_back(0);
        physicsBones.inertiaScale.push_back(0);
        physicsBones.rotRatio.push_back(0);
        physicsBones.gravity.push_back(ToVector(GetSkyrimGravity(1.0f)));
        physicsBones.offset.push_back({0, 0, 0});
        physicsBones.invMass.push_back(0);

        physicsBones.linearRotTorque.push_back(0);

        physicsBones.colMargin.push_back(0);
        physicsBones.colFriction.push_back(0);
        physicsBones.colComp.push_back(0);

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
        for (std::uint8_t i = 0; i < ANCHOR_MAX; ++i)
        {
            constraints.anchIdx.push_back(UINT32_MAX);
            constraints.restLen.push_back(0.0f);
            constraints.compSquish.push_back(0.0f);
            constraints.compStretch.push_back(0.0f);
            constraints.squishLimit.push_back(0.0f);
            constraints.stretchLimit.push_back(0.0f);
            constraints.angularLimit.push_back(0.0f);
            constraints.restDirLocal.push_back(EmptyVector);
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
        for (std::uint8_t i = 0; i < ANCHOR_MAX; ++i)
        {
            angularConstraints.anchIdx.push_back(UINT32_MAX);
            angularConstraints.restRot.push_back(EmptyVector);
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

        colliders.vertexCount.reserve(n);
        colliders.edgeCount.reserve(n);
        colliders.faceCount.reserve(n);
        colliders.convexHullData.reserve(n);
        colliders.boundingAABB.reserve(n);
        colliders.collisionCache.reserve(n);

        colliders.boundingSphere.reserve(n);
    }
    std::uint32_t XPBDWorld::AllocateCollider()
    {
        const std::uint32_t newIdx = colliders.numColliders++;
        colliders.boneIdx.push_back(UINT32_MAX);
        colliders.objIdx.push_back(UINT32_MAX);
        colliders.rootIdx.push_back(UINT32_MAX);

        colliders.noCollideCount.push_back(0);
        for (std::uint8_t i = 0; i < NOCOLLIDE_MAX; ++i)
        {
            colliders.noCollideBoneIdx.push_back(UINT32_MAX);
        }

        colliders.vertexCount.push_back(0);
        colliders.edgeCount.push_back(0);
        colliders.faceCount.push_back(0);
        colliders.convexHullData.push_back(Colliders::ConvexHullDataBatch());
        colliders.boundingAABB.push_back(AABB());
        colliders.collisionCache.push_back(Colliders::CollisionCache());

        colliders.boundingSphere.push_back(0);
        return newIdx;
    }

    void XPBDWorld::ReorderMaps()
    {
        logger::info("Reorder maps...");

        // PhysicsBones
        {
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
                if (objIdxA != UINT32_MAX && objIdxB != UINT32_MAX
                    && objectDatas.isDisable[objIdxA] != objectDatas.isDisable[objIdxB])
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
                threadPool->Execute([&] {
                    tbb::parallel_for(
                        tbb::blocked_range<std::size_t>(0, validCount),
                        [&](const tbb::blocked_range<std::size_t>& r) {
                            for (std::size_t i = r.begin(); i != r.end(); ++i)
                            {
                                const std::size_t srcIdx = physicsBonesOrder[i];

                                physicsBones.pos[i] = tmpPhysicsBones.pos[srcIdx];
                                physicsBones.pred[i] = tmpPhysicsBones.pred[srcIdx];
                                physicsBones.vel[i] = tmpPhysicsBones.vel[srcIdx];

                                physicsBones.advancedRotation[i] = tmpPhysicsBones.advancedRotation[srcIdx];
                                physicsBones.rot[i] = tmpPhysicsBones.rot[srcIdx];
                                physicsBones.predRot[i] = tmpPhysicsBones.predRot[srcIdx];
                                physicsBones.angVel[i] = tmpPhysicsBones.angVel[srcIdx];
                                physicsBones.invInertia[i] = tmpPhysicsBones.invInertia[srcIdx];

                                physicsBones.damping[i] = tmpPhysicsBones.damping[srcIdx];
                                physicsBones.inertiaScale[i] = tmpPhysicsBones.inertiaScale[srcIdx];
                                physicsBones.rotRatio[i] = tmpPhysicsBones.rotRatio[srcIdx];
                                physicsBones.gravity[i] = tmpPhysicsBones.gravity[srcIdx];
                                physicsBones.offset[i] = tmpPhysicsBones.offset[srcIdx];
                                physicsBones.invMass[i] = tmpPhysicsBones.invMass[srcIdx];

                                physicsBones.linearRotTorque[i] = tmpPhysicsBones.linearRotTorque[srcIdx];

                                physicsBones.colMargin[i] = tmpPhysicsBones.colMargin[srcIdx];
                                physicsBones.colFriction[i] = tmpPhysicsBones.colFriction[srcIdx];
                                physicsBones.colComp[i] = tmpPhysicsBones.colComp[srcIdx];

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
                        tbb::auto_partitioner()
                    );
                });

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
                physicsBones.rotRatio.resize(validCount);
                physicsBones.gravity.resize(validCount);
                physicsBones.offset.resize(validCount);
                physicsBones.invMass.resize(validCount);

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
            }
            threadPool->Execute([&] {
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

                            for (std::uint8_t a = 0; a < ANCHOR_MAX; ++a)
                            {
                                std::size_t ai = static_cast<std::size_t>(i) * ANCHOR_MAX + a;
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

                            for (std::uint8_t a = 0; a < ANCHOR_MAX; ++a)
                            {
                                std::size_t ai = static_cast<std::size_t>(i) * ANCHOR_MAX + a;
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

                            const std::size_t noColBase = static_cast<std::size_t>(i) * NOCOLLIDE_MAX;
                            for (std::uint8_t nc = 0; nc < NOCOLLIDE_MAX; ++nc)
                            {
                                const std::size_t ncIdx = noColBase + nc;
                                if (colliders.noCollideBoneIdx[ncIdx] != UINT32_MAX)
                                {
                                    colliders.noCollideBoneIdx[ncIdx] = oldToNewBoneIdx[colliders.noCollideBoneIdx[ncIdx]];
                                }
                            }
                        }
                    }
                );
            });
        }
        threadPool->Execute([&] {
            tbb::parallel_invoke(
                [&] {
                    // Constraints
                    {
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
                                tbb::blocked_range<std::size_t>(0, validCount),
                                [&](const tbb::blocked_range<std::size_t>& r) {
                                    for (std::size_t i = r.begin(); i != r.end(); ++i)
                                    {
                                        const std::size_t srcIdx = constraintsOrder[i];

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
                                tbb::auto_partitioner());
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
                    }
                },
                [&] {
                    // AngularConstraints
                    {
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
                                tbb::blocked_range<std::size_t>(0, validCount),
                                [&](const tbb::blocked_range<std::size_t>& r) {
                                    for (std::size_t i = r.begin(); i != r.end(); ++i)
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
                                tbb::auto_partitioner());

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
                    }
                },
                [&] {
                    // ConvexHullCollider
                    {
                        collidersOrder.resize(colliders.numColliders);
                        std::iota(collidersOrder.begin(), collidersOrder.end(), 0);
                        std::ranges::sort(collidersOrder, [&](std::uint32_t a, std::uint32_t b) {
                            if ((colliders.boneIdx[a] == UINT32_MAX) != (colliders.boneIdx[b] == UINT32_MAX))
                                return (colliders.boneIdx[a] != UINT32_MAX) > (colliders.boneIdx[b] != UINT32_MAX);
                            const std::uint32_t objIdxA = colliders.objIdx[a];
                            const std::uint32_t objIdxB = colliders.objIdx[b];
                            if (objIdxA != objIdxB)
                                return objIdxA < objIdxB;
                            if (objIdxA != UINT32_MAX && objIdxB != UINT32_MAX && objectDatas.isDisable[objIdxA] != objectDatas.isDisable[objIdxB])
                                return objectDatas.isDisable[objIdxA] < objectDatas.isDisable[objIdxB];
                            if (colliders.rootIdx[a] != colliders.rootIdx[b])
                                return colliders.rootIdx[a] < colliders.rootIdx[b];
                            return colliders.boneIdx[a] < colliders.boneIdx[b];
                        });

                        collidersGroup.clear();
                        std::uint32_t currentObjIdx = UINT32_MAX;
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
                            validCount++;
                        }
                        if (!collidersGroup.empty())
                            collidersGroup.push_back(validCount);

                        {
                            const Colliders tmpColliders = colliders;
                            tbb::parallel_for(
                                tbb::blocked_range<std::size_t>(0, validCount),
                                [&](const tbb::blocked_range<std::size_t>& r) {
                                    for (std::size_t i = r.begin(); i != r.end(); ++i)
                                    {
                                        const std::uint32_t srcIdx = collidersOrder[i];

                                        colliders.boneIdx[i] = tmpColliders.boneIdx[srcIdx];
                                        colliders.objIdx[i] = tmpColliders.objIdx[srcIdx];
                                        colliders.rootIdx[i] = tmpColliders.rootIdx[srcIdx];

                                        colliders.noCollideCount[i] = tmpColliders.noCollideCount[srcIdx];

                                        colliders.vertexCount[i] = tmpColliders.vertexCount[srcIdx];
                                        colliders.edgeCount[i] = tmpColliders.edgeCount[srcIdx];
                                        colliders.faceCount[i] = tmpColliders.faceCount[srcIdx];
                                        colliders.convexHullData[i] = tmpColliders.convexHullData[srcIdx];
                                        colliders.boundingAABB[i] = tmpColliders.boundingAABB[srcIdx];

                                        colliders.boundingSphere[i] = tmpColliders.boundingSphere[srcIdx];

                                        const std::uint32_t dstStrideBase = i * NOCOLLIDE_MAX;
                                        const std::uint32_t srcStrideBase = srcIdx * NOCOLLIDE_MAX;
                                        for (std::uint32_t s = 0; s < NOCOLLIDE_MAX; ++s)
                                        {
                                            const std::uint32_t dstA = dstStrideBase + s;
                                            const std::uint32_t srcA = srcStrideBase + s;
                                            colliders.noCollideBoneIdx[dstA] = tmpColliders.noCollideBoneIdx[srcA];
                                        }
                                    }
                                },
                                tbb::auto_partitioner());

                            colliders.boneIdx.resize(validCount);
                            colliders.objIdx.resize(validCount);
                            colliders.rootIdx.resize(validCount);
                            colliders.noCollideCount.resize(validCount);
                            colliders.vertexCount.resize(validCount);
                            colliders.edgeCount.resize(validCount);
                            colliders.faceCount.resize(validCount);
                            colliders.convexHullData.resize(validCount);
                            colliders.boundingAABB.resize(validCount);
                            colliders.boundingSphere.resize(validCount);

                            const std::uint32_t validNoCollideCount = validCount * NOCOLLIDE_MAX;
                            colliders.noCollideBoneIdx.resize(validNoCollideCount);
                        }

                        colliders.numColliders = validCount;
                        std::iota(collidersOrder.begin(), collidersOrder.end(), 0);
                        collidersOrder.resize(validCount);
                    }
                }
            );
        });
        logger::info("Reorder maps done");

        BuildConstraintColorGraph();
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

                        const std::size_t aiBase = ci * ANCHOR_MAX;
                        for (std::uint8_t a = 0; a < constraints.numAnchors[ci]; ++a)
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
                        const std::size_t aiBase = ci * ANCHOR_MAX;
                        for (std::uint8_t a = 0; a < constraints.numAnchors[ci]; ++a)
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
                        tbb::blocked_range<std::size_t>(0, constraints.numConstraints),
                        [&](const tbb::blocked_range<std::size_t>& r) {
                            for (std::size_t i = r.begin(); i != r.end(); ++i)
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
                        tbb::auto_partitioner()
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

                        const std::size_t aiBase = ci * ANCHOR_MAX;
                        for (std::uint8_t a = 0; a < angularConstraints.numAnchors[ci]; ++a)
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
                        const std::size_t aiBase = ci * ANCHOR_MAX;
                        for (std::uint8_t a = 0; a < angularConstraints.numAnchors[ci]; ++a)
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
                        tbb::blocked_range<std::size_t>(0, angularConstraints.numConstraints),
                        [&](const tbb::blocked_range<std::size_t>& r) {
                            for (std::size_t i = r.begin(); i != r.end(); ++i)
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
                        tbb::auto_partitioner()
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

    void XPBDWorld::ForceUpdateTransforms(RE::NiAVObject* node, bool isDirty) const
    {
        if (!node)
            return;
        RE::NiUpdateData ctx = {.time = 0.0f, .flags = isDirty ? RE::NiUpdateData::Flag::kDirty : RE::NiUpdateData::Flag::kNone};
        node->UpdateWorldData(&ctx);
        RE::NiNode* ninode = node->AsNode();
        if (!ninode)
            return;
        for (auto& child : ninode->GetChildren())
        {
            if (!child)
                continue;
            ForceUpdateTransforms(child.get(), isDirty);
        }
        return;
    }
} // namespace MXPBD
