#include "MXPBD/ConvexHull.h"
#include "QuickHull.hpp"
#include "meshoptimizer.h"

namespace MXPBD
{
    void GenerateConvexHull(const float* a_inputPoints, std::size_t vertexCount, std::vector<RE::NiPoint3>& a_outVertices, std::vector<std::uint32_t>& a_outIndices, const std::string& debugName)
    {
        quickhull::QuickHull<float> qh;
        auto qhull = qh.getConvexHull(a_inputPoints, vertexCount, true, false);
        auto& qhVertices = qhull.getVertexBuffer();
        auto& qhIndices = qhull.getIndexBuffer();

        std::vector<std::uint32_t> qhIndices_(qhIndices.begin(), qhIndices.end());
        std::vector<std::uint32_t> decimatedIndices(qhIndices.size());
        constexpr std::uint32_t targetTriCount = (COL_VERTEX_MAX * 2) - 4;
        constexpr std::uint32_t targetIndexCount = targetTriCount * 3;
        float maxError = 0.05f;
        std::size_t newIndexCount = 0;
        while (maxError <= 1.0f)
        {
            newIndexCount = meshopt_simplify(decimatedIndices.data(), qhIndices_.data(), qhIndices_.size(), &qhVertices[0].x, qhVertices.size(), sizeof(qhVertices[0]), targetIndexCount, maxError);
            if (newIndexCount <= targetIndexCount)
                break;
            maxError *= 2.0f;
        }
        decimatedIndices.resize(newIndexCount);
        std::vector<uint32_t> remapTable(qhVertices.size());
        std::size_t finalVertexCount = meshopt_optimizeVertexFetchRemap(
            &remapTable[0],
            decimatedIndices.data(),
            decimatedIndices.size(),
            qhVertices.size());
        a_outVertices.resize(finalVertexCount);
        a_outIndices.resize(decimatedIndices.size());
        meshopt_remapIndexBuffer(a_outIndices.data(), decimatedIndices.data(), decimatedIndices.size(), &remapTable[0]);
        meshopt_remapVertexBuffer(a_outVertices.data(), &qhVertices[0].x, qhVertices.size(), sizeof(qhVertices[0]), &remapTable[0]);
        if (!debugName.empty())
            writeWaveformOBJ("data/skse/plugins/MXPBD/DebugConvexHull/" + debugName + ".obj", debugName, a_outVertices, a_outIndices);
    }

    BoneVertexData GetGeometryData(const std::unordered_set<RE::BSGeometry*>& geometries)
    {
        BoneVertexData boneVertData;
        for (auto& geo : geometries)
        {
            if (!geo || geo->name.empty())
                continue;
            logger::debug("Get geometry data... {}", geo->name.c_str());
            const RE::BSTriShape* triShape = geo->AsTriShape();
            if (!triShape)
                continue;
            if (!geo->GetGeometryRuntimeData().skinInstance)
                continue;
            RE::NiSkinInstance* skinInstance = geo->GetGeometryRuntimeData().skinInstance.get();
            if (!skinInstance->skinData || !skinInstance->skinPartition)
                continue;
            const auto& skinPartition = skinInstance->skinPartition;
            if (!skinPartition)
                continue;
            std::uint32_t vertexCount = triShape->GetTrishapeRuntimeData().vertexCount;
            vertexCount = vertexCount > 0 ? vertexCount : skinPartition->vertexCount;
            auto vertexDesc = geo->GetGeometryRuntimeData().vertexDesc;
            const std::uint32_t vertexSize = vertexDesc.GetSize();
            const bool isSkinned = vertexDesc.HasFlag(RE::BSGraphics::Vertex::VF_SKINNED);

            using namespace DirectX;
            std::vector<std::string> IdxToBoneName;
            std::vector<RE::NiTransform> IdxToTransform;
            if (isSkinned)
            {
                const auto& skinData = skinInstance->skinData;
                const auto& bones = skinInstance->bones;
                if (!skinData || !bones)
                    continue;
                const std::uint32_t boneCount = skinData->bones;
                const auto boneData = skinData->boneData;
                if (!boneData || boneCount == 0)
                    continue;
                IdxToBoneName.resize(boneCount);
                IdxToTransform.resize(boneCount);
                for (std::uint32_t i = 0; i < boneCount; ++i)
                {
                    if (!bones[i] || bones[i]->name.empty())
                        continue;
                    IdxToBoneName[i] = bones[i]->name.c_str();
                    IdxToTransform[i] = boneData[i].skinToBone.Invert();
                }
            }
            else
            {
                if (!geo->parent || geo->parent->name.empty())
                    continue;
                IdxToBoneName.push_back(geo->parent->name.c_str());
                IdxToTransform.push_back(RE::NiTransform());
            }

            std::uint8_t offset = 0;
            if (vertexDesc.HasFlag(RE::BSGraphics::Vertex::VF_VERTEX))
                offset += 16;
            if (vertexDesc.HasFlag(RE::BSGraphics::Vertex::VF_UV))
                offset += 4;
            if (vertexDesc.HasFlag(RE::BSGraphics::Vertex::VF_UV_2))
                offset += 4;
            if (vertexDesc.HasFlag(RE::BSGraphics::Vertex::VF_NORMAL))
                offset += 4;
            if (vertexDesc.HasFlag(RE::BSGraphics::Vertex::VF_TANGENT))
                offset += 4;
            if (vertexDesc.HasFlag(RE::BSGraphics::Vertex::VF_COLORS))
                offset += 4;

            std::uint8_t* dynamicData = nullptr;
            if (auto dynamicTri = geo->AsDynamicTriShape(); dynamicTri)
                dynamicData = reinterpret_cast<std::uint8_t*>(dynamicTri->GetDynamicTrishapeRuntimeData().dynamicData);

            struct BoneData
            {
                std::uint16_t boneWeights[4];
                std::uint8_t boneIdx[4] = {UINT8_MAX, UINT8_MAX, UINT8_MAX, UINT8_MAX};
            };
            const auto& partition = skinPartition->partitions[0];
            for (std::uint32_t vi = 0; vi < vertexCount; ++vi)
            {
                std::uint8_t* block = &partition.buffData->rawVertexData[vi * vertexSize];
                RE::NiPoint3 pos;
                BoneData boneData;
                if (dynamicData)
                {
                    pos = *reinterpret_cast<RE::NiPoint3*>(&dynamicData[vi * 4]);
                }
                else
                {
                    pos = *reinterpret_cast<RE::NiPoint3*>(block);
                }

                if (isSkinned)
                {
                    block += offset;
                    boneData = *reinterpret_cast<BoneData*>(block);
                }
                else
                {
                    boneData.boneIdx[0] = 0;
                }
                for (std::uint8_t bdi = 0; bdi < 4; ++bdi)
                {
                    if (boneData.boneIdx[bdi] == UINT8_MAX || boneData.boneWeights[bdi] <= 0.0001f)
                        continue;
                    const std::uint8_t bi = boneData.boneIdx[bdi];
                    const std::string boneName = IdxToBoneName[bi];
                    const RE::NiPoint3 localPos = IdxToTransform[bi].rotate.Transpose() * (pos - IdxToTransform[bi].translate) * (1.0f / IdxToTransform[bi].scale);
                    boneVertData.boneVertexData[boneName].push_back(localPos);
                    for (std::uint8_t bdiAlt = 0; bdiAlt < 4; ++bdiAlt)
                    {
                        if (boneData.boneIdx[bdiAlt] == UINT8_MAX || boneData.boneWeights[bdi] <= 0.0001f || bdi == bdiAlt)
                            continue;
                        const std::uint8_t altBi = boneData.boneIdx[bdiAlt];
                        boneVertData.nearBones[boneName].insert(IdxToBoneName[altBi]);
                        boneVertData.nearBones[IdxToBoneName[altBi]].insert(boneName);
                    }
                }
            }
        }
        return boneVertData;
    }

    PhysicsInput::ConvexHullColliders GetColliders(const BoneVertexData& vertexDatas)
    {
        PhysicsInput::ConvexHullColliders colliders;
        for (const auto& data : vertexDatas.boneVertexData)
        {
            PhysicsInput::ConvexHullColliders::ConvexHullCollider collider;
            collider.boneName = data.first;
            const float* vertData = reinterpret_cast<const float*>(data.second.data());
            GenerateConvexHull(vertData, data.second.size(), collider.vertices, collider.indices);
            colliders.colliders.push_back(collider);
        }
        colliders.noCollideBones = vertexDatas.nearBones;
        return colliders;
    }

    std::unordered_set<RE::BSGeometry*> GetGeometries(RE::NiNode* root, std::uint32_t bipedSlot)
    {
        std::unordered_set<RE::BSGeometry*> geometries;
        if (!root)
            return geometries;

        RE::BSVisit::TraverseScenegraphGeometries(root, [&](RE::BSGeometry* geo) {
            using State = RE::BSGeometry::States;
            using Feature = RE::BSShaderMaterial::Feature;
            if (!geo || geo->name.empty())
                return RE::BSVisit::BSVisitControl::kContinue;
            if (isOverlayGeometry(geo->name.c_str()))
                return RE::BSVisit::BSVisitControl::kContinue;
            auto effect = geo->GetGeometryRuntimeData().properties[State::kEffect].get();
            if (!effect)
                return RE::BSVisit::BSVisitControl::kContinue;
            auto lightingShader = netimmerse_cast<RE::BSLightingShaderProperty*>(effect);
            if (!lightingShader || !lightingShader->flags.all(RE::BSShaderProperty::EShaderPropertyFlag::kModelSpaceNormals))
                return RE::BSVisit::BSVisitControl::kContinue;
            RE::BSLightingShaderMaterialBase* material = skyrim_cast<RE::BSLightingShaderMaterialBase*>(lightingShader->material);
            if (!material || !material->normalTexture)
                return RE::BSVisit::BSVisitControl::kContinue;

            auto& skinInstance = geo->GetGeometryRuntimeData().skinInstance;
            if (!skinInstance)
                return RE::BSVisit::BSVisitControl::kContinue;

            auto dismember = netimmerse_cast<RE::BSDismemberSkinInstance*>(skinInstance.get());
            if (dismember)
            {
                std::uint32_t slot;
                for (std::int32_t p = 0; p < dismember->GetRuntimeData().numPartitions; p++)
                {
                    auto pslot = dismember->GetRuntimeData().partitions[p].slot;
                    if (pslot < 30 || pslot >= RE::BIPED_OBJECT::kEditorTotal + 30)
                    {
                        if (geo->AsDynamicTriShape())
                        { // maybe head
                            slot = RE::BIPED_OBJECT::kHead;
                        }
                        else if (pslot == 0) // BP_TORSO
                        {
                            slot = RE::BIPED_OBJECT::kBody;
                        }
                        else // unknown slot
                            continue;
                    }
                    else
                        slot = pslot - 30;
                    break;
                }
                if (bipedSlot != slot)
                    return RE::BSVisit::BSVisitControl::kContinue;
            }
            geometries.insert(geo);
            return RE::BSVisit::BSVisitControl::kContinue;
        });
        return geometries;
    }
    std::unordered_set<RE::BSGeometry*> GetGeometries(RE::NiNode* root)
    {
        std::unordered_set<RE::BSGeometry*> geometries;
        if (!root)
            return geometries;

        RE::BSVisit::TraverseScenegraphGeometries(root, [&](RE::BSGeometry* geo) {
            using State = RE::BSGeometry::States;
            using Feature = RE::BSShaderMaterial::Feature;
            if (!geo || geo->name.empty())
                return RE::BSVisit::BSVisitControl::kContinue;
            if (isOverlayGeometry(geo->name.c_str()))
                return RE::BSVisit::BSVisitControl::kContinue;
            auto effect = geo->GetGeometryRuntimeData().properties[State::kEffect].get();
            if (!effect)
                return RE::BSVisit::BSVisitControl::kContinue;
            geometries.insert(geo);
            return RE::BSVisit::BSVisitControl::kContinue;
        });
        return geometries;
    }

    void writeWaveformOBJ(const std::string& filename, const std::string& objectName, const std::vector<RE::NiPoint3>& vertices, const std::vector<std::uint32_t>& indices)
    {
        std::ofstream objFile;
        objFile.open(filename);
        objFile << "o " << objectName << "\n";
        for (const auto& v : vertices)
        {
            objFile << "v " << v.x << " " << v.y << " " << v.z << "\n";
        }
        const auto& indBuf = indices;
        std::size_t triangleCount = indBuf.size() / 3;
        for (size_t i = 0; i < triangleCount; i++)
        {
            objFile << "f " << indBuf[i * 3] + 1 << " " << indBuf[i * 3 + 1] + 1 << " " << indBuf[i * 3 + 2] + 1 << "\n";
        }
        logger::debug("{} : {} vert / {} tris", objectName, vertices.size(), indices.size() / 3);
        objFile.close();
    }
}