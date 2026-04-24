#include "MXPBD/ConvexHull.h"
#include "QuickHull.hpp"
#include "meshoptimizer.h"

namespace MXPBD
{
    void GenerateRawConvexHull(const PointCloud& a_pointCloud, RawConvexHull& a_rawConvexHull)
    {
        quickhull::QuickHull<float> qh;
        auto qhull = qh.getConvexHull(reinterpret_cast<const float*>(a_pointCloud.vertices.data()), a_pointCloud.vertices.size(), false, false);
        auto& qhVertices = qhull.getVertexBuffer();
        auto& qhIndices = qhull.getIndexBuffer();

        a_rawConvexHull.boneName = a_pointCloud.boneName;
        a_rawConvexHull.indices.assign_range(qhIndices);
        a_rawConvexHull.vertices.reserve(qhVertices.size());
        a_rawConvexHull.orgVertexIndex.reserve(qhVertices.size());
        for (const auto& qhv : qhVertices)
        {
            a_rawConvexHull.vertices.push_back(RE::NiPoint3(qhv.x, qhv.y, qhv.z));

            for (std::uint32_t i = 0; i < a_pointCloud.vertices.size(); ++i)
            {
                if (std::abs(qhv.x - a_pointCloud.vertices[i].x) < 0.0001f &&
                    std::abs(qhv.y - a_pointCloud.vertices[i].y) < 0.0001f &&
                    std::abs(qhv.z - a_pointCloud.vertices[i].z) < 0.0001f)
                {
                    a_rawConvexHull.orgVertexIndex.push_back(i);
                    break;
                }
            }
        }
        //writeWaveformOBJ("Data\\SKSE\\Plugins\\MXPBD\\TEST\\" + a_pointCloud.boneName + ".obj", a_pointCloud.boneName, a_rawConvexHull.vertices, a_rawConvexHull.indices);
    }

    void UpdateRawConvexHull(const PointCloud& a_orgPointCloud, const PointCloud& a_curentPointCloud, RawConvexHull& a_rawConvexHull)
    {
        if (a_rawConvexHull.orgVertexIndex.empty())
            return;
        if (a_rawConvexHull.orgVertexIndex.size() > a_rawConvexHull.vertices.size())
            return;
        const std::uint32_t orgVertIdxSize = a_rawConvexHull.orgVertexIndex.size();
        for (std::uint32_t i = 0; i < orgVertIdxSize; ++i)
        {
            std::uint32_t idx = a_rawConvexHull.orgVertexIndex[i];
            if (idx >= a_orgPointCloud.vertices.size() || idx >= a_curentPointCloud.vertices.size())
                continue;
            a_rawConvexHull.vertices[i] += a_curentPointCloud.vertices[idx] - a_orgPointCloud.vertices[idx];
        }
    }

    void GenerateConvexHullBatch(const RawConvexHull& a_rawConvexHull, ConvexHullDataBatch& a_convexHullDataBatch)
    {
        constexpr std::uint32_t targetTriCount = (COL_VERTEX_MAX * 2) - 4;
        constexpr std::uint32_t targetIndexCount = targetTriCount * 3;
        std::vector<std::uint32_t> decimatedIndices(a_rawConvexHull.indices.size());
        float maxError = 0.05f;
        std::size_t newIndexCount = 0;
        while (maxError <= 1.0f)
        {
            newIndexCount = meshopt_simplify(decimatedIndices.data(), a_rawConvexHull.indices.data(), a_rawConvexHull.indices.size(), reinterpret_cast<const float*>(a_rawConvexHull.vertices.data()), a_rawConvexHull.vertices.size(), sizeof(a_rawConvexHull.vertices[0]), targetIndexCount, maxError);
            if (newIndexCount <= targetIndexCount)
                break;
            maxError *= 2.0f;
        }
        decimatedIndices.resize(newIndexCount);
        std::vector<uint32_t> remapTable(a_rawConvexHull.vertices.size());
        std::size_t finalVertexCount = meshopt_optimizeVertexFetchRemap(
            &remapTable[0],
            decimatedIndices.data(),
            decimatedIndices.size(),
            a_rawConvexHull.vertices.size());
        std::vector<RE::NiPoint3> a_outVertices(finalVertexCount);
        std::vector<std::uint32_t> a_outIndices(decimatedIndices.size());
        meshopt_remapIndexBuffer(a_outIndices.data(), decimatedIndices.data(), decimatedIndices.size(), &remapTable[0]);
        meshopt_remapVertexBuffer(a_outVertices.data(), reinterpret_cast<const float*>(a_rawConvexHull.vertices.data()), a_rawConvexHull.vertices.size(), sizeof(a_rawConvexHull.vertices[0]), &remapTable[0]);
    
        quickhull::QuickHull<float> qh_final;
        auto qhull_final = qh_final.getConvexHull(reinterpret_cast<const float*>(a_outVertices.data()), a_outVertices.size(), false, false);
        auto& finalVerts = qhull_final.getVertexBuffer();
        auto& finalIndices = qhull_final.getIndexBuffer();
        std::vector<RE::NiPoint3> finalConvexVerts(finalVerts.size());
        finalConvexVerts.reserve(finalVerts.size());
        for (std::uint8_t i = 0; i < finalVerts.size(); ++i)
        {
            finalConvexVerts[i] = RE::NiPoint3(finalVerts[i].x, finalVerts[i].y, finalVerts[i].z);
        }
        // writeWaveformOBJ("Data\\SKSE\\Plugins\\MXPBD\\TEST\\" + a_rawConvexHull.boneName + ".obj", a_rawConvexHull.boneName, finalConvexVerts, finalIndices);

        const std::uint8_t vCount = static_cast<std::uint8_t>(std::min(finalConvexVerts.size(), static_cast<std::size_t>(COL_VERTEX_MAX)));
        for (std::uint8_t v = 0; v < vCount; ++v)
        {
            const auto p = finalConvexVerts[v];

            a_convexHullDataBatch.vX[v] = p.x;
            a_convexHullDataBatch.vY[v] = p.y;
            a_convexHullDataBatch.vZ[v] = p.z;
        }
        a_convexHullDataBatch.vertexCount = vCount;

        {
            const float lastVX = a_convexHullDataBatch.vX[vCount - 1];
            const float lastVY = a_convexHullDataBatch.vY[vCount - 1];
            const float lastVZ = a_convexHullDataBatch.vZ[vCount - 1];
            for (std::uint8_t v = vCount; v < COL_VERTEX_MAX; ++v)
            {
                a_convexHullDataBatch.vX[v] = lastVX;
                a_convexHullDataBatch.vY[v] = lastVY;
                a_convexHullDataBatch.vZ[v] = lastVZ;
            }
        }

        {
            struct Edge
            {
                RE::NiPoint3 v;
                float length;
            };
            std::vector<Edge> tempEdges;
            tempEdges.reserve(finalIndices.size());

            for (std::size_t i = 0; i + 2 < finalIndices.size(); i += 3)
            {
                const std::uint32_t i0 = finalIndices[i + 0];
                const std::uint32_t i1 = finalIndices[i + 1];
                const std::uint32_t i2 = finalIndices[i + 2];

                auto addTempEdge = [&](std::uint32_t a, std::uint32_t b) {
                    if (a >= vCount || b >= vCount)
                        return;
                    RE::NiPoint3 d = finalConvexVerts[b] - finalConvexVerts[a];
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
            }
            a_convexHullDataBatch.edgeCount = static_cast<std::uint8_t>(finalEdges.size());

            if (finalEdges.empty())
            {
                finalEdges.push_back({1.0f, 0.0f, 0.0f});
            }

            for (std::uint8_t e = 0; e < COL_EDGE_MAX; ++e)
            {
                const std::uint8_t fe = (e < finalEdges.size()) ? e : 0;
                a_convexHullDataBatch.eX[e] = finalEdges[fe].x;
                a_convexHullDataBatch.eY[e] = finalEdges[fe].y;
                a_convexHullDataBatch.eZ[e] = finalEdges[fe].z;
            }
        }

        {
            std::vector<RE::NiPoint3> finalFaces;
            finalFaces.reserve(COL_FACE_MAX);
            for (std::size_t i = 0; i + 2 < finalIndices.size(); i += 3)
            {
                const std::uint32_t i0 = finalIndices[i + 0];
                const std::uint32_t i1 = finalIndices[i + 1];
                const std::uint32_t i2 = finalIndices[i + 2];

                if (i0 >= vCount || i1 >= vCount || i2 >= vCount)
                    continue;

                const auto& v0 = finalConvexVerts[i0];
                const auto& v1 = finalConvexVerts[i1];
                const auto& v2 = finalConvexVerts[i2];

                const RE::NiPoint3 edge1 = v1 - v0;
                const RE::NiPoint3 edge2 = v2 - v0;

                RE::NiPoint3 n = edge2.Cross(edge1);
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
            a_convexHullDataBatch.faceCount = static_cast<std::uint8_t>(finalFaces.size());

            if (finalFaces.empty())
            {
                finalFaces.push_back({1.0f, 0.0f, 0.0f});
            }

            for (std::uint8_t f = 0; f < COL_FACE_MAX; ++f)
            {
                const std::uint8_t ff = (f < finalFaces.size()) ? f : 0;
                a_convexHullDataBatch.fX[f] = finalFaces[ff].x;
                a_convexHullDataBatch.fY[f] = finalFaces[ff].y;
                a_convexHullDataBatch.fZ[f] = finalFaces[ff].z;
            }
        }
    }

    std::vector<PointCloud> ConvertPointClouds(const BoneVertexData& a_boneVertexData)
    {
        std::vector<PointCloud> result;
        result.reserve(a_boneVertexData.boneVertexData.size());
        for (auto& data : a_boneVertexData.boneVertexData)
        {
            result.emplace_back(data.second, data.first);
        }
        return result;
    }

    BoneVertexData GetGeometryData(const std::vector<RE::BSGeometry*>& geometries)
    {
        BoneVertexData boneVertData;
        for (auto& geo : geometries)
        {
            BoneVertexData newBoneVertexData = GetGeometryData(geo);
            for (auto& data : newBoneVertexData.boneVertexData)
            {
                boneVertData.boneVertexData[data.first].append_range(data.second);
            }
            for (auto& nearBone : newBoneVertexData.nearBones)
            {
                boneVertData.nearBones[nearBone.first].insert(nearBone.second.begin(), nearBone.second.end());
            }
        }
        return boneVertData;
    }

    BoneVertexData GetGeometryData(RE::BSGeometry* geometry)
    {
        BoneVertexData boneVertData;
        if (!geometry || geometry->name.empty())
            return boneVertData;
        logger::debug("Get geometry data... {}", geometry->name.c_str());
        const RE::BSTriShape* triShape = geometry->AsTriShape();
        if (!triShape)
            return boneVertData;
        if (!geometry->GetGeometryRuntimeData().skinInstance)
            return boneVertData;
        RE::NiSkinInstance* skinInstance = geometry->GetGeometryRuntimeData().skinInstance.get();
        if (!skinInstance->skinData || !skinInstance->skinPartition)
            return boneVertData;
        const auto& skinPartition = skinInstance->skinPartition;
        if (!skinPartition)
            return boneVertData;
        std::uint32_t vertexCount = triShape->GetTrishapeRuntimeData().vertexCount;
        vertexCount = vertexCount > 0 ? vertexCount : skinPartition->vertexCount;
        auto vertexDesc = geometry->GetGeometryRuntimeData().vertexDesc;
        const std::uint32_t vertexSize = vertexDesc.GetSize();
        const bool isSkinned = vertexDesc.HasFlag(RE::BSGraphics::Vertex::VF_SKINNED);

        struct Bone
        {
            std::string boneName = "";
            RE::NiTransform boneToSkin = {};
            bool isCloned = false;
        };
        std::vector<Bone> IdxToBoneData;
        if (isSkinned)
        {
            const auto& skinData = skinInstance->skinData;
            const auto& bones = skinInstance->bones;
            if (!skinData || !bones)
                return boneVertData;
            const std::uint32_t boneCount = skinData->bones;
            const auto boneData = skinData->boneData;
            if (!boneData || boneCount == 0)
                return boneVertData;
            IdxToBoneData.resize(boneCount);
            for (std::uint32_t i = 0; i < boneCount; ++i)
            {
                if (!bones[i] || bones[i]->name.empty())
                    continue;
                std::string_view boneName = bones[i]->name.c_str();
                IdxToBoneData[i].boneName = boneName;
                IdxToBoneData[i].boneToSkin = boneData[i].skinToBone.Invert();
                IdxToBoneData[i].isCloned = IsCloneNodeName(boneName);
            }
        }
        else
        {
            if (!geometry->parent || geometry->parent->name.empty())
                return boneVertData;
            std::string_view boneName = geometry->parent->name.c_str();
            Bone newBone;
            newBone.boneName = boneName;
            newBone.boneToSkin = {};
            newBone.isCloned = IsCloneNodeName(boneName);
            IdxToBoneData.push_back(newBone);
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

        RE::NiPoint3* dynamicData1 = nullptr;
        DirectX::XMFLOAT4* dynamicData2 = nullptr;
        if (auto dynamicTri = geometry->AsDynamicTriShape(); dynamicTri)
        {
            RE::BSFaceGenBaseMorphExtraData* fodData = netimmerse_cast<RE::BSFaceGenBaseMorphExtraData*>(dynamicTri->GetExtraData("FOD"));
            if (fodData)
                dynamicData1 = fodData->vertexData;
            else if (dynamicTri->GetDynamicTrishapeRuntimeData().dynamicData)
                dynamicData2 = reinterpret_cast<DirectX::XMFLOAT4*>(dynamicTri->GetDynamicTrishapeRuntimeData().dynamicData);
        }
        struct BoneData
        {
            std::uint16_t boneWeights[4];
            std::uint8_t boneIdx[4] = {UINT8_MAX, UINT8_MAX, UINT8_MAX, UINT8_MAX};
        };
        const auto& partition = skinPartition->partitions[0];
        const float validWeightThreshold = Mus::Config::GetSingleton().GetValidBoneWeightThreshold();
        const std::uint32_t boneCount = IdxToBoneData.size();
        for (std::uint32_t vi = 0; vi < vertexCount; ++vi)
        {
            std::uint8_t* block = &partition.buffData->rawVertexData[vi * vertexSize];
            RE::NiPoint3 pos;
            BoneData boneData;
            if (dynamicData1)
            {
                pos.x = dynamicData1[vi].x;
                pos.y = dynamicData1[vi].y;
                pos.z = dynamicData1[vi].z;
            }
            else if (dynamicData2)
            {
                pos.x = dynamicData2[vi].x;
                pos.y = dynamicData2[vi].y;
                pos.z = dynamicData2[vi].z;
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

            std::uint8_t isValidBit = 0;
            for (std::uint8_t bdi = 0; bdi < 4; ++bdi)
            {
                const std::uint8_t bi = boneData.boneIdx[bdi];
                const float boneWeight = DirectX::PackedVector::XMConvertHalfToFloat(boneData.boneWeights[bdi]);
                if (bi >= boneCount || boneWeight <= validWeightThreshold)
                    continue;
                isValidBit |= IdxToBoneData[bi].isCloned << bdi;
            }
            if (isValidBit == 0)
            {
                for (std::uint8_t bdi = 0; bdi < 4; ++bdi)
                {
                    const std::uint8_t bi = boneData.boneIdx[bdi];
                    const float boneWeight = DirectX::PackedVector::XMConvertHalfToFloat(boneData.boneWeights[bdi]);
                    if (bi >= boneCount || boneWeight <= validWeightThreshold)
                        continue;
                    isValidBit |= 1 << bdi;
                }
            }
            auto isValid = [isValidBit](std::uint8_t bdi) {
                return (isValidBit & 1 << bdi);
            };
            for (std::uint8_t bdi = 0; bdi < 4; ++bdi)
            {
                if (!isValid(bdi))
                    continue;
                const std::uint8_t bi = boneData.boneIdx[bdi];
                const std::string boneName = IdxToBoneData[bi].boneName;
                const RE::NiTransform transform = IdxToBoneData[bi].boneToSkin;
                const RE::NiPoint3 localPos = transform.rotate.Transpose() * (pos - transform.translate) * (1.0f / transform.scale);
                boneVertData.boneVertexData[boneName].push_back(localPos);
                for (std::uint8_t bdiAlt = 0; bdiAlt < 4; ++bdiAlt)
                {
                    if (bdi == bdiAlt || !isValid(bdiAlt))
                        continue;
                    const std::uint8_t altBi = boneData.boneIdx[bdiAlt];
                    const std::string nearBoneName = IdxToBoneData[altBi].boneName;
                    boneVertData.nearBones[boneName].insert(nearBoneName);
                }
            }
        }
        return boneVertData;
    }

    std::vector<RE::BSGeometry*> GetGeometries(RE::NiNode* root)
    {
        std::vector<RE::BSGeometry*> geometries;
        if (!root)
            return geometries;

        RE::BSVisit::TraverseScenegraphGeometries(root, [&](RE::BSGeometry* geo) {
            using State = RE::BSGeometry::States;
            using Feature = RE::BSShaderMaterial::Feature;
            if (!geo || geo->name.empty())
                return RE::BSVisit::BSVisitControl::kContinue;
            if (isOverlayGeometry(geo->name.c_str()))
                return RE::BSVisit::BSVisitControl::kContinue;
            if (!geo->GetGeometryRuntimeData().properties[RE::BSGeometry::States::kEffect])
                return RE::BSVisit::BSVisitControl::kContinue;
            auto effect = geo->GetGeometryRuntimeData().properties[RE::BSGeometry::States::kEffect].get();
            auto lightingShader = netimmerse_cast<RE::BSLightingShaderProperty*>(effect);
            if (!lightingShader)
                return RE::BSVisit::BSVisitControl::kContinue;
            if (lightingShader->alpha <= 0.0001f)
                return RE::BSVisit::BSVisitControl::kContinue;
            geometries.push_back(geo);
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

    void writeWaveformOBJ(const std::string& filename, const std::string& objectName, const std::vector<RE::NiPoint3>& vertices, const std::vector<std::size_t>& indices)
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