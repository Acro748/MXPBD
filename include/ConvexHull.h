#pragma once

namespace MXPBD {
    void GenerateConvexHull(const float* a_inputPoints, std::size_t vertexCount, std::vector<RE::NiPoint3>& a_outVertices, std::vector<std::uint32_t>& a_outIndices, const std::string& debugName = "");

    struct BoneVertexData {
        std::unordered_map<std::string, std::vector<RE::NiPoint3>> boneVertexData;
        std::unordered_map<std::string, std::unordered_set<std::string>> nearBones;
    };
    PhysicsInput::ConvexHullColliders GetColliders(const BoneVertexData& vertexDatas);
    BoneVertexData GetGeometryData(const std::unordered_set<RE::BSGeometry*>& geometries);
    std::unordered_set<RE::BSGeometry*> GetGeometries(RE::NiNode* root, std::uint32_t bipedSlot);
    std::unordered_set<RE::BSGeometry*> GetGeometries(RE::NiNode* root);

    void writeWaveformOBJ(const std::string& filename, const std::string& objectName, const std::vector<RE::NiPoint3>& vertices, const std::vector<std::uint32_t>& indices);
	
}