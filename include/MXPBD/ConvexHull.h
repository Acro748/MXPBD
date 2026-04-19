#pragma once

namespace MXPBD {
    struct PointCloud {
        std::vector<RE::NiPoint3> vertices;
        std::string boneName;
    };
    struct RawConvexHull {
        std::vector<RE::NiPoint3> vertices;
        std::vector<std::uint32_t> indices;
        std::vector<std::uint32_t> orgVertexIndex; // same as vertices
        std::string boneName;
    };
    struct RawConvexHullData {
        RE::BSGeometry* geometry; // use ptr compare only, do not access
        std::vector<RawConvexHull> rawConvexHulls;
        NearBones nearBones;
    };
    struct alignas(64) ConvexHullDataBatch {
        float vX[COL_VERTEX_MAX], vY[COL_VERTEX_MAX], vZ[COL_VERTEX_MAX];
        float eX[COL_EDGE_MAX], eY[COL_EDGE_MAX], eZ[COL_EDGE_MAX];
        float fX[COL_FACE_MAX], fY[COL_FACE_MAX], fZ[COL_FACE_MAX];
    };

    void GenerateRawConvexHull(const PointCloud& a_pointCloud, RawConvexHull& a_rawConvexHull);
    void UpdateRawConvexHull(const PointCloud& a_orgPointCloud, const PointCloud& a_curentPointCloud, RawConvexHull& a_rawConvexHull);
    void GenerateConvexHullBatch(const RawConvexHull& a_rawConvexHull, ConvexHullDataBatch& a_convexHullDataBatch);

    struct BoneVertexData {
        std::unordered_map<std::string, std::vector<RE::NiPoint3>> boneVertexData;
        NearBones nearBones;
    };
    std::vector<PointCloud> ConvertPointClouds(const BoneVertexData& a_boneVertexData);

    inline bool isOverlayGeometry(std::string name) {
        std::transform(name.begin(), name.end(), name.begin(), ::tolower);
        return name.contains("overlay") || name.contains("[ovl") || name.contains("[sovl");
    }
    BoneVertexData GetGeometryData(const std::vector<RE::BSGeometry*>& geometries);
    BoneVertexData GetGeometryData(RE::BSGeometry* geometry);
    std::vector<RE::BSGeometry*> GetGeometries(RE::NiNode* root);

    void writeWaveformOBJ(const std::string& filename, const std::string& objectName, const std::vector<RE::NiPoint3>& vertices, const std::vector<std::uint32_t>& indices);
    void writeWaveformOBJ(const std::string& filename, const std::string& objectName, const std::vector<RE::NiPoint3>& vertices, const std::vector<std::size_t>& indices);
}