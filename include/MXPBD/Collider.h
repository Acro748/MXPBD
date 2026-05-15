#pragma once

namespace MXPBD {
    enum ColliderType : std::uint8_t {
        kConvexHull,
        kSphere,
        kNone
    };
    ColliderType GetColliderType(const Mus::lString& str);
    std::string GetColliderTypeStr(const std::uint8_t type);

#if defined(AVX512)
    struct alignas(64) ConvexHullDataBatch {
#elif defined(AVX2) || defined(AVX)
    struct alignas(32) ConvexHullDataBatch {
#endif
        float vX[COL_VERTEX_MAX], vY[COL_VERTEX_MAX], vZ[COL_VERTEX_MAX];
        float eX[COL_EDGE_MAX], eY[COL_EDGE_MAX], eZ[COL_EDGE_MAX];
        float fX[COL_FACE_MAX], fY[COL_FACE_MAX], fZ[COL_FACE_MAX];

        std::uint8_t vertexCount = 0;
        std::uint8_t edgeCount = 0;
        std::uint8_t faceCount = 0;
    };

    struct alignas(32) SphereData {
        float cX[COL_SPHERE_MAX], cY[COL_SPHERE_MAX], cZ[COL_SPHERE_MAX];
        float radius[COL_SPHERE_MAX];
    };

    struct RawCollider {
        std::vector<RE::NiPoint3> vertices;
        std::vector<std::uint32_t> indices;
        std::vector<std::uint32_t> orgVertexIndex; // same as vertices
        SphereData sphereData;
        std::string boneName;
    };
    struct RawColliderData {
        RE::BSGeometry* geometry = nullptr; // use ptr compare only, do not access
        std::uint64_t hash = 0;
        std::vector<RawCollider> rawColliders;
        NearBones nearBones;
    };

    struct PointCloud {
        std::vector<RE::NiPoint3> vertices;
        std::string boneName;
    };

    struct VertexGroup {
        std::vector<RE::NiPoint3> vertices;
        RE::NiPoint3 minPt;
        RE::NiPoint3 maxPt;

        void UpdateBounds() {
            minPt = RE::NiPoint3(FLT_MAX, FLT_MAX, FLT_MAX);
            maxPt = RE::NiPoint3(-FLT_MAX, -FLT_MAX, -FLT_MAX);
            for (const auto& v : vertices) {
                minPt.x = std::min(minPt.x, v.x);
                minPt.y = std::min(minPt.y, v.y);
                minPt.z = std::min(minPt.z, v.z);
                maxPt.x = std::max(maxPt.x, v.x);
                maxPt.y = std::max(maxPt.y, v.y);
                maxPt.z = std::max(maxPt.z, v.z);
            }
        }

        float GetVolume() const {
            return (maxPt.x - minPt.x) * (maxPt.y - minPt.y) * (maxPt.z - minPt.z);
        }
    };

    void GenerateSphere(const PointCloud& a_pointCloud, RawCollider& a_rawCollider);
    void GenerateRawConvexHull(const PointCloud& a_pointCloud, RawCollider& a_rawCollider);
    void UpdateRawConvexHull(const PointCloud& a_orgPointCloud, const PointCloud& a_curentPointCloud, RawCollider& a_rawCollider);
    void GenerateConvexHullBatch(const RawCollider& a_rawCollider, ConvexHullDataBatch& a_convexHullDataBatch);

    float GetVolume(const RawCollider& a_convexHull);

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
    std::uint64_t GetGeometryHash(RE::BSGeometry* a_geometry);

    void writeWaveformOBJ(const std::string& filename, const std::string& objectName, const std::vector<RE::NiPoint3>& vertices, const std::vector<std::uint32_t>& indices);
    void writeWaveformOBJ(const std::string& filename, const std::string& objectName, const std::vector<RE::NiPoint3>& vertices, const std::vector<std::size_t>& indices);
}