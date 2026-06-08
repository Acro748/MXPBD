#pragma once

namespace MXPBD {
    enum CollisionLayer : std::uint32_t {
        kSkeleton = 1 << 0,
        kHead = 1 << 1,
        kRigidBody = 1 << 2,
        kSoftBody = 1 << 3,
        kGenitals = 1 << 4,
        kBody = kHead | kRigidBody | kSoftBody | kGenitals,
        kHair = 1 << 5,
        kFace = 1 << 6,
        kFaceGen = kHair | kFace,
        kWig = 1 << 7,
        kCloth = 1 << 8,
        kSkirt = 1 << 9,
        kCape = 1 << 10,
        kOutfit = kWig | kCloth | kSkirt | kCape,
        kWing = 1 << 11,
        kEars = 1 << 12,
        kTail = 1 << 13,
        kWeapon = 1 << 14,
        kGround = 1 << 15,
        kStatic = 1 << 16,
        kEnvironment = kGround | kStatic,
        kMisc1 = 1 << 17,
        kMisc2 = 1 << 18,
        kMisc3 = 1 << 19,
        kMisc4 = 1 << 20,
        kMisc5 = 1 << 21,
        kMisc6 = 1 << 22,
        kMisc7 = 1 << 23,
        kMisc8 = 1 << 24,
        kMisc9 = 1 << 25,
        kMisc10 = 1 << 26,
        kMisc11 = 1 << 27,
        kMisc12 = 1 << 28,
        kMisc13 = 1 << 29,
        kMisc14 = 1 << 30,
        kMisc15 = 1 << 31,
        kAllLayer = ~0u
    };
    const std::unordered_map<Mus::lString, std::uint32_t>& GetCollisionLayerEnum();
    std::uint32_t GetStringAsBitMask(const Mus::lString& str);

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
        SphereData sphereData = {};
        std::string boneName = "";
        std::unordered_set<std::string> nearBones;
    };
    struct RawColliderData {
        RE::BSGeometry* geometry = nullptr; // use ptr compare only, do not access
        std::uint64_t hash = 0;
        std::vector<RawCollider> rawColliders;
    };

    struct PointCloud {
        std::vector<RE::NiPoint3> vertices;
        std::string boneName;
        std::unordered_set<std::string> nearBones;
    };

    void GenerateSphere(const PointCloud& a_pointCloud, RawCollider& a_rawCollider);
    void GenerateRawConvexHull(const PointCloud& a_pointCloud, RawCollider& a_rawCollider);
    void UpdateRawConvexHull(const PointCloud& a_orgPointCloud, const PointCloud& a_curentPointCloud, RawCollider& a_rawCollider);
    void GenerateConvexHullBatch(const RawCollider& a_rawCollider, ConvexHullDataBatch& a_convexHullDataBatch);

    float GetVolume(const RawCollider& a_convexHull);

    struct BoneVertexData {
        struct Data {
            std::vector<RE::NiPoint3> vertices;
            std::unordered_set<std::string> nearBones;
        };
        std::unordered_map<std::string, Data> boneVertexData;
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
    std::uint32_t GetBipedSlot(RE::BSGeometry* a_geo);

    void writeWaveformOBJ(const std::string& filename, const std::string& objectName, const std::vector<RE::NiPoint3>& vertices, const std::vector<std::uint32_t>& indices);
    void writeWaveformOBJ(const std::string& filename, const std::string& objectName, const std::vector<RE::NiPoint3>& vertices, const std::vector<std::size_t>& indices);
}