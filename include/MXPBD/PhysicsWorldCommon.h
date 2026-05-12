#pragma once

namespace MXPBD {
    constexpr RE::NiPoint3 pZero = RE::NiPoint3(0, 0, 0);
    constexpr RE::NiPoint3 pOne = RE::NiPoint3(1, 1, 1);
    constexpr RE::NiPoint3 pInf = RE::NiPoint3(FLT_MAX, FLT_MAX, FLT_MAX);
    constexpr RE::NiPoint3 pHalfInf = RE::NiPoint3(10000.0f, 10000.0f, 10000.0f);
    constexpr RE::NiPoint3 pRotInf = RE::NiPoint3(DirectX::XM_PI, DirectX::XM_PI, DirectX::XM_PI);
    const RE::NiMatrix3 mZero = RE::NiMatrix3(pZero, pZero, pZero);

    // for ni world : 1m * SkyrimWorldScaleInverse
    // for havok world : 1m * SkyrimWorldUnitInverse * SkyrimWorldScale
    constexpr float SkyrimWorldScale = 0.0142875f;
    constexpr float SkyrimWorldScaleInverse = 1.0f / SkyrimWorldScale;
    constexpr float SkyrimWorldScaleVolume = SkyrimWorldScale * SkyrimWorldScale * SkyrimWorldScale * 1000.0f;
    constexpr float SkyrimWorldUnit = 0.046875f;
    constexpr float SkyrimWorldUnitInverse = 1.0f / SkyrimWorldUnit;

    constexpr float DeltaTime60 = 1.0f / 60.0f;
    const std::size_t CoreCount = std::thread::hardware_concurrency();
    constexpr float Epsilon = 1e-5f;
    constexpr float ns2ms = 1.0f / 1000000.0f;
    constexpr float toDegree = 180.0f / DirectX::XM_PI;
    constexpr float toRadian = DirectX::XM_PI / 180.0f;

    constexpr DirectX::XMVECTORF32 vZero = {0.0f, 0.0f, 0.0f, 0.0f};
    constexpr DirectX::XMVECTORF32 SkyrimGravity = {0.0f, 0.0f, -9.8f * SkyrimWorldScaleInverse, 0.0f};
    constexpr DirectX::XMVECTORF32 vEpsilon = {Epsilon, Epsilon, Epsilon, Epsilon};
    constexpr DirectX::XMVECTORF32 vXone = {1.0f, 0.0f, 0.0f, 0.0f};
    constexpr DirectX::XMVECTORF32 vYone = {0.0f, 1.0f, 0.0f, 0.0f};
    constexpr DirectX::XMVECTORF32 vZone = {0.0f, 0.0f, 1.0f, 0.0f};
    constexpr DirectX::XMVECTORF32 vWone = {0.0f, 0.0f, 0.0f, 1.0f};
    constexpr DirectX::XMVECTORF32 vHalf = {0.5f, 0.5f, 0.5f, 0.5f};
    constexpr DirectX::XMVECTORF32 vOne = {1.0f, 1.0f, 1.0f, 1.0f};
    constexpr DirectX::XMVECTORF32 vTwo = {2.0f, 2.0f, 2.0f, 2.0f};
    constexpr DirectX::XMVECTORF32 vOnePointFive = {1.5f, 1.5f, 1.5f, 1.5f};
    constexpr DirectX::XMVECTORF32 vNegOne = {-1.0f, -1.0f, -1.0f, -1.0f};
    constexpr DirectX::XMVECTORF32 vAxisSimilarityLimit = {0.998f, 0.998f, 0.998f, 0.998f};
    constexpr DirectX::XMVECTORF32 vDotOppositeThreshold = {-0.9999f, -0.9999f, -0.9999f, -0.9999f};
    constexpr DirectX::XMVECTORF32 vAxisOverlapLimit = {-0.99f, -0.99f, -0.99f, -0.99f};
    constexpr DirectX::XMVECTORF32 vCrossProductSkipLimit = {0.99f, 0.99f, 0.99f, 0.99f};
    constexpr DirectX::XMVECTORF32 vBreakThresholdSq = {0.04f, 0.04f, 0.04f, 0.04f};
    constexpr DirectX::XMVECTORF32 vContactMergeThresholdSq = {0.001f, 0.001f, 0.001f, 0.001f};
    constexpr DirectX::XMVECTORF32 vNegInf = {-FLT_MAX, -FLT_MAX, -FLT_MAX, -FLT_MAX};
    constexpr DirectX::XMVECTORF32 vInf = {FLT_MAX, FLT_MAX, FLT_MAX, FLT_MAX};
    constexpr DirectX::XMVECTORF32 vHalfInf = {10000.0f, 10000.0f, 10000.0f, 10000.0f};
    constexpr DirectX::XMVECTORF32 vFloorHigh = {0.25f, 0.25f, 0.25f, 0.25f};
    constexpr DirectX::XMVECTORF32 vFloorLow = {-0.25f, -0.25f, -0.25f, -0.25f};
    constexpr DirectX::XMVECTORF32 vVelReduce = {0.8f, 0.8f, 0.8f, 0.8f};
    constexpr DirectX::XMVECTORF32 vPi = {DirectX::XM_PI, DirectX::XM_PI, DirectX::XM_PI, DirectX::XM_PI};
    constexpr DirectX::XMVECTORF32 vNegPi = {-DirectX::XM_PI, -DirectX::XM_PI, -DirectX::XM_PI, -DirectX::XM_PI};
    constexpr DirectX::XMVECTORF32 vMinScale = {0.01f, 0.01f, 0.01f, 0.01f};
    constexpr DirectX::XMVECTORU32 maskYW = {0, 0xFFFFFFFF, 0, 0xFFFFFFFF};
    constexpr DirectX::XMVECTORU32 maskXZ = {0xFFFFFFFF, 0, 0xFFFFFFFF, 0};
    constexpr DirectX::XMVECTORU32 maskW = {0, 0, 0, 0xFFFFFFFF};
    constexpr DirectX::XMVECTORU32 maskXYZ = {0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 0};
    constexpr DirectX::XMVECTORU32 maskSign = {0x80000000, 0x80000000, 0x80000000, 0x80000000};

    constexpr DirectX::XMVECTORF32 qZero = {0.0f, 0.0f, 0.0f, 1.0f};

    const DirectX::XMMATRIX vmZero = {vZero, vZero, vZero, vWone};
    const DirectX::XMMATRIX vmZeroAll = {vZero, vZero, vZero, vZero};
    const DirectX::XMMATRIX vmIdentity = DirectX::XMMatrixIdentity();

    const __m128i hash_primes = _mm_set_epi32(0, 83492791, 19349663, 73856093);

    constexpr std::uint32_t ANCHOR_MAX = 4;
    constexpr std::uint32_t COL_VERTEX_MAX = 16; // COL_VERTEX_MAX = 16 * qualityLevel
    constexpr std::uint32_t COL_EDGE_MAX = 6;
    constexpr std::uint32_t COL_FACE_MAX = 12;
    constexpr std::uint32_t AXIS_HISTORY_MAX = 92; // AXIS_HISTORY_MAX <= 1 + (COL_FACE_MAX * 2) + (COL_EDGE_MAX * COL_EDGE_MAX);
    constexpr std::uint32_t COL_SPHERE_MAX = 4;
    constexpr std::uint32_t NOCOLLIDE_MAX = 16;
    constexpr float COMPLIANCE_SCALE = 0.0001f;
    constexpr float ROTATION_CLAMP_DEFAULT = 0.2f;
    constexpr float COL_MARGIN_MIN = 0.5f;
    constexpr float ANGULAR_FRICTION_SCALE = 0.3f;
    constexpr float VOLUME_SCALE = 0.0001f;
    constexpr float LINEAR_ANGULAR_COUPLING_MULT = 10.0f;

    const std::string_view cloneNodePrefix = "[MXPBD]";
    
    using Vector = DirectX::XMVECTOR;
    using Quaternion = DirectX::XMVECTOR;
    using Matrix = DirectX::XMMATRIX;

    using NearBones = std::unordered_map<std::string, std::unordered_set<std::string>>;
}
