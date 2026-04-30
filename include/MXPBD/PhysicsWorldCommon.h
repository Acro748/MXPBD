#pragma once

namespace MXPBD {
    constexpr RE::NiPoint3 pZero = RE::NiPoint3(0, 0, 0);
    const RE::NiMatrix3 mZero = RE::NiMatrix3(pZero, pZero, pZero);
    constexpr float Scale_havokWorld = 0.0142875f;
    constexpr float InverseScale_havokWorld = 1.0f / Scale_havokWorld;
    constexpr float Scale_skyrimUnit = 0.046875f;
    constexpr float InverseScale_skyrimUnit = 1.0f / Scale_skyrimUnit;
    constexpr float DeltaTime60 = 1.0f / 60.0f;
    const std::size_t CoreCount = std::thread::hardware_concurrency();
    constexpr float FloatPrecision = 1e-5f;
    constexpr float ns2ms = 1.0f / 1000000.0f;

    const DirectX::XMVECTOR vZero = DirectX::XMVectorZero();
    const DirectX::XMVECTOR GlobalGravity = DirectX::XMVectorSet(0.0f, 0.0f, -9.8f, 0.0f);
    const DirectX::XMVECTOR SkyrimGravity = DirectX::XMVectorScale(GlobalGravity, InverseScale_havokWorld);
    const DirectX::XMVECTOR vFloatPrecision = DirectX::XMVectorReplicate(FloatPrecision);
    const DirectX::XMVECTOR vXone = DirectX::XMVectorSet(1.0f, 0.0f, 0.0f, 0.0f);
    const DirectX::XMVECTOR vYone = DirectX::XMVectorSet(0.0f, 1.0f, 0.0f, 0.0f);
    const DirectX::XMVECTOR vZone = DirectX::XMVectorSet(0.0f, 0.0f, 1.0f, 0.0f);
    const DirectX::XMVECTOR vWone = DirectX::XMVectorSet(0.0f, 0.0f, 0.0f, 1.0f);
    const DirectX::XMVECTOR vHalf = DirectX::XMVectorReplicate(0.5f);
    const DirectX::XMVECTOR vOne = DirectX::XMVectorReplicate(1.0f);
    const DirectX::XMVECTOR vNegOne = DirectX::XMVectorReplicate(-1.0f);
    const DirectX::XMVECTOR vAxisSimilarityLimit = DirectX::XMVectorReplicate(0.998f);
    const DirectX::XMVECTOR vBreakThresholdSq = DirectX::XMVectorReplicate(0.04f);
    const DirectX::XMVECTOR vContactMergeThresholdSq = DirectX::XMVectorReplicate(0.001f);
    const DirectX::XMVECTOR vNegInf = DirectX::XMVectorReplicate(-FLT_MAX);
    const DirectX::XMVECTOR vInf = DirectX::XMVectorReplicate(FLT_MAX);
    const DirectX::XMVECTOR vFloorHigh = DirectX::XMVectorReplicate(0.25f);
    const DirectX::XMVECTOR vFloorLow = DirectX::XMVectorReplicate(-0.25f);

    const DirectX::XMMATRIX vmZero = {vZero, vZero, vZero, vWone};

    const __m128i hash_primes = _mm_set_epi32(0, 83492791, 19349663, 73856093);

    constexpr std::uint32_t ANCHOR_MAX = 4;
    constexpr std::uint32_t COL_VERTEX_MAX = 16; // COL_VERTEX_MAX = 16 * qualityLevel
    constexpr std::uint32_t COL_EDGE_MAX = 6;
    constexpr std::uint32_t COL_FACE_MAX = 12;
    constexpr std::uint32_t AXIS_HISTORY_MAX = 92; // AXIS_HISTORY_MAX <= 1 + (COL_FACE_MAX * 2) + (COL_EDGE_MAX * COL_EDGE_MAX);
    constexpr std::uint32_t NOCOLLIDE_MAX = 16;
    constexpr float COMPLIANCE_SCALE = 0.0001f;
    constexpr float ROTATION_CLAMP_DEFAULT = 0.2f;
    constexpr float COL_MARGIN_MIN = 0.5f;

    const std::string_view cloneNodePrefix = "[MXPBD]";
    [[nodiscard]] inline std::string GetArmorCloneNodePrefix(std::uint32_t bipedSlot) {
        return cloneNodePrefix.data() + std::to_string(bipedSlot) + "&";
    }
    [[nodiscard]] inline std::string GetFacegenCloneNodePrefix() {
        return cloneNodePrefix.data() + std::string("Facegen&");
    }
    [[nodiscard]] inline std::string_view GetOriginalNodeName(std::string_view name) {
        if (!name.starts_with(cloneNodePrefix))
            return name;
        std::size_t pos = name.find('&');
        if (pos != std::string_view::npos) {
            return name.substr(pos + 1);
        }
        return name;
    }
    [[nodiscard]] inline bool IsCloneNodeName(const std::string_view name) {
        return name.size() != GetOriginalNodeName(name).size();
    }

    using Vector = DirectX::XMVECTOR;
    using Quaternion = DirectX::XMVECTOR;

    using NearBones = std::unordered_map<std::string, std::unordered_set<std::string>>;

    [[nodiscard]] inline RE::NiPoint3 ToPoint3(const DirectX::XMVECTOR& v) {
        return {DirectX::XMVectorGetX(v), DirectX::XMVectorGetY(v), DirectX::XMVectorGetZ(v)};
    }

    [[nodiscard]] inline RE::NiPoint3 GetSkyrimGravity(const float gm) {
        return ToPoint3(DirectX::XMVectorScale(SkyrimGravity, gm));
    }

    [[nodiscard]] inline DirectX::XMVECTOR ToVector(const RE::NiPoint3& p3) {
        return DirectX::XMLoadFloat3(reinterpret_cast<const DirectX::XMFLOAT3*>(&p3));
    }
    [[nodiscard]] inline DirectX::XMVECTOR ToVector(const RE::NiQuaternion& q) {
        const DirectX::XMFLOAT4 fq = {q.x, q.y, q.z, q.w};
        return DirectX::XMLoadFloat4(&fq);
    }

    [[nodiscard]] inline DirectX::XMVECTOR ToQuaternion(const RE::NiMatrix3& m) {
        const float tr = m.entry[0][0] + m.entry[1][1] + m.entry[2][2];
        if (tr > 0.0f) {
            const float S = std::sqrt(tr + 1.0f) * 2.0f;
            return DirectX::XMVectorSet(
                (m.entry[2][1] - m.entry[1][2]) / S,
                (m.entry[0][2] - m.entry[2][0]) / S,
                (m.entry[1][0] - m.entry[0][1]) / S,
                0.25f * S);
        } else if ((m.entry[0][0] > m.entry[1][1]) && (m.entry[0][0] > m.entry[2][2])) {
            const float S = std::sqrt(1.0f + m.entry[0][0] - m.entry[1][1] - m.entry[2][2]) * 2.0f;
            return DirectX::XMVectorSet(
                0.25f * S,
                (m.entry[0][1] + m.entry[1][0]) / S,
                (m.entry[0][2] + m.entry[2][0]) / S,
                (m.entry[2][1] - m.entry[1][2]) / S);
        } else if (m.entry[1][1] > m.entry[2][2]) {
            const float S = std::sqrt(1.0f + m.entry[1][1] - m.entry[0][0] - m.entry[2][2]) * 2.0f;
            return DirectX::XMVectorSet(
                (m.entry[0][1] + m.entry[1][0]) / S,
                0.25f * S,
                (m.entry[1][2] + m.entry[2][1]) / S,
                (m.entry[0][2] - m.entry[2][0]) / S);
        } else {
            const float S = std::sqrt(1.0f + m.entry[2][2] - m.entry[0][0] - m.entry[1][1]) * 2.0f;
            return DirectX::XMVectorSet(
                (m.entry[0][2] + m.entry[2][0]) / S,
                (m.entry[1][2] + m.entry[2][1]) / S,
                0.25f * S,
                (m.entry[1][0] - m.entry[0][1]) / S);
        }
    }

    [[nodiscard]] inline RE::NiMatrix3 ToMatrix(const DirectX::XMVECTOR& vQ) {
        RE::NiMatrix3 m;
        const DirectX::XMMATRIX xmMat = DirectX::XMMatrixTranspose(DirectX::XMMatrixRotationQuaternion(vQ));
        DirectX::XMFLOAT3X3 f33;
        XMStoreFloat3x3(&f33, xmMat);

        m.entry[0][0] = f33._11;
        m.entry[0][1] = f33._12;
        m.entry[0][2] = f33._13;

        m.entry[1][0] = f33._21;
        m.entry[1][1] = f33._22;
        m.entry[1][2] = f33._23;

        m.entry[2][0] = f33._31;
        m.entry[2][1] = f33._32;
        m.entry[2][2] = f33._33;
        return m;
    }

    [[nodiscard]] inline DirectX::XMMATRIX NiTransformToXMMATRIX(const RE::NiTransform& t) {
        return DirectX::XMMATRIX(
            t.rotate.entry[0][0] * t.scale, t.rotate.entry[1][0] * t.scale, t.rotate.entry[2][0] * t.scale, 0.0f,
            t.rotate.entry[0][1] * t.scale, t.rotate.entry[1][1] * t.scale, t.rotate.entry[2][1] * t.scale, 0.0f,
            t.rotate.entry[0][2] * t.scale, t.rotate.entry[1][2] * t.scale, t.rotate.entry[2][2] * t.scale, 0.0f,
            t.translate.x, t.translate.y, t.translate.z, 1.0f);
    }

    [[nodiscard]] inline DirectX::XMMATRIX NiPoin3x3ToXMMATRIX(const RE::NiPoint3* p) {
        return DirectX::XMMATRIX(
            p[0].x, p[0].y, p[0].z, 0.0f,
            p[1].x, p[1].y, p[1].z, 0.0f,
            p[2].x, p[2].y, p[2].z, 0.0f,
            0.0f,   0.0f,   0.0f,   1.0f);
    }

    [[nodiscard]] inline RE::TESObjectREFR* GetREFR(const RE::FormID objectID) {
        return RE::TESForm::LookupByID<RE::TESObjectREFR>(objectID);
    }

    [[nodiscard]] inline RE::Actor* GetActor(const RE::FormID objectID) {
        return RE::TESForm::LookupByID<RE::Actor>(objectID);
    }
    [[nodiscard]] inline RE::Actor* GetActor(RE::TESObjectREFR* object) {
        return object ? object->As<RE::Actor>() : nullptr;
    }

    [[nodiscard]] inline float rsqrt(float x) {
        __m128 v = _mm_set_ss(x);
        __m128 y = _mm_rsqrt_ss(v);
        __m128 half = _mm_set_ss(0.5f);
        __m128 one_point_five = _mm_set_ss(1.5f);
        __m128 y2 = _mm_mul_ss(y, y);
        __m128 xy2 = _mm_mul_ss(v, y2);
        __m128 half_xy2 = _mm_mul_ss(half, xy2);
        __m128 term = _mm_sub_ss(one_point_five, half_xy2);
        y = _mm_mul_ss(y, term);
        return _mm_cvtss_f32(y);
    }

    [[nodiscard]] inline float reciprocal(float x) {
        __m128 vx = _mm_set_ss(x);
        __m128 vx0 = _mm_rcp_ss(vx);
        __m128 vtwo = _mm_set_ss(2.0f);
        __m128 vnx0 = _mm_mul_ss(vx, vx0);
        __m128 vSub = _mm_sub_ss(vtwo, vnx0);
        __m128 vx1 = _mm_mul_ss(vx0, vSub);
        return _mm_cvtss_f32(vx1);
    }

    inline float sin(float x) {
        constexpr float _4DIVPI = 4.0f / DirectX::XM_PI;
        constexpr float _4DIVPOWPI = 4.0f / (DirectX::XM_PI * DirectX::XM_PI);
        const float z = x * DirectX::XM_1DIV2PI;
        x = (z - std::round(z)) * DirectX::XM_2PI;
        float y = _4DIVPI * x - _4DIVPOWPI * x * std::abs(x);
        y = 0.225f * (y * std::abs(y) - y) + y;
        return y;
    }

    inline std::uint32_t rand_Hash(std::uint32_t seed) {
        constexpr float n = 1.0f / 16777215.0f;
        seed = (seed ^ 61) ^ (seed >> 16);
        seed *= 9;
        seed = seed ^ (seed >> 4);
        seed *= 0x27d4eb2d;
        seed = seed ^ (seed >> 15);
        return seed;
    }
    inline float rand_Hash_Float(std::uint32_t seed) {
        constexpr float n = 1.0f / 16777215.0f;
        return (rand_Hash(seed) & 0xFFFFFF) * n;
    }

    [[nodiscard]] inline std::uint32_t rand_PCG32(std::uint64_t& state) {
        const std::uint64_t oldstate = state;
        state = oldstate * 6364136223846793005ULL + 1ULL;
        const std::uint32_t xorshifted = static_cast<std::uint32_t>(((oldstate >> 18u) ^ oldstate) >> 27u);
        const std::uint32_t rot = static_cast<std::uint32_t>(oldstate >> 59u);
        const std::uint32_t result = (xorshifted >> rot) | (xorshifted << ((-rot) & 31));
        return result;
    }
    inline float rand_PCG32_Float(std::uint64_t& state) {
        constexpr float n = 1.0f / 16777215.0f;
        return (rand_PCG32(state) >> 8) * n;
    }

    inline void AtomicMax(float& target, float value) {
        if (std::isnan(value))
            return;
        std::atomic_ref<float> targetRef(target);
        float prev = targetRef.load(std::memory_order_relaxed);
        while (prev < value && !targetRef.compare_exchange_weak(prev, value, std::memory_order_relaxed))
            ;
    };

    [[nodiscard]] inline RE::NiNode* GetNPCNode(RE::NiAVObject* rootObj) {
        if (!rootObj)
            return nullptr;
        auto npcObj = rootObj->GetObjectByName("NPC");
        return npcObj ? npcObj->AsNode() : nullptr;
    }
    [[nodiscard]] inline RE::NiNode* GetNPCNode(RE::TESObjectREFR* refr) {
        if (!refr || !refr->loadedData || !refr->loadedData->data3D)
            return nullptr;
        auto npcObj = refr->loadedData->data3D->GetObjectByName("NPC");
        return npcObj ? npcObj->AsNode() : nullptr;
    }
}
