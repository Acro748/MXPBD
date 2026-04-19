#pragma once

namespace MXPBD {
    const DirectX::XMVECTOR EmptyVector = DirectX::XMVectorZero();
    constexpr RE::NiPoint3 EmptyPoint = RE::NiPoint3(0, 0, 0);
    constexpr float Scale_havokWorld = 0.0142875f;
    const DirectX::XMVECTOR GlobalGravity = DirectX::XMVectorSet(0.0f, 0.0f, -9.8f, 0.0f);
    const DirectX::XMVECTOR SkyrimGravity = DirectX::XMVectorScale(GlobalGravity, 1.0f / Scale_havokWorld);
    const DirectX::XMVECTOR forwardDir = DirectX::XMVectorSet(0.0f, 1.0f, 0.0f, 0.0f);
    constexpr float DeltaTime60 = 1.0f / 60.0f;
    const std::size_t CoreCount = std::thread::hardware_concurrency();
    constexpr float FloatPrecision = 1e-4f;
    constexpr float ns2ms = 1.0f / 1000000.0f;
    constexpr float colRotBias = 0.2f;

    constexpr std::uint8_t ANCHOR_MAX = 4;
    constexpr std::uint8_t COL_VERTEX_MAX = 16; // COL_VERTEX_MAX = 16 * qualityLevel
    constexpr std::uint8_t COL_EDGE_MAX = 6;
    constexpr std::uint8_t COL_FACE_MAX = 12;
    constexpr std::uint8_t AXIS_HISTORY_MAX = 64; // AXIS_HISTORY_MAX <= 1 + (COL_FACE_MAX * 2) + (COL_EDGE_MAX * COL_EDGE_MAX);
    constexpr std::uint8_t NOCOLLIDE_MAX = 16;
    constexpr float COMPLIANCE_SCALE = 0.0001f;
    constexpr float COLLIDE_ROTATE_SCALE = 0.1f;
    constexpr std::uint32_t HASH_TABLE_SIZE = 1009;

    const std::string_view cloneNodePrefix = "[MXPBD]";
    inline std::string GetArmorCloneNodePrefix(std::uint32_t bipedSlot) {
        return cloneNodePrefix.data() + std::to_string(bipedSlot) + "&";
    }
    inline std::string GetFacegenCloneNodePrefix() {
        return cloneNodePrefix.data() + std::string("Facegen&");
    }
    inline std::string_view GetOriginalNodeName(std::string_view name) {
        if (!name.starts_with(cloneNodePrefix))
            return name;
        std::size_t pos = name.find('&');
        if (pos != std::string_view::npos) {
            return name.substr(pos + 1);
        }
        return name;
    }
    inline bool IsCloneNodeName(const std::string_view name) {
        return name.size() != GetOriginalNodeName(name).size();
    }

    using Vector = DirectX::XMVECTOR;
    using Quaternion = DirectX::XMVECTOR;

    using NearBones = std::unordered_map<std::string, std::unordered_set<std::string>>;

    inline RE::NiPoint3 ToPoint3(const DirectX::XMVECTOR& v) {
        return {DirectX::XMVectorGetX(v), DirectX::XMVectorGetY(v), DirectX::XMVectorGetZ(v)};
    }

    inline RE::NiPoint3 GetSkyrimGravity(const float gm) {
        return ToPoint3(DirectX::XMVectorScale(SkyrimGravity, gm));
    }

    inline DirectX::XMVECTOR ToVector(const RE::NiPoint3& p3) {
        return DirectX::XMLoadFloat3(reinterpret_cast<const DirectX::XMFLOAT3*>(&p3));
    }
    inline DirectX::XMVECTOR ToVector(const RE::NiQuaternion& q) {
        const DirectX::XMFLOAT4 fq = {q.x, q.y, q.z, q.w};
        return DirectX::XMLoadFloat4(&fq);
    }

    inline DirectX::XMVECTOR ToQuaternion(const RE::NiMatrix3& m) {
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

    inline RE::NiMatrix3 ToMatrix(const DirectX::XMVECTOR& vQ) {
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

    inline DirectX::XMMATRIX NiTransformToXMMATRIX(const RE::NiTransform& t) {
        return DirectX::XMMATRIX(
            t.rotate.entry[0][0] * t.scale, t.rotate.entry[1][0] * t.scale, t.rotate.entry[2][0] * t.scale, 0.0f,
            t.rotate.entry[0][1] * t.scale, t.rotate.entry[1][1] * t.scale, t.rotate.entry[2][1] * t.scale, 0.0f,
            t.rotate.entry[0][2] * t.scale, t.rotate.entry[1][2] * t.scale, t.rotate.entry[2][2] * t.scale, 0.0f,
            t.translate.x, t.translate.y, t.translate.z, 1.0f);
    }

    inline RE::TESObjectREFR* GetREFR(const RE::FormID objectID) {
        return RE::TESForm::LookupByID<RE::TESObjectREFR>(objectID);
    }

    inline RE::Actor* GetActor(const RE::FormID objectID) {
        return RE::TESForm::LookupByID<RE::Actor>(objectID);
    }
    inline RE::Actor* GetActor(RE::TESObjectREFR* object) {
        return object ? object->As<RE::Actor>() : nullptr;
    }

    inline std::vector<std::uint32_t> GetBitElements(std::uint32_t bitNum) {
        std::vector<std::uint32_t> elements;
        while (bitNum) {
            const std::uint32_t lsb = bitNum & -bitNum;
            const std::uint32_t num = std::countr_zero(lsb);
            elements.push_back(num);
            bitNum &= (bitNum - 1);
        }
        return elements;
    }

    inline float rsqrt(float x) {
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

    inline void AtomicMax(float& target, float value) {
        if (std::isnan(value))
            return;
        std::atomic_ref<float> targetRef(target);
        float prev = targetRef.load(std::memory_order_relaxed);
        while (prev < value && !targetRef.compare_exchange_weak(prev, value, std::memory_order_relaxed))
            ;
    };

    enum SIMDType : std::uint8_t {
        none,
        avx,
        avx2,
        avx512,
        total
    };
    inline bool HasAVX2Support() {
        int info[4]{};
        __cpuidex(info, 7, 0);
        return (info[1] & (1 << 5)) != 0;
    }
    inline bool HasAVX512Support() {
        int info[4]{};
        __cpuidex(info, 7, 0);
        bool avx512f = (info[1] & (1 << 16)) != 0;
        if (!avx512f)
            return false;
        unsigned long long xcrFeatureMask = _xgetbv(0);
        return (xcrFeatureMask & 0xE6) == 0xE6;
    }
    inline SIMDType GetSIMDType(bool scan, std::uint8_t maximum) {
        static SIMDType SIMDType_ = SIMDType::none;
        if (SIMDType_ != SIMDType::none && !scan)
            return SIMDType_;
        if (maximum <= 0)
            maximum = SIMDType::avx512;
        SIMDType_ = SIMDType::avx;
        if (maximum >= SIMDType::avx512 && HasAVX512Support())
            SIMDType_ = SIMDType::avx512;
        else if (maximum >= SIMDType::avx2 && HasAVX2Support())
            SIMDType_ = SIMDType::avx2;
        logger::info("Set SIMD type : {}", magic_enum::enum_name(SIMDType_).data());
        return SIMDType_;
    }
}