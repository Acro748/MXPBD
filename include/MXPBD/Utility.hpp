#pragma once

namespace MXPBD {
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

    [[nodiscard]] inline float rsqrt(float x) {
        __m128 v = _mm_set_ss(x);
        __m128 y = _mm_rsqrt_ss(v);
        __m128 half = _mm_set_ss(0.5f);
        __m128 y2 = _mm_mul_ss(y, y);
        __m128 xy2 = _mm_mul_ss(v, y2);
        __m128 half_xy2 = _mm_mul_ss(half, xy2);
        __m128 term = _mm_sub_ss(vOnePointFive, half_xy2);
        y = _mm_mul_ss(y, term);
        return _mm_cvtss_f32(y);
    }

    [[nodiscard]] inline float reciprocal(float x) {
        __m128 vx = _mm_set_ss(x);
        __m128 vx0 = _mm_rcp_ss(vx);
        __m128 vnx0 = _mm_mul_ss(vx, vx0);
        __m128 vSub = _mm_sub_ss(vTwo, vnx0);
        __m128 vx1 = _mm_mul_ss(vx0, vSub);
        return _mm_cvtss_f32(vx1);
    }

    [[nodiscard]] inline RE::NiPoint3 ToNiPoint(const DirectX::XMVECTOR& v) {
        return {DirectX::XMVectorGetX(v), DirectX::XMVectorGetY(v), DirectX::XMVectorGetZ(v)};
    }

    [[nodiscard]] inline Vector GetSkyrimGravity(const float gm) {
        return DirectX::XMVectorScale(SkyrimGravity, gm);
    }

    [[nodiscard]] inline Vector ToVector(const RE::NiPoint3& p3) {
        return DirectX::XMLoadFloat3(reinterpret_cast<const DirectX::XMFLOAT3*>(&p3));
    }
    [[nodiscard]] inline Vector ToVector(const RE::NiQuaternion& q) {
        const DirectX::XMFLOAT4 fq = {q.x, q.y, q.z, q.w};
        return DirectX::XMLoadFloat4(&fq);
    }

    [[nodiscard]] inline Matrix ToMatrix(const RE::NiMatrix3& m) {
        return DirectX::XMMatrixTranspose(Matrix(
            m.entry[0][0], m.entry[0][1], m.entry[0][2], 0.0f,
            m.entry[1][0], m.entry[1][1], m.entry[1][2], 0.0f,
            m.entry[2][0], m.entry[2][1], m.entry[2][2], 0.0f,
            0.0f, 0.0f, 0.0f, 1.0f));
    }

    [[nodiscard]] inline Matrix ToMatrix(const Quaternion& q) {
        return DirectX::XMMatrixRotationQuaternion(q);
    }

    [[nodiscard]] inline RE::NiMatrix3 ToNiMatrix(Matrix xmm) {
        xmm.r[3] = vWone;
        RE::NiMatrix3 m;
        DirectX::XMFLOAT3X3 f33;
        XMStoreFloat3x3(&f33, DirectX::XMMatrixTranspose(xmm));

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

    [[nodiscard]] inline RE::NiMatrix3 ToNiMatrix(const Quaternion& q) {
        RE::NiMatrix3 m;
        const Matrix xmMat = DirectX::XMMatrixTranspose(DirectX::XMMatrixRotationQuaternion(q));
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

    [[nodiscard]] inline Quaternion ToQuaternion(Matrix m) {
        const Vector xAxis = DirectX::XMVector3Normalize(m.r[0]);
        const Vector yAxis = DirectX::XMVector3Normalize(m.r[1]);
        const Vector zAxis = DirectX::XMVector3Normalize(DirectX::XMVector3Cross(xAxis, yAxis));
        m.r[0] = xAxis;
        m.r[1] = yAxis;
        m.r[2] = zAxis;
        m.r[3] = DirectX::XMVectorSet(0.0f, 0.0f, 0.0f, 1.0f);
        return DirectX::XMQuaternionRotationMatrix(m);
    }

    [[nodiscard]] inline Quaternion ToQuaternion(const RE::NiMatrix3& m) {
        return ToQuaternion(ToMatrix(m));
    }

    [[nodiscard]] inline Matrix NiTransformToMatrix(const RE::NiTransform& t) {
        return Matrix(
            t.rotate.entry[0][0] * t.scale, t.rotate.entry[1][0] * t.scale, t.rotate.entry[2][0] * t.scale, 0.0f,
            t.rotate.entry[0][1] * t.scale, t.rotate.entry[1][1] * t.scale, t.rotate.entry[2][1] * t.scale, 0.0f,
            t.rotate.entry[0][2] * t.scale, t.rotate.entry[1][2] * t.scale, t.rotate.entry[2][2] * t.scale, 0.0f,
            t.translate.x, t.translate.y, t.translate.z, 1.0f);
    }

    [[nodiscard]] inline Matrix NiPoin3x3ToMatrix(const RE::NiPoint3* p, const float scale) {
        return Matrix(
            p[0].x * scale, p[0].y * scale, p[0].z * scale, 0.0f,
            p[1].x * scale, p[1].y * scale, p[1].z * scale, 0.0f,
            p[2].x * scale, p[2].y * scale, p[2].z * scale, 0.0f,
            0.0f, 0.0f, 0.0f, 1.0f);
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

    [[nodiscard]] inline std::uint32_t rand_Hash(std::uint32_t seed) {
        constexpr float n = 1.0f / 16777215.0f;
        seed = (seed ^ 61) ^ (seed >> 16);
        seed *= 9;
        seed = seed ^ (seed >> 4);
        seed *= 0x27d4eb2d;
        seed = seed ^ (seed >> 15);
        return seed;
    }
    [[nodiscard]] inline float rand_Hash_Float(std::uint32_t seed) {
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
    [[nodiscard]] inline float rand_PCG32_Float(std::uint64_t& state) {
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

    [[nodiscard]] inline Vector GetInvInertia(const Vector& mass, const Vector& inertia) {
        const Vector massInertia = DirectX::XMVectorMultiply(mass, inertia);
        const Vector mask = DirectX::XMVectorGreater(massInertia, vEpsilon);
        const Vector invMassInertia = DirectX::XMVectorReciprocal(massInertia);
        return DirectX::XMVectorSelect(vZero, invMassInertia, mask);
    };

    [[nodiscard]] inline RE::NiPoint3 GetABSPoint3(const RE::NiPoint3& p3) {
        return ToNiPoint(DirectX::XMVectorAbs(ToVector(p3)));
    };

    [[nodiscard]] inline bool IsAllZero(const RE::NiPoint3& p3) {
        return DirectX::XMVector3LessOrEqual(DirectX::XMVectorAbs(ToVector(p3)), vEpsilon);
    };

    inline void ClampZeroToInfinity(Vector& v) {
        if (DirectX::XMVectorGetX(DirectX::XMVector3LengthSq(v)) <= Epsilon)
            v = vHalfInf;
    }

    inline void ClampZeroToInfinityRot(Vector& v, bool negative) {
        if (DirectX::XMVectorGetX(DirectX::XMVector3LengthSq(v)) <= Epsilon)
            v = negative ? vNegPi : vPi;
    }

    inline bool IsInvalid(const Vector& v) {
        return DirectX::XMVector4IsNaN(v) || DirectX::XMVector4IsInfinite(v);
    }
    inline bool IsInvalid(const Matrix& m) {
        return DirectX::XMMatrixIsNaN(m) || DirectX::XMMatrixIsInfinite(m);
    }
}