#include "MXPBD/PhysicsWorld.h"

namespace MXPBD
{
    bool XPBDWorld::ConvexHullvsConvexHull_avx2(const std::uint32_t coiA, const std::uint32_t coiB, ContactManifold& outManifold)
    {
        const std::uint32_t biA = colliders.boneIdx[coiA];
        const std::uint32_t biB = colliders.boneIdx[coiB];

        const Vector posA = physicsBones.pred[biA];
        const Quaternion rotA = physicsBones.predRot[biA];
        const float scaleA = physicsBones.worldScale[biA];

        const Vector posB = physicsBones.pred[biB];
        const Quaternion rotB = physicsBones.predRot[biB];
        const float scaleB = physicsBones.worldScale[biB];

        const float marginA = physicsBones.colMargin[biA];
        const float marginB = physicsBones.colMargin[biB];
        const float sumMargin = marginA + marginB;

        const float rA = colliders.boundingSphere[coiA] * scaleA + marginA;
        const float rB = colliders.boundingSphere[coiB] * scaleB + marginB;
        const float sumR = rA + rB;

        const Vector sphereCenterA = DirectX::XMVectorAdd(posA, DirectX::XMVector3Rotate(DirectX::XMVectorScale(colliders.boundingSphereOffset[coiA], scaleA), rotA));
        const Vector sphereCenterB = DirectX::XMVectorAdd(posB, DirectX::XMVector3Rotate(DirectX::XMVectorScale(colliders.boundingSphereOffset[coiB], scaleB), rotB));
        const Vector centerToCenter = DirectX::XMVectorSubtract(sphereCenterB, sphereCenterA);
        const float distSq = DirectX::XMVectorGetX(DirectX::XMVector3LengthSq(centerToCenter));
        if (distSq > sumR * sumR)
            return false;

        DirectX::XMFLOAT3 cA_f3, cB_f3;
        DirectX::XMStoreFloat3(&cA_f3, sphereCenterA);
        DirectX::XMStoreFloat3(&cB_f3, sphereCenterB);

        DirectX::XMFLOAT3 pA_f3, pB_f3;
        DirectX::XMStoreFloat3(&pA_f3, posA);
        DirectX::XMStoreFloat3(&cB_f3, posB);

        const Quaternion invRotA = DirectX::XMQuaternionConjugate(rotA);
        const Quaternion invRotB = DirectX::XMQuaternionConjugate(rotB);

        const auto& hullA = colliders.convexHullData[coiA];
        const auto& hullB = colliders.convexHullData[coiB];

        float minOverlap = FLT_MAX;
        Vector bestAxis = DirectX::XMVectorZero();
        bool flip = false;

        float histX[AXIS_HISTORY_MAX], histY[AXIS_HISTORY_MAX], histZ[AXIS_HISTORY_MAX];
        std::uint8_t histCount = 0;

        auto hmin256_ps = [](const __m256 v) -> float {
            __m256 shuf = _mm256_permute2f128_ps(v, v, 1);
            __m256 m = _mm256_min_ps(v, shuf);
            shuf = _mm256_shuffle_ps(m, m, _MM_SHUFFLE(2, 3, 0, 1));
            m = _mm256_min_ps(m, shuf);
            shuf = _mm256_shuffle_ps(m, m, _MM_SHUFFLE(1, 0, 3, 2));
            m = _mm256_min_ps(m, shuf);
            return _mm_cvtss_f32(_mm256_castps256_ps128(m));
        };
        auto hmax256_ps = [](const __m256 v) -> float {
            __m256 shuf = _mm256_permute2f128_ps(v, v, 1);
            __m256 m = _mm256_max_ps(v, shuf);
            shuf = _mm256_shuffle_ps(m, m, _MM_SHUFFLE(2, 3, 0, 1));
            m = _mm256_max_ps(m, shuf);
            shuf = _mm256_shuffle_ps(m, m, _MM_SHUFFLE(1, 0, 3, 2));
            m = _mm256_max_ps(m, shuf);
            return _mm_cvtss_f32(_mm256_castps256_ps128(m));
        };

        auto TestAxis = [&](float ax, float ay, float az) -> bool {
            const float lenSq = ax * ax + ay * ay + az * az;
            if (lenSq < FloatPrecision)
                return true;

            const float invLen = rsqrt(lenSq);
            ax *= invLen;
            ay *= invLen;
            az *= invLen;

            for (std::uint8_t i = 0; i < histCount; ++i)
            {
                const float dot = ax * histX[i] + ay * histY[i] + az * histZ[i];
                if (std::abs(dot) > 0.998f)
                    return true;
            }

            if (histCount < AXIS_HISTORY_MAX)
            {
                histX[histCount] = ax;
                histY[histCount] = ay;
                histZ[histCount] = az;
                histCount++;
            }

            const Vector axis = DirectX::XMVectorSet(ax, ay, az, 0);
            const float cAdotAx = cA_f3.x * ax + cA_f3.y * ay + cA_f3.z * az;
            const float cBdotAx = cB_f3.x * ax + cB_f3.y * ay + cB_f3.z * az;
            if (std::abs(cAdotAx - cBdotAx) > sumR)
                return false;

            const float pAdotAx = pA_f3.x * ax + pA_f3.y * ay + pA_f3.z * az;
            const float pBdotAx = pB_f3.x * ax + pB_f3.y * ay + pB_f3.z * az;

            const Vector localA = DirectX::XMVector3Rotate(axis, invRotA);
            DirectX::XMFLOAT3 localA_f3;
            DirectX::XMStoreFloat3(&localA_f3, localA);
            const __m256 v_axA = _mm256_set1_ps(localA_f3.x);
            const __m256 v_ayA = _mm256_set1_ps(localA_f3.y);
            const __m256 v_azA = _mm256_set1_ps(localA_f3.z);

            const Vector localB = DirectX::XMVector3Rotate(axis, invRotB);
            DirectX::XMFLOAT3 localB_f3;
            DirectX::XMStoreFloat3(&localB_f3, localB);
            const __m256 v_axB = _mm256_set1_ps(localB_f3.x);
            const __m256 v_ayB = _mm256_set1_ps(localB_f3.y);
            const __m256 v_azB = _mm256_set1_ps(localB_f3.z);

            __m256 dotA0 = _mm256_mul_ps(_mm256_load_ps(&hullA.vX[0]), v_axA);
            dotA0 = _mm256_fmadd_ps(_mm256_load_ps(&hullA.vY[0]), v_ayA, dotA0);
            dotA0 = _mm256_fmadd_ps(_mm256_load_ps(&hullA.vZ[0]), v_azA, dotA0);

            __m256 dotA1 = _mm256_mul_ps(_mm256_load_ps(&hullA.vX[8]), v_axA);
            dotA1 = _mm256_fmadd_ps(_mm256_load_ps(&hullA.vY[8]), v_ayA, dotA1);
            dotA1 = _mm256_fmadd_ps(_mm256_load_ps(&hullA.vZ[8]), v_azA, dotA1);

            __m256 minA = _mm256_min_ps(dotA0, dotA1);
            __m256 maxA = _mm256_max_ps(dotA0, dotA1);

            __m256 dotB0 = _mm256_mul_ps(_mm256_load_ps(&hullB.vX[0]), v_axB);
            dotB0 = _mm256_fmadd_ps(_mm256_load_ps(&hullB.vY[0]), v_ayB, dotB0);
            dotB0 = _mm256_fmadd_ps(_mm256_load_ps(&hullB.vZ[0]), v_azB, dotB0);

            __m256 dotB1 = _mm256_mul_ps(_mm256_load_ps(&hullB.vX[8]), v_axB);
            dotB1 = _mm256_fmadd_ps(_mm256_load_ps(&hullB.vY[8]), v_ayB, dotB1);
            dotB1 = _mm256_fmadd_ps(_mm256_load_ps(&hullB.vZ[8]), v_azB, dotB1);

            __m256 minB = _mm256_min_ps(dotB0, dotB1);
            __m256 maxB = _mm256_max_ps(dotB0, dotB1);

            for (std::uint32_t i = 16; i < COL_VERTEX_MAX; i += 8)
            {
                __m256 dotA = _mm256_mul_ps(_mm256_load_ps(&hullA.vX[i]), v_axA);
                dotA = _mm256_fmadd_ps(_mm256_load_ps(&hullA.vY[i]), v_ayA, dotA);
                dotA = _mm256_fmadd_ps(_mm256_load_ps(&hullA.vZ[i]), v_azA, dotA);

                minA = _mm256_min_ps(dotA, minA);
                maxA = _mm256_max_ps(dotA, maxA);

                __m256 dotB = _mm256_mul_ps(_mm256_load_ps(&hullB.vX[i]), v_axB);
                dotB = _mm256_fmadd_ps(_mm256_load_ps(&hullB.vY[i]), v_ayB, dotB);
                dotB = _mm256_fmadd_ps(_mm256_load_ps(&hullB.vZ[i]), v_azB, dotB);

                minB = _mm256_min_ps(dotB, minB);
                maxB = _mm256_max_ps(dotB, maxB);
            }

            const float fminA = hmin256_ps(minA) * scaleA + pAdotAx - marginA;
            const float fmaxA = hmax256_ps(maxA) * scaleA + pAdotAx + marginA;
            const float fminB = hmin256_ps(minB) * scaleB + pBdotAx - marginB;
            const float fmaxB = hmax256_ps(maxB) * scaleB + pBdotAx + marginB;
            if (fmaxA < fminB || fmaxB < fminA)
                return false;

            const float overlap1 = fmaxA - fminB;
            const float overlap2 = fmaxB - fminA;
            if (overlap1 < 0.0f || overlap2 < 0.0f)
                return false;

            const float overlap = std::min(overlap1, overlap2);
            if (overlap < minOverlap)
            {
                minOverlap = overlap;
                bestAxis = axis;
                flip = (overlap1 < overlap2);
            }
            return true;
        };

        auto& cache = colliders.convexHullCache[GetCacheKey(coiA, coiB)];
        if (cache.lastFrame == currentFrame - 1)
        {
            if (!TestAxis(cache.ax, cache.ay, cache.az))
                return false;
        }
        
        const Vector dirAtoB = DirectX::XMVectorSubtract(posB, posA);
        for (std::uint8_t i = 0; i < COL_FACE_MAX; ++i)
        {
            const Vector nA = DirectX::XMVector3Rotate(DirectX::XMVectorSet(hullA.fX[i], hullA.fY[i], hullA.fZ[i], 0), rotA);
            if (DirectX::XMVectorGetX(DirectX::XMVector3Dot(nA, dirAtoB)) <= 0.0f)
                continue;
            DirectX::XMFLOAT3 nA_f3;
            DirectX::XMStoreFloat3(&nA_f3, nA);
            if (!TestAxis(nA_f3.x, nA_f3.y, nA_f3.z))
                return false;

            const Vector nB = DirectX::XMVector3Rotate(DirectX::XMVectorSet(hullB.fX[i], hullB.fY[i], hullB.fZ[i], 0), rotB);
            DirectX::XMFLOAT3 nB_f3;
            DirectX::XMStoreFloat3(&nB_f3, nB);
            if (!TestAxis(nB_f3.x, nB_f3.y, nB_f3.z))
                return false;
        }

        Vector wEdgeB[COL_EDGE_MAX];
        bool wEdgeBValid[COL_EDGE_MAX] = {false};
        for (std::uint8_t i = 0; i < COL_EDGE_MAX; ++i)
        {
            const Vector eB = DirectX::XMVectorSet(hullB.eX[i], hullB.eY[i], hullB.eZ[i], 0);
            wEdgeB[i] = DirectX::XMVector3Rotate(eB, rotB);
            if (DirectX::XMVectorGetX(DirectX::XMVector3LengthSq(wEdgeB[i])) >= FloatPrecision)
                wEdgeBValid[i] = true;
        }

        for (std::uint8_t eiA = 0; eiA < COL_EDGE_MAX; ++eiA)
        {
            const Vector eA = DirectX::XMVectorSet(hullA.eX[eiA], hullA.eY[eiA], hullA.eZ[eiA], 0);
            const Vector wA = DirectX::XMVector3Rotate(eA, rotA);

            if (DirectX::XMVectorGetX(DirectX::XMVector3LengthSq(wA)) < FloatPrecision)
                continue;

            for (std::uint8_t eiB = 0; eiB < COL_EDGE_MAX; ++eiB)
            {
                if (!wEdgeBValid[eiB])
                    continue;
                const float edgeDot = DirectX::XMVectorGetX(DirectX::XMVector3Dot(wA, wEdgeB[eiB]));
                if (std::abs(edgeDot) > 0.99f)
                    continue;
                const Vector crossAxis = DirectX::XMVector3Cross(wA, wEdgeB[eiB]);
                DirectX::XMFLOAT3 cross_f3;
                DirectX::XMStoreFloat3(&cross_f3, crossAxis);
                if (!TestAxis(cross_f3.x, cross_f3.y, cross_f3.z))
                    return false;
            }
        }

        const Vector normal = flip ? DirectX::XMVectorNegate(bestAxis) : bestAxis;

        DirectX::XMFLOAT3 best_f3;
        DirectX::XMStoreFloat3(&best_f3, bestAxis);
        cache.ax = best_f3.x;
        cache.ay = best_f3.y;
        cache.az = best_f3.z;
        cache.lastFrame = currentFrame;

        // manifold
        ContactManifold::ContactPoint tempPoints[16];
        std::uint8_t tempCount = 0;
        const float breakThresholdSq = 0.04f;

        for (std::uint8_t i = 0; i < cache.persistentManifold.pointCount; ++i)
        {
            const auto& cp = cache.persistentManifold.points[i];
            const Vector wA = DirectX::XMVectorAdd(posA, DirectX::XMVector3Rotate(DirectX::XMVectorScale(cp.localPointA, scaleA), rotA));
            const Vector wB = DirectX::XMVectorAdd(posB, DirectX::XMVector3Rotate(DirectX::XMVectorScale(cp.localPointB, scaleB), rotB));

            Vector diff = DirectX::XMVectorSubtract(wA, wB);
            float currentDepth = -DirectX::XMVectorGetX(DirectX::XMVector3Dot(diff, normal));
            if (currentDepth < -sumMargin)
                continue;

            Vector projDiff = DirectX::XMVectorAdd(diff, DirectX::XMVectorScale(normal, currentDepth));
            float driftSq = DirectX::XMVectorGetX(DirectX::XMVector3LengthSq(projDiff));
            if (driftSq > breakThresholdSq)
                continue;

            tempPoints[tempCount] = cp;
            tempPoints[tempCount].depth = currentDepth;
            tempCount++;
        }

        const std::uint8_t vCountA = colliders.vertexCount[coiA];
        const std::uint8_t vCountB = colliders.vertexCount[coiB];

        alignas(32) float dotA[COL_VERTEX_MAX];
        alignas(32) float dotB[COL_VERTEX_MAX];

        const Vector localNormalA = DirectX::XMVector3Rotate(normal, invRotA);
        DirectX::XMFLOAT3 lnA_f3;
        DirectX::XMStoreFloat3(&lnA_f3, localNormalA);
        const float posDotA = DirectX::XMVectorGetX(DirectX::XMVector3Dot(posA, normal));

        const __m256 v_lnAx = _mm256_set1_ps(lnA_f3.x);
        const __m256 v_lnAy = _mm256_set1_ps(lnA_f3.y);
        const __m256 v_lnAz = _mm256_set1_ps(lnA_f3.z);
        const __m256 v_scaleA = _mm256_set1_ps(scaleA);
        const __m256 v_posDotA = _mm256_set1_ps(posDotA);
        __m256 v_minDotA = _mm256_set1_ps(FLT_MAX);

        for (std::uint8_t i = 0; i < COL_VERTEX_MAX; i += 8)
        {
            const __m256 vx = _mm256_load_ps(&hullA.vX[i]);
            const __m256 vy = _mm256_load_ps(&hullA.vY[i]);
            const __m256 vz = _mm256_load_ps(&hullA.vZ[i]);

            __m256 dot = _mm256_mul_ps(vx, v_lnAx);
            dot = _mm256_fmadd_ps(vy, v_lnAy, dot);
            dot = _mm256_fmadd_ps(vz, v_lnAz, dot);
            dot = _mm256_fmadd_ps(dot, v_scaleA, v_posDotA);
            _mm256_store_ps(&dotA[i], dot);
            v_minDotA = _mm256_min_ps(v_minDotA, dot);
        }
        const float minDotA = hmin256_ps(v_minDotA);

        const Vector localNormalB = DirectX::XMVector3Rotate(normal, invRotB);
        DirectX::XMFLOAT3 lnB_f3;
        DirectX::XMStoreFloat3(&lnB_f3, localNormalB);
        const float posDotB = DirectX::XMVectorGetX(DirectX::XMVector3Dot(posB, normal));

        const __m256 v_lnBx = _mm256_set1_ps(lnB_f3.x);
        const __m256 v_lnBy = _mm256_set1_ps(lnB_f3.y);
        const __m256 v_lnBz = _mm256_set1_ps(lnB_f3.z);
        const __m256 v_scaleB = _mm256_set1_ps(scaleB);
        const __m256 v_posDotB = _mm256_set1_ps(posDotB);
        __m256 v_maxDotB = _mm256_set1_ps(-FLT_MAX);

        for (std::uint8_t i = 0; i < COL_VERTEX_MAX; i += 8)
        {
            const __m256 vx = _mm256_load_ps(&hullB.vX[i]);
            const __m256 vy = _mm256_load_ps(&hullB.vY[i]);
            const __m256 vz = _mm256_load_ps(&hullB.vZ[i]);

            __m256 dot = _mm256_mul_ps(vx, v_lnBx);
            dot = _mm256_fmadd_ps(vy, v_lnBy, dot);
            dot = _mm256_fmadd_ps(vz, v_lnBz, dot);
            dot = _mm256_fmadd_ps(dot, v_scaleB, v_posDotB);
            _mm256_store_ps(&dotB[i], dot);
            v_maxDotB = _mm256_max_ps(v_maxDotB, dot);
        }
        const float maxDotB = hmax256_ps(v_maxDotB);
        const float tolerance = 0.02f + sumMargin;
        auto AddTempPoint = [&](const Vector& lA, const Vector& lB, const float depth) {
            for (std::uint8_t i = 0; i < tempCount; ++i)
            {
                const float distSq = DirectX::XMVectorGetX(DirectX::XMVector3LengthSq(DirectX::XMVectorSubtract(tempPoints[i].localPointA, lA)));
                if (distSq < 0.001f)
                    return;
            }
            if (tempCount < 16)
            {
                tempPoints[tempCount++] = {lA, lB, depth};
            }
        };

        const __m256 v_tolA = _mm256_set1_ps(minDotA + tolerance);
        for (std::uint8_t i = 0; i < COL_VERTEX_MAX; i += 8)
        {
            const __m256 v_dotA = _mm256_load_ps(&dotA[i]);
            const __m256 cmp = _mm256_cmp_ps(v_dotA, v_tolA, _CMP_LE_OQ);
            const int mask = _mm256_movemask_ps(cmp);

            if (mask == 0)
                continue;
            for (std::uint8_t k = 0; k < 8; ++k)
            {
                if (!((mask >> k) & 1))
                    continue;
                const std::uint8_t idx = i + k;
                if (idx >= vCountA)
                    continue;

                const float pen = maxDotB - dotA[idx];
                if (pen <= 0.0f)
                    continue;
                const Vector lA = DirectX::XMVectorSet(hullA.vX[idx], hullA.vY[idx], hullA.vZ[idx], 0);
                const Vector worldA_i = DirectX::XMVectorAdd(posA, DirectX::XMVector3Rotate(DirectX::XMVectorScale(lA, scaleA), rotA));

                const Vector wB = DirectX::XMVectorAdd(worldA_i, DirectX::XMVectorScale(normal, pen));
                const Vector lB = DirectX::XMVectorScale(DirectX::XMVector3Rotate(DirectX::XMVectorSubtract(wB, posB), invRotB), 1.0f / scaleB);
                AddTempPoint(lA, lB, pen);
            }
        }

        const __m256 v_tolB = _mm256_set1_ps(maxDotB - tolerance);
        for (std::uint8_t i = 0; i < COL_VERTEX_MAX; i += 8)
        {
            const __m256 v_dotB = _mm256_load_ps(&dotB[i]);
            const __m256 cmp = _mm256_cmp_ps(v_dotB, v_tolB, _CMP_GE_OQ);
            const int mask = _mm256_movemask_ps(cmp);

            if (mask == 0)
                continue;
            for (std::uint8_t k = 0; k < 8; ++k)
            {
                if (!((mask >> k) & 1))
                    continue;
                const std::uint8_t idx = i + k;
                if (idx >= vCountB)
                    continue;

                const float pen = dotB[idx] - minDotA;
                if (pen <= 0.0f)
                    continue;
                const Vector lB = DirectX::XMVectorSet(hullB.vX[idx], hullB.vY[idx], hullB.vZ[idx], 0);
                const Vector worldB_i = DirectX::XMVectorAdd(posB, DirectX::XMVector3Rotate(DirectX::XMVectorScale(lB, scaleB), rotB));

                const Vector wA = DirectX::XMVectorSubtract(worldB_i, DirectX::XMVectorScale(normal, pen));
                const Vector lA = DirectX::XMVectorScale(DirectX::XMVector3Rotate(DirectX::XMVectorSubtract(wA, posA), invRotA), 1.0f / scaleA);
                AddTempPoint(lA, lB, pen);
            }
        }
        cache.persistentManifold.normal = normal;
        if (tempCount <= 4)
        {
            cache.persistentManifold.pointCount = tempCount;
            for (std::uint8_t i = 0; i < tempCount; ++i)
            {
                cache.persistentManifold.points[i] = tempPoints[i];
            }
        }
        else
        {
            std::int32_t p1 = 0, p2 = -1, p3 = -1, p4 = -1;
            float maxDistSq = -1.0f, maxTriAreaSq = -1.0f, maxQuadAreaSq = -1.0f;

            for (std::uint8_t i = 1; i < tempCount; ++i)
            {
                if (tempPoints[i].depth > tempPoints[p1].depth)
                    p1 = i;
            }

            for (std::uint8_t i = 0; i < tempCount; ++i)
            {
                if (i == p1)
                    continue;
                const float dSq = DirectX::XMVectorGetX(DirectX::XMVector3LengthSq(DirectX::XMVectorSubtract(tempPoints[i].localPointA, tempPoints[p1].localPointA)));
                if (dSq > maxDistSq)
                {
                    maxDistSq = dSq;
                    p2 = i;
                }
            }

            if (p2 != -1)
            {
                for (std::uint8_t i = 0; i < tempCount; ++i)
                {
                    if (i == p1 || i == p2)
                        continue;
                    const Vector edge1 = DirectX::XMVectorSubtract(tempPoints[i].localPointA, tempPoints[p1].localPointA);
                    const Vector edge2 = DirectX::XMVectorSubtract(tempPoints[p2].localPointA, tempPoints[p1].localPointA);
                    const float areaSq = DirectX::XMVectorGetX(DirectX::XMVector3LengthSq(DirectX::XMVector3Cross(edge1, edge2)));
                    if (areaSq > maxTriAreaSq)
                    {
                        maxTriAreaSq = areaSq;
                        p3 = i;
                    }
                }
            }

            if (p3 != -1)
            {
                for (std::uint32_t i = 0; i < tempCount; ++i)
                {
                    if (i == p1 || i == p2 || i == p3)
                        continue;
                    const Vector e1 = DirectX::XMVector3Cross(DirectX::XMVectorSubtract(tempPoints[i].localPointA, tempPoints[p1].localPointA), DirectX::XMVectorSubtract(tempPoints[p2].localPointA, tempPoints[p1].localPointA));
                    const Vector e2 = DirectX::XMVector3Cross(DirectX::XMVectorSubtract(tempPoints[i].localPointA, tempPoints[p2].localPointA), DirectX::XMVectorSubtract(tempPoints[p3].localPointA, tempPoints[p2].localPointA));
                    const Vector e3 = DirectX::XMVector3Cross(DirectX::XMVectorSubtract(tempPoints[i].localPointA, tempPoints[p3].localPointA), DirectX::XMVectorSubtract(tempPoints[p1].localPointA, tempPoints[p3].localPointA));
                    const float areaSqSum = DirectX::XMVectorGetX(DirectX::XMVector3LengthSq(e1)) + DirectX::XMVectorGetX(DirectX::XMVector3LengthSq(e2)) + DirectX::XMVectorGetX(DirectX::XMVector3LengthSq(e3));
                    if (areaSqSum > maxQuadAreaSq)
                    {
                        maxQuadAreaSq = areaSqSum;
                        p4 = i;
                    }
                }
            }

            std::uint8_t fCount = 0;
            cache.persistentManifold.points[fCount++] = tempPoints[p1];
            if (p2 != -1)
                cache.persistentManifold.points[fCount++] = tempPoints[p2];
            if (p3 != -1)
                cache.persistentManifold.points[fCount++] = tempPoints[p3];
            if (p4 != -1)
                cache.persistentManifold.points[fCount++] = tempPoints[p4];
            cache.persistentManifold.pointCount = fCount;
        }

        outManifold = cache.persistentManifold;
        return true;
    }
}