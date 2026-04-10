inline void RotateByQuatConj_Uniform(
    uniform float vx, uniform float vy, uniform float vz,
    uniform float qx, uniform float qy, uniform float qz, uniform float qw,
    uniform float * uniform outX, uniform float * uniform outY, uniform float * uniform outZ)
{
    uniform float nqx = -qx; uniform float nqy = -qy; uniform float nqz = -qz;
    uniform float cx = nqy * vz - nqz * vy;
    uniform float cy = nqz * vx - nqx * vz;
    uniform float cz = nqx * vy - nqy * vx;

    uniform float tx = 2.0f * cx; uniform float ty = 2.0f * cy; uniform float tz = 2.0f * cz;

    uniform float ccx = nqy * tz - nqz * ty;
    uniform float ccy = nqz * tx - nqx * tz;
    uniform float ccz = nqx * ty - nqy * tx;

    *outX = vx + qw * tx + ccx; 
    *outY = vy + qw * ty + ccy; 
    *outZ = vz + qw * tz + ccz;
}

inline void RotateByQuat_Uniform(
    uniform float vx, uniform float vy, uniform float vz,
    uniform float qx, uniform float qy, uniform float qz, uniform float qw,
    uniform float * uniform outX, uniform float * uniform outY, uniform float * uniform outZ)
{
    uniform float cx = qy * vz - qz * vy;
    uniform float cy = qz * vx - qx * vz;
    uniform float cz = qx * vy - qy * vx;

    uniform float tx = 2.0f * cx; uniform float ty = 2.0f * cy; uniform float tz = 2.0f * cz;

    uniform float ccx = qy * tz - qz * ty;
    uniform float ccy = qz * tx - qx * tz;
    uniform float ccz = qx * ty - qy * tx;

    *outX = vx + qw * tx + ccx; 
    *outY = vy + qw * ty + ccy; 
    *outZ = vz + qw * tz + ccz;
}

inline uniform bool TestAxis_SIMD(
    uniform float ax, uniform float ay, uniform float az,
    uniform const float * uniform vX_A, uniform const float * uniform vY_A, uniform const float * uniform vZ_A, uniform int vCountA,
    uniform float posAx, uniform float posAy, uniform float posAz,
    uniform float rotAx, uniform float rotAy, uniform float rotAz, uniform float rotAw, uniform float scaleA,
    uniform const float * uniform vX_B, uniform const float * uniform vY_B, uniform const float * uniform vZ_B, uniform int vCountB,
    uniform float posBx, uniform float posBy, uniform float posBz,
    uniform float rotBx, uniform float rotBy, uniform float rotBz, uniform float rotBw, uniform float scaleB,
    uniform float * uniform minOverlap, uniform float * uniform mtvX, uniform float * uniform mtvY, uniform float * uniform mtvZ)
{
    uniform float lenSq = ax*ax + ay*ay + az*az;
    if (lenSq < 1e-6f) return true;

    uniform float invLen = rsqrt(lenSq);
    ax *= invLen; ay *= invLen; az *= invLen;

    uniform float centerProjA = posAx * ax + posAy * ay + posAz * az;
    uniform float centerProjB = posBx * ax + posBy * ay + posBz * az;

    uniform float localAx_A, localAy_A, localAz_A;
    RotateByQuatConj_Uniform(ax, ay, az, rotAx, rotAy, rotAz, rotAw, &localAx_A, &localAy_A, &localAz_A);
    
    float d_min_A = 1e30f; 
    float d_max_A = -1e30f;
    // 주의: 혹시라도 foreach_active로 덮어씌워진다면 반드시 foreach로 적어주세요!
    foreach (v = 0 ... vCountA) {
        float d = vX_A[v] * localAx_A + vY_A[v] * localAy_A + vZ_A[v] * localAz_A;
        d_min_A = min(d_min_A, d); 
        d_max_A = max(d_max_A, d);
    }
    uniform float minA = reduce_min(d_min_A) * scaleA + centerProjA;
    uniform float maxA = reduce_max(d_max_A) * scaleA + centerProjA;

    uniform float localAx_B, localAy_B, localAz_B;
    RotateByQuatConj_Uniform(ax, ay, az, rotBx, rotBy, rotBz, rotBw, &localAx_B, &localAy_B, &localAz_B);
    
    float d_min_B = 1e30f; 
    float d_max_B = -1e30f;
    foreach (v = 0 ... vCountB) {
        float d = vX_B[v] * localAx_B + vY_B[v] * localAy_B + vZ_B[v] * localAz_B;
        d_min_B = min(d_min_B, d); 
        d_max_B = max(d_max_B, d);
    }
    uniform float minB = reduce_min(d_min_B) * scaleB + centerProjB;
    uniform float maxB = reduce_max(d_max_B) * scaleB + centerProjB;

    if (maxA < minB || maxB < minA) return false;

    uniform float overlap = min(maxA - minB, maxB - minA);
    if (overlap < *minOverlap) {
        *minOverlap = overlap;
        if (centerProjA < centerProjB) {
            *mtvX = -ax; *mtvY = -ay; *mtvZ = -az;
        } else {
            *mtvX = ax; *mtvY = ay; *mtvZ = az;
        }
    }
    return true;
}

export uniform bool MXPBD_CheckCollision(
    uniform const float * uniform vX_A, uniform const float * uniform vY_A, uniform const float * uniform vZ_A, uniform int vCountA,
    uniform const float * uniform fX_A, uniform const float * uniform fY_A, uniform const float * uniform fZ_A, uniform int fCountA,
    uniform const unsigned int8 * uniform edgeA_A, uniform const unsigned int8 * uniform edgeB_A, uniform int eCountA,
    uniform float posAx, uniform float posAy, uniform float posAz,
    uniform float rotAx, uniform float rotAy, uniform float rotAz, uniform float rotAw, uniform float scaleA,

    uniform const float * uniform vX_B, uniform const float * uniform vY_B, uniform const float * uniform vZ_B, uniform int vCountB,
    uniform const float * uniform fX_B, uniform const float * uniform fY_B, uniform const float * uniform fZ_B, uniform int fCountB,
    uniform const unsigned int8 * uniform edgeA_B, uniform const unsigned int8 * uniform edgeB_B, uniform int eCountB,
    uniform float posBx, uniform float posBy, uniform float posBz,
    uniform float rotBx, uniform float rotBy, uniform float rotBz, uniform float rotBw, uniform float scaleB,

    uniform float * uniform outNormal, uniform float * uniform outDepth)
{
    uniform float minOverlap = 1e30f;
    uniform float mtvX = 0.0f, mtvY = 0.0f, mtvZ = 0.0f;

    for (uniform int i = 0; i < fCountA; ++i) {
        uniform float worldNx, worldNy, worldNz;
        RotateByQuat_Uniform(fX_A[i], fY_A[i], fZ_A[i], rotAx, rotAy, rotAz, rotAw, &worldNx, &worldNy, &worldNz);

        if (!TestAxis_SIMD(worldNx, worldNy, worldNz, 
            vX_A, vY_A, vZ_A, vCountA, posAx, posAy, posAz, rotAx, rotAy, rotAz, rotAw, scaleA,
            vX_B, vY_B, vZ_B, vCountB, posBx, posBy, posBz, rotBx, rotBy, rotBz, rotBw, scaleB,
            &minOverlap, &mtvX, &mtvY, &mtvZ)) return false;
    }

    for (uniform int i = 0; i < fCountB; ++i) {
        uniform float worldNx, worldNy, worldNz;
        RotateByQuat_Uniform(fX_B[i], fY_B[i], fZ_B[i], rotBx, rotBy, rotBz, rotBw, &worldNx, &worldNy, &worldNz);

        if (!TestAxis_SIMD(worldNx, worldNy, worldNz, 
            vX_A, vY_A, vZ_A, vCountA, posAx, posAy, posAz, rotAx, rotAy, rotAz, rotAw, scaleA,
            vX_B, vY_B, vZ_B, vCountB, posBx, posBy, posBz, rotBx, rotBy, rotBz, rotBw, scaleB,
            &minOverlap, &mtvX, &mtvY, &mtvZ)) return false;
    }

    uniform float worldEdgeBx[100]; 
    uniform float worldEdgeBy[100]; 
    uniform float worldEdgeBz[100];
    for (uniform int j = 0; j < eCountB; ++j) {
        uniform int b1 = edgeA_B[j]; uniform int b2 = edgeB_B[j];
        RotateByQuat_Uniform(
            vX_B[b2] - vX_B[b1], vY_B[b2] - vY_B[b1], vZ_B[b2] - vZ_B[b1], 
            rotBx, rotBy, rotBz, rotBw, &worldEdgeBx[j], &worldEdgeBy[j], &worldEdgeBz[j]);
    }

    for (uniform int i = 0; i < eCountA; ++i) {
        uniform int a1 = edgeA_A[i]; uniform int a2 = edgeB_A[i];
        uniform float worldEdgeAx, worldEdgeAy, worldEdgeAz;
        RotateByQuat_Uniform(
            vX_A[a2] - vX_A[a1], vY_A[a2] - vY_A[a1], vZ_A[a2] - vZ_A[a1], 
            rotAx, rotAy, rotAz, rotAw, &worldEdgeAx, &worldEdgeAy, &worldEdgeAz);

        for (uniform int j = 0; j < eCountB; ++j) {
            uniform float cx = worldEdgeAy * worldEdgeBz[j] - worldEdgeAz * worldEdgeBy[j];
            uniform float cy = worldEdgeAz * worldEdgeBx[j] - worldEdgeAx * worldEdgeBz[j];
            uniform float cz = worldEdgeAx * worldEdgeBy[j] - worldEdgeAy * worldEdgeBx[j];

            if (!TestAxis_SIMD(cx, cy, cz, 
                vX_A, vY_A, vZ_A, vCountA, posAx, posAy, posAz, rotAx, rotAy, rotAz, rotAw, scaleA,
                vX_B, vY_B, vZ_B, vCountB, posBx, posBy, posBz, rotBx, rotBy, rotBz, rotBw, scaleB,
                &minOverlap, &mtvX, &mtvY, &mtvZ)) return false;
        }
    }

    outNormal[0] = mtvX; outNormal[1] = mtvY; outNormal[2] = mtvZ;
    outDepth[0] = minOverlap;
    return true;
}

