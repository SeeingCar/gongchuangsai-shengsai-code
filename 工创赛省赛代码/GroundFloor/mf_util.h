/* mf_util.h —— 运动层公用小工具：角度换算与角度误差（static inline，不产生 .c）。 */
#ifndef __MF_UTIL_H
#define __MF_UTIL_H

#define MF_DEG2RAD   0.017453293f     /* 度 → 弧度 */

static inline float mf_wrap180(float a)     /* 归一到 (-180,180] */
{
    while (a >  180.0f) a -= 360.0f;
    while (a < -180.0f) a += 360.0f;
    return a;
}

/* ⚠ 角度误差必须走这里：直接相减在 ±180 边界会得 ±360 级错值（−179° 与 +179° 应差 +2°） */
static inline float mf_ang_err(float des, float now)
{
    return mf_wrap180(des - now);
}

#endif /* __MF_UTIL_H */
