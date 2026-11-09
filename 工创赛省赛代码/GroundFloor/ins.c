/* ins.c —— 底盘运动学换算：正解（四轮 rpm → 车体速度 → 世界系 → 积分 x/y）与逆解（out → 四轮 rpm），两者必须严格互逆。
 * 单位 cm / ° / rpm；平移乘 RPM_PER_CMPS、旋转乘 ROT_RPM_PER_DPS；逆时针为正，坐标系见 chassis_cfg.h。 */
#include "ins.h"
#include "chassis_cfg.h"
#include <math.h>
#include "mf_util.h"

void INS_Init(InsState_t *s)
{
    s->x = s->y = 0.0f;
    s->yaw = 0.0f;
    s->body_vx = s->body_vy = 0.0f;
    s->world_vx = s->world_vy = 0.0f;
}

void INS_ZeroPos(InsState_t *s)
{
    s->x = s->y = 0.0f;
    s->body_vx = s->body_vy = 0.0f;
    s->world_vx = s->world_vy = 0.0f;
}

/* 四轮 rpm → 车体系速度 cm/s；是 INS_OutToWheels 的严格反（两者必须互逆）。
 * ⚠ vy 符号实测定死：vy > 0 = 车体朝左，改它必须与 INS_OutToWheels 的 vy 列同时改（两处一对）； 正逆解一起错就是个自洽的谎，位置环自己发现不了（IMU 只给航向，给不了位移）。 */
static void wheel_to_body(const InsInput_t *in, float *vx, float *vy)
{
    float w[4];
    int i;
    for (i = 0; i < 4; i++) {
        w[i] = in->wheel_rpm[i] / RPM_PER_CMPS;
    }
    *vx = (+w[0] + w[1] + w[2] + w[3]) / 4.0f;
    *vy = (-w[0] - w[1] + w[2] + w[3]) / 4.0f;    /* 与逆解 vy 列成对 */
}

void INS_Update(InsState_t *s, const InsInput_t *in)
{
    const float dt = CTRL_DT_S;      /* 控制周期唯一出处 */
    float body_vx, body_vy, yaw_r;

    /* 航向：全链路唯一来源 = IMU 绝对角，直接抄，不融合/不外推（轮速打滑会把航向带偏）。 无新帧时保持最后值（不判失效、不清零）。 */
    s->yaw = in->imu_yaw;

    wheel_to_body(in, &body_vx, &body_vy);
    s->body_vx = body_vx;            /* 遥测：横移漂移看的就是它（正 = 车体朝前） */
    s->body_vy = body_vy;

    yaw_r = s->yaw * MF_DEG2RAD;
    s->world_vx =  body_vx * cosf(yaw_r) - body_vy * sinf(yaw_r);
    s->world_vy =  body_vx * sinf(yaw_r) + body_vy * cosf(yaw_r);

    s->x += s->world_vx * dt;
    s->y += s->world_vy * dt;
}

/* 逆解：out（世界系速度 + 角速度）+ 当前航向 → 四轮 rpm，是 INS_Update 的反向 */
void INS_OutToWheels(float yaw, PidOut_t out, float *wheel_rpm)
{
    int i;
    float yaw_r, bvx, bvy, wrpm, mx = 0.0f, scale;

    yaw_r = yaw * MF_DEG2RAD;
    bvx = ( out.vx * cosf(yaw_r) + out.vy * sinf(yaw_r)) * RPM_PER_CMPS;
    bvy = (-out.vx * sinf(yaw_r) + out.vy * cosf(yaw_r)) * RPM_PER_CMPS;
    wrpm = out.w * ROT_RPM_PER_DPS;

    /* 麦轮混合（课本形式；正 = 该轮朝前进方向转，与 WHEEL_DIR 校正后的实测同一套符号）： wheel[0..3] = 右后(轴1) / 左前(轴6) / 右前(轴3) / 左后(轴4)；对角两轮平移项同、旋转项反（前进同向、自转对角反向）。
     * ⚠ vy 列课本原样、不取反，改它必须与 wheel_to_body 的 vy 同时改；轴号/接线反了归 WHEEL_DIR 管，别塞进本矩阵。 */
    wheel_rpm[0] = +bvx - bvy + wrpm;   /* 轴1 右后 */
    wheel_rpm[1] = +bvx - bvy - wrpm;   /* 轴6 左前 */
    wheel_rpm[2] = +bvx + bvy + wrpm;   /* 轴3 右前 */
    wheel_rpm[3] = +bvx + bvy - wrpm;   /* 轴4 左后 */

    /* 限幅：四轮**同比例**缩（方向与相对关系不变；逐轮各自截会破坏麦轮关系） */
    for (i = 0; i < 4; i++) {
        float a = (wheel_rpm[i] < 0.0f) ? -wheel_rpm[i] : wheel_rpm[i];
        if (a > mx) mx = a;
    }
    if (mx > GO_WHEEL_RPM_MAX) {
        scale = GO_WHEEL_RPM_MAX / mx;
        for (i = 0; i < 4; i++) wheel_rpm[i] *= scale;
    }
}
