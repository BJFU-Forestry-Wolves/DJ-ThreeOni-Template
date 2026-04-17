#include "cmsis_os.h"
#include "sys_const.h"
#include "module_chassis.h"
#include "periph_motor.h"
#include <math.h>

#define CHASSIS_OMNI_WHEEL_NUM 3
#define CHASSIS_PI 3.1415926f

Chassis_ChassisTypeDef Chassis_ControlData[CHASSIS_OMNI_WHEEL_NUM];
Chassis_StatusTypeDef Chassis_StatusData;

void Chassis_InitChassis(void) {
    Chassis_ChassisTypeDef *chassis = Chassis_ChassisPtr();
    Chassis_StatusTypeDef *s = Chassis_StatusPtr();
    s->chassis_mode = Chassis_NULL;
    s->Chassis_LeftFront_SpeedRef = 0.0f;
    s->Chassis_RightFront_SpeedRef = 0.0f;
    s->Chassis_Rear_SpeedRef = 0.0f;

    for (int i = 0; i < CHASSIS_OMNI_WHEEL_NUM; i++) {
        chassis[i].control_state = 1;
        chassis[i].output_state = 1;
        chassis[i].chassis_ref = 0.0f;
        chassis[i].chassis_count = 0;
        PID_ClearPID(&chassis[i].spdPID);
    }

    Motor_SetMotorOutput(&Motor_ChassisLeftFrontMotor, 0.0f);
    Motor_SetMotorOutput(&Motor_ChassisRightFrontMotor, 0.0f);
    Motor_SetMotorOutput(&Motor_ChassisRearMotor, 0.0f);

    PID_InitPIDParam(&chassis[0].spdPIDParam, Const_ChassisFontLeftSpdParam[0][0], Const_ChassisFontLeftSpdParam[0][1], Const_ChassisFontLeftSpdParam[0][2], Const_ChassisFontLeftSpdParam[0][3],
                    Const_ChassisFontLeftSpdParam[0][4], Const_ChassisFontLeftSpdParam[1][0], Const_ChassisFontLeftSpdParam[1][1], Const_ChassisFontLeftSpdParam[2][0], Const_ChassisFontLeftSpdParam[2][1],
                    Const_ChassisFontLeftSpdParam[3][0], Const_ChassisFontLeftSpdParam[3][1], PID_POSITION);
    PID_InitPIDParam(&chassis[1].spdPIDParam, Const_ChassisFontRightSpdParam[0][0], Const_ChassisFontRightSpdParam[0][1], Const_ChassisFontRightSpdParam[0][2], Const_ChassisFontRightSpdParam[0][3],
                    Const_ChassisFontRightSpdParam[0][4], Const_ChassisFontRightSpdParam[1][0], Const_ChassisFontRightSpdParam[1][1], Const_ChassisFontRightSpdParam[2][0], Const_ChassisFontRightSpdParam[2][1],
                    Const_ChassisFontRightSpdParam[3][0], Const_ChassisFontRightSpdParam[3][1], PID_POSITION);
    PID_InitPIDParam(&chassis[2].spdPIDParam, Const_ChassisBackRightSpdParam[0][0], Const_ChassisBackRightSpdParam[0][1], Const_ChassisBackRightSpdParam[0][2], Const_ChassisBackRightSpdParam[0][3],
                    Const_ChassisBackRightSpdParam[0][4], Const_ChassisBackRightSpdParam[1][0], Const_ChassisBackRightSpdParam[1][1], Const_ChassisBackRightSpdParam[2][0], Const_ChassisBackRightSpdParam[2][1],
                    Const_ChassisBackRightSpdParam[3][0], Const_ChassisBackRightSpdParam[3][1], PID_POSITION);
}

void Chassis_SetChassisMode(Chassis_ModeEnum mode)
{

}

// 定义加减速步长（根据你的控制周期调整，假设是1ms一次，步长设小一点）
// 也可以根据物理量计算：步长 = 加速度(m/s^2) * 控制周期(s)
float CHASSIS_ACCEL_STEP = 1.8f;  // 加速步长
float CHASSIS_DECEL_STEP = 1.9f;  // 减速步长（通常减速比加速快一点，手感更好）
// 辅助函数：限幅线性趋近
float Ramp_Calc(float target, float current, float accel, float decel) {
    float diff = target - current;
    
    if (diff > 0) {
        // 目标值大于当前值 -> 向上加速或减速回归
        // 如果 target > 0 且 current >= 0，则是加速；如果 target > 0 且 current < 0，则是减速回归
        float step = (current >= 0) ? accel : decel;
        if (diff > step) return current + step;
        else return target;
    } else if (diff < 0) {
        // 目标值小于当前值 -> 向下加速或减速回归
        float step = (current <= 0) ? accel : decel;
        if (diff < -step) return current - step;
        else return target;
    }
    return target;
}

void Chassis_SetChassisRef(float RC_Vx, float RC_Vy, float RC_Wz) {
    const float R = CHASSIS_OMNI_RADIUS;
    const float half = 0.5f;
    const float sqrt3_half = 0.8660254f;
    
    Chassis_StatusTypeDef *s = Chassis_StatusPtr();

    // 1. 获取遥控器原始目标值
    float raw_vx = RC_Vx * REMOTE_CHASSIS_VX_GAIN;
    float raw_vy = RC_Vy * REMOTE_CHASSIS_VY_GAIN;
    float raw_wz = RC_Wz * REMOTE_CHASSIS_XTL_WZ_GAIN;

    // 2. 核心逻辑：斜坡函数处理 (Ramp Control)
    // 每一帧让当前的 Ramp 速度向 raw 速度靠近一点点
    s->Chassis_Vx_Ramp = Ramp_Calc(raw_vx, s->Chassis_Vx_Ramp, CHASSIS_ACCEL_STEP, CHASSIS_DECEL_STEP);
    s->Chassis_Vy_Ramp = Ramp_Calc(raw_vy, s->Chassis_Vy_Ramp, CHASSIS_ACCEL_STEP, CHASSIS_DECEL_STEP);
    s->Chassis_Wz_Ramp = Ramp_Calc(raw_wz, s->Chassis_Wz_Ramp, CHASSIS_ACCEL_STEP, CHASSIS_DECEL_STEP);

    // 更新状态供调试观察
    s->Chassis_Vx = s->Chassis_Vx_Ramp;
    s->Chassis_Vy = s->Chassis_Vy_Ramp;
    s->Chassis_Wz = s->Chassis_Wz_Ramp;

    /* 3. 使用处理后的平滑速度进行逆运动学计算 */
    float vx = s->Chassis_Vx_Ramp;
    float vy = s->Chassis_Vy_Ramp;
    float wz = s->Chassis_Wz_Ramp;

    s->Chassis_LeftFront_SpeedRef  =  sqrt3_half * vx + half * vy + wz * R;
    s->Chassis_RightFront_SpeedRef = -sqrt3_half * vx + half * vy + wz * R;
    s->Chassis_Rear_SpeedRef       = -vy + wz * R;
}
void Chassis_Control()
{
    Chassis_ChassisTypeDef *chassis = Chassis_ChassisPtr();
    Chassis_StatusTypeDef *s = Chassis_StatusPtr();
    Motor_MotorTypeDef *wheel[CHASSIS_OMNI_WHEEL_NUM] = {
        &Motor_ChassisLeftFrontMotor,
        &Motor_ChassisRightFrontMotor,
        &Motor_ChassisRearMotor
    };
    const float ref[CHASSIS_OMNI_WHEEL_NUM] = {
        s->Chassis_LeftFront_SpeedRef,
        s->Chassis_RightFront_SpeedRef,
        s->Chassis_Rear_SpeedRef
    };
    const float sign[CHASSIS_OMNI_WHEEL_NUM] = {
        CHASSIS_OMNI_LF_SIGN,
        CHASSIS_OMNI_RF_SIGN,
        CHASSIS_OMNI_RR_SIGN
    };

    for (int i = 0; i < CHASSIS_OMNI_WHEEL_NUM; i++) {
        if (chassis[i].control_state != 1) return;
    }

    for (int i = 0; i < CHASSIS_OMNI_WHEEL_NUM; i++) {
        PID_SetPIDRef(&chassis[i].spdPID, ref[i]);
        PID_SetPIDFdb(&chassis[i].spdPID, wheel[i]->encoder.speed * sign[i]);
        PID_CalcPID(&chassis[i].spdPID, &chassis[i].spdPIDParam);
        Motor_SetMotorOutput(wheel[i], PID_GetPIDOutput(&chassis[i].spdPID) * sign[i]);
    }
}

void Chassis_Output(void) {
    Chassis_ChassisTypeDef *chassis = Chassis_ChassisPtr();
    for (int i = 0; i < CHASSIS_OMNI_WHEEL_NUM; i++) {
        if (chassis[i].output_state != 1) return;
    }
    Motor_SendMotorGroupOutput(&Motor_ChassisMotors);
}

Chassis_StatusTypeDef* Chassis_StatusPtr(void) {
    return &Chassis_StatusData;
}
Chassis_ChassisTypeDef* Chassis_ChassisPtr(void) {
    return Chassis_ControlData;
}