#ifndef MODULE_CHASSIS
#define MODULE_CHASSIS

#ifdef __cplusplus
extern "C" {
#endif

#include "periph_motor.h"
#include "alg_math.h"
#include "alg_pid.h"

typedef enum {
		Chassis_NULL = 0u,
		Chassis_SEP = 1u,
		Chassis_FOLLOW = 2u,
		Chassis_XTL = 3u,
		/* 摇杆 vx/vy 视为云台坐标系（前/右），经云台相对底盘角变换后再做舵轮解算，使平移时轮向与云台朝向一致 */
		Chassis_GIMBAL = 4u
}Chassis_ModeEnum;	
	
typedef struct {
    float chassis_ref;   //速度期望值                               
    float chassis_position_fdb;//位置反馈值                         
    float chassis_speed_fdb;  //速度反馈值                      
    uint8_t chassis_ref_limit_status;//底盘限位标志位                   
    uint8_t chassis_count;

    uint8_t control_state;//控制状态                          
    uint8_t output_state;                          
    uint8_t pending_state;                          

    PID_PIDTypeDef spdPID;//
    PID_PIDParamTypeDef spdPIDParam;//速度pid参数配置
    PID_PIDTypeDef angPID;
    PID_PIDParamTypeDef angPIDParam;//角度pid参数配置

} Chassis_ChassisTypeDef;

typedef struct {
	  Chassis_ModeEnum chassis_mode;
	
		float Chassis_Vx;
		float Chassis_Vy;
		float Chassis_Wz;
	
		float Chassis_Yaw_Angle;//原始偏航角，来自c板陀螺仪,范围是 0° ~ 360°													
		float Chassis_Yaw_Rad;
		
		/* 三轮全向：轮向目标速度 [左前, 右前, 正后] */
		float Chassis_LeftFront_SpeedRef;
		float Chassis_RightFront_SpeedRef;
		float Chassis_Rear_SpeedRef;

		float Chassis_Vx_Ramp; // 经过平滑处理的 Vx
   		 float Chassis_Vy_Ramp; // 经过平滑处理的 Vy
    	float Chassis_Wz_Ramp; // 经过平滑处理的 Wz
	
} Chassis_StatusTypeDef;

extern Chassis_ChassisTypeDef Chassis_ControlData[3];
extern Chassis_StatusTypeDef Chassis_StatusData;

/* 全向轮输出方向：对应 [左前, 右前, 正后]，反向就设为 -1 */
#ifndef CHASSIS_OMNI_LF_SIGN
#define CHASSIS_OMNI_LF_SIGN   (1)
#endif
#ifndef CHASSIS_OMNI_RF_SIGN
#define CHASSIS_OMNI_RF_SIGN   (1)
#endif
#ifndef CHASSIS_OMNI_RR_SIGN
#define CHASSIS_OMNI_RR_SIGN   (1)
#endif

void Chassis_InitChassis(void);

Chassis_ChassisTypeDef* Chassis_ChassisPtr(void);
void Chassis_SetChassisControlState(uint8_t state);
void Chassis_SetChassisOutputState(uint8_t state);

Chassis_StatusTypeDef* Chassis_StatusPtr(void);
void Chassis_SetChassisMode(Chassis_ModeEnum mode);
void Chassis_SetChassisYawAngle(float yaw_angle,float yaw_angle_offset);
void Chassis_SetChassisRef(float RC_Vx,float RC_Vy,float RC_Wz);

void Chasssis_SetChasssisFontRightRef(float chassisfontright_ref);
void Chassis_SetChassisFontLeftRef(float chassisfontleft_ref);
void Chassis_SetChassisBackLeftRef(float chassisbackleft_ref);
void Chassis_SetChassisBackRightRef(float chassisbackright_ref);

void Chassis_Control(void);
void Chassis_Output(void);


#endif
	
#ifdef __cplusplus
}
#endif

