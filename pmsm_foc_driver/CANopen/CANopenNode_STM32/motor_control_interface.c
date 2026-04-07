/*
 * motor_control_interface.c
 *
 * CIA402 <-> ST Motor Control SDK 之间的接口层
 *
 * 实现 CIA402 所需的 motor_xxx 函数，对接 ST Motor Control SDK
 */

#include "motor_control_interface.h"
#include "mc_api.h"
#include "mc_config.h"

/* 速度转换因子: ST SDK 使用 0.1 RPM 单位 */
#define RPM_TO_01RPM(rpm) ((int16_t)((rpm) * 10))
#define RPM01_TO_RPM(rpm01) (((int16_t)(rpm01)) / 10.0f)

/* 位置单位: ST SDK 使用 PPR (脉冲) */
#define PPR_TO_INT32(ppr) ((int32_t)(ppr))
#define INT32_TO_PPR(val) ((int32_t)(val))

/* 速度阈值判断 (RPM)，低于此值认为已停止 */
#define STOPPED_SPEED_THRESHOLD_RPM  5

/* 模式切换状态机 */
typedef enum {
    SWITCH_IDLE,           /* 空闲，无切换进行中 */
    SWITCH_TO_VELOCITY,    /* 正在切换到速度模式 */
    SWITCH_TO_POSITION     /* 正在切换到位置模式 */
} Motor_SwitchState_t;

static Motor_SwitchState_t switch_state = SWITCH_IDLE;
static int32_t switch_target_value = 0;

/*===========================================================================
 * CIA402 需要的电机控制函数实现
 *===========================================================================*/

void motor_control_interface_init(void)
{
    /* 初始化后默认停止电机 */
    MC_StopMotor1();
}

/* 启动电机 */
void motor_start(void)
{
    MC_StartMotor1();
}

/* 设置目标速度 (RPM) */
void motor_set_target_velocity(int32_t vel)
{
    /* vel 单位是 RPM，MC_ProgramSpeedRampMotor1_F 使用 rpm */
    float_t speed_rpm = (float_t)vel;

    /* 速度模式时禁用位置控制 */
    pPosCtrl[M1]->PositionControlRegulation = false;

    /* 设置为目标速度，立即执行 (时间设为0) */
    MC_ProgramSpeedRampMotor1_F(speed_rpm, 0);
}

/* 设置目标位置 (PPR) */
void motor_set_target_position(int32_t pos)
{
    /* pos 单位是 PPR (编码器脉冲数) */
    float_t position_ppr = (float_t)pos;

    /* ST SDK 需要启用位置控制才能执行位置命令 */
    pPosCtrl[M1]->PositionControlRegulation = true;

    /* 设置位置命令，0ms表示立即执行 */
    MC_ProgramPositionCommandMotor1(position_ppr, 0);
}

/* 设置目标力矩 (0.1%) */
void motor_set_target_torque(int16_t torque)
{
    /* torque 单位是 0.1% (千分之一额定扭矩) */
    /* 需要转换为安培 - 这里简化处理 */
    float_t torque_amp = (float_t)torque / 1000.0f; /* 假设1000 = 1A */
    MC_ProgramTorqueRampMotor1_F(torque_amp, 0);
}

/* 正常停止 - 减速停止 */
void motor_stop(void)
{
    /* 速度设为0，100ms减速时间 */
    MC_ProgramSpeedRampMotor1_F(0, 100);
}

/* 紧急停止 - 立即切断输出 */
void motor_emergency_stop(void)
{
    /* 立即停止电机 */
    MC_StopMotor1();
}

/* 获取当前位置 (PPR) */
int32_t motor_get_position(void)
{
    /* 返回编码器位置 (PPR) */
    return (int32_t)MC_GetCurrentPosition1();
}

/* 获取当前速度 (RPM) */
int32_t motor_get_velocity(void)
{
    /* ST SDK 返回 int16_t，单位 0.1 RPM */
    int16_t speed_01rpm = MC_GetMecSpeedAverageMotor1();

    /* 转换为 RPM (int32) */
    return (int32_t)(speed_01rpm / 10);
}

/* 获取当前力矩 (0.1%) */
int16_t motor_get_torque(void)
{
    /* 获取扭矩参考值 (电气单位) */
    int16_t teref = MC_GetTerefMotor1();

    /* 转换为 0.1% 单位 (简化) */
    return (int16_t)(teref * 10);
}

/* 检查是否有故障 */
uint8_t motor_has_fault(void)
{
    uint16_t faults = MC_GetCurrentFaultsMotor1();
    return (faults != 0) ? 1 : 0;
}

/* 清除故障 */
void motor_clear_fault(void)
{
    MC_AcknowledgeFaultMotor1();
}

/*===========================================================================
 * 安全模式切换函数
 *===========================================================================*/

/* 检查电机是否已停止 */
uint8_t motor_is_stopped(void)
{
    int32_t speed = motor_get_velocity();
    return (speed < STOPPED_SPEED_THRESHOLD_RPM && speed > -STOPPED_SPEED_THRESHOLD_RPM) ? 1 : 0;
}

/* 安全切换到速度模式 - 先停止再切换 */
void motor_safe_switch_to_velocity(int32_t target_vel)
{
    if (switch_state != SWITCH_IDLE) {
        /* 切换已在进行中，忽略新请求 */
        return;
    }

    if (motor_is_stopped()) {
        /* 已经停止，直接切换 */
        pPosCtrl[M1]->PositionControlRegulation = false;
        STC_SetControlMode(pSTC[M1], MCM_SPEED_MODE);

        /* 设置新目标速度 */
        MC_ProgramSpeedRampMotor1_F((float_t)target_vel, 0);
    } else {
        /* 需要先停止 */
        switch_state = SWITCH_TO_VELOCITY;
        switch_target_value = target_vel;
        motor_stop();
    }
}

/* 安全切换到位置模式 - 先停止再切换 */
void motor_safe_switch_to_position(int32_t target_pos)
{
    if (switch_state != SWITCH_IDLE) {
        /* 切换已在进行中，忽略新请求 */
        return;
    }

    if (motor_is_stopped()) {
        /* 已经停止，直接切换 */
        pPosCtrl[M1]->PositionControlRegulation = true;
        STC_SetControlMode(pSTC[M1], MCM_TORQUE_MODE);

        /* 设置新目标位置 */
        MC_ProgramPositionCommandMotor1((float_t)target_pos, 0);
    } else {
        /* 需要先停止 */
        switch_state = SWITCH_TO_POSITION;
        switch_target_value = target_pos;
        motor_stop();
    }
}

/* 模式切换状态机处理 - 需要在1ms周期调用 */
void motor_switch_process(void)
{
    if (switch_state == SWITCH_IDLE) {
        return;
    }

    /* 检查是否已停止 */
    if (!motor_is_stopped()) {
        return;
    }

    /* 已停止，完成切换 */
    switch (switch_state) {
        case SWITCH_TO_VELOCITY:
            pPosCtrl[M1]->PositionControlRegulation = false;
            STC_SetControlMode(pSTC[M1], MCM_SPEED_MODE);
            MC_ProgramSpeedRampMotor1_F((float_t)switch_target_value, 0);
            break;

        case SWITCH_TO_POSITION:
            pPosCtrl[M1]->PositionControlRegulation = true;
            STC_SetControlMode(pSTC[M1], MCM_TORQUE_MODE);
            MC_ProgramPositionCommandMotor1((float_t)switch_target_value, 0);
            break;

        default:
            break;
    }

    switch_state = SWITCH_IDLE;
}
