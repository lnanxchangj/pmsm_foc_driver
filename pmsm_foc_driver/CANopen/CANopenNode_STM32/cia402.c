/* CIA402.c */
#include "CIA402.h"
#include "mc_api.h"
#include "mc_interface.h"
#include "pmsm_motor_parameters.h"
#include "OD.h"
#include "main.h"
#include "trajectory_ctrl.h" // 必须包含此头文件以操作 PositionCtrlStatus
#include <string.h>
#include <stdio.h>
#include <math.h>
#include <stdlib.h>

/* 外部引用 MCSDK 句柄 */
extern MCI_Handle_t *pMCI[];

/* ============================================================
 * 模块内部状态
 * ============================================================ */
static CIA402_State_t s_state = CIA402_NOT_READY_TO_SWITCH_ON;
static uint16_t s_cw_prev = 0u;
static bool s_pp_setpoint_pending = false;
static int32_t s_target_pos_inc = 0;
static int32_t s_last_target_vel = 0;
static int8_t s_last_mode = 0;
static uint32_t s_fault_reaction_tick = 0;

/* 连续多圈位置跟踪变量 */
static float_t s_continuous_pos_float = 0.0f;
static float_t s_last_mcsdk_pos_rad = 0.0f;
static bool s_pos_tracker_ready = false;

/* 窗口逻辑计时器 */
static uint32_t s_target_reached_timer = 0;

#define CIA402_PI 3.141592653589793f
#define CIA402_2PI 6.283185307179586f

/* 单圈模数轴定义 */
#define MODULO_RANGE 8000
#define MODULO_HALF 4000
#define CIA402_POS_SCALE ((float_t)MODULO_RANGE / CIA402_2PI)

/* ============================================================
 * 内部辅助函数
 * ============================================================ */
static uint16_t CIA402_BuildStatusword(void)
{
    uint16_t sw = 0u;
    switch (s_state)
    {
    case CIA402_NOT_READY_TO_SWITCH_ON:
        sw = SW_NOT_READY_TO_SWITCH_ON_VAL;
        break;
    case CIA402_SWITCH_ON_DISABLED:
        sw = SW_SWITCH_ON_DISABLED_VAL;
        break;
    case CIA402_READY_TO_SWITCH_ON:
        sw = SW_READY_TO_SWITCH_ON_VAL;
        break;
    case CIA402_SWITCHED_ON:
        sw = SW_SWITCHED_ON_VAL;
        break;
    case CIA402_OPERATION_ENABLED:
        sw = SW_OPERATION_ENABLED_VAL;
        break;
    case CIA402_QUICK_STOP_ACTIVE:
        sw = SW_QUICK_STOP_ACTIVE_VAL;
        break;
    case CIA402_FAULT_REACTION_ACTIVE:
        sw = SW_FAULT_REACTION_ACTIVE_VAL;
        break;
    case CIA402_FAULT:
        sw = SW_FAULT_VAL;
        break;
    default:
        sw = SW_FAULT_VAL;
        break;
    }
    sw |= SW_REMOTE;
    if (s_state != CIA402_NOT_READY_TO_SWITCH_ON)
        sw |= SW_VOLTAGE_ENABLED;
    if (MC_GetCurrentFaultsMotor1() != 0u)
        sw |= SW_WARNING;

    /* 握手应答 */
    if (s_pp_setpoint_pending)
        sw |= SW_SETPOINT_ACK;

    return sw;
}

static void CIA402_UpdateActualValues(void)
{
    OD_RAM.x606C_velocityActualValue = (int32_t)(MC_GetAverageMecSpeedMotor1_F() * CIA402_VEL_SCALE);

    if (!s_pos_tracker_ready)
        return;

    float_t mcsdk_pos_rad = MC_GetCurrentPosition1();
    float_t delta_rad = mcsdk_pos_rad - s_last_mcsdk_pos_rad;

    if (delta_rad > CIA402_PI)
        delta_rad -= CIA402_2PI;
    else if (delta_rad < -CIA402_PI)
        delta_rad += CIA402_2PI;

    s_continuous_pos_float += (delta_rad * CIA402_POS_SCALE);
    s_last_mcsdk_pos_rad = mcsdk_pos_rad;

    while (s_continuous_pos_float >= (float_t)MODULO_RANGE)
        s_continuous_pos_float -= (float_t)MODULO_RANGE;
    while (s_continuous_pos_float < 0.0f)
        s_continuous_pos_float += (float_t)MODULO_RANGE;

    OD_RAM.x6064_positionActualValue = (int32_t)(s_continuous_pos_float + 0.5f);
    if (OD_RAM.x6064_positionActualValue >= MODULO_RANGE)
        OD_RAM.x6064_positionActualValue = 0;

    float_t nominal = (float_t)NOMINAL_CURRENT_A;
    if (nominal < 0.1f)
        nominal = 1.0f;
    OD_RAM.x6077_torqueActualValue = (int16_t)((MC_GetIqdMotor1_F().q / nominal) * CIA402_TRQ_SCALE);
}

static void CIA402_StateMachineProcess(void)
{
    uint16_t cw = OD_RAM.x6040_controlword;
    MCI_State_t mc_state = MC_GetSTMStateMotor1();
    int8_t mode = OD_RAM.x6060_modesOfOperation;

    if (MC_GetCurrentFaultsMotor1() != 0u && s_state != CIA402_FAULT && s_state != CIA402_FAULT_REACTION_ACTIVE)
    {
        s_state = CIA402_FAULT_REACTION_ACTIVE;
        s_fault_reaction_tick = HAL_GetTick();
        MC_StopMotor1();
        return;
    }

    switch (s_state)
    {
    case CIA402_NOT_READY_TO_SWITCH_ON:
        if (mc_state != ICLWAIT && mc_state != OFFSET_CALIB && mc_state != CHARGE_BOOT_CAP)
            s_state = CIA402_SWITCH_ON_DISABLED;
        break;
    case CIA402_SWITCH_ON_DISABLED:
        if ((cw & 0x0007) == 0x0006)
            s_state = CIA402_READY_TO_SWITCH_ON;
        break;
    case CIA402_READY_TO_SWITCH_ON:
        if ((cw & 0x0007) == 0x0007)
            s_state = CIA402_SWITCHED_ON;
        break;
    case CIA402_SWITCHED_ON:
        if ((cw & 0x000F) == 0x000F)
        {
            if (mc_state == IDLE)
            {
                if (MC_StartMotor1())
                    printf("[CIA402] Motor starting...\r\n");
            }
            else if (mc_state == RUN)
            {
                /* 如果编码器 Z-index 对齐正在执行中（TC_MoveCommand 控制电机旋转
                 * 寻找零点），必须等待对齐完成再进入 OPERATION_ENABLED。
                 * 否则 MC_ProgramPositionCommandMotor1 会覆盖对齐 ramp 的
                 * PositionCtrlStatus，中断寻零过程，导致零点丢失。 */
                if (pPosCtrl[0] && pPosCtrl[0]->AlignmentStatus == TC_ZERO_ALIGNMENT_START)
                {
                    /* 对齐进行中 —— 维持 SWITCHED_ON，更新跟踪参考但不干预位置环 */
                    s_last_mcsdk_pos_rad = MC_GetCurrentPosition1();
                    s_continuous_pos_float = 0.0f;
                    break;
                }

                /* 双零点同步法：保证物理寻零后的坐标偏移正确，避免初次起步的跳变与额外一圈 */
                s_last_mcsdk_pos_rad = MC_GetCurrentPosition1();
                if (!s_pos_tracker_ready) {
                    s_continuous_pos_float = 0.0f;
                }
                s_pos_tracker_ready = true;

                /* 根据模式初始化底层 */
                if (mode == 1)
                {
                    float_t sync_pos_rad = MC_GetCurrentPosition1();
                    MC_SetCtrlPositionAngle1(sync_pos_rad);
                    if (pMCI[0]->pPosCtrl)
                    {
                        pMCI[0]->pPosCtrl->PositionCtrlStatus = TC_READY_FOR_COMMAND;
                    }
                    /* 使能位置环：放在最后，确保 Theta 已同步 */
                    if (pPosCtrl[0])
                        pPosCtrl[0]->PositionControlRegulation = ENABLE;
                }
                else if (mode == 3)
                {
                    if (pPosCtrl[0])
                        pPosCtrl[0]->PositionControlRegulation = DISABLE;
                    if (pSTC[0])
                        STC_SetControlMode(pSTC[0], MCM_SPEED_MODE);
                    MC_ProgramSpeedRampMotor1_F(0.0f, 0.0f);
                }

                s_last_target_vel = 0;
                s_last_mode = mode;
                
                /* 保持当前的实际位置，并将目标位置同步到当前位置，避免跳变 */
                CIA402_UpdateActualValues();
                s_target_pos_inc = OD_RAM.x6064_positionActualValue;
                OD_RAM.x607A_targetPosition = OD_RAM.x6064_positionActualValue;
                
                s_state = CIA402_OPERATION_ENABLED;
                printf("[CIA402] ENABLED: Mode %d Ready\r\n", mode);
            }
        }
        break;
    case CIA402_OPERATION_ENABLED:
        if ((cw & 0x000F) != 0x000F)
        {
            printf("[CIA402] Disabling from OPERATION_ENABLED. CW: 0x%04X, MC State: %d\r\n", cw, mc_state);
            MC_StopMotor1();
            s_state = CIA402_SWITCHED_ON;
        }
        else
        {
            /* 动态模式切换 */
            if (mode != s_last_mode)
            {
                if (mode == 1)
                {
                    float_t sync_pos_rad = MC_GetCurrentPosition1();
                    MC_SetCtrlPositionAngle1(sync_pos_rad);
                    
                    if (pMCI[0]->pPosCtrl)
                    {
                        pMCI[0]->pPosCtrl->PositionCtrlStatus = TC_READY_FOR_COMMAND;
                    }
                    
                    /* 保持 CIA402 连续位置跟踪不归零，仅更新基准 */
                    s_last_mcsdk_pos_rad = sync_pos_rad;
                    
                    /* 更新最新的实际位置，并吸附目标位置避免跳变 */
                    CIA402_UpdateActualValues();
                    s_target_pos_inc = OD_RAM.x6064_positionActualValue;
                    OD_RAM.x607A_targetPosition = OD_RAM.x6064_positionActualValue;
                    
                    /* 最后使能位置环，确保 Theta 已同步完毕 */
                    if (pPosCtrl[0])
                        pPosCtrl[0]->PositionControlRegulation = ENABLE;
                    printf("[CIA402] Mode Switched to PP (1)\r\n");
                }
                else if (mode == 3)
                {
                    if (pPosCtrl[0])
                        pPosCtrl[0]->PositionControlRegulation = DISABLE;
                    if (pSTC[0])
                        STC_SetControlMode(pSTC[0], MCM_SPEED_MODE);
                    MC_ProgramSpeedRampMotor1_F(0.0f, 0.0f);
                    printf("[CIA402] Mode Switched to PV (3) - Pos Loop Disabled\r\n");
                }
                s_last_mode = mode;
            }
            if (OD_RAM.x6060_modesOfOperation == 1)
            {
                bool new_setpoint = (cw & CW_NEW_SETPOINT) != 0u;
                bool last_setpoint = (s_cw_prev & CW_NEW_SETPOINT) != 0u;

                /* 握手逻辑：上升沿触发 */
                if (new_setpoint && !last_setpoint)
                {
                    int32_t current_inc = (int32_t)OD_RAM.x6064_positionActualValue;
                    int32_t target_inc;

                    if (cw & CW_ABS_REL)
                        target_inc = (current_inc + OD_RAM.x607A_targetPosition) % MODULO_RANGE;
                    else
                        target_inc = OD_RAM.x607A_targetPosition % MODULO_RANGE;

                    if (target_inc < 0)
                        target_inc += MODULO_RANGE;

                    int32_t diff_inc = target_inc - current_inc;
                    if (diff_inc > MODULO_HALF)
                        diff_inc -= MODULO_RANGE;
                    if (diff_inc < -MODULO_HALF)
                        diff_inc += MODULO_RANGE;

                    float_t diff_rad = (float_t)diff_inc / CIA402_POS_SCALE;

                    /* 核心修复：强制底层状态进入 READY，确保新指令不被丢弃 */
                    if (pMCI[0]->pPosCtrl)
                    {
                        pMCI[0]->pPosCtrl->PositionCtrlStatus = TC_READY_FOR_COMMAND;
                    }

                    float_t final_target_rad = MC_GetCtrlPositionAngle1() + diff_rad;
                    float_t vel_rpm = (float_t)OD_RAM.x6081_profileVelocity;
                    if (vel_rpm < 1.0f)
                        vel_rpm = 60.0f;
                    float_t vel_rad_s = (vel_rpm * CIA402_2PI) / 60.0f;
                    float_t dur = fabsf(diff_rad) / vel_rad_s;
                    if (dur < 0.01f)
                        dur = 0.01f;

                    s_target_pos_inc = target_inc;
                    s_target_reached_timer = 0;
                    s_pp_setpoint_pending = true; // 设置 Ack

                    MC_ProgramPositionCommandMotor1(final_target_rad, dur);
                    printf("[CIA402] PP %s: To %d, Dur=%.3fs\r\n",
                           (cw & CW_ABS_REL) ? "REL" : "ABS", (int)target_inc, dur);
                }

                /* 握手逻辑：Master 撤回 Bit 4 后，驱动器才撤回 Bit 12 */
                if (!new_setpoint)
                    s_pp_setpoint_pending = false;
            }
            else if (mode == 3) /* PV Mode */
            {
                int32_t target_vel = OD_RAM.x60FF_targetVelocity;
                if (target_vel != s_last_target_vel)
                {
                    float_t target_rpm = (float_t)target_vel / CIA402_VEL_SCALE;
                    
                    /* 适配 0x6083 为加速度单元 (默认 RPM/s) */
                    float_t acc_val = (float_t)OD_RAM.x6083_profileAcceleration * CIA402_ACC_SCALE;
                    if (acc_val < 1.0f)
                        acc_val = 1000.0f; // 默认 1000 RPM/s，防止除零

                    float_t current_rpm = MC_GetAverageMecSpeedMotor1_F();
                    float_t diff_rpm = fabsf(target_rpm - current_rpm);
                    
                    /* 动态计算斜坡时间 (ms): ramp_ms = (|dv| / a) * 1000 */
                    uint16_t ramp_ms = (uint16_t)((diff_rpm / acc_val) * 1000.0f);
                    if (ramp_ms < 10)
                        ramp_ms = 10; // 最小允许10ms斜坡时间

                    MC_ProgramSpeedRampMotor1_F(target_rpm, ramp_ms);
                    s_last_target_vel = target_vel;
                    printf("[CIA402] PV Target: %d RPM, Acc: %d, Ramp: %d ms\r\n", (int)target_vel, (int)OD_RAM.x6083_profileAcceleration, ramp_ms);
                }
            }
        }
        break;
    case CIA402_FAULT:
        if (cw & 0x0080)
        {
            MC_AcknowledgeFaultMotor1();
            s_state = CIA402_SWITCH_ON_DISABLED;
        }
        break;
    default:
        break;
    }
}

void CIA402_Init(void)
{
    memset(&OD_RAM.x6040_controlword, 0, 2);
    OD_RAM.x6041_statusword = SW_NOT_READY_TO_SWITCH_ON_VAL;
    OD_RAM.x6060_modesOfOperation = 1;
    OD_RAM.x6081_profileVelocity = 60;
    OD_RAM.x6083_profileAcceleration = 3000;
    s_state = CIA402_NOT_READY_TO_SWITCH_ON;
    s_pos_tracker_ready = false;
    s_pp_setpoint_pending = false;
    s_cw_prev = 0u;
}

void CIA402_Process(void)
{
    CIA402_StateMachineProcess();
    CIA402_UpdateActualValues();
    uint16_t sw = CIA402_BuildStatusword();
    int8_t mode = OD_RAM.x6060_modesOfOperation;
    bool reached = false;
    if (mode == 1)
    {
        int32_t diff = (int32_t)OD_RAM.x6064_positionActualValue - s_target_pos_inc;
        if (diff > MODULO_HALF)
            diff -= MODULO_RANGE;
        if (diff < -MODULO_HALF)
            diff += MODULO_RANGE;
        reached = (abs(diff) <= POS_WINDOW_DEFAULT) && (!s_pp_setpoint_pending);
    }
    if (reached)
    {
        if (s_target_reached_timer < WINDOW_TIME_MS)
            s_target_reached_timer++;
        else
            sw |= SW_TARGET_REACHED;
    }
    else
    {
        s_target_reached_timer = 0;
    }

    OD_RAM.x6041_statusword = sw;
    static uint32_t last_log = 0;
    if (HAL_GetTick() - last_log > 500)
    {
        last_log = HAL_GetTick();
        /* 增加对底层算法状态的日志输出 */
        int mcsdk_pos_status = (pMCI[0]->pPosCtrl) ? pMCI[0]->pPosCtrl->PositionCtrlStatus : -1;
        printf("[CIA402] State:%d SW:0x%04X Act:%d Tar:%d PS:%d\r\n",
               (int)s_state, sw, (int)OD_RAM.x6064_positionActualValue, (int)s_target_pos_inc, mcsdk_pos_status);
    }
    s_cw_prev = OD_RAM.x6040_controlword;
}
