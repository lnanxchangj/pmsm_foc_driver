/* CIA402.c */
#include "CIA402.h"
#include "mc_api.h"
#include "mc_interface.h"  /* for PosCtrlStatus_t, TC_TARGET_POSITION_REACHED */
#include "OD.h"
#include "main.h"          /* for HAL_GetTick() */
#include <string.h>
#include <stdio.h>

/* ============================================================
 * 模块内部状态
 * ============================================================ */
static CIA402_State_t  s_state       = CIA402_NOT_READY_TO_SWITCH_ON;
static uint16_t        s_cw_prev     = 0u;   /* 上一周期控制字，用于边沿检测 */
static bool            s_pp_setpoint_pending = false; /* PP模式新目标点待处理 */
static int32_t         s_last_target_vel = 0; /* 上一周期目标速度，用于PV/CSV边沿检测 */
static int32_t         s_last_target_pos = 0; /* 上一周期目标位置，用于检测x607A变化 */
static int32_t         s_target_pos_inc  = 0;   /* 绝对目标位置 (increments)，用于状态位判断 */
static float_t         s_target_pos_abs  = 0.0f; /* 绝对目标位置 (rad)，用于 MC 指令 */
static uint32_t        s_fault_reaction_tick = 0; /* FAULT_REACTION进入时刻 */

/* 支持的操作模式位掩码 */
#define SUPPORTED_MODES  (CIA402_MODE_PP | CIA402_MODE_PV | CIA402_MODE_PT | \
                          CIA402_MODE_CSP | CIA402_MODE_CSV | CIA402_MODE_CST)

/* ============================================================
 * 内部辅助：构建标准状态字基础值
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

    /* 始终置 Remote 位（由网络控制） */
    sw |= SW_REMOTE;

    /* Voltage Enabled: bit4, 主电源就绪（不在 NOT_READY 状态即表示就绪） */
    if (s_state != CIA402_NOT_READY_TO_SWITCH_ON)
    {
        sw |= SW_VOLTAGE_ENABLED;
    }

    /* 检查是否有告警 */
    if (MC_GetCurrentFaultsMotor1() != 0u)
    {
        sw |= SW_WARNING;
    }

    return sw;
}

/* ============================================================
 * 内部辅助：根据操作模式更新 Target Reached 及模式专用位
 * ============================================================ */
static void CIA402_UpdateModeSpecificStatus(uint16_t *sw)
{
    int8_t mode = OD_RAM.x6060_modesOfOperation;

    switch (mode)
    {
        case CIA402_MODE_PV:
        case CIA402_MODE_CSV:
        {
            /* 速度模式：实际速度与目标速度之差在阈值内则 Target Reached */
            float_t actual_rpm  = MC_GetAverageMecSpeedMotor1_F();
            float_t target_rpm  = (float_t)OD_RAM.x60FF_targetVelocity / CIA402_VEL_SCALE;
            float_t diff        = actual_rpm - target_rpm;
            if (diff < 0.0f) diff = -diff;
            if (diff < 10.0f) /* 10rpm 容差，可调 */
            {
                *sw |= SW_TARGET_REACHED;
            }
            break;
        }

        case CIA402_MODE_PP:
        case CIA402_MODE_CSP:
        {
            /* 位置模式：结合整数容差 (5 counts) 判断 Target Reached */
            int32_t act_pos_inc = OD_RAM.x6064_positionActualValue;
            int32_t diff_inc    = act_pos_inc - s_target_pos_inc;
            if (diff_inc < 0) diff_inc = -diff_inc;

            if (diff_inc <= 5 || 
                MC_GetControlPositionStatusMotor1() == TC_TARGET_POSITION_REACHED)
            {
                *sw |= SW_TARGET_REACHED;
            }

            /* PP 模式：Setpoint acknowledge */
            if (s_pp_setpoint_pending)
            {
                *sw |= SW_SETPOINT_ACK;
            }
            break;
        }

        case CIA402_MODE_PT:
        case CIA402_MODE_CST:
        {
            /* 转矩模式：转矩到达额定范围认为 Target Reached */
            *sw |= SW_TARGET_REACHED;
            break;
        }

        default:
            break;
    }
}

/* ============================================================
 * 内部辅助：更新实际值到 OD
 * ============================================================ */
static void CIA402_UpdateActualValues(void)
{
    /* 实际速度 rpm → int32 (×CIA402_VEL_SCALE) */
    float_t raw_speed = MC_GetAverageMecSpeedMotor1_F();
    OD_RAM.x606C_velocityActualValue =
        (int32_t)(raw_speed * CIA402_VEL_SCALE);

    /* 实际位置 rad → int32 (×CIA402_POS_SCALE) */
    float_t raw_pos = MC_GetCurrentPosition1();
    OD_RAM.x6064_positionActualValue =
        (int32_t)(raw_pos * CIA402_POS_SCALE);

    /* 调试打印：每5秒一次，显示MC SDK原始值 */
    static uint32_t s_debug_tick = 0;
    s_debug_tick++;
    if (s_debug_tick % 5000 == 0) {
        printf("[CIA402] DEBUG: raw_speed=%.2f rpm, raw_pos=%.4f rad\r\n",
               raw_speed, raw_pos);
    }

    /* 实际转矩: MCSDK 返回 A，需映射到额定转矩的 0.1%
     * 此处简化：直接用内部 digit 值，实际项目请按额定电流换算 */
    OD_RAM.x6077_torqueActualValue =
        MC_GetPhaseCurrentAmplitudeMotor1(); /* 替换为真实转矩换算 */

    /* 操作模式显示：仅支持的模式才同步，否则显示 0 (NO_MODE) */
    int8_t req_mode = OD_RAM.x6060_modesOfOperation;
    OD_RAM.x6061_modesOfOperationDisplay =
        (req_mode == CIA402_MODE_PV  || req_mode == CIA402_MODE_CSV ||
         req_mode == CIA402_MODE_PP  || req_mode == CIA402_MODE_CSP ||
         req_mode == CIA402_MODE_PT  || req_mode == CIA402_MODE_CST)
        ? req_mode : CIA402_MODE_NO_MODE;
}

/* ============================================================
 * 内部辅助：执行操作模式指令（在 Operation Enabled 状态下周期调用）
 * ============================================================ */
static void CIA402_ExecuteModeCommand(void)
{
    uint16_t cw   = OD_RAM.x6040_controlword;
    int8_t   mode = OD_RAM.x6060_modesOfOperation;
    bool     halt = (cw & CW_HALT) != 0u;

    /* 每秒打印一次模式入口信息（调试用） */
    static uint32_t s_mode_print_tick = 0;
    s_mode_print_tick++;
    if (s_mode_print_tick % 1000 == 0) {
        printf("[CIA402] ExecuteMode: mode=%d halt=%d CW=0x%04X vel_target=%d pos_target=%d\r\n",
               mode, halt, cw, OD_RAM.x60FF_targetVelocity, OD_RAM.x607A_targetPosition);
    }

    if (halt)
    {
        /* Halt：根据当前模式执行不同的停止策略 */
        switch (mode)
        {
            case CIA402_MODE_PV:
            case CIA402_MODE_CSV:
                MC_ProgramSpeedRampMotor1_F(0.0f, 500u);
                break;
            case CIA402_MODE_PP:
            case CIA402_MODE_CSP:
            {
                /* 位置模式：停在当前位置 */
                float_t cur_pos = MC_GetCurrentPosition1();
                MC_ProgramPositionCommandMotor1(cur_pos, 0.0f);
                break;
            }
            case CIA402_MODE_PT:
            case CIA402_MODE_CST:
                MC_ProgramTorqueRampMotor1_F(0.0f, 100u);
                break;
            default:
                MC_ProgramSpeedRampMotor1_F(0.0f, 500u);
                break;
        }
        return;
    }

    switch (mode)
    {
        /* ---- 轮廓速度模式 PV ---- */
        case CIA402_MODE_PV:
        {
            int32_t target_vel_raw = OD_RAM.x60FF_targetVelocity;
            /* 仅在目标速度变化时重设斜坡，避免每周期重写 */
            if (target_vel_raw != s_last_target_vel)
            {
                float_t target_rpm = (float_t)target_vel_raw / CIA402_VEL_SCALE;
                /* profileVelocity 作为斜坡时间 (ms) */
                uint16_t duration_ms = (uint16_t)(OD_RAM.x6081_profileVelocity & 0xFFFFu);
                if (duration_ms == 0u) duration_ms = 100u;
                MC_ProgramSpeedRampMotor1_F(target_rpm, duration_ms);
                s_last_target_vel = target_vel_raw;
            }
            break;
        }

        /* ---- 周期同步速度 CSV ---- */
        case CIA402_MODE_CSV:
        {
            /* CSV：每周期直接下发速度目标 */
            float_t target_rpm = (float_t)OD_RAM.x60FF_targetVelocity / CIA402_VEL_SCALE;
            MC_ProgramSpeedRampMotor1_F(target_rpm, 0u);
            s_last_target_vel = OD_RAM.x60FF_targetVelocity;
            break;
        }

        /* ---- 轮廓转矩模式 PT ---- */
        case CIA402_MODE_PT:
        case CIA402_MODE_CST:
        {
            float_t target_torque =
                (float_t)OD_RAM.x6071_targetTorque / CIA402_TRQ_SCALE;
            MC_ProgramTorqueRampMotor1_F(target_torque, 0u);
            break;
        }

        /* ---- 轮廓位置模式 PP ---- */
        case CIA402_MODE_PP:
        {
            /* 标准 CiA 402 PP 握手：检测 New Setpoint (bit 4) 上升沿 */
            bool new_setpoint = (cw & CW_NEW_SETPOINT) != 0u;
            bool last_setpoint = (s_cw_prev & CW_NEW_SETPOINT) != 0u;

            if (new_setpoint && !last_setpoint)
            {
                /* 触发新移动 */
                int32_t target_val_inc = OD_RAM.x607A_targetPosition;
                int32_t cur_pos_inc    = OD_RAM.x6064_positionActualValue;

                /* 绝对/相对位置处理：在整数域完成 */
                if ((cw & CW_ABS_REL) != 0u)
                {
                    /* 相对模式：目标 = 当前 + 偏移 */
                    s_target_pos_inc = cur_pos_inc + target_val_inc;
                }
                else
                {
                    /* 绝对模式 */
                    s_target_pos_inc = target_val_inc;
                }

                /* 转换为 Rad 用于 MC 指令 */
                s_target_pos_abs = (float_t)s_target_pos_inc / CIA402_POS_SCALE;

                /* 根据 Profile Velocity (0x6081) 计算持续时间 */
                float_t velocity_rpm = (float_t)OD_RAM.x6081_profileVelocity / CIA402_VEL_SCALE;
                if (velocity_rpm < 1.0f) velocity_rpm = 100.0f; /* 默认 100 RPM */
                
                float_t cur_pos_rad = MC_GetCurrentPosition1();
                float_t distance_rad = s_target_pos_abs - cur_pos_rad;
                if (distance_rad < 0) distance_rad = -distance_rad;
                
                float_t velocity_rads = (velocity_rpm * 2.0f * 3.14159f) / 60.0f;
                float_t duration = distance_rad / velocity_rads;
                if (duration < 0.01f) duration = 0.01f;

                printf("[CIA402] PP Start: target=%d inc (%.2f rad), vel=%.1f rpm, dur=%.3f s\r\n",
                       s_target_pos_inc, s_target_pos_abs, velocity_rpm, duration);
                
                MC_ProgramPositionCommandMotor1(s_target_pos_abs, duration);
                s_pp_setpoint_pending = true;
            }
            
            /* 握手应答：如果 New Setpoint 变为 0，清除 pending (ACK 变为 0) */
            if (!new_setpoint)
            {
                s_pp_setpoint_pending = false;
            }
            break;
        }

        /* ---- 周期同步位置 CSP ---- */
        case CIA402_MODE_CSP:
        {
            /* CSP: duration=0 表示立即执行（follow mode），主站负责插补 */
            float_t target_pos =
                (float_t)OD_RAM.x607A_targetPosition / CIA402_POS_SCALE;
            MC_ProgramPositionCommandMotor1(target_pos, 0.0f);
            break;
        }

        default:
            break;
    }
}

/* ============================================================
 * 内部：CiA 402 状态转移逻辑
 * ============================================================ */
static void CIA402_StateMachineProcess(void)
{
    uint16_t cw          = OD_RAM.x6040_controlword;
    bool fault_reset_edge = ((cw & CW_FAULT_RESET) != 0u) &&
                            ((s_cw_prev & CW_FAULT_RESET) == 0u); /* 上升沿 */

    /* ---- 检查 MC 层是否有故障 ---- */
    uint16_t mc_faults = MC_GetCurrentFaultsMotor1();
    MCI_State_t mc_state = MC_GetSTMStateMotor1();

    /* MC 层进入故障 → CIA402 进入故障响应 */
    if (mc_faults != 0u &&
        s_state != CIA402_FAULT &&
        s_state != CIA402_FAULT_REACTION_ACTIVE &&
        s_state != CIA402_NOT_READY_TO_SWITCH_ON)
    {
        s_state = CIA402_FAULT_REACTION_ACTIVE;
        s_fault_reaction_tick = (uint32_t)HAL_GetTick();
        MC_StopMotor1();
        return;
    }

    /* FAULT_REACTION_ACTIVE → FAULT（停止完成后或超时） */
    if (s_state == CIA402_FAULT_REACTION_ACTIVE)
    {
        uint32_t elapsed = (uint32_t)HAL_GetTick() - s_fault_reaction_tick;
        /* MC 停止完成（STOP/IDLE）或故障状态（FAULT_NOW/FAULT_OVER）或超时200ms */
        if (mc_state == STOP || mc_state == IDLE ||
            mc_state == FAULT_NOW || mc_state == FAULT_OVER ||
            elapsed >= 200u)
        {
            s_state = CIA402_FAULT;
            s_fault_reaction_tick = 0;
        }
        return;
    }

    switch (s_state)
    {
        /* ================================================================
         * NOT_READY_TO_SWITCH_ON
         * MC层初始化完成后自动跳到 SWITCH_ON_DISABLED
         * ================================================================ */
        case CIA402_NOT_READY_TO_SWITCH_ON:
            if (mc_state != ICLWAIT && mc_state != OFFSET_CALIB && mc_state != CHARGE_BOOT_CAP) /* MC初始化完成 */
            {
                s_state = CIA402_SWITCH_ON_DISABLED;
            }
            break;

        /* ================================================================
         * SWITCH_ON_DISABLED
         * 控制字: bit1(Enable Voltage)=1 → READY_TO_SWITCH_ON
         * ================================================================ */
        case CIA402_SWITCH_ON_DISABLED:
            if ((cw & CW_ENABLE_VOLTAGE) != 0u &&
                (cw & CW_QUICK_STOP)     != 0u &&  /* Quick Stop=1 表示不触发 */
                (cw & CW_FAULT_RESET)    == 0u)
            {
                s_state = CIA402_READY_TO_SWITCH_ON;
            }
            break;

        /* ================================================================
         * READY_TO_SWITCH_ON
         * 控制字: bit0(Switch On)=1 → SWITCHED_ON
         * 控制字: bit1=0 → SWITCH_ON_DISABLED
         * ================================================================ */
        case CIA402_READY_TO_SWITCH_ON:
            if ((cw & CW_ENABLE_VOLTAGE) == 0u)
            {
                s_state = CIA402_SWITCH_ON_DISABLED;
            }
            else if ((cw & CW_SWITCH_ON) != 0u)
            {
                s_state = CIA402_SWITCHED_ON;
            }
            break;

        /* ================================================================
         * SWITCHED_ON
         * 控制字: bit3(Enable Operation)=1 → OPERATION_ENABLED
         * 控制字: bit0=0 → READY_TO_SWITCH_ON
         * 控制字: bit1=0 → SWITCH_ON_DISABLED
         * ================================================================ */
        case CIA402_SWITCHED_ON:
            if ((cw & CW_ENABLE_VOLTAGE) == 0u)
            {
                s_state = CIA402_SWITCH_ON_DISABLED;
            }
            else if ((cw & CW_SWITCH_ON) == 0u)
            {
                s_state = CIA402_READY_TO_SWITCH_ON;
            }
            else if ((cw & CW_ENABLE_OPERATION) != 0u)
            {
                /* 启动电机 */
                bool start_ok = MC_StartMotor1();
                printf("[CIA402] SWITCHED_ON: MC_StartMotor1() = %d, MC_state=%d\r\n", start_ok, MC_GetSTMStateMotor1());
                if (start_ok == true)
                {
                    s_state = CIA402_OPERATION_ENABLED;
                }
                /* 启动失败则保持当前状态，等待下次重试 */
            }
            break;

        /* ================================================================
         * OPERATION_ENABLED  (正常运行)
         * 控制字: bit3=0 → SWITCHED_ON
         * 控制字: bit0=0 → READY_TO_SWITCH_ON
         * 控制字: bit2(Quick Stop)=0 → QUICK_STOP_ACTIVE
         * 控制字: bit1=0 → SWITCH_ON_DISABLED
         * ================================================================ */
        case CIA402_OPERATION_ENABLED:
            if ((cw & CW_ENABLE_VOLTAGE) == 0u)
            {
                MC_StopMotor1();
                s_state = CIA402_SWITCH_ON_DISABLED;
            }
            else if ((cw & CW_QUICK_STOP) == 0u)
            {
                /* 快速停止：触发快速减速 */
                MC_ProgramSpeedRampMotor1_F(0.0f, 200u); /* 200ms快速停 */
                s_state = CIA402_QUICK_STOP_ACTIVE;
            }
            else if ((cw & CW_SWITCH_ON) == 0u)
            {
                MC_StopMotor1();
                s_state = CIA402_READY_TO_SWITCH_ON;
            }
            else if ((cw & CW_ENABLE_OPERATION) == 0u)
            {
                MC_StopMotor1();
                s_state = CIA402_SWITCHED_ON;
            }
            else
            {
                /* 正常运行中，执行模式指令 */
                CIA402_ExecuteModeCommand();
            }
            break;

        /* ================================================================
         * QUICK_STOP_ACTIVE
         * 速度到0后 → SWITCH_ON_DISABLED
         * 控制字 Enable Voltage=1 + Quick Stop=1 → OPERATION_ENABLED (可选)
         * ================================================================ */
        case CIA402_QUICK_STOP_ACTIVE:
        {
            float_t speed = MC_GetAverageMecSpeedMotor1_F();
            if (speed < 1.0f && speed > -1.0f) /* 速度接近0 */
            {
                MC_StopMotor1();
                s_state = CIA402_SWITCH_ON_DISABLED;
            }
            break;
        }

        /* ================================================================
         * FAULT
         * 控制字: bit7(Fault Reset) 上升沿 → SWITCH_ON_DISABLED
         * ================================================================ */
        case CIA402_FAULT:
            if (fault_reset_edge)
            {
                if (MC_AcknowledgeFaultMotor1() == true)
                {
                    s_state = CIA402_SWITCH_ON_DISABLED;
                }
            }
            break;

        default:
            s_state = CIA402_SWITCH_ON_DISABLED;
            break;
    }
}

/* ============================================================
 * 公共接口实现
 * ============================================================ */
void CIA402_Init(void)
{
    memset(&OD_RAM.x6040_controlword, 0, sizeof(uint16_t));
    OD_RAM.x6041_statusword             = SW_NOT_READY_TO_SWITCH_ON_VAL;
    printf("[CIA402] Init called, state=%d, SW=0x%04X\r\n", s_state, OD_RAM.x6041_statusword);
    OD_RAM.x6060_modesOfOperation       = CIA402_MODE_PP; /* 默认位置模式 (Profile Position) */
    OD_RAM.x6061_modesOfOperationDisplay= CIA402_MODE_PP;
    OD_RAM.x6064_positionActualValue    = 0;
    OD_RAM.x606C_velocityActualValue    = 0;
    OD_RAM.x6071_targetTorque           = 0;
    OD_RAM.x6077_torqueActualValue      = 0;
    OD_RAM.x607A_targetPosition         = 0;
    OD_RAM.x6081_profileVelocity        = 60u; /* PP模式下的默认匀速运行转速: 60 RPM */
    OD_RAM.x60FF_targetVelocity         = 0;

    s_state            = CIA402_NOT_READY_TO_SWITCH_ON;
    s_cw_prev          = 0u;
    s_pp_setpoint_pending = false;
    s_last_target_vel  = 0;
    s_last_target_pos  = 0;
    s_fault_reaction_tick = 0;
}

/*
 * CIA402_Process() 必须在固定周期任务中调用（推荐 1ms）
 * 例如放在 MediumFrequencyTask 或单独 1ms SysTick 回调中
 */
void CIA402_Process(void)
{
    /* 调试: 每1000次打印一次状态 */
    static uint32_t s_process_count = 0;
    s_process_count++;
    if (s_process_count % 1000 == 0) {
        MCI_State_t mc_st = MC_GetSTMStateMotor1();
        printf("[CIA402] Process #%u, CIA_state=%d, MC_state=%d, CW=0x%04X, SW=0x%04X\r\n",
               s_process_count, s_state, mc_st, OD_RAM.x6040_controlword, OD_RAM.x6041_statusword);
    }

    /* 1. 运行状态机 */
    CIA402_StateMachineProcess();

    /* 2. 更新实际值到 OD */
    CIA402_UpdateActualValues();

    /* 3. 构建并写入状态字 */
    uint16_t sw = CIA402_BuildStatusword();
    CIA402_UpdateModeSpecificStatus(&sw);
    OD_RAM.x6041_statusword = sw;

    /* 4. 更新调试镜像 */
    OD_RAM.x2001_controlwordMirror = OD_RAM.x6040_controlword;
    OD_RAM.x2002_statuswordMirror  = OD_RAM.x6041_statusword;

    /* 5. 保存本周期控制字供下周期边沿检测 */
    s_cw_prev = OD_RAM.x6040_controlword;
}