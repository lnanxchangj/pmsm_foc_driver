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
/* s_pp_setpoint_pending 已存在: bit4=1 时置位, bit4=0 时清零 → 防止重复触发 */
static uint32_t s_fault_reaction_tick = 0;

/* 连续多圈位置跟踪变量 */
static float_t s_continuous_pos_float = 0.0f;
static float_t s_last_mcsdk_pos_rad = 0.0f;
static bool s_pos_tracker_ready = false;

/* 窗口逻辑计时器 */
static uint32_t s_target_reached_timer = 0;

/* 回零模式状态 */
static CIA402_HomingState_t s_homing_state = CIA402_HM_IDLE;
static uint32_t s_homing_start_tick = 0;
static int8_t s_homing_method = 35;  /* 缓存 0x6098 */
static bool s_homing_halted = false; /* 控制字 bit8 暂停标志 */

#define CIA402_PI 3.141592653589793f
#define CIA402_2PI 6.283185307179586f

/* ============================================================
 * 功能开关：固定运动参数
 * 开启后将忽略上位机通过 0x6081/0x6083 设置的值，改用硬编码固定值
 * ============================================================ */
#define CIA402_USE_FIXED_DYNAMICS

/* 单圈模数轴定义 */
#define MODULO_RANGE 8000
#define MODULO_HALF 4000
#define CIA402_POS_SCALE ((float_t)MODULO_RANGE / CIA402_2PI)

/* ============================================================
 * 内部辅助函数
 * ============================================================ */

/* 前向声明 */
static void CIA402_HomingStart(void);
static void CIA402_HomingStop(void);

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

    /* 握手应答 (PP 模式) */
    if (s_pp_setpoint_pending)
        sw |= SW_SETPOINT_ACK;

    /* ============================================================
     * 回零模式状态字编码 (CiA 402 表 7.3.3)
     * bits 13/12/10 的组合表达回零状态
     * ============================================================ */
    if (OD_RAM.x6060_modesOfOperation == 6)
    {
        switch (s_homing_state)
        {
        case CIA402_HM_IDLE:
            /* 模式已选但未启动回零 → 中断或未启动 */
            sw |= HM_SW_NOT_STARTED;
            break;
        case CIA402_HM_SEARCHING:
            /* 回零进行中 → 不置任何位 */
            /* sw already has bits 13=0,12=0,10=0 after state base */
            break;
        case CIA402_HM_HALTED:
            /* 回零被暂停 → 目标未到达 */
            sw |= HM_SW_NOT_STARTED;
            break;
        case CIA402_HM_COMPLETED:
            /* 回零成功完成 */
            sw |= HM_SW_COMPLETED;
            break;
        case CIA402_HM_ERROR:
            /* 回零错误：检查速度是否为0 */
            {
                float_t spd = MC_GetAverageMecSpeedMotor1_F();
                if (fabsf(spd) < 1.0f)
                    sw |= HM_SW_ERR_STOP; /* 13=1,12=0,10=1: 错误，速度为0 */
                else
                    sw |= HM_SW_ERR_SPEED; /* 13=1,12=0,10=0: 错误，速度≠0 */
            }
            break;
        default:
            break;
        }
    }

    return sw;
}

static uint32_t CIA402_MapMCSDKFaults(uint16_t mc_faults)
{
    uint32_t err_2000 = 0;
    
    if (mc_faults & MC_OVER_VOLT) err_2000 |= (1 << 12);
    if (mc_faults & MC_UNDER_VOLT) err_2000 |= (1 << 13);
    if (mc_faults & MC_OVER_TEMP) err_2000 |= (1 << 11); // Drive overtemp
    if (mc_faults & MC_SPEED_FDBK) err_2000 |= (1 << 5); // Feedback error
    if (mc_faults & MC_OVER_CURR) err_2000 |= (1 << 8); // Overcurrent
    if (mc_faults & MC_DP_FAULT) err_2000 |= (1 << 1); // Short circuit
    if (mc_faults & MC_SW_ERROR) err_2000 |= (1 << 14); // Command error
    if (mc_faults & MC_START_UP) err_2000 |= (1 << 14); // Command error
    if (mc_faults & MC_DURATION) err_2000 |= (1 << 3); // Control error
    
    /* 自定义检测：主控先上电但驱动板未上电时，电流偏置校准会失败（读取到 0V）
       正常的 1.65V 偏置在 ADC 中应该远大于 500（12位下约2048）。
       如果测出的偏置值过低，则判断为“设置数据无效”。 */
    PolarizationOffsets_t offsets;
    if (MC_GetPolarizationOffsetsMotor1(&offsets)) {
        if (offsets.phaseAOffset < 500 && offsets.phaseBOffset < 500) {
            err_2000 |= (1 << 2); // Bit 2: 设置数据无效 (Offset calibration invalid)
        }
    }
    
    return err_2000;
}

static void CIA402_UpdateActualValues(void)
{
    OD_RAM.x606C_velocityActualValue = (int32_t)(MC_GetAverageMecSpeedMotor1_F() * CIA402_VEL_SCALE);
    OD_RAM.x2000_manufacturerParameter = CIA402_MapMCSDKFaults(MC_GetCurrentFaultsMotor1());

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

    int32_t pos_inc = (int32_t)(s_continuous_pos_float + 0.5f);
    if (pos_inc >= MODULO_RANGE)
        pos_inc = 0;

    /* 优化显示：如果位置非常接近一圈的终点（例如 7992），则显示为负数（例如 -8），
     * 避免用户在零点附近调试时看到跳变的数值。 */
    if (pos_inc > (MODULO_RANGE - 100))
    {
        pos_inc -= MODULO_RANGE;
    }
    OD_RAM.x6064_positionActualValue = pos_inc;

    float_t nominal = (float_t)NOMINAL_CURRENT_A;
    if (nominal < 0.1f)
        nominal = 1.0f;
    OD_RAM.x6077_torqueActualValue = (int16_t)((MC_GetIqdMotor1_F().q / nominal) * CIA402_TRQ_SCALE);
}

/* ============================================================
 * 回零启动 —— 由控制字 bit4 (开始回零操作) 0→1 上升沿触发
 * ============================================================ */
static void CIA402_HomingStart(void)
{
    /* 读取回零参数 */
    s_homing_method = OD_RAM.x6098_homingMethod;
    if (s_homing_method == 0)
        s_homing_method = 35; /* 默认方法 35: 仅 Z 脉冲 */

    /* 支持所有基于 Z 脉冲的方法 (33~37) */
    if (s_homing_method < 33 || s_homing_method > 37)
    {
        s_homing_state = CIA402_HM_ERROR;
        // printf("[CIA402] Homing ERROR: unsupported method %d\r\n", (int)s_homing_method);
        return;
    }

    /* 强制硬编码回零速度为 60 RPM */
    float_t hm_speed_rpm = 30.0f;

    /* 启动回零前，重置对齐状态 */
    if (pPosCtrl[0])
    {
        /* 读取当前角度，准备进行多圈位置轨迹移动以寻找Z脉冲 */
        int32_t wMecAngleRef = SPD_GetMecAngle(STC_GetSpeedSensor(pPosCtrl[0]->pSTC));

        /* 计算回零速度对应的 duration (ST默认是1圈2秒 = 30RPM) */
        float_t target_rpm = hm_speed_rpm;
        if (target_rpm < 1.0f)
            target_rpm = 10.0f;

        /* 计算移动一圈(2PI)需要的时间 */
        float_t duration = 60.0f / target_rpm;

        /* 发起一个 2PI (1圈) 的位置控制移动 */
        TC_MoveCommand(pPosCtrl[0], (float)(wMecAngleRef) / 10430.378f, 6.283185f, duration);

        pPosCtrl[0]->EncoderAbsoluteAligned = false;
        pPosCtrl[0]->AlignmentStatus = TC_ZERO_ALIGNMENT_START;
        pPosCtrl[0]->PositionControlRegulation = ENABLE;
    }

    s_homing_state = CIA402_HM_SEARCHING;
    s_homing_start_tick = HAL_GetTick();
    s_homing_halted = false;

    // printf("[CIA402] Homing STARTED (Position Mode): method=%d, speed=%.0f RPM\r\n", (int)s_homing_method, (double)hm_speed_rpm);
}

/* ============================================================
 * 回零停止 (暂停或中断)
 * ============================================================ */
static void CIA402_HomingStop(void)
{
    /* 立即在速度模式下停止电机 */
    MC_ProgramSpeedRampMotor1_F(0.0f, 100);
    s_homing_halted = true;
    s_homing_state = CIA402_HM_HALTED;
    // printf("[CIA402] Homing HALTED\r\n");
}

/* ============================================================
 * 回零过程监控 —— 在 OPERATION_ENABLED 每个周期检查
 * ============================================================ */
static void CIA402_HomingProcess(void)
{
    /* 处理控制字 bit8 (Halt) */
    uint16_t cw = OD_RAM.x6040_controlword;
    bool halt_requested = (cw & CW_HALT) != 0u;

    if (halt_requested && s_homing_state == CIA402_HM_SEARCHING)
    {
        CIA402_HomingStop();
        return;
    }

    if (!halt_requested && s_homing_state == CIA402_HM_HALTED)
    {
        /* 恢复回零：重新启动 Z 搜索 */
        if (pPosCtrl[0])
        {
            pPosCtrl[0]->AlignmentStatus = TC_AWAITING_FOR_ALIGNMENT;
            TC_EncAlignmentCommand(pPosCtrl[0]);
        }
        s_homing_state = CIA402_HM_SEARCHING;
        s_homing_start_tick = HAL_GetTick();
        s_homing_halted = false;
        // printf("[CIA402] Homing RESUMED\r\n");
        return;
    }

    if (s_homing_state != CIA402_HM_SEARCHING)
        return;

    /* 检查超时 */
    if (HAL_GetTick() - s_homing_start_tick > HOMING_TIMEOUT_MS)
    {
        s_homing_state = CIA402_HM_ERROR;
        if (pSTC[0])
            STC_StopRamp(pSTC[0]);
        // printf("[CIA402] Homing ERROR: timeout (>%d ms)\r\n", HOMING_TIMEOUT_MS);
        return;
    }

    /* 检查对齐状态 */
    if (pPosCtrl[0] == NULL)
        return;

    AlignStatus_t align = pPosCtrl[0]->AlignmentStatus;

    if (align == TC_ALIGNMENT_COMPLETED)
    {
        /* Z 脉冲已找到，编码器已被 TC_EncoderReset (ISR) 清零 */

        /* 应用原点偏置 (0x607C) */
        int32_t home_offset = OD_RAM.x607C_homeOffset;
        if (s_homing_method == 37 && home_offset != 0)
        {
            /* 方法 37: 原点偏置模式，将位置设置为 home offset */
            s_continuous_pos_float = (float_t)(home_offset % MODULO_RANGE);
            if (s_continuous_pos_float < 0.0f)
                s_continuous_pos_float += (float_t)MODULO_RANGE;
        }
        else
        {
            /* 方法 35: 零点模式，位置归零 */
            s_continuous_pos_float = 0.0f;
        }

        s_last_mcsdk_pos_rad = MC_GetCurrentPosition1();
        s_target_pos_inc = (int32_t)(s_continuous_pos_float + 0.5f);
        OD_RAM.x6064_positionActualValue = s_target_pos_inc;
        OD_RAM.x607A_targetPosition = s_target_pos_inc;

        /* 找到零点后必须立即停止电机，避免持续高速运转导致失步或故障 */
        float_t acc_val = (float_t)OD_RAM.x609A_homingAcceleration * CIA402_ACC_SCALE;
        if (acc_val < 1.0f)
            acc_val = 1000.0f;
        float_t current_rpm = MC_GetAverageMecSpeedMotor1_F();
        uint16_t stop_ramp_ms = (uint16_t)((fabsf(current_rpm) / acc_val) * 1000.0f);
        if (stop_ramp_ms < 10)
            stop_ramp_ms = 10;
        MC_ProgramSpeedRampMotor1_F(0.0f, stop_ramp_ms);

        s_homing_state = CIA402_HM_COMPLETED;
        // printf("[CIA402] Homing COMPLETED: pos=%ld, offset=%ld. Stopping motor...\r\n", (long)s_target_pos_inc, (long)home_offset);
    }
    else if (align == TC_ALIGNMENT_ERROR)
    {
        s_homing_state = CIA402_HM_ERROR;
        // printf("[CIA402] Homing ERROR: Z-index not found (no Z signal or ramp failed)\r\n");
    }
    /* TC_ZERO_ALIGNMENT_START: 搜索 ramp 运行中，继续等待 */
    /* TC_AWAITING_FOR_ALIGNMENT: 刚重置，等待 TC_PositionRegulation 启动 ramp */
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
        if ((cw & 0x0006) == 0x0006 && (cw & 0x0001) == 0) /* Shutdown (Transition 2) */
            s_state = CIA402_READY_TO_SWITCH_ON;
        break;

    case CIA402_READY_TO_SWITCH_ON:
        if ((cw & 0x0002) == 0) /* Disable Voltage (Transition 7) */
            s_state = CIA402_SWITCH_ON_DISABLED;
        else if ((cw & 0x0006) == 0x0002) /* Quick Stop (Transition 7) */
            s_state = CIA402_SWITCH_ON_DISABLED;
        else if ((cw & 0x000F) == 0x0007) /* Switch On (Transition 3) */
            s_state = CIA402_SWITCHED_ON;
        break;

    case CIA402_SWITCHED_ON:
        if ((cw & 0x0002) == 0) /* Disable Voltage (Transition 10) */
            s_state = CIA402_SWITCH_ON_DISABLED;
        else if ((cw & 0x0006) == 0x0002) /* Quick Stop (Transition 10) */
            s_state = CIA402_SWITCH_ON_DISABLED;
        else if ((cw & 0x000F) == 0x0006) /* Shutdown (Transition 6) */
            s_state = CIA402_READY_TO_SWITCH_ON;
        else if ((cw & 0x000F) == 0x000F) /* Enable Operation (Transition 4) */
        {
            if (mc_state == IDLE)
            {
                if (MC_StartMotor1()) {
                    // printf("[CIA402] Motor starting...\r\n");
                }
            }
            else if (mc_state == RUN)
            {
                /* [FIX] 状态清理：防止残留指令干扰。
                 * 刚从 IDLE 启动到 RUN 时，清除所有之前积压的 SDO/PDO 缓冲指令 */
                if (pMCI[0])
                {
                    pMCI[0]->CommandState = MCI_COMMAND_EXECUTED_SUCCESSFULLY;
                    pMCI[0]->DirectCommand = MCI_NO_COMMAND;
                }
                if (pSTC[0])
                {
                    /* 将 ST 底层速度环的参考点强置为当前实际速度（防止上次停止时的速度残余） */
                    STC_ForceSpeedReferenceToCurrentSpeed(pSTC[0]);
                }

                /* 初始化连续位置跟踪器 —— 从当前编码器值开始，不强制清零 */
                s_last_mcsdk_pos_rad = MC_GetCurrentPosition1();
                if (!s_pos_tracker_ready)
                {
                    /* 首次启动：基于编码器当前实际位置初始化（而非清零） */
                    float_t init_pos_rad = MC_GetCurrentPosition1();
                    s_continuous_pos_float = init_pos_rad * CIA402_POS_SCALE;
                    while (s_continuous_pos_float >= (float_t)MODULO_RANGE)
                        s_continuous_pos_float -= (float_t)MODULO_RANGE;
                    while (s_continuous_pos_float < 0.0f)
                        s_continuous_pos_float += (float_t)MODULO_RANGE;
                }
                s_pos_tracker_ready = true;

                /* 根据模式初始化底层 */
                if (mode == 1) /* PP: 位置模式 —— 直接切换，不改编码器值 */
                {
                    float_t sync_pos_rad = MC_GetCurrentPosition1();
                    MC_SetCtrlPositionAngle1(sync_pos_rad);
                    if (pMCI[0]->pPosCtrl)
                    {
                        pMCI[0]->pPosCtrl->PositionCtrlStatus = TC_READY_FOR_COMMAND;
                    }
                    if (pPosCtrl[0])
                        pPosCtrl[0]->PositionControlRegulation = ENABLE;
                }
                else if (mode == 3) /* PV: 速度模式 —— 直接切换，不改编码器值 */
                {
                    if (pPosCtrl[0])
                        pPosCtrl[0]->PositionControlRegulation = DISABLE;
                    if (pSTC[0])
                        STC_SetControlMode(pSTC[0], MCM_SPEED_MODE);
                    MC_ProgramSpeedRampMotor1_F(0.0f, 0.0f);
                }
                else if (mode == 6) /* HM: 回零模式 —— 等待控制字 bit4 启动 */
                {
                    /* [FIX] 使能回零模式时，先通过位置环锁死当前位置，防止电机漂移或疯转 */
                    float_t sync_pos_rad = MC_GetCurrentPosition1();
                    MC_SetCtrlPositionAngle1(sync_pos_rad);
                    if (pPosCtrl[0])
                    {
                        pPosCtrl[0]->PositionCtrlStatus = TC_READY_FOR_COMMAND;
                        pPosCtrl[0]->PositionControlRegulation = ENABLE;
                    }
                    s_homing_state = CIA402_HM_IDLE;
                    s_homing_halted = false;
                    // printf("[CIA402] ENABLED: Homing Mode (Position Locked) - waiting for start (bit4)\r\n");
                }

                s_last_target_vel = 0;
                s_last_mode = mode;

                /* 将目标位置吸附到当前实际位置，避免跳变 */
                CIA402_UpdateActualValues();
                s_target_pos_inc = OD_RAM.x6064_positionActualValue;
                OD_RAM.x607A_targetPosition = OD_RAM.x6064_positionActualValue;

                s_state = CIA402_OPERATION_ENABLED;
                // printf("[CIA402] ENABLED: Mode %d Ready\r\n", mode);
            }
        }
        break;

    case CIA402_OPERATION_ENABLED:
        if ((cw & 0x0002) == 0) /* Disable Voltage (Transition 9) */
        {
            // printf("[CIA402] Disable Voltage from OPERATION_ENABLED\r\n");
            MC_StopMotor1();
            s_homing_state = CIA402_HM_IDLE;
            s_state = CIA402_SWITCH_ON_DISABLED;
        }
        else if ((cw & 0x0004) == 0) /* Quick Stop (Transition 11) */
        {
            // printf("[CIA402] Quick Stop from OPERATION_ENABLED\r\n");
            if (pPosCtrl[0] && mode == 1)
            {
                float_t cur_pos = MC_GetCurrentPosition1();
                MC_ProgramPositionCommandMotor1(cur_pos, 0.1f);
            }
            else if (mode == 3)
            {
                float_t acc_val;
#ifdef CIA402_USE_FIXED_DYNAMICS
                acc_val = 600.0f;
#else
                acc_val = (float_t)OD_RAM.x6083_profileAcceleration * CIA402_ACC_SCALE;
                if (acc_val < 1.0f)
                    acc_val = 1000.0f;
#endif
                float_t current_rpm = MC_GetAverageMecSpeedMotor1_F();
                uint16_t ramp_ms = (uint16_t)((fabsf(current_rpm) / acc_val) * 1000.0f);
                if (ramp_ms < 10)
                    ramp_ms = 10;
                MC_ProgramSpeedRampMotor1_F(0.0f, ramp_ms);
            }
            else if (mode == 6)
            {
                MC_ProgramSpeedRampMotor1_F(0.0f, 100);
                s_homing_state = CIA402_HM_IDLE;
            }
            s_state = CIA402_QUICK_STOP_ACTIVE;
        }
        else if ((cw & 0x000F) == 0x0006) /* Shutdown (Transition 8) */
        {
            // printf("[CIA402] Shutdown from OPERATION_ENABLED\r\n");
            MC_StopMotor1();
            s_homing_state = CIA402_HM_IDLE;
            s_state = CIA402_READY_TO_SWITCH_ON;
        }
        else if ((cw & 0x000F) == 0x0007) /* Disable Operation (Transition 5) */
        {
            // printf("[CIA402] Disable Operation from OPERATION_ENABLED\r\n");
            MC_StopMotor1();
            s_homing_state = CIA402_HM_IDLE;
            s_state = CIA402_SWITCHED_ON;
        }
        else
        {
            /* ---- 动态模式切换 ---- */
            if (mode != s_last_mode)
            {
                if (mode == 1) /* PP: 位置模式 */
                {
                    float_t sync_pos_rad = MC_GetCurrentPosition1();
                    MC_SetCtrlPositionAngle1(sync_pos_rad);

                    if (pMCI[0]->pPosCtrl)
                    {
                        pMCI[0]->pPosCtrl->PositionCtrlStatus = TC_READY_FOR_COMMAND;
                    }

                    s_last_mcsdk_pos_rad = sync_pos_rad;

                    CIA402_UpdateActualValues();
                    s_target_pos_inc = OD_RAM.x6064_positionActualValue;
                    OD_RAM.x607A_targetPosition = OD_RAM.x6064_positionActualValue;

                    if (pPosCtrl[0])
                        pPosCtrl[0]->PositionControlRegulation = ENABLE;

                    s_homing_state = CIA402_HM_IDLE;
                    // printf("[CIA402] Mode Switched to PP (1)\r\n");
                }
                else if (mode == 3) /* PV: 速度模式 */
                {
                    if (pPosCtrl[0])
                        pPosCtrl[0]->PositionControlRegulation = DISABLE;
                    if (pSTC[0])
                        STC_SetControlMode(pSTC[0], MCM_SPEED_MODE);
                    MC_ProgramSpeedRampMotor1_F(0.0f, 0.0f);

                    s_homing_state = CIA402_HM_IDLE;
                    // printf("[CIA402] Mode Switched to PV (3) - Pos Loop Disabled\r\n");
                }
                else if (mode == 6) /* HM: 回零模式 —— 等待控制字 bit4 启动 */
                {
                    if (pPosCtrl[0])
                        pPosCtrl[0]->PositionControlRegulation = DISABLE;
                    s_homing_state = CIA402_HM_IDLE;
                    s_homing_halted = false;
                    // printf("[CIA402] Mode Switched to HM (6) - waiting for start (bit4)\r\n");
                }
                s_last_mode = mode;
            }

            /* ---- 回零模式：监控控制字 bit4 (开始回零操作) ---- */
            if (mode == 6)
            {
                bool bit4_now = (cw & CW_NEW_SETPOINT) != 0u; /* HM 模式中 bit4 = 开始回零 */
                bool bit4_prev = (s_cw_prev & CW_NEW_SETPOINT) != 0u;

                /* bit4 上升沿或电平 → 启动回零 (边沿丢失时电平兜底) */
                bool hm_edge = (bit4_now && !bit4_prev);
                bool hm_level = (bit4_now && (s_homing_state == CIA402_HM_IDLE ||
                                              s_homing_state == CIA402_HM_ERROR ||
                                              s_homing_state == CIA402_HM_HALTED));

                if (hm_edge || hm_level)
                {
                    if (s_homing_state == CIA402_HM_IDLE ||
                        s_homing_state == CIA402_HM_ERROR ||
                        s_homing_state == CIA402_HM_HALTED)
                    {
                        CIA402_HomingStart();
                    }
                }

                /* bit4 下降沿 → 中断回零 (如果正在进行中) */
                if (!bit4_now && bit4_prev)
                {
                    if (s_homing_state == CIA402_HM_SEARCHING)
                    {
                        CIA402_HomingStop();
                    }
                }

                /* 回零过程监控 */
                CIA402_HomingProcess();
            }

            /* ---- PP 模式：set-point 握手 ---- */
            if (mode == 1)
            {
                bool new_setpoint = (cw & CW_NEW_SETPOINT) != 0u;
                bool last_setpoint = (s_cw_prev & CW_NEW_SETPOINT) != 0u;

                /* 触发条件：
                 * ① 标准: bit4 0→1 上升沿
                 * ② 兼容: bit4=1 且未在等待应答 (主机先清零再置位但边沿被跨周期吞掉) */
                bool edge_trigger = (new_setpoint && !last_setpoint);
                bool level_trigger = (new_setpoint && !s_pp_setpoint_pending);

                if (edge_trigger || level_trigger)
                {
                    int32_t current_inc = (int32_t)OD_RAM.x6064_positionActualValue;
                    int32_t target_inc;
                    int32_t raw_target = OD_RAM.x607A_targetPosition;

                    if (cw & CW_ABS_REL)
                        target_inc = (current_inc + raw_target) % MODULO_RANGE;
                    else
                        target_inc = raw_target % MODULO_RANGE;

                    if (target_inc < 0)
                        target_inc += MODULO_RANGE;

                    int32_t diff_inc = target_inc - current_inc;
                    if (diff_inc > MODULO_HALF)
                        diff_inc -= MODULO_RANGE;
                    if (diff_inc < -MODULO_HALF)
                        diff_inc += MODULO_RANGE;

                    float_t diff_rad = (float_t)diff_inc / CIA402_POS_SCALE;

                    if (pMCI[0]->pPosCtrl)
                    {
                        pMCI[0]->pPosCtrl->PositionCtrlStatus = TC_READY_FOR_COMMAND;
                    }

                    float_t final_target_rad = MC_GetCtrlPositionAngle1() + diff_rad;
                    float_t vel_rpm;
#ifdef CIA402_USE_FIXED_DYNAMICS
                    vel_rpm = 60.0f; /* 固定 60 RPM */
#else
                    vel_rpm = (float_t)OD_RAM.x6081_profileVelocity;
                    if (vel_rpm < 1.0f)
                        vel_rpm = 60.0f;
#endif
                    float_t vel_rad_s = (vel_rpm * CIA402_2PI) / 60.0f;
                    float_t dur = fabsf(diff_rad) / vel_rad_s;
                    if (dur < 0.01f)
                        dur = 0.01f;

                    s_target_pos_inc = target_inc;
                    s_target_reached_timer = 0;
                    s_pp_setpoint_pending = true;

                    MC_ProgramPositionCommandMotor1(final_target_rad, dur);
                    // printf("[CIA402] PP %s: To %d, Dur=%.3fs\r\n", (cw & CW_ABS_REL) ? "REL" : "ABS", (int)target_inc, dur);
                }

                if (!new_setpoint)
                    s_pp_setpoint_pending = false;
            }

            /* ---- PV 模式：速度控制 ---- */
            if (mode == 3)
            {
                int32_t target_vel = OD_RAM.x60FF_targetVelocity;
                if (target_vel != s_last_target_vel)
                {
                    float_t target_rpm = (float_t)target_vel / CIA402_VEL_SCALE;
                    float_t acc_val;

#ifdef CIA402_USE_FIXED_DYNAMICS
                    acc_val = 600.0f; /* 固定加速度 600 RPM/s */
#else
                    acc_val = (float_t)OD_RAM.x6083_profileAcceleration * CIA402_ACC_SCALE;
                    if (acc_val < 1.0f)
                        acc_val = 1000.0f;
#endif

                    float_t current_rpm = MC_GetAverageMecSpeedMotor1_F();
                    float_t diff_rpm = fabsf(target_rpm - current_rpm);

                    uint16_t ramp_ms = (uint16_t)((diff_rpm / acc_val) * 1000.0f);
                    if (ramp_ms < 10)
                        ramp_ms = 10;

                    MC_ProgramSpeedRampMotor1_F(target_rpm, ramp_ms);
                    s_last_target_vel = target_vel;
                    // printf("[CIA402] PV Target: %d RPM, Acc: %d, Ramp: %d ms\r\n", (int)target_vel, (int)OD_RAM.x6083_profileAcceleration, ramp_ms);
                }
            }
        }
        break;

    case CIA402_QUICK_STOP_ACTIVE:
        if ((cw & 0x0002) == 0) /* Disable Voltage (Transition 12) */
        {
            MC_StopMotor1();
            s_state = CIA402_SWITCH_ON_DISABLED;
        }
        else if ((cw & 0x000F) == 0x000F) /* Enable Operation (Transition 16) */
        {
            // printf("[CIA402] Enable Operation from QUICK_STOP_ACTIVE\r\n");
            s_state = CIA402_OPERATION_ENABLED;
        }
        break;

    case CIA402_FAULT_REACTION_ACTIVE:
        if (HAL_GetTick() - s_fault_reaction_tick > 100) /* Fault Reaction completed (Transition 14) */
        {
            s_state = CIA402_FAULT;
        }
        break;

    case CIA402_FAULT:
        if ((cw & 0x0080) && !(s_cw_prev & 0x0080)) /* Fault Reset (Transition 15) */
        {
            MC_AcknowledgeFaultMotor1();
            if (MC_GetCurrentFaultsMotor1() == 0)
            {
                s_state = CIA402_SWITCH_ON_DISABLED;
            }
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
    OD_RAM.x6083_profileAcceleration = 600;
    s_state = CIA402_NOT_READY_TO_SWITCH_ON;
    s_pos_tracker_ready = false;
    s_pp_setpoint_pending = false;
    s_cw_prev = 0u;
    s_homing_state = CIA402_HM_IDLE;
    s_homing_start_tick = 0;
    s_homing_method = 35;
    s_homing_halted = false;
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
    else if (mode == 6)
    {
        /* 回零完成视为 target reached (已在 BuildStatusword 中编码) */
        reached = (s_homing_state == CIA402_HM_COMPLETED);
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
        // int mcsdk_pos_status = (pMCI[0]->pPosCtrl) ? pMCI[0]->pPosCtrl->PositionCtrlStatus : -1;
        // int align_status = (pMCI[0]->pPosCtrl) ? pMCI[0]->pPosCtrl->AlignmentStatus : -1;
    }
    s_cw_prev = OD_RAM.x6040_controlword;
}
