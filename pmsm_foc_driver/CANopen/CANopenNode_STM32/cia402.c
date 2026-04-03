/*
 * CIA402.c - CANopen CIA 402 State Machine Implementation
 *
 * CiA 402运动控制协议的状态机实现
 */

#include "cia402.h"
#include "motor_control_interface.h"
#include "OD.h"

static CIA402_State_t state = STATE_SWITCH_ON_DISABLED;

/* 故障标志 */
static uint8_t fault_pending = 0;

/*===========================================================================
 * 状态转移命令 (Controlword 0x6040)
 *===========================================================================
 * 状态转移图:
 *
 *   [Not Ready] --> [Switch On Disabled]
 *        |                    |
 *        v                    v
 *   [Ready to Switch On] <--+
 *        |                    |
 *        v                    |
 *   [Switched On] -------->--+
 *        |                    |
 *        v                    |
 *   [Operation Enabled] ----+
 *        |     |              |
 *        v     v              |
 *   [Quick Stop Active]      |
 *        |                   |
 *        +------------------>+
 *        |                   |
 *        v                   v
 *   [Fault] <-- [Fault Reaction Active]
 *        |
 *        v
 *   (Fault Reset) --> [Switch On Disabled]
 *===========================================================================*/

/* 将 Controlword 位转换为控制命令 */
static uint8_t controlword_to_target_state(uint16_t cw)
{
    uint8_t bits = 0;
    if (cw & CW_SWITCH_ON) bits |= 0x01;
    if (cw & CW_ENABLE_VOLTAGE) bits |= 0x02;
    if (cw & CW_QUICK_STOP) bits |= 0x04;
    if (cw & CW_ENABLE_OPERATION) bits |= 0x08;
    return bits;
}

/* 更新 Statusword */
static void update_statusword(CIA402_State_t state)
{
    uint16_t sw = 0;

    /* 基本状态位 */
    switch (state) {
        case STATE_NOT_READY_TO_SWITCH_ON:
            sw = 0x0000;
            break;

        case STATE_SWITCH_ON_DISABLED:
            sw = SW_SWITCH_ON_DISABLED;
            break;

        case STATE_READY_TO_SWITCH_ON:
            sw = SW_READY_TO_SWITCH_ON | SW_SWITCH_ON_DISABLED;
            break;

        case STATE_SWITCHED_ON:
            sw = SW_READY_TO_SWITCH_ON | SW_SWITCHED_ON | SW_SWITCH_ON_DISABLED | SW_VOLTAGE_ENABLED;
            break;

        case STATE_OPERATION_ENABLED:
            sw = SW_READY_TO_SWITCH_ON | SW_SWITCHED_ON | SW_OPERATION_ENABLED
               | SW_VOLTAGE_ENABLED | SW_SWITCH_ON_DISABLED;
            break;

        case STATE_QUICK_STOP_ACTIVE:
            sw = SW_READY_TO_SWITCH_ON | SW_SWITCHED_ON | SW_OPERATION_ENABLED
               | SW_VOLTAGE_ENABLED | SW_QUICK_STOP | SW_SWITCH_ON_DISABLED;
            break;

        case STATE_FAULT_REACTION_ACTIVE:
            sw = SW_READY_TO_SWITCH_ON | SW_SWITCHED_ON | SW_OPERATION_ENABLED
               | SW_VOLTAGE_ENABLED | SW_SWITCH_ON_DISABLED | SW_FAULT;
            break;

        case STATE_FAULT:
            sw = SW_FAULT | SW_SWITCH_ON_DISABLED;
            break;
    }

    OD_RAM.x6041_statusword = sw;
}

/*===========================================================================
 * 公共接口函数
 *===========================================================================*/

void cia402_init(void)
{
    state = STATE_SWITCH_ON_DISABLED;
    fault_pending = 0;

    /* 初始状态字 */
    OD_RAM.x6041_statusword = SW_SWITCH_ON_DISABLED;

    /* 默认模式 */
    OD_RAM.x6060_modesOfOperation = MODE_VELOCITY;  /* 速度模式 */
}

CIA402_State_t cia402_get_state(void)
{
    return state;
}

void cia402_process(void)
{
    uint16_t cw = OD_RAM.x6040_controlword;
    uint8_t target_bits = controlword_to_target_state(cw);

    /* 故障检测 */
    if (motor_has_fault() && state != STATE_FAULT && state != STATE_FAULT_REACTION_ACTIVE) {
        state = STATE_FAULT_REACTION_ACTIVE;
        motor_emergency_stop();
    }

    /* 故障恢复 */
    if (state == STATE_FAULT) {
        if (cw & CW_FAULT_RESET) {
            state = STATE_SWITCH_ON_DISABLED;
            motor_clear_fault();
            fault_pending = 0;
        }
        update_statusword(state);
        return;
    }

    /* 故障反应状态 - 等待故障处理完成 */
    if (state == STATE_FAULT_REACTION_ACTIVE) {
        state = STATE_FAULT;
        update_statusword(state);
        return;
    }

    /* 正常状态转移 */
    switch (state) {
        /*-------- 初始状态 --------*/
        case STATE_NOT_READY_TO_SWITCH_ON:
            /* 等待初始化完成 -> 自动转移到 Switch On Disabled */
            state = STATE_SWITCH_ON_DISABLED;
            break;

        /*-------- 关机禁止 --------*/
        case STATE_SWITCH_ON_DISABLED:
            if (target_bits == 0x06) {  /* Shutdown: bit1&2=1, bit0=0 */
                state = STATE_READY_TO_SWITCH_ON;
            }
            break;

        /*-------- 准备就绪 --------*/
        case STATE_READY_TO_SWITCH_ON:
            if (target_bits == 0x07) {  /* Switch On: bit0&1&2=1 */
                state = STATE_SWITCHED_ON;
            } else if (target_bits != 0x06) {
                /* 收到非法命令 -> 回到初始状态 */
                state = STATE_SWITCH_ON_DISABLED;
            }
            break;

        /*-------- 已通电 --------*/
        case STATE_SWITCHED_ON:
            if (target_bits == 0x0F) {  /* Enable Operation: bit0&1&2&3=1 */
                state = STATE_OPERATION_ENABLED;
            } else if (target_bits == 0x00) {  /* Disable Voltage */
                state = STATE_SWITCH_ON_DISABLED;
            } else if ((target_bits & 0x07) == 0x06) {
                /* 收到 Shutdown -> 回到 Ready */
                state = STATE_READY_TO_SWITCH_ON;
            }
            break;

        /*-------- 运行使能 --------*/
        case STATE_OPERATION_ENABLED:
            if (target_bits == 0x07) {  /* Disable Operation */
                state = STATE_SWITCHED_ON;
                motor_stop();
            } else if (target_bits == 0x00) {  /* Disable Voltage */
                state = STATE_SWITCH_ON_DISABLED;
                motor_stop();
            } else if (!(cw & CW_QUICK_STOP)) {  /* Quick Stop 激活 */
                state = STATE_QUICK_STOP_ACTIVE;
            } else if (target_bits == 0x06) {  /* Shutdown */
                state = STATE_READY_TO_SWITCH_ON;
                motor_stop();
            } else {
                /* ==== 执行运动控制 ==== */
                switch (OD_RAM.x6060_modesOfOperation) {
                    case MODE_VELOCITY:      /* 速度模式 (CSV) */
                    case MODE_PROFILE_VELOCITY:  /* 轮廓速度模式 (PV) */
                        motor_set_target_velocity(OD_RAM.x60FF_targetVelocity);
                        break;

                    case MODE_PROFILE_POSITION: /* 轮廓位置模式 (PP) */
                        motor_set_target_position(OD_RAM.x607A_targetPosition);
                        break;

                    case MODE_TORQUE:          /* 力矩模式 (TC) */
                        motor_set_target_torque(OD_RAM.x6071_targetTorque);
                        break;

                    default:
                        /* 未知模式，不输出 */
                        break;
                }
            }
            break;

        /*-------- 快速停止 --------*/
        case STATE_QUICK_STOP_ACTIVE:
            motor_emergency_stop();
            if (target_bits == 0x00) {  /* Disable Voltage -> 回到初始 */
                state = STATE_SWITCH_ON_DISABLED;
            } else if (cw & CW_QUICK_STOP) {
                /* Quick Stop 取消 -> 回到 Operation Enabled */
                state = STATE_OPERATION_ENABLED;
            }
            break;

        default:
            state = STATE_SWITCH_ON_DISABLED;
            break;
    }

    /* 更新状态字 */
    update_statusword(state);

    /* 更新反馈值 */
    OD_RAM.x6061_modesOfOperationDisplay = OD_RAM.x6060_modesOfOperation;
    OD_RAM.x6064_positionActualValue = motor_get_position();
    OD_RAM.x606C_velocityActualValue = motor_get_velocity();
    OD_RAM.x6077_torqueActualValue = motor_get_torque();
}
