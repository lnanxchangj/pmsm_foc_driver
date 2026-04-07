/*
 * CIA402.c - CANopen CIA 402 State Machine Implementation
 *
 * 基于 cia402device 库的标准化实现
 * 集成 ST Motor Control SDK 接口
 */

#include "cia402.h"
#include "motor_control_interface.h"
#include "OD.h"
#include "mc_type.h"
#include <stdio.h>

/* CIA402 状态机句柄 */
static CIA402_Handle_t cia402;

/* 当前运行模式 */
static int8_t current_mode = MODE_NO_MODE;

/* 上一个状态 (用于状态变化检测) */
static CIA402_State_t last_state = SWITCH_ON_DISABLED;

/*===========================================================================
 * CIA402 初始化
 *===========================================================================*/

/* ALstatus 变量 - 用于 CIA402 状态机
 * CIA402规范要求：当应用层初始化完成后，ALstatus应该被设置为AL_STATUS_OP
 * 这里我们创建一个变量，在初始化时设置为AL_STATUS_OP让状态机正确转移
 */
static uint16_t AL_status = AL_STATUS_OP;

void cia402_init(void)
{
    /* 初始化 CIA402 轴 (使用标准库)
     * ALstatus需要指向一个有效的变量，让状态机能从NOT_READY_TO_SWITCH_ON转移
     */
    cia402_initialize(&cia402.axis, &OD_RAM.x6041_statusword, &AL_status);

    /* 清除任何残留故障 */
    motor_clear_fault();

    /* 初始化扩展字段 */
    cia402.target_mode = MODE_NO_MODE;
    cia402.last_mode = MODE_NO_MODE;
    cia402.switch_target_value = 0;

    /* 初始模式 */
    current_mode = MODE_NO_MODE;

    /* 初始状态 - 与状态机一致 */
    last_state = cia402.axis.state;

    /* Debug: 打印初始化后的状态 */
    printf("[CIA402] Init: state=0x%02X, SW=0x%04X, AL_status=0x%04X\r\n",
           cia402.axis.state, OD_RAM.x6041_statusword, AL_status);
}

/*===========================================================================
 * CIA402 主处理函数 (1ms周期调用)
 *===========================================================================*/

void cia402_process(void)
{
    uint16_t controlword = OD_RAM.x6040_controlword;
    uint16_t statusword = 0;
    CIA402_State_t new_state;

    /* Debug: 打印每次调用时收到的controlword */
    static uint16_t last_cw = 0xFFFF;
    if (controlword != last_cw) {
        printf("[CIA402] CW: 0x%04X\r\n", controlword);
        last_cw = controlword;
    }

    /* 调用标准库状态机 */
    cia402_state_machine(&cia402.axis, controlword);

    /* 获取更新后的状态字 */
    statusword = OD_RAM.x6041_statusword;
    new_state = cia402.axis.state;

    /* 检测状态变化 */
    if (new_state != last_state) {
        /* 状态变化处理 */
        switch (new_state) {
            case OPERATION_ENABLED:
                /* 进入运行状态 - 启动电机 */
                if (last_state == SWITCHED_ON || last_state == READY_TO_SWITCH_ON) {
                    motor_start();
                }
                break;

            case SWITCH_ON_DISABLED:
            case READY_TO_SWITCH_ON:
            case SWITCHED_ON:
                /* 进入非运行状态 - 确保电机停止 */
                if (last_state == OPERATION_ENABLED) {
                    motor_stop();
                }
                break;

            case FAULT:
                /* 进入故障状态 - 紧急停止 */
                motor_emergency_stop();
                break;

            default:
                break;
        }
        last_state = new_state;
    }

    /* 故障检测 - 实时检查 */
    if (motor_has_fault() && new_state != FAULT && new_state != FAULT_REACTION_ACTIVE) {
        /* 触发故障转移 */
        cia402.axis.state = FAULT_REACTION_ACTIVE;
        motor_emergency_stop();
    }

    /* 根据状态机标志执行相应动作 */
    if (cia402.axis.flags.axis_func_enabled) {
        /* 运行使能 - 执行运动控制 */
        switch (current_mode) {
            case MODE_VELOCITY:
            case MODE_PROFILE_VELOCITY:
                /* 速度模式 */
                motor_set_target_velocity(OD_RAM.x60FF_targetVelocity);
                break;

            case MODE_PROFILE_POSITION:
                /* 轮廓位置模式 */
                motor_set_target_position(OD_RAM.x607A_targetPosition);
                break;

            case MODE_TORQUE:
                /* 力矩模式 */
                motor_set_target_torque(OD_RAM.x6071_targetTorque);
                break;

            default:
                /* 未设置模式时默认速度模式 */
                if (OD_RAM.x6060_modesOfOperation != MODE_NO_MODE) {
                    current_mode = OD_RAM.x6060_modesOfOperation;
                }
                break;
        }
    }

    /* 检测模式切换请求 */
    if (OD_RAM.x6060_modesOfOperation != current_mode &&
        new_state == OPERATION_ENABLED) {
        /* 模式发生变化且处于运行状态 - 需要安全切换 */
        int8_t new_mode = OD_RAM.x6060_modesOfOperation;

        if (motor_is_stopped()) {
            /* 电机已停止，直接切换 */
            current_mode = new_mode;
        } else {
            /* 电机还在转，先停止 */
            cia402.target_mode = new_mode;
            /* 根据模式存储目标值 */
            if (new_mode == MODE_VELOCITY || new_mode == MODE_PROFILE_VELOCITY) {
                cia402.switch_target_value = OD_RAM.x60FF_targetVelocity;
            } else if (new_mode == MODE_PROFILE_POSITION) {
                cia402.switch_target_value = OD_RAM.x607A_targetPosition;
            } else if (new_mode == MODE_TORQUE) {
                cia402.switch_target_value = OD_RAM.x6071_targetTorque;
            }
            motor_stop();
        }
    }

    /* 检查停止后的模式切换 */
    if (motor_is_stopped() && cia402.target_mode != MODE_NO_MODE &&
        new_state == OPERATION_ENABLED) {
        current_mode = cia402.target_mode;
        cia402.target_mode = MODE_NO_MODE;

        /* 根据新模式配置电机 */
        if (current_mode == MODE_VELOCITY || current_mode == MODE_PROFILE_VELOCITY) {
            motor_switch_to_velocity_mode(cia402.switch_target_value);
        } else if (current_mode == MODE_PROFILE_POSITION) {
            motor_switch_to_position_mode(cia402.switch_target_value);
        } else if (current_mode == MODE_TORQUE) {
            motor_switch_to_torque_mode(cia402.switch_target_value);
        }
    }

    /* 更新反馈值 */
    OD_RAM.x6061_modesOfOperationDisplay = current_mode;
    OD_RAM.x6064_positionActualValue = motor_get_position();
    OD_RAM.x606C_velocityActualValue = motor_get_velocity();
    OD_RAM.x6077_torqueActualValue = motor_get_torque();
}

/*===========================================================================
 * 公共接口函数
 *===========================================================================*/

CIA402_State_t cia402_get_state(void)
{
    return cia402.axis.state;
}

int8_t cia402_get_mode(void)
{
    return current_mode;
}
