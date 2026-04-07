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

/* CIA402 状态机句柄 */
static CIA402_Handle_t cia402;

/* 当前运行模式 */
static int8_t current_mode = MODE_NO_MODE;

/*===========================================================================
 * CIA402 初始化
 *===========================================================================*/

void cia402_init(void)
{
    /* 初始化 CIA402 轴 (使用标准库) */
    cia402_initialize(&cia402.axis, &OD_RAM.x6041_statusword, NULL);

    /* 清除任何残留故障 */
    motor_clear_fault();

    /* 初始化扩展字段 */
    cia402.target_mode = MODE_VELOCITY;
    cia402.last_mode = MODE_NO_MODE;
    cia402.switch_target_value = 0;

    /* 初始模式 */
    current_mode = MODE_NO_MODE;

    /* 初始状态字 - SWITCH_ON_DISABLED */
    OD_RAM.x6041_statusword = SWITCH_ON_DISABLED;
}

/*===========================================================================
 * CIA402 主处理函数 (1ms周期调用)
 *===========================================================================*/

void cia402_process(void)
{
    uint16_t controlword = OD_RAM.x6040_controlword;
    uint16_t statusword = 0;

    /* 调用标准库状态机 */
    cia402_state_machine(&cia402.axis, controlword);

    /* 获取更新后的状态字 */
    statusword = OD_RAM.x6041_statusword;

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
                break;
        }
    }

    /* 检测模式切换请求 */
    if (OD_RAM.x6060_modesOfOperation != current_mode &&
        cia402.axis.state == OPERATION_ENABLED) {
        /* 模式发生变化且处于运行状态 - 需要安全切换 */
        int8_t new_mode = OD_RAM.x6060_modesOfOperation;

        if (motor_is_stopped()) {
            /* 电机已停止，直接切换 */
            current_mode = new_mode;
        } else {
            /* 电机还在转，先停止 */
            cia402.target_mode = new_mode;
            cia402.switch_target_value = (new_mode == MODE_VELOCITY || new_mode == MODE_PROFILE_VELOCITY)
                                         ? OD_RAM.x60FF_targetVelocity
                                         : OD_RAM.x607A_targetPosition;
            motor_stop();
        }
    }

    /* 检查停止后的模式切换 */
    if (motor_is_stopped() && cia402.target_mode != 0 &&
        cia402.axis.state == OPERATION_ENABLED) {
        current_mode = cia402.target_mode;
        cia402.target_mode = 0;

        /* 根据新模式配置电机 */
        if (current_mode == MODE_VELOCITY || current_mode == MODE_PROFILE_VELOCITY) {
            motor_safe_switch_to_velocity(cia402.switch_target_value);
        } else if (current_mode == MODE_PROFILE_POSITION) {
            motor_safe_switch_to_position(cia402.switch_target_value);
        }
    }

    /* 故障检测 - 使用标准方式 */
    if (motor_has_fault() && cia402.axis.state != FAULT) {
        /* 触发故障转移 */
        cia402.axis.state = FAULT_REACTION_ACTIVE;
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
