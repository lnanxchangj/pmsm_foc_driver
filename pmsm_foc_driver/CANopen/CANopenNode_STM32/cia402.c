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

/* 控制字已收到标志 - 用于跳过初始的0值控制字
 * CIA402规范：controlword=0是DISABLE_VOLTAGE命令
 * 在CANopen通信未建立前，避免误触发状态转移
 */
static uint8_t controlword_received = 0;

/*===========================================================================
 * 辅助函数
 *===========================================================================*/

/* CIA402 状态名称（用于调试） */
static const char* state_name(CIA402_State_t state)
{
    switch (state)
    {
        case NOT_READY_TO_SWITCH_ON: return "NOT_READY";
        case SWITCH_ON_DISABLED:      return "SWITCH_ON_DISABLED";
        case READY_TO_SWITCH_ON:      return "READY_TO_SWITCH_ON";
        case SWITCHED_ON:              return "SWITCHED_ON";
        case OPERATION_ENABLED:        return "OPERATION_ENABLED";
        case QUICK_STOP_ACTIVE:        return "QUICK_STOP_ACTIVE";
        case FAULT_REACTION_ACTIVE:    return "FAULT_REACTION_ACTIVE";
        case FAULT:                    return "FAULT";
        default:                       return "UNKNOWN";
    }
}

/* 编码器PPR - 需要根据实际电机编码器设置
 * MC SDK使用弧度，CANopen使用PPR
 * 例如: 4000 PPR = 2π rad
 */
#define ENCODER_PPR     4000

/* PPR转弧度: rad = ppr * (2π / ppr_per_rev) */
static inline float ppr_to_rad(int32_t ppr)
{
    return (float)ppr * (2.0f * 3.14159265359f) / (float)ENCODER_PPR;
}

/* 设置位置目标（带持续时间计算）
 * 根据 x6081_profileVelocity 计算运动时间，实现平滑的位置控制
 *
 * 注意: MC SDK使用弧度作为位置单位，CANopen使用PPR
 * 这里做单位转换: PPR -> 弧度
 */
static void cia402_set_position_with_duration(int32_t target_pos)
{
    int32_t current_pos = motor_get_position();
    int32_t distance = (target_pos > current_pos) ?
                       (target_pos - current_pos) :
                       (current_pos - target_pos);
    int32_t profile_vel = OD_RAM.x6081_profileVelocity;

    /* 如果profile velocity为0，使用默认值10 PPR/s (安全速度) */
    if (profile_vel <= 0) {
        profile_vel = 10;
    }

    /* 计算运动时间（秒）: t = distance / velocity */
    float duration = (float)distance / (float)profile_vel;

    /* 确保最小时间，避免0 duration导致的立即跳变 */
    if (duration < 0.1f) {
        duration = 0.1f;
    }

    /* 转换为弧度后设置目标位置 */
    float target_rad = ppr_to_rad(target_pos);
    motor_set_target_position_with_duration(target_rad, duration);
}

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

    /* 重置控制字接收标志 - 等待主站发送第一个有效命令 */
    controlword_received = 0;
}

/*===========================================================================
 * CIA402 主处理函数 (1ms周期调用)
 *===========================================================================*/

void cia402_process(void)
{
    uint16_t controlword = OD_RAM.x6040_controlword;
    uint16_t statusword = 0;
    CIA402_State_t new_state;

    /* 跟踪控制字变化，用于检测首次有效命令
     * CIA402规范中controlword=0是DISABLE_VOLTAGE命令
     * 但在没有CANopen主站发送命令时，应该忽略初始的0值
     */
    static uint16_t last_controlword = 0;
    static uint32_t same_cw_count = 0;  /* 连续相同控制字的计数 */

    /* 如果控制字变化了，重置计数器 */
    if (controlword != last_controlword)
    {
        if (same_cw_count > 1 && last_controlword != 0)
        {
            printf("[CIA402] Controlword unchanged for %lu cycles\r\n", same_cw_count);
        }
        same_cw_count = 0;

        /* 如果控制字首次从0变为非0，说明主站已开始发送有效命令 */
        if (controlword != 0 && last_controlword == 0)
        {
            controlword_received = 1;
            printf("[CIA402] First valid controlword received: 0x%04X\r\n", controlword);
        }

        /* 只在变化时打印调试信息 */
        printf("[CIA402] RX controlword=0x%04X, current_state=%d\r\n",
               controlword, cia402.axis.state);
        last_controlword = controlword;
    }
    else
    {
        same_cw_count++;
    }

    /* 如果尚未收到有效控制字，跳过状态机处理
     * 这避免了controlword=0导致的误触发DISABLE_VOLTAGE命令
     */
    if (!controlword_received)
    {
        return;
    }

    /* 调用标准库状态机 */
    cia402_state_machine(&cia402.axis, controlword);

    /* 获取更新后的状态字 */
    statusword = OD_RAM.x6041_statusword;
    new_state = cia402.axis.state;

    /* 检测状态变化 */
    if (new_state != last_state)
    {
        printf("[CIA402] State changed: %d(%s) -> %d(%s), CW=0x%04X\r\n",
               last_state, state_name(last_state), new_state, state_name(new_state), controlword);

        /* 状态变化处理 */
        switch (new_state)
        {
        case OPERATION_ENABLED:
            printf("[CIA402] -> OPERATION_ENABLED\r\n");
            /* 进入运行状态 - 启动电机
             * 如果有待处理的模式切换，先处理模式切换
             */
            if (cia402.target_mode != MODE_NO_MODE && motor_is_stopped())
            {
                /* 有待处理的模式切换，先执行切换
                 * 注意：motor_switch_to_position_mode已经设置了控制模式，不需要再motor_start()
                 */
                printf("[CIA402] Executing pending mode switch to %d\r\n", cia402.target_mode);
                current_mode = cia402.target_mode;
                cia402.target_mode = MODE_NO_MODE;

                if (current_mode == MODE_VELOCITY || current_mode == MODE_PROFILE_VELOCITY)
                {
                    motor_switch_to_velocity_mode(OD_RAM.x60FF_targetVelocity);
                    motor_start();
                }
                else if (current_mode == MODE_PROFILE_POSITION)
                {
                    /* 执行时读取当前OD中的目标位置 */
                    cia402_set_position_with_duration(OD_RAM.x607A_targetPosition);
                    /* 位置模式不需要调用motor_start()，已经在上面设置好了 */
                }
                else if (current_mode == MODE_TORQUE)
                {
                    motor_switch_to_torque_mode(OD_RAM.x6071_targetTorque);
                    motor_start();
                }
            }
            else if (last_state == SWITCHED_ON || last_state == READY_TO_SWITCH_ON)
            {
                printf("[CIA402] Normal start from %d\r\n", last_state);
                motor_start();
            }
            break;

        case SWITCH_ON_DISABLED:
            printf("[CIA402] -> SWITCH_ON_DISABLED\r\n");
            if (last_state == OPERATION_ENABLED)
            {
                printf("[CIA402] Calling motor_stop()\r\n");
                motor_stop();
            }
            break;

        case READY_TO_SWITCH_ON:
            printf("[CIA402] -> READY_TO_SWITCH_ON\r\n");
            if (last_state == OPERATION_ENABLED)
            {
                printf("[CIA402] Calling motor_stop()\r\n");
                motor_stop();
            }
            break;

        case SWITCHED_ON:
            printf("[CIA402] -> SWITCHED_ON\r\n");
            if (last_state == OPERATION_ENABLED)
            {
                printf("[CIA402] Calling motor_hold_position()\r\n");
                motor_hold_position();
            }
            break;

        case QUICK_STOP_ACTIVE:
            printf("[CIA402] -> QUICK_STOP_ACTIVE\r\n");
            if (last_state == OPERATION_ENABLED)
            {
                printf("[CIA402] Calling motor_quick_stop()\r\n");
                motor_quick_stop();
            }
            break;

        case FAULT:
            printf("[CIA402] -> FAULT\r\n");
            motor_emergency_stop();
            break;

        default:
            break;
        }
        last_state = new_state;
    }

    /* 故障检测 - 实时检查 */
    if (motor_has_fault() && new_state != FAULT && new_state != FAULT_REACTION_ACTIVE)
    {
        /* 触发故障转移 */
        cia402.axis.state = FAULT_REACTION_ACTIVE;
        motor_emergency_stop();
    }

    /* 根据状态机标志执行相应动作 */
    if (cia402.axis.flags.axis_func_enabled)
    {
        /* 如果处于Quick Stop状态，不执行位置/扭矩命令，让电机减速停止 */
        if (new_state == QUICK_STOP_ACTIVE)
        {
            /* Quick Stop状态下只执行速度模式减速到0 */
            /* 不 reprogram 轨迹 */
        }
        else switch (current_mode)
        {
        case MODE_VELOCITY:
        case MODE_PROFILE_VELOCITY:
            /* 速度模式 - 每次调用都设置速度（确保SDO写入的值被传递） */
            motor_set_target_velocity(OD_RAM.x60FF_targetVelocity);
            break;

        case MODE_PROFILE_POSITION:
            /* 轮廓位置模式 - 使用profile velocity计算运动时间 */
            cia402_set_position_with_duration(OD_RAM.x607A_targetPosition);
            break;

        case MODE_TORQUE:
            /* 力矩模式 */
            motor_set_target_torque(OD_RAM.x6071_targetTorque);
            break;

        default:
            /* 未设置模式时默认速度模式 */
            if (OD_RAM.x6060_modesOfOperation != MODE_NO_MODE)
            {
                current_mode = OD_RAM.x6060_modesOfOperation;
            }
            break;
        }
    }

    /* 检测模式切换请求 */
    if (OD_RAM.x6060_modesOfOperation != current_mode &&
        new_state == OPERATION_ENABLED)
    {
        /* 模式发生变化且处于运行状态 - 需要安全切换 */
        int8_t new_mode = OD_RAM.x6060_modesOfOperation;

        if (motor_is_stopped())
        {
            /* 电机已停止，直接切换 */
            current_mode = new_mode;

            /* 根据新模式配置电机 */
            if (new_mode == MODE_VELOCITY || new_mode == MODE_PROFILE_VELOCITY)
            {
                motor_switch_to_velocity_mode(OD_RAM.x60FF_targetVelocity);
                motor_start();
            }
            else if (new_mode == MODE_PROFILE_POSITION)
            {
                cia402_set_position_with_duration(OD_RAM.x607A_targetPosition);
                /* 位置模式不需要motor_start()，已在上面设置好 */
            }
            else if (new_mode == MODE_TORQUE)
            {
                motor_switch_to_torque_mode(OD_RAM.x6071_targetTorque);
                motor_start();
            }
        }
        else
        {
            /* 电机还在转，先停止 */
            cia402.target_mode = new_mode;
            /* 根据模式存储目标值 */
            if (new_mode == MODE_VELOCITY || new_mode == MODE_PROFILE_VELOCITY)
            {
                cia402.switch_target_value = OD_RAM.x60FF_targetVelocity;
            }
            else if (new_mode == MODE_PROFILE_POSITION)
            {
                cia402.switch_target_value = OD_RAM.x607A_targetPosition;
            }
            else if (new_mode == MODE_TORQUE)
            {
                cia402.switch_target_value = OD_RAM.x6071_targetTorque;
            }
            motor_stop();
        }
    }

    /* 检查停止后的模式切换 */
    if (motor_is_stopped() && cia402.target_mode != MODE_NO_MODE &&
        new_state == OPERATION_ENABLED)
    {
        current_mode = cia402.target_mode;
        cia402.target_mode = MODE_NO_MODE;

        /* 根据新模式配置电机 */
        if (current_mode == MODE_VELOCITY || current_mode == MODE_PROFILE_VELOCITY)
        {
            motor_switch_to_velocity_mode(cia402.switch_target_value);
        }
        else if (current_mode == MODE_PROFILE_POSITION)
        {
            cia402_set_position_with_duration(cia402.switch_target_value);
        }
        else if (current_mode == MODE_TORQUE)
        {
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
