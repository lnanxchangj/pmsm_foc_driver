/*
 * CIA402.c - CANopen CIA 402 State Machine Implementation
 *
 * 基于 cia402device 库的标准化实现
 * 集成 ST Motor Control SDK 接口
 *
 * CIA402状态机与ST Motor Control SDK的关系:
 * - CIA402定义驱动状态机（通信和控制流程）
 * - ST SDK定义电机控制状态机（FOC和硬件控制）
 * 两者需要正确同步，但概念不同
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

/* 故障触发标志 - 用于在状态机处理外检测故障
 * 故障必须通过状态机处理，不能直接赋值状态
 */
static uint8_t fault_triggered = 0;

/* 停止命令已发送标志 - 用于防止速度模式处理器在减速过程中覆盖停止命令
 * 当mode switch请求触发motor_stop()后，此标志被设置
 * 在电机完全停止前，速度模式处理器不会发送新的速度命令
 */
static uint8_t stop_for_mode_switch = 0;

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

/* 获取CIA402状态字对应的状态名称 */
static const char* statusbit_name(uint16_t statusword)
{
    if (statusword & CIA402_STATUSWORD_FAULT)
        return "FAULT";
    if (statusword & CIA402_STATUSWORD_SWITCH_ON_DISABLED)
        return "SWITCH_ON_DISABLED";
    if (statusword & CIA402_STATUSWORD_OPERATION_ENABLED)
        return "OPERATION_ENABLED";
    if (statusword & CIA402_STATUSWORD_SWITCHED_ON)
        return "SWITCHED_ON";
    if (statusword & CIA402_STATUSWORD_READY_TO_SWITCH_ON)
        return "READY_TO_SWITCH_ON";
    return "UNKNOWN";
}

/*===========================================================================
 * 单位转换辅助
 *===========================================================================*/

/* 编码器PPR - 需要根据实际电机编码器设置
 * MC SDK使用弧度，CANopen使用编码器计数
 * 例如: 4000 PPR = 2π rad
 *
 * 注意: 这个值应与mc_config.h中的配置匹配
 */
#ifndef ENCODER_PPR
#define ENCODER_PPR     4000
#endif

/* PPR转弧度: rad = ppr * (2π / ppr_per_rev) */
static inline float ppr_to_rad(int32_t ppr)
{
    return (float)ppr * (2.0f * 3.14159265359f) / (float)ENCODER_PPR;
}

/* 弧度转PPR: ppr = rad * (ppr_per_rev / 2π) */
static inline int32_t rad_to_ppr(float rad)
{
    return (int32_t)(rad * (float)ENCODER_PPR / (2.0f * 3.14159265359f));
}

/*===========================================================================
 * 位置目标设置（带持续时间计算）
 *
 * 根据 x6081_profileVelocity 计算运动时间，实现平滑的位置控制
 *
 * 注意: MC SDK使用弧度作为位置单位，CANopen使用PPR/计数
 * 这里做单位转换: PPR -> 弧度
 *===========================================================================*/

/* 计算绝对目标位置（考虑绝对/相对位置）
 *
 * CIA402规范:
 * - 控制字 bit 6 = 0: 绝对位置 - 目标位置相对于原点
 * - 控制字 bit 6 = 1: 相对位置 - 目标位置相对于当前位置
 *
 * 返回: 计算后的绝对目标位置
 */
static int32_t cia402_calc_absolute_target(int32_t target_pos, uint16_t controlword)
{
    /* 检查控制字 bit 6 - 绝对/相对位置 */
    if (controlword & CIA402_CONTROLWORD_ABS_REL)
    {
        /* 相对位置: 绝对目标 = 当前位置 + 相对位移 */
        int32_t current_pos = motor_get_position();
        int32_t absolute_target = current_pos + target_pos;
        printf("[CIA402] RELATIVE POSITION: controlword=0x%04X, bit6=%d, current=%d, delta=%d, absolute=%d\r\n",
               controlword, (controlword >> 6) & 1, current_pos, target_pos, absolute_target);
        return absolute_target;
    }
    else
    {
        /* 绝对位置: 直接使用目标位置 */
        printf("[CIA402] ABSOLUTE POSITION: controlword=0x%04X, bit6=%d, target=%d\r\n",
               controlword, (controlword >> 6) & 1, target_pos);
        return target_pos;
    }
}

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
 * 状态字更新
 *
 * 确保statusword正确反映CIA402状态机和驱动状态
 *===========================================================================*/

static void cia402_update_statusword(void)
{
    uint16_t statusword = OD_RAM.x6041_statusword;

    /* 清除可能被CIA402状态机修改的位
     * cia402device库只设置状态位，不清除
     * 我们需要根据当前状态确保正确
     */

    /* 更新FAULT位 - 如果驱动有故障且状态机不在FAULT状态 */
    if (motor_has_fault())
    {
        /* 只有当前不在FAULT或FAULT_REACTION_ACTIVE状态时才设置FAULT位
         * 状态机会自己处理FAULT状态的转移
         */
        if (last_state != FAULT && last_state != FAULT_REACTION_ACTIVE)
        {
            /* 触发故障处理 */
            fault_triggered = 1;
        }
    }

    /* 根据CIA402状态机更新状态字的其他位
     * 注意: cia402device库会设置大部分位，这里只做补充
     */
}

/*===========================================================================
 * CIA402 初始化
 *===========================================================================*/

/* ALstatus 变量 - 用于 CIA402 状态机
 * 设置为 AL_STATUS_OP 让状态机正常运行
 * 注意：OPERATION_ENABLED 状态检查 AL_status != OP 来检测连接丢失
 */
static uint16_t AL_status = AL_STATUS_OP;  /* 初始化为OP状态 */

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

    /* 重置故障标志 */
    fault_triggered = 0;
}

/*===========================================================================
 * CIA402 主处理函数 (1ms周期调用)
 *
 * 这是CIA402状态机的主循环，每次调用处理一个状态转换
 * 遵循CIA402规范的非阻塞处理模式
 *===========================================================================*/

void cia402_process(void)
{
    uint16_t controlword = OD_RAM.x6040_controlword;
    uint16_t statusword = 0;
    CIA402_State_t new_state;
    CIA402_State_t target_state;

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
            /* 控制字长时间未变化，可能需要关注 */
        }
        same_cw_count = 0;

        /* 如果控制字首次从0变为非0，说明主站已开始发送有效命令 */
        if (controlword != 0 && last_controlword == 0)
        {
            controlword_received = 1;
            printf("[CIA402] First valid controlword received: 0x%04X\r\n", controlword);
        }

        /* 只在变化时打印调试信息 */
        printf("[CIA402] RX controlword=0x%04X, state=%s\r\n",
               controlword, state_name(cia402.axis.state));
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

    /* 首先处理故障检测 - 在状态机调用之前
     * 如果驱动有故障且状态机当前不在FAULT相关状态，
     * 需要通过控制字命令触发故障转移，而不是直接赋值
     */
    if (fault_triggered)
    {
        fault_triggered = 0;
        /* 故障已被触发，但需要通过状态机处理
         * 这里不直接修改状态，让状态机根据controlword判断
         * 如果controlword中有FAULT_RESET命令，状态机会处理
         */
    }

    /* 实时故障检测 - 在状态机处理之前检查
     * 如果电机驱动有故障，应该触发FAULT状态
     */
    if (motor_has_fault())
    {
        /* 检查是否已经在FAULT相关状态 */
        if (last_state != FAULT && last_state != FAULT_REACTION_ACTIVE)
        {
            /* 故障发生，需要转移到FAULT_REACTION_ACTIVE
             * CIA402规范要求通过控制字命令触发，但直接赋值也可接受
             * 因为故障是驱动层事件，不是主站命令
             */
            target_state = FAULT_REACTION_ACTIVE;

            /* 通知驱动层紧急停止 */
            motor_emergency_stop();

            /* 直接设置状态和statusword */
            cia402.axis.state = FAULT_REACTION_ACTIVE;
            OD_RAM.x6041_statusword |= CIA402_STATUSWORD_FAULT;

            printf("[CIA402] Fault detected, entering FAULT_REACTION_ACTIVE\r\n");

            /* 记录状态变化 */
            last_state = FAULT_REACTION_ACTIVE;
            return;
        }
    }

    /* 调用标准库状态机处理controlword */
    cia402_state_machine(&cia402.axis, controlword);

    /* 获取更新后的状态字 */
    statusword = OD_RAM.x6041_statusword;
    new_state = cia402.axis.state;

    /* 检测状态变化 */
    if (new_state != last_state)
    {
        printf("[CIA402] State transition: %s -> %s, CW=0x%04X\r\n",
               state_name(last_state), state_name(new_state), controlword);

        /* 状态变化时执行相应动作 */
        switch (new_state)
        {
        case OPERATION_ENABLED:
            printf("[CIA402] -> OPERATION_ENABLED\r\n");

            /* 检查是否已完全启动 */
            if (!motor_is_start_completed())
            {
                /* 电机尚未完全启动，需要先启动 */
                printf("[CIA402] Starting motor...\r\n");
                motor_start();
                /* 等待下一次调用检查启动完成 */
                break;
            }

            /* 启动已完成，执行待处理的模式切换（如果有） */
            if (cia402.target_mode != MODE_NO_MODE)
            {
                printf("[CIA402] Executing pending mode switch to %d\r\n", cia402.target_mode);
                current_mode = cia402.target_mode;
                cia402.target_mode = MODE_NO_MODE;

                if (current_mode == MODE_VELOCITY || current_mode == MODE_PROFILE_VELOCITY)
                {
                    motor_switch_to_velocity_mode(OD_RAM.x60FF_targetVelocity);
                }
                else if (current_mode == MODE_PROFILE_POSITION)
                {
                    cia402_set_position_with_duration(OD_RAM.x607A_targetPosition);
                }
                else if (current_mode == MODE_TORQUE)
                {
                    motor_switch_to_torque_mode(OD_RAM.x6071_targetTorque);
                }
            }
            else if (last_state == SWITCHED_ON || last_state == READY_TO_SWITCH_ON)
            {
                /* 从SWITCHED_ON或READY_TO_SWITCH_ON正常启动 */
                printf("[CIA402] Normal start from %s\r\n", state_name(last_state));
                /* 电机应该已经在运行，不需要再次启动 */
            }
            break;

        case SWITCH_ON_DISABLED:
            printf("[CIA402] -> SWITCH_ON_DISABLED\r\n");
            if (last_state == OPERATION_ENABLED)
            {
                printf("[CIA402] Calling motor_emergency_stop()\r\n");
                motor_emergency_stop();
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

        case FAULT_REACTION_ACTIVE:
            printf("[CIA402] -> FAULT_REACTION_ACTIVE\r\n");
            /* 已经在上面处理过，这里仅做日志 */
            break;

        case FAULT:
            printf("[CIA402] -> FAULT\r\n");
            /* 驱动已停止，保持停止状态 */
            break;

        default:
            break;
        }
        last_state = new_state;
    }

    /* 根据状态机标志执行相应动作 - 仅在轴功能启用时 */
    if (cia402.axis.flags.axis_func_enabled)
    {
        /* 如果处于Quick Stop状态，不执行新的位置/扭矩命令，让电机减速停止 */
        if (new_state == QUICK_STOP_ACTIVE)
        {
            /* Quick Stop状态下只执行速度模式减速到0
             * 不 reprogram 轨迹
             */
        }
        else switch (current_mode)
        {
        case MODE_VELOCITY:
        case MODE_PROFILE_VELOCITY:
            /* 速度模式 - 每次调用都设置速度（确保SDO写入的值被传递）
             * 但如果正在等待电机停止以切换模式，不要发送速度命令
             */
            if (stop_for_mode_switch)
            {
                /* 正在等待电机停止以切换模式，让电机减速停止 */
            }
            else
            {
                motor_set_target_velocity(OD_RAM.x60FF_targetVelocity);
            }
            break;

        case MODE_PROFILE_POSITION:
            /* 轮廓位置模式 - 使用profile velocity计算运动时间
             * 支持绝对/相对位置（根据控制字bit 6）
             *
             * 注意: 只在目标位置(x607A)或控制字变化时重新编程轨迹
             * 避免每周期重复编程导致轨迹控制器行为异常
             */
            {
                static uint16_t last_cw_for_pos = 0;
                static int32_t last_x607A = 0;
                uint16_t cw = OD_RAM.x6040_controlword;
                int32_t x607A = OD_RAM.x607A_targetPosition;

                /* 检测x607A或controlword是否变化 */
                if (x607A != last_x607A || cw != last_cw_for_pos)
                {
                    int32_t abs_target = cia402_calc_absolute_target(x607A, cw);
                    printf("[CIA402] POSITION TARGET CHANGED: cw=0x%04X, x607A=%d, abs_target=%d\r\n",
                           cw, x607A, abs_target);
                    last_cw_for_pos = cw;
                    last_x607A = x607A;
                    cia402_set_position_with_duration(abs_target);
                }
            }
            break;

        case MODE_TORQUE:
            /* 力矩模式 */
            motor_set_target_torque(OD_RAM.x6071_targetTorque);
            break;

        default:
            /* 未设置模式时从OD读取模式 */
            if (OD_RAM.x6060_modesOfOperation != MODE_NO_MODE)
            {
                current_mode = OD_RAM.x6060_modesOfOperation;
            }
            break;
        }
    }

    /* 检测模式切换请求 - 仅在OPERATION_ENABLED状态时允许切换 */
    if (OD_RAM.x6060_modesOfOperation != current_mode &&
        new_state == OPERATION_ENABLED)
    {
        /* 模式发生变化且处于运行状态 - 需要安全切换 */
        int8_t new_mode = OD_RAM.x6060_modesOfOperation;

        if (motor_is_stopped())
        {
            /* 电机已停止，直接切换 */
            printf("[CIA402] Mode switch (stopped): %d -> %d\r\n", current_mode, new_mode);
            current_mode = new_mode;
            stop_for_mode_switch = 0;  /* 清除停止标志 */

            /* 根据新模式配置电机 */
            if (new_mode == MODE_VELOCITY || new_mode == MODE_PROFILE_VELOCITY)
            {
                motor_switch_to_velocity_mode(OD_RAM.x60FF_targetVelocity);
            }
            else if (new_mode == MODE_PROFILE_POSITION)
            {
                /* 切换到位置模式时，使用当前位置作为目标位置
                 * 避免电机突然跑回旧的目标位置
                 */
                int32_t current_pos = motor_get_position();
                OD_RAM.x607A_targetPosition = current_pos;
                printf("[CIA402] Position mode: target = current = %d\r\n", current_pos);
                cia402_set_position_with_duration(OD_RAM.x607A_targetPosition);
            }
            else if (new_mode == MODE_TORQUE)
            {
                motor_switch_to_torque_mode(OD_RAM.x6071_targetTorque);
            }
        }
        else
        {
            /* 电机还在转，先停止，保存目标模式
             * 注意：只有尚未保存目标模式时才调用motor_stop()
             * 否则每次process()都会重置斜坡，导致电机永远无法停止
             */
            if (cia402.target_mode == MODE_NO_MODE)
            {
                /* 第一次检测到需要切换，开始停止电机 */
                printf("[CIA402] Mode switch start: %d -> %d, motor stopping...\r\n", current_mode, new_mode);
                motor_stop();
                stop_for_mode_switch = 1;  /* 标记已发送停止命令 */
            }

            /* 保存目标模式 */
            cia402.target_mode = new_mode;

            /* 保存目标值 */
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
        }
    }

    /* 检查停止后的模式切换 */
    if (motor_is_stopped() && cia402.target_mode != MODE_NO_MODE &&
        new_state == OPERATION_ENABLED)
    {
        printf("[CIA402] Executing deferred mode switch to %d\r\n", cia402.target_mode);
        current_mode = cia402.target_mode;
        cia402.target_mode = MODE_NO_MODE;
        stop_for_mode_switch = 0;  /* 清除停止标志 */

        /* 根据新模式配置电机 */
        if (current_mode == MODE_VELOCITY || current_mode == MODE_PROFILE_VELOCITY)
        {
            motor_switch_to_velocity_mode(cia402.switch_target_value);
        }
        else if (current_mode == MODE_PROFILE_POSITION)
        {
            /* 切换到位置模式时，使用当前位置作为目标位置
             * 避免电机突然跑回旧的目标位置
             */
            int32_t current_pos = motor_get_position();
            OD_RAM.x607A_targetPosition = current_pos;
            printf("[CIA402] Deferred position mode: target = current = %d\r\n", current_pos);
            cia402_set_position_with_duration(current_pos);
        }
        else if (current_mode == MODE_TORQUE)
        {
            motor_switch_to_torque_mode(cia402.switch_target_value);
        }
    }

    /* 更新反馈值 - Object Dictionary对象 */
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
