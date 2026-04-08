/*
 * motor_control_interface.c
 *
 * CIA402 <-> ST Motor Control SDK 之间的接口层
 *
 * 遵循MCP协议标准方式：所有控制指令通过MC_ProgramXxxMotor1缓冲命令
 * 在MC_RunMotorControlTasks任务循环中自动调用MCI_ExecBufferedCommands执行
 */

#include "motor_control_interface.h"
#include "mc_api.h"
#include "mc_config.h"
#include "mc_interface.h"
#include "trajectory_ctrl.h"
#include <stdio.h>

/*
 * 单位转换说明:
 *
 * MC SDK内部单位:
 * - 位置: 弧度 (rad)
 * - 速度: 0.1 RPM (int16_t)
 * - 扭矩: 电气扭矩参考 (电流Iq参考值)
 *
 * CIA402对象字典单位:
 * - 6064_positionActualValue: 计数 (counts) - 编码器脉冲数
 * - 606C_velocityActualValue: 计数/秒 (counts/s) - 根据单元计算
 * - 6077_torqueActualValue: 千分之一Nm (mNm)
 *
 * 注意: 实际单位取决于您的电机和编码器配置
 */

/*===========================================================================
 * CIA402 需要的电机控制函数实现
 *===========================================================================*/

/* 电机启动状态跟踪 - 用于非阻塞启动 */
static uint8_t motor_start_pending = 0;
static uint8_t motor_start_completed = 0;

/* 启动电机 - 非阻塞方式
 *
 * CIA402规范要求状态机处理不能阻塞，因此使用非阻塞启动
 * 启动流程:
 * 1. 调用MC_StartMotor1()启动电机
 * 2. 后续调用motor_check_start_completed()检查启动是否完成
 *
 * 注意: motor_start()只负责启动FOC，不设置控制模式
 *       控制模式应在调用此函数之前由motor_switch_to_xxx_mode设置好
 */
void motor_start(void)
{
    /* 如果上次启动尚未完成，先完成上次启动 */
    if (motor_start_pending)
    {
        return;
    }

    /* 启动电机控制 */
    MC_StartMotor1();

    /* 标记启动进行中 */
    motor_start_pending = 1;
    motor_start_completed = 0;
}

/* 检查启动是否完成 - 非阻塞方式
 *
 * 返回值:
 *   1: 启动已完成或无需启动
 *   0: 启动进行中
 */
uint8_t motor_check_start_completed(void)
{
    AlignStatus_t align_status;

    if (!motor_start_pending)
    {
        /* 没有待完成的启动 */
        return 1;
    }

    align_status = MC_GetAlignmentStatusMotor1();

    if (align_status == TC_ALIGNMENT_COMPLETED)
    {
        /* 对齐完成，启动成功 */
        motor_start_pending = 0;
        motor_start_completed = 1;
        return 1;
    }
    else if (align_status == TC_ALIGNMENT_FAILED)
    {
        /* 对齐失败 - 触发故障 */
        motor_start_pending = 0;
        return 0;
    }

    /* 仍在对齐中 */
    return 0;
}

/* 电机是否已完全启动 */
uint8_t motor_is_start_completed(void)
{
    return motor_start_completed;
}

/* 停止电机 - 正常减速停止 */
void motor_stop(void)
{
    /* 速度设为0，200ms减速时间（标准方式：通过缓冲命令） */
    MC_ProgramSpeedRampMotor1_F(0.0f, 200);
}

/* 紧急停止 - 立即切断输出 */
void motor_emergency_stop(void)
{
    /* 重置启动状态 */
    motor_start_pending = 0;
    motor_start_completed = 0;

    MC_StopMotor1();
}

/* 暂停电机 - 用于Disable Operation命令
 *
 * CIA402规范：Disable Op后电机应减速停止，保持位置
 *
 * 与motor_stop()的区别：
 * - motor_stop()用于Shutdown，完全停止（200ms）
 * - motor_hold_position()用于Disable Op，暂停后可以快速恢复（100ms）
 *
 * MC SDK方式：
 * 1. 停止当前斜坡（如果有）
 * 2. 禁用位置控制
 * 3. 切换到速度模式并减速到0
 *
 * 注意: 此函数应在SWITCHED_ON状态下调用，此时FOC已启用
 */
void motor_hold_position(void)
{
    /* 1. 先停止当前斜坡 */
    MC_StopRampMotor1();

    /* 2. 禁用位置控制 */
    pPosCtrl[M1]->PositionControlRegulation = DISABLE;

    /* 3. 切换到速度模式 */
    STC_SetControlMode(pSTC[M1], MCM_SPEED_MODE);

    /* 4. 设置速度为0，让电机减速停止 */
    MC_ProgramSpeedRampMotor1_F(0.0f, 100); /* 100ms减速，比Shutdown更快 */
}

/* 快速停止 - 用于Quick Stop命令
 *
 * CIA402规范：Quick Stop后电机应最快速减速停止
 *
 * 与motor_stop()的区别：
 * - motor_stop()用于Shutdown（200ms减速）
 * - motor_quick_stop()用于Quick Stop（50ms快速减速）
 */
void motor_quick_stop(void)
{
    /* 1. 先停止当前斜坡 */
    MC_StopRampMotor1();

    /* 2. 禁用位置控制 */
    pPosCtrl[M1]->PositionControlRegulation = DISABLE;

    /* 3. 切换到速度模式 */
    STC_SetControlMode(pSTC[M1], MCM_SPEED_MODE);

    /* 4. 设置速度为0，让电机快速减速停止 */
    MC_ProgramSpeedRampMotor1_F(0.0f, 50); /* 50ms快速减速 */
}

/* 设置目标速度 (RPM) - 标准MCP方式
 *
 * 内部调用链：
 * MC_ProgramSpeedRampMotor1_F() → MCI_ExecSpeedRamp() [缓冲命令]
 *                                          ↓
 *                         在MC_RunMotorControlTasks()中自动执行
 *                                          ↓
 *                                         STC_SetControlMode(MCM_SPEED_MODE)
 */
void motor_set_target_velocity(int32_t vel)
{
    /* vel 单位是 RPM */
    MC_ProgramSpeedRampMotor1_F((float_t)vel, 0);
}

/* 设置目标力矩 (mNm) - 标准MCP方式
 *
 * 内部调用链：
 * MC_ProgramTorqueRampMotor1_F() → MCI_ExecTorqueRamp() [缓冲命令]
 *                                            ↓
 *                           在MC_RunMotorControlTasks()中自动执行
 *                                            ↓
 *                                           STC_SetControlMode(MCM_TORQUE_MODE)
 *
 * 注意: mNm转A的转换取决于电机参数，这里使用简化模型
 *       实际应根据电机的KT常数和电流增益进行转换
 */
void motor_set_target_torque(int32_t torque_mnm)
{
    /* 简化转换: mNm -> A
     * 实际应使用: torque_amp = torque_mnm / (1000.0f * motor_KT)
     * 这里假设电机KT使得 1A 对应 1000 mNm (即KT = 1 Nm/A)
     */
    float_t torque_amp = (float_t)torque_mnm / 1000.0f;
    MC_ProgramTorqueRampMotor1_F(torque_amp, 0);
}

/* 设置目标位置 (弧度) - 标准MCP方式
 *
 * 内部调用链：
 * MC_ProgramPositionCommandMotor1() → MCI_ExecPositionCommand()
 *                                                ↓
 *                              MCI_ExecPositionCommand内部调用TC_MoveCommand()
 *                                                ↓
 *                                               TC_MoveCommand设置
 *                                               PositionControlRegulation = ENABLE
 *
 * 注意：Duration=0表示立即到达（follow模式），>0表示梯形速度轨迹
 *
 * 警告: MC SDK内部使用弧度，但CIA402使用PPR/计数
 *       调用者应确保传入正确的单位，或进行适当转换
 */
void motor_set_target_position(float position)
{
    /* position 单位是弧度(rad) - 与MC SDK一致 */
    /* Duration=0 表示立即执行（follow模式） */
    MC_ProgramPositionCommandMotor1((float_t)position, 0.0f);
}

/* 设置目标位置，带运动时间 (弧度, seconds)
 *
 * Duration > 0 时使用梯形速度轨迹：
 * 1. 加速阶段
 * 2. 匀速阶段
 * 3. 减速阶段
 */
void motor_set_target_position_with_duration(float position, float duration_sec)
{
    MC_ProgramPositionCommandMotor1((float_t)position, duration_sec);
}

/* 获取当前控制模式 */
MC_ControlMode_t motor_get_control_mode(void)
{
    return MC_GetControlModeMotor1();
}

/* 获取ST SDK电机状态
 *
 * 注意: 这是ST Motor Control SDK的内部状态机状态，不同于CIA402状态机
 * 返回值见MCI_State_t枚举:
 *   IDLE, ALIGNMENT, START, SWITCH_OVER, RUN, STOP, FAULT_NOW, FAULT_OVER等
 */
MCI_State_t motor_get_state(void)
{
    return MC_GetSTMStateMotor1();
}

/* 获取当前位置
 *
 * 返回: 编码器计数 (counts)
 *
 * MC_GetCurrentPosition1()返回float，单位是弧度
 * 转换为计数: counts = rad * (encoder_ppr / 2π)
 *
 * 注意: 需要与ENCODER_PPR配置匹配
 */
int32_t motor_get_position(void)
{
    float_t position_rad = MC_GetCurrentPosition1();

    /* 转换为计数
     * counts = rad * PPR / (2π)
     * 使用M1_ENCODER_PPR (定义于pmsm_motor_parameters.h，值为1000)
     */
    extern uint16_t M1_ENCODER_PPR;
    int32_t counts = (int32_t)(position_rad * (float_t)M1_ENCODER_PPR / (2.0f * 3.14159265359f));
    return counts;
}

/* 获取当前速度
 *
 * 返回: 速度值 (RPM)
 *
 * MC_GetMecSpeedAverageMotor1()返回int16_t，单位是0.1 RPM
 */
int32_t motor_get_velocity(void)
{
    /* ST SDK返回int16_t，单位0.1 RPM */
    int16_t speed_01rpm = MC_GetMecSpeedAverageMotor1();

    /* 转换为 RPM: rpm = 0.1rpm / 10 = rpm */
    return (int32_t)(speed_01rpm / 10);
}

/* 获取当前扭矩
 *
 * 返回: 扭矩值 (mNm)
 *
 * 注意: MC_GetTerefMotor1()返回电气扭矩参考(电流Iq参考)
 *       实际扭矩输出还受电机参数影响
 *       这里返回的是扭矩参考值，非实际测量值
 *
 *       若需要实际扭矩，应使用:
 *       - FOC计算的Iq电流 * 电机KT常数
 */
int32_t motor_get_torque(void)
{
    /* 获取扭矩参考值 (电气单位，int16_t) */
    int16_t teref = MC_GetTerefMotor1();

    /* 简化转换: 电气扭矩参考 -> mNm
     * 实际应使用: torque_mnm = teref * current_scale * motor_KT * 1000
     * 这里使用简化假设: teref直接对应某个比例的mNm
     */
    return (int32_t)(teref * 100);
}

/* 检查是否有故障 */
uint8_t motor_has_fault(void)
{
    uint16_t faults = MC_GetCurrentFaultsMotor1();
    return (faults != 0) ? 1 : 0;
}

/* 获取当前故障代码
 *
 * 返回: 故障位域，每位代表一种故障类型
 *       具体定义见mc_config.h中的故障码定义
 */
uint16_t motor_get_current_faults(void)
{
    return MC_GetCurrentFaultsMotor1();
}

/* 获取历史故障代码
 *
 * 返回: 自上次清除故障以来发生过的所有故障的位域
 */
uint16_t motor_get_occurred_faults(void)
{
    return MC_GetOccurredFaultsMotor1();
}

/* 清除故障
 *
 * 返回: 1 - 故障清除成功
 *       0 - 故障清除失败（可能仍有故障存在）
 */
uint8_t motor_clear_fault(void)
{
    return MC_AcknowledgeFaultMotor1();
}

/* 获取命令执行状态 */
MCI_CommandState_t motor_get_command_state(void)
{
    return MC_GetCommandStateMotor1();
}

/* 获取位置控制状态 */
PosCtrlStatus_t motor_get_position_control_status(void)
{
    return MC_GetControlPositionStatusMotor1();
}

/* 获取对准状态 */
AlignStatus_t motor_get_alignment_status(void)
{
    return MC_GetAlignmentStatusMotor1();
}

/*===========================================================================
 * 模式切换
 *
 * MC SDK的核心逻辑：
 * - Speed/Torque模式：使用STC_SetControlMode()切换控制模式
 * - Position模式：使用TC_MoveCommand()启用位置控制（不禁用STC模式）
 *
 * 关键：从Position模式切换到Speed/Torque模式时，必须先禁用位置控制！
 *===========================================================================*/

/* 切换到速度模式并设置目标速度
 *
 * MC SDK方式：
 * 1. 禁用位置控制（pPosCtrl->PositionControlRegulation = DISABLE）
 * 2. 设置控制模式为速度模式（STC_SetControlMode）
 * 3. 设置目标速度（MC_ProgramSpeedRampMotor1_F）
 *
 * 注意: 此函数只配置速度和模式，不启动电机
 *       调用motor_start()来启动电机
 */
void motor_switch_to_velocity_mode(int32_t target_vel)
{
    /* 1. 必须先禁用位置控制！ */
    pPosCtrl[M1]->PositionControlRegulation = DISABLE;

    /* 2. 设置控制模式为速度模式 */
    STC_SetControlMode(pSTC[M1], MCM_SPEED_MODE);

    /* 3. 设置目标速度 */
    MC_ProgramSpeedRampMotor1_F((float_t)target_vel, 0);
}

/* 切换到扭矩模式并设置目标力矩
 *
 * MC SDK方式：
 * 1. 禁用位置控制（pPosCtrl->PositionControlRegulation = DISABLE）
 * 2. 设置控制模式为扭矩模式（STC_SetControlMode）
 * 3. 设置目标扭矩（MC_ProgramTorqueRampMotor1_F）
 *
 * 注意: 此函数只配置扭矩和模式，不启动电机
 *       调用motor_start()来启动电机
 */
void motor_switch_to_torque_mode(int32_t target_torque)
{
    /* 1. 必须先禁用位置控制！ */
    pPosCtrl[M1]->PositionControlRegulation = DISABLE;

    /* 2. 设置控制模式为扭矩模式 */
    STC_SetControlMode(pSTC[M1], MCM_TORQUE_MODE);

    /* 3. 设置目标扭矩 */
    float_t torque_amp = (float_t)target_torque / 1000.0f;
    MC_ProgramTorqueRampMotor1_F(torque_amp, 0);
}

/* 切换到位置模式并设置目标位置
 *
 * MC SDK方式：
 * 1. 调用TC_MoveCommand启用位置控制（自动设置PositionControlRegulation = ENABLE）
 * 2. 设置目标位置（MC_ProgramPositionCommandMotor1）
 *
 * 注意: Position模式不通过STC_SetControlMode切换，而是通过TC_MoveCommand
 *       此函数只配置位置模式，不启动电机
 */
void motor_switch_to_position_mode(float target_pos)
{
    /* TC_MoveCommand会启用位置控制，设置目标位置和运动时间 */
    /* Duration=0表示立即到达（follow模式） */
    MC_ProgramPositionCommandMotor1((float_t)target_pos, 0.0f);
}

/* 停止当前斜坡 - 保持当前值
 *
 * 标准方式：调用MC_StopRampMotor1
 */
void motor_stop_ramp(void)
{
    MC_StopRampMotor1();
}

/* 检查斜坡是否完成 */
uint8_t motor_ramp_completed(void)
{
    return MC_HasRampCompletedMotor1() ? 1 : 0;
}

/* 检查电机是否已停止
 *
 * 判断条件:
 * 1. 速度参考为0且实际速度足够小（< 5 RPM，考虑死区）
 * 2. 斜坡已完成
 * 3. 命令执行完成
 *
 * 注意: 单独的速度判断可能不可靠，因为速度可能有波动
 *       使用多种条件综合判断更可靠
 */
uint8_t motor_is_stopped(void)
{
    int32_t vel = motor_get_velocity();
    int16_t speed_ref = MC_GetMecSpeedReferenceMotor1();
    uint8_t ramp_done = MC_HasRampCompletedMotor1();
    MCI_CommandState_t cmd_state = MC_GetCommandStateMotor1();

    /* 检查速度参考是否为0 */
    int8_t speed_ref_is_zero = (speed_ref == 0);

    /* 检查实际速度是否足够小 (阈值5 RPM) */
    int8_t speed_is_low = (vel < 5 && vel > -5);

    /* 检查斜坡和命令状态 */
    return (speed_ref_is_zero && speed_is_low && ramp_done &&
            (cmd_state == MCI_BUFFER_EMPTY || cmd_state == MCI_COMMAND_EXECUTED_SUCCESSFULLY)) ? 1 : 0;
}
