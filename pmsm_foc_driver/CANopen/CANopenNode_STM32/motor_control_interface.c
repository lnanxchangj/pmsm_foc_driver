/*
 * motor_control_interface.c
 *
 * CIA402 <-> ST Motor Control SDK 之间的接口层
 *
 * 模仿MCP协议标准方式：所有控制指令通过MC_ProgramXxxMotor1缓冲命令
 * 在MC_RunMotorControlTasks任务循环中自动调用MCI_ExecBufferedCommands执行
 */

#include "motor_control_interface.h"
#include "mc_api.h"
#include "mc_config.h"
#include "mc_interface.h"
#include "trajectory_ctrl.h"
#include <stdio.h>
/*
 * MCP标准方式说明：
 *
 * 速度模式：MC_ProgramSpeedRampMotor1_F() → MCI_ExecSpeedRamp() [缓冲命令]
 *                                          ↓
 *                         TIM1_BRK中断 → MC_RunMotorControlTasks()
 *                                          ↓
 *                                         MCI_ExecBufferedCommands()
 *                                          ↓
 *                                         STC_SetControlMode(MCM_SPEED_MODE) [真正切换]
 *
 * 扭矩模式：MC_ProgramTorqueRampMotor1_F() → MCI_ExecTorqueRamp() [缓冲命令]
 *                                            ↓
 *                           TIM1_BRK中断 → MC_RunMotorControlTasks()
 *                                            ↓
 *                                           MCI_ExecBufferedCommands()
 *                                            ↓
 *                                           STC_SetControlMode(MCM_TORQUE_MODE) [真正切换]
 *
 * 位置模式：MC_ProgramPositionCommandMotor1() → MCI_ExecPositionCommand()
 *                                                ↓
 *                              MCI_ExecPositionCommand内部调用TC_MoveCommand()
 *                                                ↓
 *                                               TC_MoveCommand设置
 *                                               PositionControlRegulation = ENABLE
 */

/*===========================================================================
 * CIA402 需要的电机控制函数实现
 *===========================================================================*/

/* 启动电机 - 完整的ST SDK标准启动流程
 * 注意：这个函数只负责启动电机，不设置控制模式
 * 控制模式应该在调用此函数之前由motor_switch_to_xxx_mode设置好
 */
void motor_start(void)
{
    MC_StartMotor1();

    /* 等待alignment完成 */
    while (MC_GetAlignmentStatusMotor1() != TC_ALIGNMENT_COMPLETED)
        ;
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
    MC_StopMotor1();
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
 */
void motor_set_target_torque(int32_t torque_mnm)
{
    /* torque 单位是 mNm，转换为 A
     * 假设电机额定扭矩对应某个电流值，这里简化处理
     * 实际应根据电机参数转换
     */
    float_t torque_amp = (float_t)torque_mnm / 1000.0f;
    MC_ProgramTorqueRampMotor1_F(torque_amp, 0);
}

/* 设置目标位置 (pulse) - 标准MCP方式
 *
 * 内部调用链：
 * MC_ProgramPositionCommandMotor1() → MCI_ExecPositionCommand()
 *                                                ↓
 *                              TC_MoveCommand() [启用位置控制]
 *                                                ↓
 *                            PositionControlRegulation = ENABLE
 *
 * 注意：Duration=0表示立即到达（follow模式），>0表示梯形速度轨迹
 */
void motor_set_target_position(float position)
{
    /* position 单位是弧度(rad) */
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

/* 获取电机状态 */
MCI_State_t motor_get_state(void)
{
    return MC_GetSTMStateMotor1();
}

/* 获取当前位置 (pulse) */
int32_t motor_get_position(void)
{
    return (int32_t)MC_GetCurrentPosition1();
}

/* 获取当前速度 (RPM) */
int32_t motor_get_velocity(void)
{
    /* ST SDK返回int16_t，单位0.1 RPM */
    int16_t speed_01rpm = MC_GetMecSpeedAverageMotor1();
    return (int32_t)(speed_01rpm / 10);
}

/* 获取当前力矩 (mNm) */
int32_t motor_get_torque(void)
{
    /* 获取扭矩参考值（电气单位），转换为mNm */
    int16_t teref = MC_GetTerefMotor1();
    return (int32_t)(teref * 100); /* 简化：假设teref*100对应mNm */
}

/* 检查是否有故障 */
uint8_t motor_has_fault(void)
{
    uint16_t faults = MC_GetCurrentFaultsMotor1();
    return (faults != 0) ? 1 : 0;
}

/* 获取当前故障代码 */
uint16_t motor_get_current_faults(void)
{
    return MC_GetCurrentFaultsMotor1();
}

/* 获取历史故障代码 */
uint16_t motor_get_occurred_faults(void)
{
    return MC_GetOccurredFaultsMotor1();
}

/* 清除故障 */
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
 * 注意：Position模式不通过STC_SetControlMode切换，而是通过TC_MoveCommand
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

/* 检查电机是否已停止 */
uint8_t motor_is_stopped(void)
{
    /* 检查速度是否为0且斜坡已完成 */
    int32_t vel = motor_get_velocity();
    return (vel == 0 && MC_HasRampCompletedMotor1()) ? 1 : 0;
}
