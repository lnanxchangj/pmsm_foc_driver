/*
 * motor_control_interface.h
 *
 * CIA402 <-> ST Motor Control SDK 之间的接口层
 *
 * 遵循MCP协议标准方式：
 * - 所有控制指令通过MC_ProgramXxxMotor1系列函数缓冲命令
 * - 在MC_RunMotorControlTasks任务循环中自动执行
 */

#ifndef MOTOR_CONTROL_INTERFACE_H
#define MOTOR_CONTROL_INTERFACE_H

#include <stdint.h>
#include "mc_type.h"
#include "mc_interface.h"
#include "trajectory_ctrl.h"

/*===========================================================================
 * CIA402 电机控制函数 - 由本文件实现
 *===========================================================================*/

/* 电机启停控制 */
void motor_start(void);               /* 启动电机 */
void motor_stop(void);                /* 正常停止（减速停止） */
void motor_emergency_stop(void);      /* 紧急停止（立即切断输出） */

/* 速度模式控制 - 标准MCP方式
 *
 * 调用: MC_ProgramSpeedRampMotor1_F() → MCI_ExecSpeedRamp() [缓冲]
 *                                           ↓
 *                          在MC_RunMotorControlTasks()中自动执行
 *                                           ↓
 *                                          STC_SetControlMode(MCM_SPEED_MODE)
 */
void motor_set_target_velocity(int32_t vel);      /* 设置目标速度 (RPM) */
void motor_switch_to_velocity_mode(int32_t target_vel); /* 切换到速度模式 */

/* 扭矩模式控制 - 标准MCP方式
 *
 * 调用: MC_ProgramTorqueRampMotor1_F() → MCI_ExecTorqueRamp() [缓冲]
 *                                             ↓
 *                            在MC_RunMotorControlTasks()中自动执行
 *                                             ↓
 *                                            STC_SetControlMode(MCM_TORQUE_MODE)
 */
void motor_set_target_torque(int32_t torque_mnm); /* 设置目标力矩 (mNm) */
void motor_switch_to_torque_mode(int32_t target_torque); /* 切换到扭矩模式 */

/* 位置模式控制 - 标准MCP方式
 *
 * 调用: MC_ProgramPositionCommandMotor1() → MCI_ExecPositionCommand()
 *                                                      ↓
 *                                     TC_MoveCommand() [启用位置控制]
 *                                                      ↓
 *                                    PositionControlRegulation = ENABLE
 */
void motor_set_target_position(int32_t position);               /* 设置目标位置 (PPR), 立即执行 */
void motor_set_target_position_with_duration(int32_t position, float duration_sec); /* 带运动时间 */
void motor_switch_to_position_mode(int32_t target_pos); /* 切换到位置模式 */

/* 斜坡控制 */
void motor_stop_ramp(void);               /* 停止当前斜坡，保持当前值 */
uint8_t motor_ramp_completed(void);       /* 检查斜坡是否完成 */
uint8_t motor_is_stopped(void);           /* 检查电机是否已停止（速度为0且无运动） */

/*===========================================================================
 * 状态查询函数
 *===========================================================================*/

/* 获取当前值 */
int32_t motor_get_position(void);          /* 获取当前位置 (PPR) */
int32_t motor_get_velocity(void);          /* 获取当前速度 (RPM) */
int32_t motor_get_torque(void);            /* 获取当前力矩 (mNm) */

/* 获取控制状态 */
MC_ControlMode_t motor_get_control_mode(void);     /* 获取当前控制模式 */
MCI_State_t motor_get_state(void);                 /* 获取电机状态 */
MCI_CommandState_t motor_get_command_state(void);    /* 获取命令执行状态 */
PosCtrlStatus_t motor_get_position_control_status(void); /* 获取位置控制状态 */
AlignStatus_t motor_get_alignment_status(void);      /* 获取对准状态 */

/* 故障管理 */
uint8_t motor_has_fault(void);             /* 检查是否有故障 */
uint16_t motor_get_current_faults(void);    /* 获取当前故障代码 */
uint16_t motor_get_occurred_faults(void);    /* 获取历史故障代码 */
uint8_t motor_clear_fault(void);            /* 清除故障 */

#endif /* MOTOR_CONTROL_INTERFACE_H */
