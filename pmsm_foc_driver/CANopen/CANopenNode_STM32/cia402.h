/*
 * CIA402.h - CANopen CIA 402 State Machine
 *
 * 基于 cia402device 库的标准化实现
 */

#ifndef CIA402_H
#define CIA402_H

#include <stdint.h>

/* 引入标准 CIA402 设备库 */
#include "cia402device.h"

/* 运行模式 */
#define MODE_NO_MODE         0
#define MODE_PROFILE_POSITION 1
#define MODE_VELOCITY        2
#define MODE_PROFILE_VELOCITY 3
#define MODE_TORQUE          4
#define MODE_HOMING          6
#define MODE_INTERPOLATED    7

/* CIA402 Handle - 封装库类型并添加扩展字段 */
typedef struct {
    cia402_axis_t axis;
    int8_t target_mode;          /* 目标运行模式 */
    int8_t last_mode;            /* 上一次运行的模式 */
    int32_t switch_target_value; /* 模式切换时的目标值 */
} CIA402_Handle_t;

/* 状态别名 - 兼容旧代码 */
#define CIA402_State_t cia402_axis_state_t
#define STATE_NOT_READY_TO_SWITCH_ON NOT_READY_TO_SWITCH_ON
#define STATE_SWITCH_ON_DISABLED     SWITCH_ON_DISABLED
#define STATE_READY_TO_SWITCH_ON     READY_TO_SWITCH_ON
#define STATE_SWITCHED_ON            SWITCHED_ON
#define STATE_OPERATION_ENABLED      OPERATION_ENABLED
#define STATE_QUICK_STOP_ACTIVE      QUICK_STOP_ACTIVE
#define STATE_FAULT_REACTION_ACTIVE  FAULT_REACTION_ACTIVE
#define STATE_FAULT                  FAULT

/* 函数声明 */
void cia402_init(void);
void cia402_process(void);
CIA402_State_t cia402_get_state(void);

/* 获取当前运行模式 */
int8_t cia402_get_mode(void);

#endif /* CIA402_H */
