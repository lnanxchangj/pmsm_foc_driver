/*
 * CIA402.h - CANopen CIA 402 State Machine
 *
 * CiA 402运动控制协议的状态机实现
 */

#ifndef CIA402_H
#define CIA402_H

#include <stdint.h>
#include "motor_control_interface.h"

/* CIA 402 状态机状态 */
typedef enum {
    STATE_NOT_READY_TO_SWITCH_ON = 0,
    STATE_SWITCH_ON_DISABLED,
    STATE_READY_TO_SWITCH_ON,
    STATE_SWITCHED_ON,
    STATE_OPERATION_ENABLED,
    STATE_QUICK_STOP_ACTIVE,
    STATE_FAULT_REACTION_ACTIVE,
    STATE_FAULT
} CIA402_State_t;

/* 控制字 (0x6040) 位定义 */
#define CW_SWITCH_ON         (1<<0)   /* 0: 不能运行 1: 运行使能请求 */
#define CW_ENABLE_VOLTAGE    (1<<1)   /* 0: 禁用电压 1: 使能电压 */
#define CW_QUICK_STOP        (1<<2)   /* 0: 快速停止 1: 继续运行 */
#define CW_ENABLE_OPERATION  (1<<3)   /* 0: 禁用运行 1: 使能运行 */
#define CW_NEW_SETPOINT      (1<<4)   /* 新设定点 (PP模式) */
#define CW_IMMEDIATE         (1<<5)   /* 立即更新 (PP模式) */
#define CW_FAULT_RESET       (1<<7)   /* 故障复位 */
#define CW_HALT              (1<<8)   /* 暂停 */

/* 状态字 (0x6041) 位定义 */
#define SW_READY_TO_SWITCH_ON    (1<<0)  /* 准备就绪 */
#define SW_SWITCHED_ON           (1<<1)  /* 已通电 */
#define SW_OPERATION_ENABLED     (1<<2)  /* 运行使能 */
#define SW_FAULT                 (1<<3)  /* 故障 */
#define SW_VOLTAGE_ENABLED       (1<<4)  /* 电压使能 */
#define SW_QUICK_STOP            (1<<5)  /* 快速停止 */
#define SW_SWITCH_ON_DISABLED    (1<<6)  /* 开关断开 */
#define SW_WARNING              (1<<7)  /* 警告 */
#define SW_MANUFACTURER_SPECIFIC (1<<8) /* 制造商特定 */
#define SW_REMOTE               (1<<9)  /* 远程 */
#define SW_TARGET_REACHED        (1<<10) /* 目标到达 */
#define SW_INTERNAL_LIMIT_ACTIVE (1<<11) /* 内部限制激活 */

/* 运行模式 */
#define MODE_NO_MODE         0
#define MODE_PROFILE_POSITION 1
#define MODE_VELOCITY        2
#define MODE_PROFILE_VELOCITY 3
#define MODE_TORQUE          4
#define MODE_HOMING          6
#define MODE_INTERPOLATED    7

/* 函数声明 */
void cia402_init(void);
void cia402_process(void);  /* 1ms周期调用 */

CIA402_State_t cia402_get_state(void);

#endif /* CIA402_H */
