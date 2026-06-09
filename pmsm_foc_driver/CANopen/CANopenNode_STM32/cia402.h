/* CIA402.h */
#ifndef CIA402_H
#define CIA402_H

#include "stdint.h"
#include "stdbool.h"

/* ============================================================
 * CiA 402 状态机状态定义
 * ============================================================ */
typedef enum {
    CIA402_NOT_READY_TO_SWITCH_ON = 0,  /* 上电初始化中 */
    CIA402_SWITCH_ON_DISABLED,          /* 禁止合闸 (默认待机态) */
    CIA402_READY_TO_SWITCH_ON,          /* 准备合闸 */
    CIA402_SWITCHED_ON,                 /* 已合闸 */
    CIA402_OPERATION_ENABLED,           /* 运行使能 (电机运转) */
    CIA402_QUICK_STOP_ACTIVE,           /* 快速停止中 */
    CIA402_FAULT_REACTION_ACTIVE,       /* 故障响应中 */
    CIA402_FAULT                        /* 故障 */
} CIA402_State_t;

/* ============================================================
 * 操作模式定义 (0x6060)
 * ============================================================ */
typedef enum {
    CIA402_MODE_NO_MODE  =  0,
    CIA402_MODE_PP       =  1,   /* Profile Position     轮廓位置 */
    CIA402_MODE_PV       =  3,   /* Profile Velocity     轮廓速度 */
    CIA402_MODE_PT       =  4,   /* Profile Torque       轮廓转矩 */
    CIA402_MODE_HM       =  6,   /* Homing               回零     */
    CIA402_MODE_CSP      =  8,   /* Cyclic Sync Position 周期同步位置 */
    CIA402_MODE_CSV      =  9,   /* Cyclic Sync Velocity 周期同步速度 */
    CIA402_MODE_CST      = 10,   /* Cyclic Sync Torque   周期同步转矩 */
} CIA402_ModeOfOperation_t;

/* ============================================================
 * 控制字 Bit 定义 (0x6040)
 * ============================================================ */
#define CW_SWITCH_ON          (1u << 0)   /* 合闸       */
#define CW_ENABLE_VOLTAGE     (1u << 1)   /* 使能电压   */
#define CW_QUICK_STOP         (1u << 2)   /* 快速停止 (0=触发) */
#define CW_ENABLE_OPERATION   (1u << 3)   /* 使能运行   */
#define CW_NEW_SETPOINT       (1u << 4)   /* PP模式新目标点 */
#define CW_CHANGE_SET_IMMED   (1u << 5)   /* PP模式立即更新 */
#define CW_ABS_REL            (1u << 6)   /* PP模式绝对/相对 */
#define CW_FAULT_RESET        (1u << 7)   /* 故障复位   */
#define CW_HALT               (1u << 8)   /* 暂停       */

/* ============================================================
 * 状态字 Bit 定义 (0x6041)
 * ============================================================ */
#define SW_READY_TO_SWITCH_ON (1u << 0)
#define SW_SWITCHED_ON        (1u << 1)
#define SW_OPERATION_ENABLED  (1u << 2)
#define SW_FAULT              (1u << 3)
#define SW_VOLTAGE_ENABLED    (1u << 4)
#define SW_QUICK_STOP         (1u << 5)   /* 1=正常, 0=快速停止激活 */
#define SW_SWITCH_ON_DISABLED (1u << 6)
#define SW_WARNING            (1u << 7)
#define SW_REMOTE             (1u << 9)
#define SW_TARGET_REACHED     (1u << 10)
#define SW_INTERNAL_LIMIT     (1u << 11)
#define SW_SETPOINT_ACK       (1u << 12)  /* PP模式应答 */
#define SW_HOMING_ATTAINED    (1u << 12)  /* HM模式回零完成 */

/* ============================================================
 * 状态字掩码 (低7位表示状态)
 * ============================================================ */
#define SW_STATE_MASK                    0x006Fu

/* 各状态对应的状态字低位模式
 * bit5 (SW_QUICK_STOP): 1=normal/not-in-quick-stop, 0=quick-stop-active or fault
 * CiA 402 DS: bit5=1 在所有非 QSA/FAULT/FAULT_REACTION 状态下
 */
#define SW_NOT_READY_TO_SWITCH_ON_VAL    0x0000u  /* bit5=1 */
#define SW_SWITCH_ON_DISABLED_VAL        0x0060u  /* bit6=1, bit5=1 */
#define SW_READY_TO_SWITCH_ON_VAL        0x0021u  /* bit5=1, bit0=1 */
#define SW_SWITCHED_ON_VAL               0x0023u  /* bit5=1, bit1=1, bit0=1 */
#define SW_OPERATION_ENABLED_VAL         0x0027u  /* bit5=1, bit2=1, bit1=1, bit0=1 */
#define SW_QUICK_STOP_ACTIVE_VAL         0x0007u  /* bit5=0, bit2=1, bit1=1, bit0=1 */
#define SW_FAULT_REACTION_ACTIVE_VAL     0x002Fu  /* bit5=1, bit3=1, bit2=1, bit1=1, bit0=1 */
#define SW_FAULT_VAL                     0x0008u  /* bit5=0, bit3=1 */

/* ============================================================
 * 速度单位换算
 * MCSDK 内部: rpm (float)
 * CIA402 0x60FF: 用户自定义，需求 1:1, 即 = 1 rpm (int32)
 * ============================================================ */
#define CIA402_VEL_SCALE    1.0f

/* ============================================================
 * 加速度单位换算 (0x6083)
 * 用户可以根据需求修改此缩放系数。
 * 默认 1.0f 表示 6083h 单位为 1 RPM/s。
 * 如果要使用 "65536 IU = 1 inc / s^2" 的换算，
 * 根据 1 圈 = 8000 inc，可以自行映射计算。
 * ============================================================ */
#define CIA402_ACC_SCALE    1.0f

/* 转矩单位: 0x6071/6077 单位为额定转矩的 0.1%，即 1000 = 100%
 * MCSDK 使用 A (float)
 * 1000 = NOMINAL_CURRENT_A
 */
#define CIA402_TRQ_SCALE    1000.0f

/* ============================================================
 * 容差窗口 (Windows) - 模拟 0x6067/0x606D
 * ============================================================ */
#define POS_WINDOW_DEFAULT      10      /* counts */
#define VEL_WINDOW_DEFAULT      10      /* rpm */
#define TRQ_WINDOW_DEFAULT      10      /* 0.1% units */
#define WINDOW_TIME_MS          10      /* 持续 10ms 认为到达 */
void CIA402_Init(void);
void CIA402_Process(void);          /* 放入 1ms 周期任务中调用 */

#endif /* CIA402_H */