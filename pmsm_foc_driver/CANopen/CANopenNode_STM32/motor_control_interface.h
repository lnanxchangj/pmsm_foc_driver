/*
 * motor_control_interface.h
 *
 * CIA402 <-> ST Motor Control SDK 之间的接口层
 */

#ifndef MOTOR_CONTROL_INTERFACE_H
#define MOTOR_CONTROL_INTERFACE_H

#include <stdint.h>

/* 初始化电机控制接口 */
void motor_control_interface_init(void);

/* CIA402 调用的函数 - 由本文件实现 */
void motor_set_target_velocity(int32_t vel);      /* 设置目标速度 (RPM) */
void motor_set_target_position(int32_t pos);     /* 设置目标位置 (PPR) */
void motor_set_target_torque(int16_t torque);    /* 设置目标力矩 (0.1%) */

void motor_start(void);                          /* 启动电机 */
void motor_stop(void);                          /* 正常停止 */
void motor_emergency_stop(void);                 /* 紧急停止 */

int32_t motor_get_position(void);               /* 获取当前位置 (PPR) */
int32_t motor_get_velocity(void);                /* 获取当前速度 (RPM) */
int16_t motor_get_torque(void);                 /* 获取当前力矩 (0.1%) */

uint8_t motor_has_fault(void);                  /* 检查故障 */
void motor_clear_fault(void);                    /* 清除故障 */

#endif /* MOTOR_CONTROL_INTERFACE_H */
