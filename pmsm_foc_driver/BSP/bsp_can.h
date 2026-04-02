/* BSP CAN - Board Support Package for CAN testing */
#ifndef __BSP_CAN_H__
#define __BSP_CAN_H__

#include "can.h"

void BSP_CAN_Init(void);
void BSP_CAN_SendTestFrame(void);
void BSP_CAN_Process(void);

#endif /* __BSP_CAN_H__ */
