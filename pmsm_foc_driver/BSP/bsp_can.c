/* BSP CAN - Board Support Package for CAN testing */
#include "bsp_can.h"
#include "main.h"

static uint32_t txMailbox;
static uint8_t testCounter = 0;

/* CAN TX header - standard ID, 8 bytes data */
static CAN_TxHeaderTypeDef TxHeader = {
    .StdId = 0x123,          /* Test ID */
    .ExtId = 0,
    .IDE = CAN_ID_STD,
    .RTR = CAN_RTR_DATA,
    .DLC = 8,
    .TransmitGlobalTime = DISABLE
};

void BSP_CAN_Init(void)
{
    /* CAN filter: accept all IDs */
    CAN_FilterTypeDef sFilterConfig;
    sFilterConfig.FilterBank = 0;
    sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
    sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
    sFilterConfig.FilterIdHigh = 0x0000;
    sFilterConfig.FilterIdLow = 0x0000;
    sFilterConfig.FilterMaskIdHigh = 0x0000;
    sFilterConfig.FilterMaskIdLow = 0x0000;
    sFilterConfig.FilterFIFOAssignment = CAN_FilterFIFO0;
    sFilterConfig.FilterActivation = ENABLE;
    sFilterConfig.SlaveStartFilterBank = 14;

    if (HAL_CAN_ConfigFilter(&hcan1, &sFilterConfig) != HAL_OK)
    {
        Error_Handler();
    }

    /* Start CAN */
    if (HAL_CAN_Start(&hcan1) != HAL_OK)
    {
        Error_Handler();
    }

    /* Enable TX interrupt */
    if (HAL_CAN_ActivateNotification(&hcan1, CAN_IT_TX_MAILBOX_EMPTY) != HAL_OK)
    {
        Error_Handler();
    }
}

void BSP_CAN_SendTestFrame(void)
{
    uint8_t txData[8] = {0};

    /* Fill test data */
    txData[0] = 0x01;           /* Header */
    txData[1] = testCounter++;  /* Incrementing counter */
    txData[2] = 0xAA;
    txData[3] = 0xBB;
    txData[4] = 0xCC;
    txData[5] = 0xDD;
    txData[6] = 0xEE;
    txData[7] = 0xFF;

    /* Send CAN frame */
    if (HAL_CAN_AddTxMessage(&hcan1, &TxHeader, txData, &txMailbox) == HAL_OK)
    {
        /* Success - LED toggle or counter increment could go here */
    }
}

void BSP_CAN_Process(void)
{
    /* Nothing needed for TX-only test */
}
