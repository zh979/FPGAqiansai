#include "can.h"

__IO CAN_t can = {0};

CAN_HandleTypeDef hcan;

void MX_CAN_Init(void)
{
  hcan.Instance = CAN1;
  hcan.Init.Prescaler = 12;
  hcan.Init.Mode = CAN_MODE_NORMAL;
  hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan.Init.TimeSeg1 = CAN_BS1_4TQ;
  hcan.Init.TimeSeg2 = CAN_BS2_1TQ;
  hcan.Init.TimeTriggeredMode = DISABLE;
  hcan.Init.AutoBusOff = ENABLE;
  hcan.Init.AutoWakeUp = DISABLE;

  hcan.Init.AutoRetransmission = DISABLE;
  hcan.Init.ReceiveFifoLocked = DISABLE;
  hcan.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan) != HAL_OK)
  {
    Error_Handler();
  }
}

void HAL_CAN_MspInit(CAN_HandleTypeDef* canHandle)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(canHandle->Instance==CAN1)
  {
    __HAL_RCC_CAN1_CLK_ENABLE();

    __HAL_RCC_GPIOB_CLK_ENABLE();

    GPIO_InitStruct.Pin = GPIO_PIN_8;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    __HAL_AFIO_REMAP_CAN1_2();

    HAL_NVIC_SetPriority(USB_LP_CAN1_RX0_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(USB_LP_CAN1_RX0_IRQn);
  }
}

void HAL_CAN_MspDeInit(CAN_HandleTypeDef* canHandle)
{
  if(canHandle->Instance==CAN1)
  {
    __HAL_RCC_CAN1_CLK_DISABLE();

    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_8|GPIO_PIN_9);

    HAL_NVIC_DisableIRQ(USB_LP_CAN1_RX0_IRQn);
  }
}

void USER_CAN1_Filter_Init(void)
{
    CAN_FilterTypeDef sFilterConfig;

    __IO uint8_t id_o, im_o;
    __IO uint16_t id_l, id_h, im_l, im_h;
    id_o = (0x00);
    id_h = (uint16_t)((uint16_t)id_o >> 5);
    id_l = (uint16_t)((uint16_t)id_o << 11) | CAN_ID_EXT;
    im_o = (0x00);
    im_h = (uint16_t)((uint16_t)im_o >> 5);
    im_l = (uint16_t)((uint16_t)im_o << 11) | CAN_ID_EXT;

    sFilterConfig.FilterBank           = 0;
    sFilterConfig.FilterMode           = CAN_FILTERMODE_IDMASK;
    sFilterConfig.FilterScale          = CAN_FILTERSCALE_32BIT;
    sFilterConfig.FilterIdHigh         = id_h;
    sFilterConfig.FilterIdLow          = id_l;
    sFilterConfig.FilterMaskIdHigh     = im_h;
    sFilterConfig.FilterMaskIdLow      = im_l;
    sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
    sFilterConfig.FilterActivation     = ENABLE;
    sFilterConfig.SlaveStartFilterBank = 0;

    while (HAL_CAN_ConfigFilter(&hcan, &sFilterConfig) != HAL_OK);
}

#define CAN_TX_TIMEOUT_MS   2

volatile uint32_t can_tx_fail = 0;

void can_SendCmd(__IO uint8_t *cmd, uint8_t len)
{
    static uint32_t TxMailbox;
    __IO uint8_t i = 0, j = 0, k = 0, l = 0, packNum = 0;

    j = len - 2;

    while (i < j)
    {
        k = j - i;

        can.CAN_TxMsg.StdId = 0x00;
        can.CAN_TxMsg.ExtId = ((uint32_t)cmd[0] << 8) | (uint32_t)packNum;
        can.txData[0]       = cmd[1];
        can.CAN_TxMsg.IDE   = CAN_ID_EXT;
        can.CAN_TxMsg.RTR   = CAN_RTR_DATA;

        if (k < 8)
        {
            for (l = 0; l < k; l++, i++) { can.txData[l + 1] = cmd[i + 2]; }
            can.CAN_TxMsg.DLC = k + 1;
        }
        else
        {
            for (l = 0; l < 7; l++, i++) { can.txData[l + 1] = cmd[i + 2]; }
            can.CAN_TxMsg.DLC = 8;
        }

        {
            uint32_t t0 = HAL_GetTick();
            HAL_StatusTypeDef st;

            do {
                st = HAL_CAN_AddTxMessage(&hcan,
                                          (CAN_TxHeaderTypeDef *)(&can.CAN_TxMsg),
                                          (uint8_t *)(&can.txData),
                                          &TxMailbox);
            } while (st != HAL_OK && (HAL_GetTick() - t0) < CAN_TX_TIMEOUT_MS);

            if (st != HAL_OK) {
                can_tx_fail++;
                HAL_CAN_AbortTxRequest(&hcan, CAN_TX_MAILBOX0 |
                                              CAN_TX_MAILBOX1 |
                                              CAN_TX_MAILBOX2);
                return;
            }
        }
        ++packNum;
    }
}

