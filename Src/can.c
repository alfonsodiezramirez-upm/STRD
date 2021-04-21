/*
 * Copyright © 2021 - present | can.c by Javinator9889
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see https://www.gnu.org/licenses/.
 *
 * Created by Javinator9889 on 10/04/21 - can.c.
 */
#include "can.h"
#include <stm32f4xx_hal.h>
#include <FreeRTOS.h>
#include <FreeRTOSConfig.h>
#include <task.h>
#include <stdint.h>

const uint32_t STD_ID = 0x6FF;

static volatile CAN_HandleTypeDef hcan1;
static volatile CAN_TxHeaderTypeDef tx_header;
static volatile CAN_RxHeaderTypeDef rx_header;
static volatile uint32_t tx_mailbox;

static uint8_t byte_sent = 0;
static uint8_t byte_recv = 0;

static volatile CAN_FilterTypeDef filter_config;

/**
  * @brief CAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN1_Init(void) {
    hcan1.Instance = CAN1;
    hcan1.Init.Prescaler = 21U;
    hcan1.Init.Mode = CAN_MODE_NORMAL;
    hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
    hcan1.Init.TimeSeg1 = CAN_BS1_12TQ;
    hcan1.Init.TimeSeg2 = CAN_BS2_4TQ;
    hcan1.Init.TimeTriggeredMode = DISABLE;
    hcan1.Init.AutoBusOff = DISABLE;
    hcan1.Init.AutoWakeUp = DISABLE;
    hcan1.Init.AutoRetransmission = DISABLE;
    hcan1.Init.ReceiveFifoLocked = DISABLE;
    hcan1.Init.TransmitFifoPriority = DISABLE;

    configASSERT(HAL_CAN_Init(&hcan1) == HAL_OK);
}

void CAN_init(void) {
    MX_CAN1_Init();
    // Message size of 1 byte
    tx_header.DLC = 1U;
    // Identifier to standard
    tx_header.IDE = CAN_ID_STD;
    // Data type to remote transmission
    tx_header.RTR = CAN_RTR_DATA;
    // Standard identifier
    tx_header.StdId = STD_ID;

    // Filter one (stack light blink)
    filter_config.FilterFIFOAssignment = CAN_FILTER_FIFO0;
    // ID we're looking for
    filter_config.FilterIdHigh = STD_ID << 5;
    filter_config.FilterIdLow = 0U;

    filter_config.FilterMaskIdHigh = 0U;
    filter_config.FilterMaskIdLow = 0U;

    filter_config.FilterScale = CAN_FILTERSCALE_32BIT;
    filter_config.FilterActivation = ENABLE;

    // Setup CAN filter
    HAL_CAN_ConfigFilter(&hcan1, &filter_config);
    HAL_CAN_Start(&hcan1);
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
}

void CAN_send(uint8_t b) {
    byte_sent = b;
    HAL_CAN_AddTxMessage(&hcan1, &tx_header, &byte_sent, &tx_mailbox);
}

uint8_t CAN_recv(void) {
    return byte_recv;
}

void CAN_Handle_IRQ(void) {
    HAL_CAN_IRQHandler(&hcan1);
    HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &rx_header, &byte_recv);
}
