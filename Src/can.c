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
#include "utils.h"
#ifdef NODE_2
#include "node1.h"
#endif

const uint32_t STD_ID1 = 0x6FA;
const uint32_t STD_ID2 = 0x6FB;
const uint32_t HFILTER_ID = 0x6FF << 5;

#ifdef NODE_2
const uint32_t HFILTER_MASK = 0x7F0 << 5;
#endif

static volatile CAN_HandleTypeDef hcan1;
#ifndef NODE_2
static volatile CAN_TxHeaderTypeDef tx_header;
static volatile CAN_TxHeaderTypeDef tx_header2;
#else
static volatile CAN_RxHeaderTypeDef rx_header;
#endif
static volatile uint32_t tx_mailbox;

static volatile uint8_t byte_sent = 0;
static volatile uint8_t byte_recv = 0;
static volatile float float_recv = .0F;

static volatile CAN_FilterTypeDef filter_config;

/**
  * @brief CAN1 Initialization Function - extracted from main.
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

/**
 * @brief Initializes CANBus communications. This function
 * must be called early during initialization at main() as
 * the CAN functions won't work (and will block forever)
 * if called.
 */
void CAN_init(void) {
    MX_CAN1_Init();
#ifndef NODE_2
    // Message size of 1 byte
    tx_header.DLC = 1U;
    // Identifier to standard
    tx_header.IDE = CAN_ID_STD;
    // Data type to remote transmission
    tx_header.RTR = CAN_RTR_DATA;
    // Standard identifier
    tx_header.StdId = STD_ID1;

    // Message size of 4 bytes (float)
    tx_header2.DLC = 4U;
    // Identifier to standard
    tx_header2.IDE = CAN_ID_STD;
    // Data type to remote transmission
    tx_header2.RTR = CAN_RTR_DATA;
    // Standard identifier
    tx_header2.StdId = STD_ID2;
#endif

    // Filter one (stack light blink)
    filter_config.FilterFIFOAssignment = CAN_FILTER_FIFO0;
    // ID we're looking for
    filter_config.FilterIdHigh = HFILTER_ID;
    filter_config.FilterIdLow = 0U;

#ifndef NODE_2
    filter_config.FilterMaskIdHigh = 0U;
#else
    filter_config.FilterMaskIdHigh = HFILTER_MASK;
    filter_config.FilterMode = CAN_FILTERMODE_IDMASK;
#endif
    filter_config.FilterMaskIdLow = 0U;

    filter_config.FilterScale = CAN_FILTERSCALE_32BIT;
    filter_config.FilterActivation = ENABLE;

    // Setup CAN filter
    HAL_CAN_ConfigFilter(&hcan1, &filter_config);
    HAL_CAN_Start(&hcan1);
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
}

/**
 * @brief Sends a byte through the CANBus using the #STD_ID1
 *        message identifier.
 * 
 *        Note: this method will do nothing if NODE_2 is defined.
 * 
 * @param b the byte to send.
 */
void CAN_sendi(uint8_t b) {
#ifndef NODE_2
    byte_sent = b;
    HAL_CAN_AddTxMessage(&hcan1, &tx_header, &byte_sent, &tx_mailbox);
#endif
}

/**
 * @brief Sends a float through the CANBus using the #STD_ID2
 *        message identifier.
 * 
 *        Note: this method will do nothing if NODE_2 is defined.
 * 
 * @param value the float value to send.
 * @see   f2b(float, uint8_t*)
 */
void CAN_sendf(float value) {
#ifndef NODE_2
    uint8_t bytes[4];
    f2b(value, bytes);
    HAL_CAN_AddTxMessage(&hcan1, &tx_header2, &bytes[0], &tx_mailbox);
#endif
}

/**
 * @brief When a message arrives, the received byte (if any) is stored in
 *        a private variable. Use this method to recover its value.
 * 
 *        Notice that this value will only be updated when the received
 *        message ID matches the #STD_ID1.
 * 
 * @return uint8_t the stored byte.
 */
uint8_t CAN_recv(void) {
    return byte_recv;
}

/**
 * @brief When a message arrives, the received float (if any) is stored in
 *        a private variable. Use this method to recover its value.
 * 
 *        Notice that this value will only be updated when the received
 *        message ID matches the #STD_ID2.
 * 
 * @return float - the stored float.
 */
float CAN_recvf(void) {
    return float_recv;
}

/**
 * @brief This method must be called if the board wants to receive
 *        CANBus messages during the CANBus interruption routine.
 * 
 *        By filtering the ID, identifies whether the received array
 *        is either a single byte or a float value.
 * 
 *        In addition, this method sets a flag at the respective
 *        protected objects indicating that a new message is received
 *        and is ready to be used (notice that this method is called
 *        from an IRQ, so the processing must be as efficient as
 *        possible. In this function, setting a flag is easy and
 *        not blocking - at least not as much as changing a lock/semaphore).
 * 
 *        The affected protected objects are the ones that store the
 *        SPEED and the DISTANCE.
 * 
 * @see node1.h
 */
void CAN_Handle_IRQ(void) {
    HAL_CAN_IRQHandler(&hcan1);
#ifdef NODE_2
    uint8_t bytes[4];
    HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &rx_header, &bytes);
    if (rx_header.StdId == STD_ID1) {
        byte_recv = bytes[0];
        SPEED_set_recv();
    }
    if (rx_header.StdId == STD_ID2) {
        float_recv = b2f(&bytes[0]);
        DISTANCE_set_recv();
    }
#endif
}
