/********************************************************************************************************
 * @file     hci_cmd.h
 *
 * @brief    for TLSR chips
 *
 * @author	 BLE Group
 * @date     Sep. 18, 2015
 *
 * @par      Copyright (c) Telink Semiconductor (Shanghai) Co., Ltd.
 *           All rights reserved.
 *
 *			 The information contained herein is confidential and proprietary property of Telink
 * 		     Semiconductor (Shanghai) Co., Ltd. and is available under the terms
 *			 of Commercial License Agreement between Telink Semiconductor (Shanghai)
 *			 Co., Ltd. and the licensee in separate contract or the terms described here-in.
 *           This heading MUST NOT be removed from this file.
 *
 * 			 Licensees are granted free, non-transferable use of the information in this
 *			 file under Mutual Non-Disclosure Agreement. NO WARRENTY of ANY KIND is provided.
 *
 *******************************************************************************************************/
#ifndef HCI_CMD_H_
#define HCI_CMD_H_

#include "tl_common.h"
#include "stack/ble/ble.h"

/* Type Define */
/**
 *  @brief  Return Parameters for "HCI LE Read PHY Command"
 */
typedef struct {
	u8 status;
	u8 handle[2];
	u8 tx_phy;
	u8 rx_phy;
} hci_le_readPhyCmd_retParam_t;


/**
 *  @brief  Command Parameters for "HCI LE Set PHY Command"
 */
typedef struct {
	u16 connHandle;
	u8 	all_phys;
	u8 	tx_phys;
	u8 	rx_phys;
	u16 phy_options;
} hci_le_setPhyCmd_param_t;


/* Function declaration */
ble_sts_t blc_hci_le_setPhy(hci_le_setPhyCmd_param_t* para);

#endif /* HCI_CMD_H_ */
