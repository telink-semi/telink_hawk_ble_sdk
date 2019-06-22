/********************************************************************************************************
 * @file     feature_ll_state.c 
 *
 * @brief    for TLSR chips
 *
 * @author	 public@telink-semi.com;
 * @date     May. 10, 2018
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
#include "app.h"
#include <stack/ble/ble.h>
#include "tl_common.h"
#include "drivers.h"
#include "app_config.h"
#include "vendor/common/blt_led.h"
#include "vendor/common/blt_soft_timer.h"
#include "vendor/common/blt_common.h"

#if (   FEATURE_TEST_MODE == TEST_ADVERTISING_ONLY  )



#define RX_FIFO_SIZE	64
#define RX_FIFO_NUM		8

#define TX_FIFO_SIZE	40
#define TX_FIFO_NUM		16






u8 		 	blt_rxfifo_b[RX_FIFO_SIZE * RX_FIFO_NUM] = {0};
my_fifo_t	blt_rxfifo = {
												RX_FIFO_SIZE,
												RX_FIFO_NUM,
												0,
												0,
												blt_rxfifo_b,};


u8 		 	blt_txfifo_b[TX_FIFO_SIZE * TX_FIFO_NUM] = {0};
my_fifo_t	blt_txfifo = {
												TX_FIFO_SIZE,
												TX_FIFO_NUM,
												0,
												0,
												blt_txfifo_b,};





void	task_connect (u8 e, u8 *p, int n)
{

}


void	task_terminate (u8 e, u8 *p, int n)
{

}







void  func_suspend_enter (u8 e, u8 *p, int n)
{

}

#define		MY_RF_POWER_INDEX					RF_POWER_7P9dBm

_attribute_ram_code_ void  func_suspend_exit (u8 e, u8 *p, int n)
{
	rf_set_power_level_index (MY_RF_POWER_INDEX);
}







void feature_linklayer_state_test_init(void)
{
	/* load customized freq_offset CAP value and TP value.*/
	blc_app_loadCustomizedParameters();


	u8  mac_public[6];
	u8  mac_random_static[6];
	blc_initMacAddress(CFG_ADR_MAC, mac_public, mac_random_static);

	rf_set_power_level_index (MY_RF_POWER_INDEX);

	////// Controller Initialization  //////////
	blc_ll_initBasicMCU(mac_public);   //mandatory


#if (FEATURE_TEST_MODE == TEST_ADVERTISING_ONLY)

	printf("\n\rtst adv only\n");
	blc_ll_initAdvertising_module(mac_public);


	u8 tbl_advData[] = {
		0x08, 0x09, 't', 'e', 's', 't', 'a', 'd', 'v',
		0x02, 0x01, 0x05,
	};
	u8	tbl_scanRsp [] = {
		0x08, 0x09, 'T', 'E', 'S', 'T', 'A', 'D', 'V',	//scan name
	};
	bls_ll_setAdvData( (u8 *)tbl_advData, sizeof(tbl_advData) );
	bls_ll_setScanRspData( (u8 *)tbl_scanRsp, sizeof(tbl_scanRsp));

	u8 status = bls_ll_setAdvParam( ADV_INTERVAL_100MS, ADV_INTERVAL_100MS, \
									ADV_TYPE_NONCONNECTABLE_UNDIRECTED, OWN_ADDRESS_PUBLIC, \
									 0,  NULL,  BLT_ENABLE_ADV_ALL, ADV_FP_NONE);


	if(status != BLE_SUCCESS){     //adv setting err
		write_reg8(0x8000, 0x11);  //debug
		while(1);
	}

	blc_ll_setAdvCustomedChannel(37, 38, 39);
	bls_ll_setAdvEnable(1);  //adv enable

#endif




#if(BLE_PM_ENABLE)
	blc_ll_initPowerManagement_module();

	bls_pm_setSuspendMask (SUSPEND_ADV | SUSPEND_CONN);

	//bls_app_registerEventCallback (BLT_EV_FLAG_SUSPEND_ENTER, &func_suspend_enter);
//	bls_app_registerEventCallback (BLT_EV_FLAG_SUSPEND_EXIT, &func_suspend_exit);
#else
	bls_pm_setSuspendMask (SUSPEND_DISABLE);
#endif

}



#endif


