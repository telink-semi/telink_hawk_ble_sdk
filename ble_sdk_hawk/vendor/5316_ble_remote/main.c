/********************************************************************************************************
 * @file     main.c
 *
 * @brief    for TLSR chips
 *
 * @author	 BLE Group
 * @date     May. 12, 2018
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
#include <tl_common.h>
#include "drivers.h"
#include "stack/ble/ble.h"
#include "../common/user_config.h"

extern void rc_ir_irq_prc(void);

_attribute_ram_code_ void irq_handler(void)
{
#if(REMOTE_IR_ENABLE)
	rc_ir_irq_prc();
#endif

	irq_blt_sdk_handler();

}

int main(void){

	blc_pm_select_internal_32k_crystal();

	cpu_wakeup_init();


	clock_init(SYS_CLK_16M_Crystal);

	gpio_init();

	#if(RC_BTN_ENABLE)
		deep_wakeup_proc();
	#endif

	rf_drv_init(RF_MODE_BLE_1M);

	user_init ();

    irq_enable();

	while (1) {
	#if (MODULE_WATCHDOG_ENABLE)
		wd_clear(); //clear watch dog
	#endif
		main_loop ();
	}
}

