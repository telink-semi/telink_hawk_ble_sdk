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
#include "../common/blt_fw_sign.h"

#include "../link_layer/rf_ll.h"
#include "rf_2p4g.h"


#if(REMOTE_IR_ENABLE)
	extern void rc_ir_irq_prc(void);
#endif

_attribute_ram_code_ void irq_handler(void)
{
	if(sys_status.sys_mode == SYS_2P4G_MODE){  //2p4g mode

		u16  src_rf = reg_rf_irq_status;
		if(src_rf & FLD_RF_IRQ_RX){
			DBG_CHN2_TOGGLE;
			irq_device_rx();
		}

		if(src_rf & FLD_RF_IRQ_TX){
			DBG_CHN3_TOGGLE;
			irq_device_tx();
		}

	}
	else{

	#if(REMOTE_IR_ENABLE)
		rc_ir_irq_prc();
	#endif
		irq_blt_sdk_handler();

	}

}

int main(void){

	blc_pm_select_internal_32k_crystal();
//	blc_pm_select_external_32k_crystal();

	/***********************************************
	 * if the bin size is less than 48K, we recommend using this setting.
	 */
	#if (FLASH_SIZE_OPTION == FLASH_SIZE_OPTION_128K) ///FLASH_SIZE_OPTION_128K
		bls_ota_setFirmwareSizeAndOffset(48, 0x10000);///default : ota_firmware_size_k=128;ota_program_bootAddr=0x20000; it is for hawk 128K flash
		bls_smp_configParingSecurityInfoStorageAddr(0x1C000);
	#endif

	cpu_wakeup_init();

	#if (CLOCK_SYS_CLOCK_HZ == 16000000)
		clock_init(SYS_CLK_16M_Crystal);
	#elif (CLOCK_SYS_CLOCK_HZ == 32000000)
		clock_init(SYS_CLK_32M_Crystal);
	#elif (CLOCK_SYS_CLOCK_HZ == 48000000)
		clock_init(SYS_CLK_48M_Crystal);
	#endif

	gpio_init();

	u8 system_mode = analog_read(PM_REG_SYS_MODE);
	sys_status.sys_mode = (system_mode & (SYS_BLE_MODE_1M | SYS_2P4G_MODE)) ? system_mode : SYS_2P4G_MODE; // default is 2p4g mode;

	/* load customized freq_offset CAP value and TP value.*/
	blc_app_loadCustomizedParameters();//do this operation before "rf_drv_init"

	rf_drv_init(sys_status.sys_mode);

	user_init();

    irq_enable();

	while (1) {
	#if (MODULE_WATCHDOG_ENABLE)
		wd_clear(); //clear watch dog
	#endif
		main_loop ();
	}
}

