/********************************************************************************************************
 * @file     analog.c
 *
 * @brief    This is the source file for TLSR8232
 *
 * @author	 BLE Group
 * @date     May 8, 2018
 *
 * @par      Copyright (c) 2018, Telink Semiconductor (Shanghai) Co., Ltd.
 *           All rights reserved.
 *
 *           The information contained herein is confidential property of Telink
 *           Semiconductor (Shanghai) Co., Ltd. and is available under the terms
 *           of Commercial License Agreement between Telink Semiconductor (Shanghai)
 *           Co., Ltd. and the licensee or the terms described here-in. This heading
 *           MUST NOT be removed from this file.
 *
 *           Licensees are granted free, non-transferable use of the information in this
 *           file under Mutual Non-Disclosure Agreement. NO WARRENTY of ANY KIND is provided.
 *
 *******************************************************************************************************/

#include "analog.h"

#include "compiler.h"
#include "register.h"
#include "irq.h"

/**
 * @brief      This function serves to wait for analog register ready.
 * @param[in]  none.
 * @return     none.
 */
static inline void analog_wait(){
	while(reg_ana_ctrl & FLD_ANA_BUSY){}
}

/**
 * @brief      This function serves to analog register read.
 * @param[in]  addr - address need to be read.
 * @return     the result of read.
 */
_attribute_ram_code_ unsigned char analog_read(unsigned char addr){
	unsigned char r = irq_disable();

	reg_ana_addr = addr;
	reg_ana_ctrl = (FLD_ANA_START);
//   Can't use one line setting "reg_ana_ctrl32 = ((FLD_ANA_START | FLD_ANA_RSV) << 16) | addr;"
//   This will fail because of time sequence and more over size is bigger
	analog_wait();

	unsigned char data = reg_ana_data;

	reg_ana_ctrl = 0;		// finish

	irq_restore(r);

	return data;
}

/**
 * @brief      This function serves to analog register write.
 * @param[in]  addr - address need to be write.
 * @param[in]  v - the value need to be write.
 * @return     none.
 */
_attribute_ram_code_ void analog_write(unsigned char addr, unsigned char v){
	unsigned char r = irq_disable();

	reg_ana_addr = addr;
	reg_ana_data = v;
	reg_ana_ctrl = (FLD_ANA_START | FLD_ANA_RW);
//	 Can't use one line setting "reg_ana_ctrl32 = ((FLD_ANA_START | FLD_ANA_RW) << 16) | (v << 8) | addr;"
//   This will fail because of time sequence and more over size is bigger
	analog_wait();
	reg_ana_ctrl = 0; 		// finish

	
	irq_restore(r);
}

void analog_read_multi(unsigned char addr, unsigned char *v, int len){
	unsigned char r = irq_disable();

	reg_ana_ctrl = 0;		// issue clock
	reg_ana_addr = addr;
	while(len--){
		reg_ana_ctrl = FLD_ANA_CYC | FLD_ANA_START;
		analog_wait();

		*v++ = reg_ana_data;

	}
	reg_ana_ctrl = 0; 		// finish

	irq_restore(r);
}

void analog_write_multi(unsigned char addr, unsigned char *v, int len){
	unsigned char r = irq_disable();

	reg_ana_addr = addr;
	while(len--){
		reg_ana_data = *v++;

		reg_ana_ctrl = FLD_ANA_CYC | FLD_ANA_START | FLD_ANA_RW; 	// multi write
		analog_wait();
	}
	reg_ana_ctrl = 0; 		// finish

	irq_restore(r);
}


