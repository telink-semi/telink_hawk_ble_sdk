/********************************************************************************************************
 * @file	flash_mid1140c8.h
 *
 * @brief	This is the source file for TLSR8232
 *
 * @author	Driver Group
 * @date	May 8, 2018
 *
 * @par     Copyright (c) 2018, Telink Semiconductor (Shanghai) Co., Ltd. ("TELINK")
 *          All rights reserved.
 *
 *          Redistribution and use in source and binary forms, with or without
 *          modification, are permitted provided that the following conditions are met:
 *
 *              1. Redistributions of source code must retain the above copyright
 *              notice, this list of conditions and the following disclaimer.
 *
 *              2. Unless for usage inside a TELINK integrated circuit, redistributions
 *              in binary form must reproduce the above copyright notice, this list of
 *              conditions and the following disclaimer in the documentation and/or other
 *              materials provided with the distribution.
 *
 *              3. Neither the name of TELINK, nor the names of its contributors may be
 *              used to endorse or promote products derived from this software without
 *              specific prior written permission.
 *
 *              4. This software, with or without modification, must only be used with a
 *              TELINK integrated circuit. All other usages are subject to written permission
 *              from TELINK and different commercial license may apply.
 *
 *              5. Licensee shall be solely responsible for any claim to the extent arising out of or
 *              relating to such deletion(s), modification(s) or alteration(s).
 *
 *          THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 *          ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 *          WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *          DISCLAIMED. IN NO EVENT SHALL COPYRIGHT HOLDER BE LIABLE FOR ANY
 *          DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 *          (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *          LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 *          ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *          (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 *          SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *******************************************************************************************************/
#ifndef __MID1140C8_H__
#define __MID1140C8_H__

/*
 * MID = 0x1140c8 Flash include GD25D10c GD25D10B.the function used in driver are all the sane.
 */


#include "../compiler.h"
#include "flash_type.h"
#include "../flash.h"
#include "../spi_i.h"
#include "../irq.h"
#include "../timer.h"

typedef enum{
	FLASH_LOCK_NONE_MID1140C8        =   0,
	FLASH_LOCK_LOW_120K_MID1140C8    =   0x04,     //000000H-01DFFFH
	FLASH_LOCK_LOW_112K_MID1140C8    =   0x08,     //000000H-01BFFFH
	FLASH_LOCK_LOW_96K_MID1140C8     =   0x0c,     //000000h¨C017FFFh
	FLASH_LOCK_LOW_64K_MID1140C8   	 =   0x10,     //000000h¨C00FFFFh
	FLASH_LOCK_LOW_128K_MID1140C8    =   0x14,     //000000h¨C01FFFFh
}mid1140c8_lock_block_e;

typedef enum{
	FLASH_WRITE_STATUS_BP_MID1140C8	=	0x1c,
}mid1140c8_write_status_bit_e;

#if FLASH_LOCK_EN

/**
 * @brief This function write the status of flash.
 * @param[in]  the value of status
 * @return status
 */
_attribute_ram_code_ void flash_write_status_mid1140c8(unsigned char data, mid1140c8_write_status_bit_e bit);

/**
 * @brief This function reads the status of flash.
 * @param[in]  none
 * @return none
 */
_attribute_ram_code_ unsigned char flash_read_status_mid1140c8(void);

/**
 * @brief This function serves to protect data for flash.
 * @param[in]   data - refer to Driver API Doc or Flash_ProtectedAreaDef in "flash.h".
 * @return none
 */
_attribute_ram_code_ void flash_lock_mid1140c8(mid1140c8_lock_block_e data);


/**
 * @brief This function serves to protect data for flash.
 * @return none
 */
_attribute_ram_code_ void flash_unlock_mid1140c8(void);
#endif

#endif /* __MID1140C8_H__ */
