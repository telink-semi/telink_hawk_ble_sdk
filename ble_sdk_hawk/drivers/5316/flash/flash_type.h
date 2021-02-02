/********************************************************************************************************
 * @file	flash_type.h
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
#ifndef __FLASH_COMPATIBLE_H__
#define __FLASH_COMPATIBLE_H__

#include "flash_mid1140c8.h"   	// GD25D10C & GD25D10B
#include "flash_mid134051.h"	// MD25D40DGIG
#include "flash_mid12325e.h"	// ZB25WD20A
#include "flash_mid13325e.h"	// ZB25WD40B

#define FLASH_LOCK_EN          0

/*
 * The declaration API in this file just used for internal.
 */

#if FLASH_LOCK_EN
/**
 * @brief This function reads the status of flash.
 * @param[in]  cmd - the cmd of read status
 * @param[in]  none
 * @return none
 */
_attribute_ram_code_ unsigned char flash_read_status(unsigned char cmd);

/**
 * @brief This function write the status of flash.
 * @param[in]  data - the value of status
 * @return     status
 */
_attribute_ram_code_ void flash_write_status(Flash_Status_Typedef_e type , unsigned short data);
#endif

#endif /* __FLASH_COMPATIBLE_H__ */
