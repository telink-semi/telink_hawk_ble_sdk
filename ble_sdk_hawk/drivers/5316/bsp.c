/********************************************************************************************************
 * @file     bsp.c
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

#include "bsp.h"
#include "clock.h"
#include "analog.h"

int load_tbl_cmd_set(const TBLCMDSET * pt, int size){
	int l=0;

	while (l<size) {
		unsigned int  cadr = ((unsigned int)0x800000) | pt[l].adr;
		unsigned char cdat = pt[l].dat;
		unsigned char ccmd = pt[l].cmd;
		unsigned char cvld =(ccmd & TCMD_UNDER_WR);
		ccmd &= TCMD_MASK;
		if (cvld) {
			if (ccmd == TCMD_WRITE) {
				write_reg8 (cadr, cdat);
			}
			else if (ccmd == TCMD_WAREG) {
				WriteAnalogReg (cadr, cdat);
			}
			else if (ccmd == TCMD_WAIT) {
				WaitUs (pt[l].adr*256 + cdat);
			}
		}
		l++;
	}
	return size;
}




void sub_wr_ana(unsigned int addr, unsigned char value, unsigned char e, unsigned char s)
{
	unsigned char v, mask, tmp1, target, tmp2;

	v = ReadAnalogReg(addr);
	mask = BIT_MASK_LEN(e - s + 1);
	tmp1 = value & mask;

	tmp2 = v & (~BIT_RNG(s,e));

	target = (tmp1 << s) | tmp2;
	WriteAnalogReg(addr, target);
}



void sub_wr(unsigned int addr, unsigned char value, unsigned char e, unsigned char s)
{
	unsigned char v, mask, tmp1, target, tmp2;

	v = read_reg8(addr);
	mask = BIT_MASK_LEN(e - s + 1);
	tmp1 = value & mask;

	tmp2 = v & (~BIT_RNG(s,e));

	target = (tmp1 << s) | tmp2;
	write_reg8(addr, target);
}
