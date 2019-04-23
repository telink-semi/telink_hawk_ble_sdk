/********************************************************************************************************
 * @file     myprintf.c
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

#include "tl_common.h"
#include "myprintf.h"
#include "drivers.h"

#if(PRINT_DEBUG_INFO)
#define va_start(ap,v)    (ap = (char *)((int)&v + sizeof(v)))
#define va_arg(ap,t)      ((t *)(ap += sizeof(t)))[-1]

#ifndef		BIT_INTERVAL
#define		BIT_INTERVAL	(CLOCK_SYS_CLOCK_HZ/115200)
#endif

static void uart_put_char(u8 byte){
	u8 j = 0;
	u32 t1 = 0,t2 = 0;
	
	REG_ADDR8(0x582+((DEBUG_INFO_TX_PIN>>8)<<3)) &= ~(DEBUG_INFO_TX_PIN & 0xff) ;//Enable output


	u32 pcTxReg = (0x583+((DEBUG_INFO_TX_PIN>>8)<<3));//register GPIO output
	u8 tmp_bit0 = read_reg8(pcTxReg) & (~(DEBUG_INFO_TX_PIN & 0xff));
	u8 tmp_bit1 = read_reg8(pcTxReg) | (DEBUG_INFO_TX_PIN & 0xff);


	u8 bit[10] = {0};
	bit[0] = tmp_bit0;
	bit[1] = (byte & 0x01)? tmp_bit1 : tmp_bit0;
	bit[2] = ((byte>>1) & 0x01)? tmp_bit1 : tmp_bit0;
	bit[3] = ((byte>>2) & 0x01)? tmp_bit1 : tmp_bit0;
	bit[4] = ((byte>>3) & 0x01)? tmp_bit1 : tmp_bit0;
	bit[5] = ((byte>>4) & 0x01)? tmp_bit1 : tmp_bit0;
	bit[6] = ((byte>>5) & 0x01)? tmp_bit1 : tmp_bit0;
	bit[7] = ((byte>>6) & 0x01)? tmp_bit1 : tmp_bit0;
	bit[8] = ((byte>>7) & 0x01)? tmp_bit1 : tmp_bit0;
	bit[9] = tmp_bit1;

	//u8 r = irq_disable();
	t1 = read_reg32(0x740);
	for(j = 0;j<10;j++)
	{
		t2 = t1;
		while(t1 - t2 < BIT_INTERVAL){
			t1  = read_reg32(0x740);
		}
		write_reg8(pcTxReg,bit[j]);        //send bit0
	}
	//irq_restore(r);
}
void printb(unsigned char byte)
{
	unsigned char half_byte_high = byte >> 4;
	unsigned char half_byte_low = byte & 0x0f;

	if (half_byte_high > 9) {                                       //if high-half-byte is 10/11/12/13/14/15 
        half_byte_high += 'W';                                      //10+'W'='a'/11+'W'='b'/.../15+'W'='f'
	}
	else {
		half_byte_high += '0';
	}
	uart_put_char(half_byte_high);

	if (half_byte_low > 9) {                                       //if high-half-byte is 10/11/12/13/14/15 
        half_byte_low += 'W';                                      //10+'W'='a'/11+'W'='b'/.../15+'W'='f'
	}
	else {
		half_byte_low += '0';
	}
	uart_put_char(half_byte_low);
}

static void sysPutNumber(int n, int len)
{
    int len_adj = (len > 4 || len < 1) ? 4 : len;
    int i = 0;
    int tmp = n;

    for (i = len_adj - 1; i >= 0; i--) {
        tmp = n >> (i*8);
        printb(tmp);
    }
}

static char *FormatItem(char *f, int n)
{
    char ch;
    int fieldwidth = 0;
    int flag = 0;

    while (ch = *f++) {
        if ((ch >= '0') && (ch <= '9')) {
            fieldwidth = (fieldwidth * 10) + (ch - '0');
        }
        else {
        	switch (ch) {
        		case 'x': 
        		    //this case is the same as follow
        		case 'X':
                    flag = 16;
                    break;
                default:
                    flag = -1;
                    uart_put_char('*');
                    break;
        	}
        }
        if (flag != 0) {
            break;
        }
    }
    sysPutNumber(n, fieldwidth);
    return f;   
}
u8 enter_critical_section = 0; //filter function re-enter problem
void mini_printf(const char *format, ...)
{
	if(!enter_critical_section){
		enter_critical_section = 1;
		const char *pcStr = format;
		char *args = 0;

		const char *s = 0;

		va_start(args,format);

		while(*pcStr) {
			if (*pcStr == '%') {
				if ((*(pcStr+1) == 's') || (*(pcStr+1) == 'S')) {
					s = va_arg(args, const char *);
					for(; *s; s++)
						uart_put_char(*s);
					pcStr += 2;
				}
				else {
					pcStr = FormatItem(pcStr + 1, va_arg(args, int));
				}
			}
			else {
				uart_put_char(*pcStr++);
			}
		}
		enter_critical_section = 0;
	}
}

void array_printf(u8*data, u8 len){
     for(int i=0; i<len; i++){
         uart_put_char(data[i]);
     }

}
#endif
