/********************************************************************************************************
 * @file     clock.c
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

#include "register.h"
#include "compiler.h"
#include "clock.h"
#include "irq.h"
#include "analog.h"


void clock_init(SYS_CLK_TYPEDEF SYS_CLK)
{


	reg_clk_sel = (unsigned char)SYS_CLK;


	/* WatchDog Configuration */
	#if(MODULE_WATCHDOG_ENABLE)
		wd_startEx(WATCHDOG_INIT_TIMEOUT);
	#endif

	/* Timer0 initialization for BLE */
	reg_tmr_ctrl8 &= (~FLD_TMR0_EN);
	reg_tmr0_tick = (clock_time()*SYS_TICK_DIV);
	reg_irq_mask |=	FLD_IRQ_TMR0_EN;  //Enable Timer0 Interrupt
	write_reg8(0x63c,0x01); //continuous tick mode
	reg_tmr_ctrl8 |= FLD_TMR0_EN;  //Enable Timer0
}

//_attribute_ram_code_ unsigned int clock_time(void)
//{
// 	#if 0
// 		return reg_tmr0_tick;
// 	#else
// 		return reg_system_tick;
// 	#endif
// }

unsigned int clock_time_exceed(unsigned int ref, unsigned int span_us)
{
 	return ((unsigned int)(clock_time() - ref) > span_us * sys_tick_per_us);
}


_attribute_ram_code_ void sleep_us (unsigned int us)
{
	unsigned int t = clock_time();
	while(!clock_time_exceed(t, us)){}
}


/**
 * @Brief:  24M RC Calibration.(error: 0.01%)
 * @Param:  None.
 * @Return: None.
 */
#if 0 ///old
void MCU_24M_RC_ClockCalibrate(void)
{
	unsigned char temp = 0;

	temp = analog_read(0x02);
	temp |= (1<<4);
	analog_write(0x02,temp);

	/* Enable 24M RC calibration. */
	temp = analog_read(0x83);
	temp |= (1<<0);
	temp &= ~(1<<1);
	analog_write(0x83,temp);

	/* Wait Calibration completely. */
	while(!(analog_read(0x84) & 0x01));

	unsigned char CalValue = 0;
	CalValue = analog_read(0x85);
	analog_write(0x30,CalValue);

	/* Disable 24M RC calibration. */
	temp = analog_read(0x83);
	temp &= ~(1<<0);
	analog_write(0x83,temp);

	temp = analog_read(0x02);
	temp &= ~(1<<4);
	analog_write(0x02,temp);
}
#else
void MCU_24M_RC_ClockCalibrate(void)
{
	unsigned char temp = 0;

	/* Reset to default value */
	analog_write(0x83,0x34);

	/* cap from analog register */
	temp = analog_read(0x02);
	temp |= (1<<4);
	analog_write(0x02,temp);

	/*Disable 24M RC calibration.*/
	temp = analog_read(0x83);
	temp &= ~(1<<0);
	temp &= ~(1<<1);
	analog_write(0x83,temp);

	for(volatile int i=0; i<100; i++);

	/* Enable 24M RC calibration. */
	temp = analog_read(0x83);
	temp |= (1<<0);
	analog_write(0x83,temp);

	/* Wait Calibration completely. */
	for(volatile int i=0; i<10000; i++)
	{
	   if((analog_read(0x84) & 0x01))
	   {
			unsigned char CalValue = 0;
			CalValue = analog_read(0x85);
			analog_write(0x30,CalValue);

			break;
	   }
	}

	/* cap from pm_top */
	temp = analog_read(0x02);
	temp &= ~(1<<4);
	analog_write(0x02,temp);
}
#endif

/*----------------------------- End of File ----------------------------------*/
