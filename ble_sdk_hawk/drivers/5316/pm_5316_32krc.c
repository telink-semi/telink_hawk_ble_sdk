/********************************************************************************************************
 * @file     pm_5316_32krc.c
 *
 * @brief    This is the source file for TLSR8232
 *
 * @author	 qiu.gao
 * @date     Jan 24, 2019
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


#include "../../stack/ble/blt_config.h"
#include "../../stack/ble/service/ble_ll_ota.h"

#include "rf_drv.h"
#include "pm.h"


#if (PM_TIM_RECOVER_MODE)
	pm_tim_recover_t	pm_timRecover;
#endif

/**
 * @Brief:
 * @Param:
 * @Return:
 */

unsigned char pm_empty_run(unsigned int span)
{
	unsigned int t = clock_time ();
	analog_write (0x44, 0x0f);			//clear all status
	#if 1
		unsigned char st;
		do {
			st = analog_read (0x44) & 0x0f;
		} while ( ((unsigned int)clock_time () - t < span) && !st);
		return st;
	#else  //save power
		if(span > 150){
			cpu_stall_wakeup_by_timer0(span - 100);
		}
		return (analog_read (0x44) & 0x0f);
	#endif
}

#define LONG_SUSPEND_EN		0

_attribute_ram_code_ int cpu_sleep_wakeup_32krc (SleepMode_TypeDef sleep_mode,  SleepWakeupSrc_TypeDef wakeup_src, unsigned int  wakeup_tick)
{
	unsigned short tick_32k_calib = REG_ADDR16(0x748);
	unsigned short tick_32k_halfCalib = tick_32k_calib >> 1;

	unsigned int span = (u32)(wakeup_tick - clock_time ());

	if(wakeup_src & PM_WAKEUP_TIMER){
		if (span > 0xc0000000)  //BIT(31)+BIT(30)   3/4 cylce of 32bit
		{
			return  analog_read (0x44) & 0x0f;
		}
		else if (span < EMPTYRUN_TIME_US * sys_tick_per_us) // 0 us base
		{
			return pm_empty_run(span);
		}
#if 0
		else
		{
			if( span > 0x0ff00000 ){  //BIT(28) = 0x10000000   16M:16S; 32M:8S  48M: 5.5S
				long_suspend = 1;
			}
		}
#endif
	}

#if 0
	/* Execute the callback function. */
	if(func_before_suspend){
		if (!func_before_suspend())
		{
			return PM_WAKEUP_CORE;
		}
	}
#endif

	/* Enter critical region. */
	unsigned char r = irq_disable ();

#if 0
	u32 tick_cur = clock_time ();
	u32 tick_32k_cur = pm_get_32k_tick ();//3.75us
#else
	reg_sys_timer_ctrl = 0xa0;//0xa8;//Enable 32k tick read,close calibrate.
	reg_sys_timer_cmd_state = BIT(5);//Clear 32k read latch update flag.

	asm("tnop");asm("tnop");asm("tnop");asm("tnop");
	asm("tnop");asm("tnop");asm("tnop");asm("tnop");
	asm("tnop");asm("tnop");asm("tnop");asm("tnop");
	asm("tnop");asm("tnop");asm("tnop");asm("tnop");

	while(!(reg_sys_timer_cmd_state & BIT(5)));//Wait 32k latch register update.

	unsigned int tick_32k_cur = reg_32k_timer_counter_latch;

	unsigned int tick_cur = clock_time ();

	reg_sys_timer_ctrl = 0x20;
#endif



#if (PM_TIM_RECOVER_MODE)
	pm_timRecover.recover_flag = 0;
	if(wakeup_src & PM_TIM_RECOVER_START){
		pm_timRecover.tick_sysClk = tick_cur;
		pm_timRecover.tick_32k = tick_32k_cur;
	}
	if(wakeup_src & PM_TIM_RECOVER_END){
		if(pm_timRecover.tick_sysClk && (unsigned int)(clock_time() - pm_timRecover.tick_sysClk) < BIT(26) ){  //BIT(26) = 2^26 = (16<<20)*4   about 4S
			pm_timRecover.recover_flag = 0x01;
		}
	}
#endif


	unsigned int tick_wakeup_reset = wakeup_tick - EARLYWAKEUP_TIME_US * sys_tick_per_us;

	/* Set digital/analog wake-up source. */
	analog_write(0x26, wakeup_src|(PM_PAD_FILTER_EN ? BIT(3) : 0x00)); ///

	//if digital wake-up is enabled,Select digital wake-up.
	write_reg8(0x6e, (wakeup_src & PM_WAKEUP_CORE) ? 0x08 : 0x00);///bit3:enable wakeup from gpio

	analog_write (0x44, 0x0f);//Clear wake-up flag. ////note, GQ delete this.

    /* Power down the corresponding module. */
	unsigned char rc32k_power_down = 0;
	if (wakeup_src & PM_WAKEUP_TIMER ) {
		rc32k_power_down = 0x00;//32K RC need be enabled
	}
	else {
		write_reg8(0x74a,0x20);
		rc32k_power_down = (PM_PAD_FILTER_EN? 0:1);//32K RC power
	}

	// Switch system clock to 24M RC.
	unsigned char reg66 = read_reg8(0x66);
	write_reg8 (0x66, 0x00);

	//32K RC power-down.
//	analog_write(0x2c, (deepsleep ? 0xfe : 0x7e) | rc32k_power_down); ///it is moved below
//	analog_write(0x2d, 0x68);//default:0x48 -> 0x68 ///it is set in cpu_wakeup_init()

	/* Set power-on delay time when MCU is waked up. */
	span = (RESET_TIME_US * sys_tick_per_us * 16 + tick_32k_halfCalib) / tick_32k_calib;

	/* Set 32k wake-up tick. */
	unsigned int tick_32k_wakeup;

	if(LONG_SUSPEND_EN){
		tick_32k_wakeup = tick_32k_cur + (unsigned int)(tick_wakeup_reset - tick_cur)/ tick_32k_calib * 16;
	}
	else{
		tick_32k_wakeup = tick_32k_cur + ((unsigned int)(tick_wakeup_reset - tick_cur) * 16 + tick_32k_halfCalib) / tick_32k_calib;
	}

	unsigned char rst_cycle = 0xff - span;
	analog_write (0x20, rst_cycle);



#if (PM_TIM_RECOVER_MODE)
	unsigned int tick_reset_timRecover;
	if( (wakeup_src & PM_TIM_RECOVER_END) && pm_timRecover.recover_flag){

		if(LONG_SUSPEND_EN){
			tick_reset_timRecover = pm_timRecover.tick_32k + (unsigned int)(tick_wakeup_reset - pm_timRecover.tick_sysClk)/ tick_32k_calib * 16;
		}
		else{
			tick_reset_timRecover = pm_timRecover.tick_32k + ((unsigned int)(tick_wakeup_reset - pm_timRecover.tick_sysClk) * 16 + tick_32k_halfCalib) / tick_32k_calib;
		}

		int rest_32k_cycle = (int )(tick_reset_timRecover - tick_32k_cur);
		//if( rest_32k_cycle > 6  &&  abs(tick_reset - tick_reset_timRecover) < BIT(7)  ){  //128 * 30us(1/32k) = 3800us
		if( rest_32k_cycle > 6  &&  (unsigned int)(tick_32k_wakeup + BIT(7) - tick_reset_timRecover) < BIT(8)  ){  //128 * 30us(1/32k) = 3800us
			pm_timRecover.recover_flag = 0x03;
			tick_32k_wakeup = tick_reset_timRecover;
		}
		else{
			//debug
		}
	}
#endif

	pm_set_32k_tick(tick_32k_wakeup);


	analog_write(0x44, 0x0f);//Clear wake-up flag.


#if 0
	//50k pull-up resistor power-down.
	u8 temp = 0;
	u8 pullup50K = 0;
	temp = analog_read(0x05);
	pullup50K = temp;//Save register's value.
	temp |= (1<<7);
	analog_write(0x05,temp);
#endif
	/*********************************************************************************
	 * 			   ana_05<7>	  ana_26<2>                                         	 ana_01<7>
	 *  Mode	 50K pullup 	pd_vddsw_3p3	PD_LDO_3v	pulldn_vo_3v	ISO			Native_LDO
	 *
	 * Active		 1				0				0			0/1			0				0
	 *  Deep		 0				1				1			0			1				1
	 * Suspend		 0				0				1			1			0				1
	 ********************************************************************************/
	///analog_0x05<7> is different from 5317
	//Native LDO power down.(Must)
	///cause analog0x01 is set to 0x77 in tbl_rf_init.
	///driver: deep--analog_0x01|0x80;suspend--analog0x01|0x08
	///so deep write 0xf7;suspend write 0x7f
	if(sleep_mode)
	{
		analog_write(0x01,0xf7);
		analog_write(0x26, analog_read(0x26) | BIT(2)); ///refer to 5317. both should be same
	}
	else//Digital power down.(save 10uA)
	{
		analog_write(0x01,0x7f);
	}
	analog_write(0x2c, (sleep_mode ? 0xfe : 0x7e) | rc32k_power_down);

//	analog_write(0x44, 0x0f);//Clear wake-up flag.
//	analog_write(0x81,0xcf);//Increase XTAL current.(user for PM driver test.)

	/* Enter low power mode. */
	DBG_CHN0_LOW;   //GPIO debug
	if(analog_read(0x44)&0x0f){

	}
	else
	{
		sleep_start();
	}
	DBG_CHN0_HIGH;  //GPIO debug

	if(sleep_mode){
		write_reg8(0x6f, 0x20);  //reboot
	}

	/* suspend recover setting. */
	analog_write (0x2c, 0x00);
	analog_write(0x01, 0x77);//4.2us
	analog_write(0x05, 0xc0);  ///????why not 0xc2 i.e power down 32K xtal;bit1: power down 32K xtal.1:power down,0:active
#if 0
	if(blt_miscParam.pad32k_en){
		analog_write(0x05, 0xc0);
	}else{
		analog_write(0x05, 0xc2);
	}
#endif


#if 0
	//Recover 50K pull-up resistor settings.
	analog_write(0x05,pullup50K);
#endif

	unsigned int now_tick_32k = pm_get_32k_tick();//3us - 40us


#if (PM_TIM_RECOVER_MODE)
	if(pm_timRecover.recover_flag == 0x03){
		if(LONG_SUSPEND_EN){
			tick_cur = pm_timRecover.tick_sysClk+ (unsigned int)(now_tick_32k - pm_timRecover.tick_32k) / 16 * tick_32k_calib;
		}
		else{
			tick_cur = pm_timRecover.tick_sysClk + (unsigned int)(now_tick_32k - pm_timRecover.tick_32k) * tick_32k_calib / 16;		// current clock
		}
	}
	else
#endif
	{
		if(LONG_SUSPEND_EN){
			tick_cur += (unsigned int)(now_tick_32k - tick_32k_cur) / 16 * tick_32k_calib;
			//tick_cur += (u32)((tick_32k<<1) - (tick_32k_cur<<1) - 3) / 32 * tick_32k_calib;
		}
		else{
			tick_cur += (unsigned int)(now_tick_32k - tick_32k_cur) * tick_32k_calib / 16 ;		// current clock
			//tick_cur += (u32)((tick_32k<<1) - (tick_32k_cur<<1) - 3) * tick_32k_calib / 32;
		}
	}

	REG_ADDR8(0x66) = reg66;//restore system clock

	//if (tick_cur >= wakeup_tick)
	//if((wakeup_src & PM_WAKEUP_TIMER) && (u32)((wakeup_tick - 16) - tick_cur ) > BIT(30))
	if((unsigned int)((wakeup_tick - 16) - tick_cur ) > BIT(30))
	{
		tick_cur = wakeup_tick - 32;
	}
	reg_tmr0_tick = (tick_cur*SYS_TICK_DIV);
	reg_system_tick = tick_cur;
	REG_ADDR8(0x74a) = 0x00;//clear
	REG_ADDR8(0x74a) = 0xa8;//recover system timer and enable calibrate


	unsigned char anareg44 = analog_read(0x44) & 0x0f;

	if(!anareg44){ //GPIO 错误的无法进入suspend
		anareg44 = STATUS_GPIO_ERR_NO_ENTER_PM;
	}
	else if ( (anareg44 & WAKEUP_STATUS_TIMER) && (wakeup_src & PM_WAKEUP_TIMER) )	//wakeup from timer only
	{
		while ((unsigned int)(clock_time() -  wakeup_tick) > BIT(30));
	}

	/* Exit critical region. */
	irq_restore(r);

//	return anareg44; ///v1.0
	return (anareg44 ? (anareg44 | STATUS_ENTER_SUSPEND) : STATUS_GPIO_ERR_NO_ENTER_PM );
}



/**
 * @Brief:
 * @Param:
 * @Return:
 */
void blc_pm_select_internal_32k_crystal(void)
{
	cpu_sleep_wakeup = cpu_sleep_wakeup_32krc;
	blt_miscParam.pm_enter_en = 1;
}








