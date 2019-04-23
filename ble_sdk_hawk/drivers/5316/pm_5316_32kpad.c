/********************************************************************************************************
 * @file     pm_5316_32kpad.c
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



#if(__TL_LIB_5316__ || MCU_CORE_TYPE == MCU_CORE_5316)


#include "../../stack/ble/blt_config.h"
#include "../../stack/ble/service/ble_ll_ota.h"

#include "rf_drv.h"
#include "pm.h"




#if (CLOCK_SYS_CLOCK_HZ == 16000000)
	#define CRYSTAL32000_TICK_PER_16CYCLE		8000
	#define CRYSTAL32768_TICK_PER_16CYCLE		7813   //7812.5
	#define CRYSTAL32768_TICK_PER_32CYCLE		15625
	#define USE_32CYCLE_EN				    	1
#elif (CLOCK_SYS_CLOCK_HZ == 24000000)
	#define CRYSTAL32000_TICK_PER_16CYCLE		12000
	#define CRYSTAL32768_TICK_PER_16CYCLE		11719   //11718.75
	#define CRYSTAL32768_TICK_PER_32CYCLE		24438	//23437.5
	#define CRYSTAL32768_TICK_PER_64CYCLE		46875
#elif(CLOCK_SYS_CLOCK_HZ == 32000000)
	#define CRYSTAL32000_TICK_PER_16CYCLE		16000
	#define CRYSTAL32768_TICK_PER_16CYCLE		15625
	#define CRYSTAL32768_TICK_PER_32CYCLE		31250
#elif(CLOCK_SYS_CLOCK_HZ == 48000000)
	#define CRYSTAL32000_TICK_PER_16CYCLE		24000
	#define CRYSTAL32768_TICK_PER_16CYCLE		23438  //23437.5
	#define CRYSTAL32768_TICK_PER_32CYCLE		46875
#else  //master lib, no clock define
	#define CRYSTAL32000_TICK_PER_16CYCLE		(sys_tick_per_us*500)
	#define CRYSTAL32768_TICK_PER_16CYCLE		(sys_tick_per_us*488)    //488.28 * sys_tick_per_us
	#define CRYSTAL32768_TICK_PER_32CYCLE		(sys_tick_per_us*977)
#endif



#ifndef 	USE_32CYCLE_EN
#define 	USE_32CYCLE_EN		0
#endif

#if 0
_attribute_ram_code_ int cpu_sleep_wakeup_32kpad (int deepsleep, int wakeup_src, u32 wakeup_tick)
{
	if(!blt_miscParam.pm_enter_en){
		return 0;
	}

	u32 span = (u32)(wakeup_tick - clock_time ());

	if(wakeup_src & PM_WAKEUP_TIMER){
		if (span > 0xc0000000)  //BIT(31)+BIT(30)   3/4 cylce of 32bit
		{
			return  analog_read (0x44) & 0x0f;
		}
		else if (span < EMPTYRUN_TIME_US * sys_tick_per_us) // 0 us base
		{
			u32 t = clock_time ();
			analog_write (0x44, 0x0f);			//clear all status
#if 1
			u8 st;
			do {
				st = analog_read (0x44) & 0x0f;
			} while ( ((u32)clock_time () - t < span) && !st);
			return st;
#else  //save power
			if(span > 150){
				cpu_stall_wakeup_by_timer0(span - 100);
			}
			return (analog_read (0x44) & 0x0f);
#endif
		}
		else
		{

		}
	}

	/* Execute the callback function. */
	if(func_before_suspend){
		if (!func_before_suspend())
		{
			return PM_WAKEUP_CORE;
		}
	}

	/* Enter critical region. */
	u8 r = irq_disable ();

	/* Set digital/analog wake-up source. */
	analog_write(0x26, (wakeup_src & 0xff));

	//if digital wake-up is enabled,Select digital wake-up.
	REG_ADDR8(0x6e) = 0x00;
	write_reg8(0x6e, (wakeup_src & PM_WAKEUP_CORE) ? 0x08 : 0x00);

	analog_write (0x44, 0x0f);//Clear wake-up flag.

	/* Power down the corresponding module. */
	u8 rc32k_power_down = 0;
	if (wakeup_src & PM_WAKEUP_TIMER ) {
		rc32k_power_down = 0x00;//32K XTAL need be enabled
	}
	else {
		write_reg8(0x74a,0x20);
		rc32k_power_down = (1<<1);//32K XTAL power-down
	}

	// Switch system clock to 24M RC.
	u8 reg66 = read_reg8(0x66);
	write_reg8 (0x66, 0x00);

	//32K RC power-down.
	analog_write(0x2c, ((deepsleep ? 0xfd : 0x7d) | rc32k_power_down));
	analog_write(0x2d, analog_read(0x2d) | (1<<7));

	/* Set power-on delay time when MCU is waked up. */
	span = (RESET_TIME_US * sys_tick_per_us * 16 + (CRYSTAL32000_TICK_PER_16CYCLE>>1))/ CRYSTAL32000_TICK_PER_16CYCLE;
	u8 rst_cycle =  0xff - span;
	analog_write (0x20, SWAP_BIT0_BIT6(rst_cycle));// quick wake-up, short reset time

	/* Set 32k wake-up tick. */
	u8 long_suspend = 0;
	u32 tick_cur = 0;
	u32 tick_32k_cur = 0;
	u32 tick_wakeup_reset = 0;
	if(wakeup_src & PM_WAKEUP_TIMER)
	{
		tick_32k_cur = pm_get_32k_tick ();
		tick_cur = clock_time ();
		tick_wakeup_reset = wakeup_tick - EARLYWAKEUP_TIME_US * sys_tick_per_us;
		u32 tick_span = tick_wakeup_reset - tick_cur;
		u32 tick_32k_wakeup;
#if (USE_32CYCLE_EN)
		if( tick_span >= BIT(27) ){
			long_suspend = 1;
			tick_32k_wakeup = tick_32k_cur + tick_span / CRYSTAL32768_TICK_PER_16CYCLE * 16;
		}
		else{
			tick_32k_wakeup = tick_32k_cur + tick_span * 32 / CRYSTAL32768_TICK_PER_32CYCLE;
		}
#else
		if( tick_span >= BIT(28) ){
			long_suspend = 1;
			tick_32k_wakeup = tick_32k_cur + tick_span / CRYSTAL32768_TICK_PER_16CYCLE * 16;
		}
		else{
			tick_32k_wakeup = tick_32k_cur + tick_span * 16 / CRYSTAL32768_TICK_PER_16CYCLE;
		}
#endif
		pm_set_32k_tick(tick_32k_wakeup);
	}

	//50k pull-up resistor power-down.
	u8 temp = 0;
	u8 pullup50K = 0;
	temp = analog_read(0x05);
	pullup50K = temp;//Save register's value.
	temp |= (1<<7);
	analog_write(0x05,temp);

	//Native LDO power down.(Must)
	u8 anareg01 = 0;
	if(deepsleep)
	{
		temp = analog_read(0x01);
		temp |= (1<<7);
		analog_write(0x01,temp);
	}
	else//Digital power down.
	{
		temp = analog_read(0x01);
		anareg01 = temp;
		temp |= BIT(3);
		analog_write(0x01,temp);
	}

	/* Disable input of GPIOs. */
	u8 pa_ie_reg_l = 0;//digital register.
	u8 pa_ie_reg_h = 0;//analog register.
	u8 pb_ie_reg = 0;  //analog register.
	u8 pc_ie_reg = 0;  //digital register.
	u8 pe_ie_reg = 0;  //digital register.
	if(!deepsleep)
	{
		//Save GPIO IE register value.
		pa_ie_reg_l = REG_ADDR8(0x581);//PA IE register's value(LOW)
		pa_ie_reg_h = analog_read(0xb6);//PA IE register's value(HIGH)
		pb_ie_reg = analog_read(0xb9);//PB IE register's value
		pc_ie_reg = REG_ADDR8(0x591);//PC IE register's value
		pe_ie_reg = REG_ADDR8(0x5a1);//PE IE register's value

		/**
		 * if Wake-up source is Digital GPIO,IE register of Digital GPIO which is
		 * used for wake-up should be set as "1" and IE register of GPIO of others
		 * should be set as "0", which can wake up MCU when MCU is in low power mode.
		 */
		if(wakeup_src & PM_WAKEUP_CORE)
		{
			u8 i = 0;
			u8 j = 0;
			u8 gpio_irq_reg_val;
			for(i = 0; i < 3; i++)
			{
				gpio_irq_reg_val = read_reg8(0x587 + (i<<3));//GPIO IRQ Enable register.

				u8 gpio_ie_reg_val = 0;
				for(j = 0; j < 8; j++)
				{
					if((gpio_irq_reg_val>>j) & 0x01)
					{
						gpio_ie_reg_val |= BIT(j);
					}
				}

				if(i == 0)//PA
				{
					REG_ADDR8(0x581) = gpio_ie_reg_val;//PC7 is SWS.
					analog_write(0xb6,gpio_ie_reg_val);
				}
				else if(i == 1)//PB
				{
					analog_write(0xb9, gpio_ie_reg_val);
				}
				else if(i == 2)//PC
				{
					REG_ADDR8(0x591) = (gpio_ie_reg_val|(1<<7));//PC7 is SWS.
				}
			}
			REG_ADDR8(0x5a1) = 0x00; //Clear PE IE
		}
		else/* IE register of GPIOs should be set "0" when wake-up source is PAD/32k timer. */
		{
			REG_ADDR8(0x581) = 0x02;//Clear PA IE
			analog_write(0xb6,0x00);//Clear PA IE
			analog_write(0xb9,0x00);//Clear PB IE
			REG_ADDR8(0x591) = (0x00|(1<<7)); //Clear PC IE; Register 0x591[7] use for SWS.
			REG_ADDR8(0x5a1) = 0x00; //Clear PE IE
		}
	}

	analog_write(0x44, 0x0f);//Clear wake-up flag.

	/* Enter low power mode. */
	DBG_CHN0_LOW;   //GPIO debug
	if(analog_read(0x44)&0x0f){

	}
	else
	{
		sleep_start();
	}
	DBG_CHN0_HIGH;  //GPIO debug

	/* MCU is waked-up from low power mode. */
	if(deepsleep){
		write_reg8(0x6f, 0x20);  //reboot
	}

	/* suspend recover setting. */
	analog_write(0x01,anareg01);

	//Recover 50K pull-up resistor settings.
	analog_write(0x05,pullup50K);

	//Recover GPIO IE register's value.
	REG_ADDR8(0x581) = pa_ie_reg_l;  //Recover PA IE
	analog_write(0xb6,pa_ie_reg_h);//Recover PA IE
	analog_write(0xb9,pb_ie_reg);  //Recover PB IE
	REG_ADDR8(0x591) = (pc_ie_reg | (1<<7));//Recover PC IE, PC7 is SWS.
	REG_ADDR8(0x5a1) = pe_ie_reg;//Recover PE IE

	analog_write (0x2c, 0x00);

	if(long_suspend){
		tick_cur += (u32)(pm_get_32k_tick () - tick_32k_cur) / 16 * CRYSTAL32768_TICK_PER_16CYCLE;
	}
	else{
		#if (USE_32CYCLE_EN)
			tick_cur += (u32)(pm_get_32k_tick () - tick_32k_cur) * CRYSTAL32768_TICK_PER_32CYCLE / 32;
		#else
			tick_cur += (u32)(pm_get_32k_tick () - tick_32k_cur) * CRYSTAL32768_TICK_PER_16CYCLE / 16;
		#endif
	}
	REG_ADDR8(0x66) = reg66;//restore system clock
	reg_system_tick = tick_cur;
	REG_ADDR8(0x74a) = 0xa8;//recover system timer
//	REG_ADDR8(0x74c) = 0x00;
//	//REG_ADDR8(0x74c) = 0x90;
//	REG_ADDR8(0x74c) = 0x92;  //reg_system_tick_mode |= FLD_SYSTEM_TICK_IRQ_EN;
//	REG_ADDR8(0x74f) = BIT(0);

	u8 anareg44 = analog_read(0x44);

	if(!anareg44){ //GPIO 错误的无法进入suspend
		anareg44 = STATUS_GPIO_ERR_NO_ENTER_PM;
	}
	else if ( (anareg44 & WAKEUP_STATUS_TIMER) && (wakeup_src & PM_WAKEUP_TIMER) )	//wakeup from timer only
	{
		while ((u32)(clock_time () -  wakeup_tick) > BIT(30));
	}

	/* Exit critical region. */
	irq_restore(r);

	return anareg44;
}
#else

_attribute_ram_code_ int cpu_sleep_wakeup_32kpad (int deepsleep, int wakeup_src, u32 wakeup_tick)
{
//	if(!blt_miscParam.pm_enter_en){
//		return 0;
//	}

	u32 span = (u32)(wakeup_tick - clock_time ());

	if(wakeup_src & PM_WAKEUP_TIMER){
		if (span > 0xc0000000)  //BIT(31)+BIT(30)   3/4 cylce of 32bit
		{
			return  analog_read (0x44) & 0x0f;
		}
		else if (span < EMPTYRUN_TIME_US * sys_tick_per_us) // 0 us base
		{
			u32 t = clock_time ();
			analog_write (0x44, 0x0f);			//clear all status
#if 1
			u8 st;
			do {
				st = analog_read (0x44) & 0x0f;
			} while ( ((u32)clock_time () - t < span) && !st);
			return st;
#else  //save power
			if(span > 150){
				cpu_stall_wakeup_by_timer0(span - 100);
			}
			return (analog_read (0x44) & 0x0f);
#endif
		}
		else
		{

		}
	}

	/* Execute the callback function. */
	if(func_before_suspend){
		if (!func_before_suspend())
		{
			return PM_WAKEUP_CORE;
		}
	}
	/* Enter critical region. */
	u8 r = irq_disable ();

	/* Get Current system tick and 32k tick */
	reg_sys_timer_ctrl = 0xa0;//0xa8;//Enable 32k tick read,close calibrate.
	reg_sys_timer_cmd_state = BIT(5);//Clear 32k read latch update flag.

	asm("tnop");asm("tnop");asm("tnop");asm("tnop");
	asm("tnop");asm("tnop");asm("tnop");asm("tnop");
	asm("tnop");asm("tnop");asm("tnop");asm("tnop");
	asm("tnop");asm("tnop");asm("tnop");asm("tnop");

	while(!(reg_sys_timer_cmd_state & BIT(5)));//Wait 32k latch register update.

	u32 tick_32k_cur = reg_32k_timer_counter_latch;
	u32 tick_cur = clock_time ();

	reg_sys_timer_ctrl = 0x20;

	u32 tick_wakeup_reset = wakeup_tick - EARLYWAKEUP_TIME_US * sys_tick_per_us;

	/* Set digital/analog wake-up source. */
	analog_write(0x26, wakeup_src|(PM_PAD_FILTER_EN ? BIT(3) : 0x00));

	//if digital wake-up is enabled,Select digital wake-up source.
	REG_ADDR8(0x6e) = 0x00;
	write_reg8(0x6e, (wakeup_src & PM_WAKEUP_CORE) ? 0x08 : 0x00);

//	analog_write (0x44, 0x0f);//Clear wake-up flag.

	/* Power down the corresponding module. */
	u8 rc32k_power_down = 0;
	if (wakeup_src & PM_WAKEUP_TIMER ) {
		rc32k_power_down = 0x00;//32K XTAL need be enabled
	}
	else {
		write_reg8(0x74a,0x20);
		if(deepsleep)
		{
			rc32k_power_down = 0x00|BIT(1);//32K RC power-off
		}
		else
		{
			rc32k_power_down = 0x00 | (PM_PAD_FILTER_EN ? 0:BIT(1));//32K RC power
		}
	}

	// Switch system clock to 24M RC.
	u8 reg66 = read_reg8(0x66);
	write_reg8 (0x66, 0x00);

	//32K XTAL auto power-down.
	analog_write(0x2c, ((deepsleep ? 0xfd : 0x7d) | rc32k_power_down));
	analog_write(0x2d, 0xe8);//default: 0x48 -> 0xe8

	/* Set power-on delay time when MCU is waked up. */
	span = (RESET_TIME_US * sys_tick_per_us * 16 + (CRYSTAL32000_TICK_PER_16CYCLE>>1))/ CRYSTAL32000_TICK_PER_16CYCLE;

	/* Set 32k wake-up tick. */
	u8 long_suspend = 0;
	u32 tick_span = tick_wakeup_reset - tick_cur;
	u32 tick_32k_wakeup;
#if (USE_32CYCLE_EN)
	if( tick_span >= BIT(27) ){
		long_suspend = 1;
		tick_32k_wakeup = tick_32k_cur + tick_span / CRYSTAL32768_TICK_PER_16CYCLE * 16;
	}
	else{
		tick_32k_wakeup = tick_32k_cur + tick_span * 32 / CRYSTAL32768_TICK_PER_32CYCLE;
	}
#else
	if( tick_span >= BIT(28) ){
		long_suspend = 1;
		tick_32k_wakeup = tick_32k_cur + tick_span / CRYSTAL32768_TICK_PER_16CYCLE * 16;
	}
	else{
		tick_32k_wakeup = tick_32k_cur + tick_span * 16 / CRYSTAL32768_TICK_PER_16CYCLE;
	}
#endif
	u8 rst_cycle =  0xff - span;
	analog_write (0x20, rst_cycle);// quick wake-up, short reset time
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

	//Native LDO power down.(Must)
	if(deepsleep)
	{
		analog_write(0x01, 0xf7);
	}
	else//Digital power down.
	{
		analog_write(0x01, 0x7f);
	}

	analog_write(0x44, 0x0f);//Clear wake-up flag.

//	analog_write(0x81,0xcf);//Increase XTAL current.(user for PM driver test.)

	/* Enter low power mode. */
//	DBG_CHN0_LOW;   //GPIO debug
	if(analog_read(0x44)&0x0f){

	}
	else
	{
		sleep_start();
	}
//	DBG_CHN0_HIGH;  //GPIO debug

	/* MCU is waked-up from low power mode. */
	if(deepsleep){
		write_reg8(0x6f, 0x20);  //reboot
	}

	/* suspend recover setting. */
	analog_write (0x2c, 0x00);

	analog_write(0x01, 0x77);

#if 0
	//Recover 50K pull-up resistor settings.
	analog_write(0x05, pullup50K);
#endif

	if(long_suspend){
		tick_cur += (u32)(pm_get_32k_tick () - tick_32k_cur) / 16 * CRYSTAL32768_TICK_PER_16CYCLE;
	}
	else{
		#if (USE_32CYCLE_EN)
			tick_cur += (u32)(pm_get_32k_tick () - tick_32k_cur) * CRYSTAL32768_TICK_PER_32CYCLE / 32;
		#else
			tick_cur += (u32)(pm_get_32k_tick () - tick_32k_cur) * CRYSTAL32768_TICK_PER_16CYCLE / 16;
		#endif
	}
	REG_ADDR8(0x66) = reg66;//restore system clock

	//if(tick_cur >= wakeup_tick)
	if((u32)((wakeup_tick - 16) - tick_cur ) > BIT(30))
	{
		tick_cur = wakeup_tick - 32;
	}

	reg_tmr0_tick = tick_cur;
	reg_system_tick = tick_cur;
	REG_ADDR8(0x74a) = 0x00;
	REG_ADDR8(0x74a) = 0xa8;//recover system timer

	u8 anareg44 = analog_read(0x44) & 0x0f;

	if(!anareg44){ //GPIO 错误的无法进入suspend
		anareg44 = STATUS_GPIO_ERR_NO_ENTER_PM;
	}
	else if ( (anareg44 & WAKEUP_STATUS_TIMER) && (wakeup_src & PM_WAKEUP_TIMER) )	//wakeup from timer only
	{
		while ((u32)(clock_time () -  wakeup_tick) > BIT(30));
	}

	/* Exit critical region. */
	irq_restore(r);

	return anareg44;
}

#endif



void blc_pm_select_external_32k_crystal(void)
{
	cpu_sleep_wakeup = cpu_sleep_wakeup_32kpad;
	blt_miscParam.pad32k_en = 1;
}



void blt_pm_ext32k_crystal_init(void)
{
	if(blt_miscParam.pm_enter_en){
		return;
	}
	else if(clock_time_exceed(0, 5000000)){  //
		blt_miscParam.pm_enter_en = 1;
		analog_write(SYS_DEEP_ANA_REG, blt_miscParam.pm_enter_en);
	}
}



#endif
