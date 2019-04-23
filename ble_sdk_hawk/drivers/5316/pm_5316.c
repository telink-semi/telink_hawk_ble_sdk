/********************************************************************************************************
 * @file     pm_5316.c
 *
 * @brief    This is the source file for TLSR8232
 *
 * @author	 junwei.lu
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

#include "rf_drv.h"
#include "pm.h"
#include "register.h"
#include "../../stack/ble/blt_config.h"
#include "../../stack/ble/service/ble_ll_ota.h"

cpu_pm_handler_t cpu_sleep_wakeup;
suspend_handler_t	func_before_suspend = 0;

unsigned char PM_PAD_FILTER_EN = 0;//Pad filter enable/disable; 0:disable; 1:enable

////ram size: case 0: no flash deep 0x38 = 56 bytes
////          case 1:    flash deep 0x88 = 136 bytes. (80 bytes)
////here is same as driver. but sdk5317 is i<2
_attribute_ram_code_ _attribute_no_inline_ void  sleep_start(void){

#if 1  ///send deep command to flash.prevent some flash's leakage
	REG_ADDR8 (0x0d) = 0;
	REG_ADDR8 (0x0c) = 0xb9;
	for(volatile int i=0; i<4; i++); //1440ns when 32M clk;before (i<2)
	REG_ADDR8 (0x0d) = 1;
#endif

	//0x5a1 only low 4bits is available.although write 0xff,it is still 0x0f.
	REG_ADDR8(0x5a1) &= 0x00; //MSPI ie disable
	write_reg8(0x6f,0x81);
	//for(volatile int i=0; i<0x30; i++);//28.5uS(24MHz)
	for(volatile int i = 0; i<6; i++);//4.25us(24MHz)
	REG_ADDR8(0x5a1) |= 0xff; //MSPI ie enable

#if 1 /// send flash wakeup cmd when reboot
	REG_ADDR8 (0x0d) = 0;
	REG_ADDR8 (0x0c) = 0xab;
	for(volatile int i=0; i<4; i++); //1440ns when 32M clk ;before(i<2)
	REG_ADDR8 (0x0d) = 1;
#endif
}


#if 0 ///old
///0x66 = 0;junwei email
const TBLCMDSET  tbl_cpu_wakeup_init[] = {
	{0x60, 0x00,   TCMD_UNDER_BOTH | TCMD_WRITE},//open all the clk,disable all the rst
	{0x61, 0x00,   TCMD_UNDER_BOTH | TCMD_WRITE},//open all the clk,disable all the rst
	{0x62, 0x00,   TCMD_UNDER_BOTH | TCMD_WRITE},//open all the clk,disable all the rst
	{0x63, 0xff,   TCMD_UNDER_BOTH | TCMD_WRITE},//open all the clk,disable all the rst
	{0x64, 0xff,   TCMD_UNDER_BOTH | TCMD_WRITE},//open all the clk,disable all the rst
	{0x65, 0xff,   TCMD_UNDER_BOTH | TCMD_WRITE},

	{0x5b5, 0x0c,  TCMD_UNDER_BOTH | TCMD_WRITE},//Enable gpio(core) irq and wakeup

	{0x03, 0x4b,   TCMD_UNDER_BOTH | TCMD_WAREG},//Increase Flash current
	{0x06, 0x00,   TCMD_UNDER_BOTH | TCMD_WAREG},
//	{0x80, 0x61,   TCMD_UNDER_BOTH | TCMD_WAREG},
//	{0x81, 0x4f,   TCMD_UNDER_BOTH | TCMD_WAREG},
//	{0x82, 0x5f,   TCMD_UNDER_BOTH | TCMD_WAREG},

//	{0x620, 0x01,  TCMD_UNDER_BOTH | TCMD_WRITE},//Timer0 Enable.
	///wakeup from deep,register 0x2d will be default 0x60,so no need to set;suspend will be set other value
	{0x20, 0x00,   TCMD_UNDER_BOTH | TCMD_WAREG},//wakeup reset time: (0xff - 0xc1)*32 = 2000 us
	{0x2d, 0x48,   TCMD_UNDER_BOTH | TCMD_WAREG},//quick settle: 200 us; it is set in code below
};
#else
const TBLCMDSET  tbl_cpu_wakeup_init[] = {
	//disable all the rst,, open essential clk
	{0x60, 0x00,   TCMD_UNDER_BOTH | TCMD_WRITE},
	{0x61, 0x00,   TCMD_UNDER_BOTH | TCMD_WRITE},
	{0x62, 0x00,   TCMD_UNDER_BOTH | TCMD_WRITE},
	{0x63,  FLD_CLK0_MCU_EN
		  | FLD_CLK0_FPU_EN
		  | FLD_CLK0_AIF_EN
		  | FLD_CLK0_ZB_EN,    TCMD_UNDER_BOTH | TCMD_WRITE},
	{0x64,  FLD_CLK1_SYS_TIMER_EN
		  | FLD_CLK1_ALGM_EN
		  | FLD_CLK1_DMA_EN
		  | FLD_CLK1_PWM_EN
		  | FLD_CLK1_AES_EN
		  | FLD_CLK1_32K_CLK_EN
		  | FLD_CLK1_SWIRE_EN, TCMD_UNDER_BOTH | TCMD_WRITE},
	{0x65, FLD_CLK2_MCIC_EN,   TCMD_UNDER_BOTH | TCMD_WRITE},

//	{0x5b5, 0x0c,  TCMD_UNDER_BOTH | TCMD_WRITE},//Enable gpio(core) irq and wakeup /////move this code to other place

	//though MCU poweron is default 24M RC clock, here set MCU to 24M RC clock again for some special reason(by junwei & gaoqiu)
	{0x66, 0x00,   TCMD_UNDER_BOTH | TCMD_WRITE},

	{0x620, 0x01,  TCMD_UNDER_BOTH | TCMD_WRITE}, //Timer0 Enable.

	//poweron_dft 0x03, lc_ldo_dig_sel<2:0>,:lc_ldo_dig output voltage trim,"0b11" for 1.75V;bit3:dcdc soft power up enable;0x03<7:5>100 in track list
	{0x03, 0x8b,   TCMD_UNDER_BOTH | TCMD_WAREG},

	{0x06, 0x00,   TCMD_UNDER_BOTH | TCMD_WAREG}, //poweron_dft 0xde(LDO all power down),   0x00 all LDO power on
};
#endif
//void cpu_wakeup_init(int xtal_type)
void cpu_wakeup_init()
{
	load_tbl_cmd_set(tbl_cpu_wakeup_init, sizeof (tbl_cpu_wakeup_init)/sizeof (TBLCMDSET));

	WriteAnalogReg(0x81, 0xe8); //increase XTAL current. 0xe8 is best based on sihui

	/* Open 24M XTAL. */
	WriteAnalogReg(0x05, 0xca);
	for(volatile u32 i =0; i<10*24; i++); //166us
	WriteAnalogReg(0x05, 0xc2);
//	for(volatile u32 i =0; i<1000*24; i++);
	for(volatile u32 i =0; i<210*24; i++); //3.026ms based on sihui

//	reg_dma_chn_irq_msk = 0; //move the ending of this function

	/* Set 24M XTAL buffer and doubler. */
	//ana_80,   poweron_dft 0x79        0b 0111'1001
	//<4>: 24MHz clock to digital power down, <3>: 24MHz clock to rfpll power down   "0b0" for power on
	WriteAnalogReg(0x80, 0x61); //0x61  0b 0110'0001
	WriteAnalogReg(0x81, 0xd4);
	WriteAnalogReg(0x82, 0x5f); //poweron_dft 0x50,  <0>: doubler_en  <3:1>:clk_rfpll_en/clk_limiter_en/clk_dig_en

	/* Initialization random */
//	extern void random_generator_pre_init(void);
//	random_generator_pre_init();

	/* 24M RC calibrate. */
	MCU_24M_RC_ClockCalibrate(); ///copy 5317 code. I think that both should be same about this section

	/* 32K RC calibration. */
	mcu_32k_rc_clock_calibration();///copy 5317 code.

	/* System Timer enable. */
	reg_sys_timer_ctrl |= FLD_SYS_TIMER_EN;

	/* 32K RC calibration. */
	//cpu_rc_clock_calibration(); ///it is same as mcu_32k_rc_clock_calibration()

	/* Must */
	write_reg8(0x74a,0x29);//Enable calibration and close system timer 0x29
	write_reg8(0x74a,0x28);//End calibration,calibration value is to be written to register 0x749|0x748.0x28
	write_reg8(0x74a,0xa8);//Enable system timer and disable calibration

	write_reg16(0x748, 0x8000);  //set 32KTimer tracking Result register a value

	if(blt_miscParam.pad32k_en)
	{
//		MCU_32kClockSourceSelect(LSC_32K_XTAL); ////compare with the following code.the function is same as following code.
		////////
		analog_write(0x2d, 0xe8);
		/**
		 *  Power on 32K RC/32K XTAL.
		 *  ana_05,   poweron_dft 0xC2
		 *  ana_0x05<0>: 32k RC power   -> 0:power on,1:power off; default = 0
		 *  ana_0x05<1>: 32k XTAL power -> 0:power on,1:power off; default value = 1
		 */
		analog_write(0x05, 0xc0); ///default 0xc2
		/**
		 * Set 32k XTAL duty :50%
		 * ana_07,   poweron_dft 0x01
		 * ana_0x07<1:0>:trim bias current in 32k crystal
		 */
		analog_write(0x07,analog_read(0x07)&0xfc);
	}else{
		analog_write(0x2d, 0x68);
	}

	/* Select OTA flash address offset */
	if(REG_ADDR8(0x63e)){
		REG_ADDR16(0x63e) = (REG_ADDR16(0x63e) & 0x07) | ((ota_firmware_size_k>>2)<<3);
		ota_program_offset = 0;
	}else{
		ota_program_offset = 0x20000;
	}


	u32 mark0, mark1;
	flash_read_page(0x8,     4, (u8 *)&mark0);
	flash_read_page(0x20008, 4, (u8 *)&mark1);
	if( mark0 == 0x544c4e4b && mark1 == 0x544c4e4b){
		blt_miscParam.conn_mark = 1;
	}

	/**
	 * Set system clock source and system clock source divider factor.
	 * The code to set the system clock must be before the timer initializes the code.
	 * 保证timer0的时钟和system timer时钟相同以及tick值的同步，避免在无PM时导致BLE_STATE_BRX_S中断无法进入，从而导致BLE断连
	 */
//	write_reg8(0x66, 0x43);  ////

	//enable timer0 for system tick irq mode
	//todo: avoid cpu stall wakeup by timer0
	///move timer0 to clock_init()

	///////
	reg_dma_chn_irq_msk = 0;
	reg_gpio_wakeup_and_irq_en |= (FLD_GPIO_CORE_WAKEUP_EN | FLD_GPIO_CORE_INTERRUPT_EN);  //Enable gpio(core) irq and wakeup
}

#if 0
#define LSC_32K_RC          (1<<0)
#define LSC_32K_XTAL        (1<<1)
void MCU_32kClockSourceSelect(u8 LSC_32kSource)
{
	u8 temp = 0;

	/* Power on 32K RC/32K XTAL. */
	temp = analog_read(0x05);
	temp |= 0x03;
	temp &= ~LSC_32kSource;
	analog_write(0x05,temp);

//	temp = analog_read(0x2c);
//	temp |= 0x03;
//	temp &= ~LSC_32kSource;
//	analog_write(0x2c,temp);

	/* Set 32k Clock source as 32K RC. */
	if(LSC_32kSource == LSC_32K_RC)
	{
		temp = analog_read(0x2d);
		temp &= ~(1<<7);
		analog_write(0x2d,temp);
	}
	else if(LSC_32kSource == LSC_32K_XTAL)/* Set 32k Clock source as 32K XTAL. */
	{
		//Disable 50K pull-up resistor of 32K XTAL pin(PB6 PB7).
		//gpio_set_50k_pullup(0);

		temp = analog_read(0x2d);
		temp |= (1<<7);
		analog_write(0x2d,temp|0x68);

		//Set 32k XTAL duty :50%
		temp = analog_read(0x07);
		temp &= ~(0x03);
		analog_write(0x07,temp);
	}
}
#endif

/**
 * @Brief: MCU internal 32K RC calibrate.Calibration accuracy is 1.6%
 * @Param: None.
 * @ReVal: None.
 */
void mcu_32k_rc_clock_calibration(void)
{
	unsigned char temp = 0;

	/* Reset to default value */
	analog_write(0x83,0x34);

	/* cap from analog register */
	temp = analog_read(0x02);
	temp |= (1<<2);
	analog_write(0x02, temp);

	/* Disable 32K RC calibration. */
	temp = analog_read(0x83);
	temp &= ~(1<<0);//disable
	temp |= (1<<1);//Select calibrate 32k RC
	analog_write(0x83,temp);

	for(volatile int i=0; i<100; i++);

	/* Enable 32K RC calibration. */
	temp = analog_read(0x83);
	temp |= (1<<0);//Enable
	analog_write(0x83,temp);

	/* Wait Calibration completely. */
	for(volatile int i=0; i<10000; i++)
	{
		if((analog_read(0x84) & 0x01))
		{
			unsigned char CalValue = 0;
			CalValue = analog_read(0x85);
			analog_write(0x2f,CalValue);

			break;
		}
	}

	/* cap from pm_top */
	temp = analog_read(0x02);
	temp &= ~(1<<2);
	analog_write(0x02, temp);
}


/**
 * @brief      This function configures a GPIO pin as the wakeup pin.
 * @param[in]  Pin - the pin needs to be configured as wakeup pin
 * @param[in]  Pol - the wakeup polarity of the pad pin(0: low-level wakeup, 1: high-level wakeup)
 * @param[in]  En  - enable or disable the wakeup function for the pan pin(1: Enable, 0: Disable)
 * @return     none
 * checked and it is same as driver
 */
#define AREG_PAD_WAKEUP_EN(i)		((i>>8) + 0x27)
#define AREG_PAD_WAKEUP_POL(i)		((i>>8) + 0x21)
void cpu_set_gpio_wakeup (int pin, int pol, int en) {
	///////////////////////////////////////////////////////////
	// 		  PA[7:0]	    PB[7:0]		PC[7:0]
	// pol: ana_21<7:0>	 ana_22<7:0>  ana_23<7:0>
	// en:	ana_27<7:0>	 ana_28<7:0>  ana_29<7:0>

    unsigned char mask = pin & 0xff;
	unsigned char areg;
	unsigned char val;

	////////////////////////// polarity ////////////////////////
	areg = AREG_PAD_WAKEUP_POL(pin);
	val = ReadAnalogReg(areg);
	if (pol) {
		val &= ~mask;
	}
	else {
		val |= mask;
	}
	WriteAnalogReg (areg, val);

	/////////////////////////// enable /////////////////////
	areg = AREG_PAD_WAKEUP_EN(pin);
	val = ReadAnalogReg(areg);
	if (en) {
		val |= mask;
	}
	else {
		val &= ~mask;
	}
	WriteAnalogReg (areg, val);

//	pm_set_filter(1);
}


/**
 * @Brief: Set pad filter.
 * @Param:
 * @Return: None.
 */
void pm_set_filter(u8 en)
{
#if 1
	PM_PAD_FILTER_EN = en;
#else
	unsigned char tmp;
	tmp = ReadAnalogReg(0x26);
	if(en)
	{
		tmp |= BIT(3);
	}
	else
	{
		tmp &= ~BIT(3);
	}

	PM_PAD_FILTER_EN = en;
	WriteAnalogReg(0x26, tmp);
#endif
}


/**
 * @Brief:  Get 32k timer tick value.
 * @Param:  None.
 * @Return: 32k timer tick value.
 */
_attribute_ram_code_ u32 pm_get_32k_tick(void)
{
	reg_sys_timer_ctrl = 0x20;//0xa8;//Enable 32k tick read,close calibrate.
//	reg_sys_timer_cmd_state = 0x00;//Clear 32k read latch update flag.
	reg_sys_timer_cmd_state |= BIT(5);//Clear 32k read latch update flag.

	asm("tnop");asm("tnop");asm("tnop");asm("tnop");
	asm("tnop");asm("tnop");asm("tnop");asm("tnop");
	asm("tnop");asm("tnop");asm("tnop");asm("tnop");
	asm("tnop");asm("tnop");asm("tnop");asm("tnop");

	while(!(reg_sys_timer_cmd_state & BIT(5)));//Wait 32k latch register update.

	return reg_32k_timer_counter_latch;
}

/**
 * @Brief:  Set 32k timer tick value.
 * @Param:  None.
 * @Return: None.
 * different with driver: //	while((reg_sys_timer_cmd_state & BIT(3)) == 0);
 * this line code is not deleted in driver.
 * here just refer to sdk5317
 */
_attribute_ram_code_ void pm_set_32k_tick(u32 tick)
{
	reg_sys_timer_ctrl = 0x21;//Enable 32k tick write.
	while(reg_sys_timer_cmd_state & 0x40);

	reg_32k_timer_tick = tick;
	reg_sys_timer_cmd_state |= BIT(3);//Start 32k tick write.

//	while((reg_sys_timer_cmd_state & BIT(3)) == 0);//Wait

	asm("tnop");asm("tnop");asm("tnop");asm("tnop");
	asm("tnop");asm("tnop");asm("tnop");asm("tnop");
	asm("tnop");asm("tnop");asm("tnop");asm("tnop");
	asm("tnop");asm("tnop");asm("tnop");asm("tnop");

	while(reg_sys_timer_cmd_state & BIT(3));//Wait done.
}

void	bls_pm_registerFuncBeforeSuspend (suspend_handler_t func )
{
	func_before_suspend = func;
}

/**
 * @brief   This function serves to wake up cpu from stall mode by timer0.
 * @param[in]   tick - capture value of timer0.
 * @return  none.
 * checked and it is same as driver
 */
void cpu_stall_wakeup_by_timer0(u32 tick_stall)
{
   /*Write 0x00 here may cause problem, it is removed to blt_sleep_wakeup*/
   //write_reg8(0x6f,0x00);//clear bit[0] suspend enable

    reg_tmr0_tick = 0;

    reg_tmr0_capt = tick_stall;
    reg_tmr_ctrl8 |= FLD_TMR0_EN;//enable TIMER1,mode:using sclk
    reg_mcu_wakeup_mask |= FLD_IRQ_TMR0_EN;//timer1 mask enable
    reg_tmr_sta = FLD_TMR_STA_TMR0; // clean interrupt

    write_reg8(0x6f,0x80);//stall mcu
    asm("tnop");
    asm("tnop");

    reg_tmr_sta = FLD_TMR_STA_TMR0; // clean interrupt
    reg_tmr_ctrl8 &= ~FLD_TMR0_EN;//disable TIMER1
}

/**
 * @brief   This function serves to wake up cpu from stall mode by timer1.
 * @param   tick - capture value of timer1.
 * @return  none.
 * checked with driver &SDK5317， the three are same.
 */
void cpu_stall_wakeup_by_timer1(u32 tick_stall)
{
   /*Write 0x00 here may cause problem, it is removed to blt_sleep_wakeup*/
   //write_reg8(0x6f,0x00);//clear bit[0] suspend enable

    reg_tmr1_tick = 0;

    reg_tmr1_capt = tick_stall;
    reg_tmr_ctrl8 |= FLD_TMR1_EN;//enable TIMER1,mode:using sclk
    reg_mcu_wakeup_mask |= FLD_IRQ_TMR1_EN;//timer1 mask enable
    reg_tmr_sta = FLD_TMR_STA_TMR1; // clean interrupt

    write_reg8(0x6f,0x80);//stall mcu
    asm("tnop");
    asm("tnop");

    reg_tmr_sta = FLD_TMR_STA_TMR1; // clean interrupt
    reg_tmr_ctrl8 &= ~FLD_TMR1_EN;//disable TIMER1
}

/**
 * @brief   This function serves to wake up cpu from stall mode by timer2.
 * @param[in]   tick - capture value of timer2.
 * @return  none.
 * checked and it is same as driver
 */
void cpu_stall_wakeup_by_timer2(u32 tick_stall)
{
   /*Write 0x00 here may cause problem, it is removed to blt_sleep_wakeup*/
   //write_reg8(0x6f,0x00);//clear bit[0] suspend enable

    reg_tmr2_tick = 0;

    reg_tmr2_capt = tick_stall;
    reg_tmr_ctrl8 |= FLD_TMR2_EN;//enable TIMER1,mode:using sclk
    reg_mcu_wakeup_mask |= FLD_IRQ_TMR2_EN;//timer1 mask enable
    reg_tmr_sta = FLD_TMR_STA_TMR2; // clean interrupt

    write_reg8(0x6f,0x80);//stall mcu
    asm("tnop");
    asm("tnop");

    reg_tmr_sta = FLD_TMR_STA_TMR2; // clean interrupt
    reg_tmr_ctrl8 &= ~FLD_TMR2_EN;//disable TIMER1
}


//just for pm test
void shutdown_gpio(void)
{
	//output disable
	(*(volatile unsigned char*) 0x800582) = 0xff;
	(*(volatile unsigned char*) 0x80058a) = 0xff;
	(*(volatile unsigned char*) 0x800592) = 0xff;
	(*(volatile unsigned char*) 0x80059a) = 0xff;
	(*(volatile unsigned char*) 0x8005a2) = 0xff;
	(*(volatile unsigned char*) 0x8005aa) = 0x03;

	//dataO = 0
	(*(volatile unsigned char*) 0x800583) = 0x00;
	(*(volatile unsigned char*) 0x80058b) = 0x00;
	(*(volatile unsigned char*) 0x800593) = 0x00;
	(*(volatile unsigned char*) 0x80059b) = 0x00;
	(*(volatile unsigned char*) 0x8005a3) = 0x00;
	(*(volatile unsigned char*) 0x8005ab) = 0x00;

	//ie = 0
	//SWS   589<0>
	//MSPI  5a1<7:4>
	(*(volatile unsigned char*) 0x800581) = 0x00;
	(*(volatile unsigned char*) 0x800589) = 0x01;
	(*(volatile unsigned char*) 0x800591) = 0x00;
	(*(volatile unsigned char*) 0x800599) = 0x00;
	(*(volatile unsigned char*) 0x8005a1) = 0xf0;
	(*(volatile unsigned char*) 0x8005a9) = 0x00;

}


