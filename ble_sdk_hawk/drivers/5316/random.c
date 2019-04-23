/********************************************************************************************************
 * @file     random.c
 *
 * @brief    This is the source file for TLSR8232
 *
 * @author	 junyuna.zhang
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

#include "random.h"
#include "adc.h"

unsigned int rnd_m_w = 0;
unsigned int rnd_m_z = 0;

typedef union
{
	unsigned int rng32;

	struct {
		unsigned int bit0:1;
		unsigned int bit1:1;
		unsigned int bit2:1;
		unsigned int bit3:1;
		unsigned int bit4:1;
		unsigned int bit5:1;
		unsigned int bit6:1;
		unsigned int bit7:1;
		unsigned int bit8:1;
		unsigned int bit9:1;
		unsigned int bit10:1;
		unsigned int bit11:1;
		unsigned int bit12:1;
		unsigned int bit13:1;
		unsigned int bit14:1;
		unsigned int bit15:1;
		unsigned int bit16:1;

	}rng_bits;

}acd_rng;

volatile static acd_rng rng = {0};

static unsigned short rng_made(void)
{

	rng.rng_bits.bit16 = rng.rng_bits.bit16 ^ rng.rng_bits.bit15 ^ rng.rng_bits.bit13 ^ rng.rng_bits.bit4 ^ rng.rng_bits.bit0;
	if(rng.rng_bits.bit16)
	{
		rng.rng32 = (rng.rng32<<1)+ 1;
	}
	else
	{
		rng.rng32 = (rng.rng32<<1);
	}

	return ((unsigned short)rng.rng32);
}

/**
 * @brief This function serves to set adc sampling and get results.
 * @param[in]  none.
 * @return the result of sampling.
 */
unsigned short adc_rng_result(void)
{
	volatile signed short adc_dat_buf_rd[16];
	volatile unsigned short rng_result;

	unsigned char i;

	unsigned int t0 = clock_time();


	//dfifo setting will lose in suspend/deep, so we need config it every time
	adc_aif_set_misc_buf((unsigned short *)adc_dat_buf_rd,16);  //size: ADC_SAMPLE_NUM*4
	adc_aif_set_m_chn_en(1);
	adc_aif_set_use_raw_data_en();


	while(!clock_time_exceed(t0, 25));  //wait at least 2 sample cycle(f = 96K, T = 10.4us)

	for(i=0;i<16;i++)
	{
		while((!adc_dat_buf_rd[i])&&(!clock_time_exceed(t0,16)));  //wait for new adc sample data,

		t0 = clock_time();

		rng.rng32 &= 0x0000ffff;
		if(adc_dat_buf_rd[i] & BIT(0))
		{
			rng.rng_bits.bit16 = 1;
		}

		rng_result = rng_made();

	}

	adc_aif_set_m_chn_en(0);
	return rng_result;

}


/**
 * @brief This function is used for ADC configuration of ADC supply voltage sampling.
 * @param[in]   GPIO_PinTypeDef pin
 * @return none
 */

void rng_init(void)
{
	//set R_max_mc,R_max_c,R_max_s
	adc_set_misc_rns_capture_state_length(0xf0);		    //max_mc
	adc_set_all_set_state_length(0x0a);									//max_s

	//set total length for sampling state machine and channel
	adc_set_chn_en(ADC_MISC_CHN);
	adc_set_max_state_cnt(0x02);

	//set channel Vref
	adc_set_all_vref(ADC_MISC_CHN, ADC_VREF_1P2V);

	//set Vbat divider select,
	adc_set_vref_vbat_div(ADC_VBAT_DIVIDER_OFF);


	//set channel mode and channel
	adc_set_all_input_mode(ADC_MISC_CHN, DIFFERENTIAL_MODE);
	adc_set_all_differential_p_n_ain(ADC_MISC_CHN, VBAT, GND);

	//set resolution for RNG
	adc_set_all_resolution(ADC_MISC_CHN, RES14);

	//Number of ADC clock cycles in sampling phase
	adc_set_all_tsample_cycle(ADC_MISC_CHN, SAMPLING_CYCLES_6); ///SAMPLING_CYCLES_3

	//set Analog input pre-scaling and
	adc_set_all_ain_pre_scaler(ADC_PRESCALER_1F8);//  ADC_PRESCALER_1F8
	//set RNG mode
	adc_set_mode(ADC_NORMAL_MODE);

}

/**
 * @brief     This function performs to preparatory initials random generator.
 * @param[in] none.
 * @return    none.
 */

void rng_pre_init(void)
{
	rng.rng32 = 0x0000ffff;

	//ADC modle init
	adc_init();

//	vbat_Init(GPIO_PA7);
	rng_init();

	//After setting the ADC parameters, turn on the ADC power supply control bit
	adc_power_on(1);

	rnd_m_w = adc_rng_result()<<16 | adc_rng_result();
	rnd_m_z = adc_rng_result()<<16 | adc_rng_result();

	adc_power_on(0);

}

//16M clock, code in flash 23us, code in sram 4us
//_attribute_ram_code_
unsigned int rand(void)
{
	rnd_m_w = 18000 * (rnd_m_w & 0xffff) + (rnd_m_w >> 16);
	rnd_m_z = 36969 * (rnd_m_z & 0xffff) + (rnd_m_z >> 16);
	unsigned int result = (rnd_m_z << 16) + rnd_m_w;

	return (unsigned int)( result ^ clock_time() );
}

/*********************************************************************
 * @fn          generateRandomNum
 *
 * @brief       generate random number
 *
 * @param       len - len
 *
 * @param       data -  buffer
 *
 * @return      None
 */
void generateRandomNum(int len, unsigned char *data)
{
	int i;
	unsigned int randNums = 0;
    /* if len is odd */
	for (i=0; i<len; i++ ) {
		if( (i & 3) == 0 ){
			randNums = rand();
		}

		data[i] = randNums & 0xff;
		randNums >>=8;
	}
}

