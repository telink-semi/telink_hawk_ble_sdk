/********************************************************************************************************
 * @file	flash.c
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


#include "flash.h"
#include "watchdog.h"

_attribute_ram_code_ static inline int flash_is_busy(){
	return mspi_read() & 0x01;				//  the busy bit, pls check flash spec
}

/**
 * @brief     This function serves to set flash write command.
 * @param[in] cmd - set command.
 * @return    none
 */
_attribute_ram_code_ static void flash_send_cmd(unsigned char cmd){
	mspi_high();
	sleep_us(1);
	mspi_low();
	mspi_write(cmd);
	mspi_wait();
}

/**
 * @brief     This function serves to send flash address.
 * @param[in] addr - the flash address.
 * @return    none
 */
_attribute_ram_code_ static void flash_send_addr(unsigned int addr){
	mspi_write((unsigned char)(addr>>16));
	mspi_wait();
	mspi_write((unsigned char)(addr>>8));
	mspi_wait();
	mspi_write((unsigned char)(addr));
	mspi_wait();
}

/**
 * @brief     This function serves to wait flash done.
 *            (make this a asynchorous version).
 * @param[in] none.
 * @return    none.
 */
_attribute_ram_code_ static void flash_wait_done(void)
{
	sleep_us(100);
	flash_send_cmd(FLASH_READ_STATUS_CMD_LOWBYTE);

	int i;
	for(i = 0; i < 10000000; ++i){
		if(!flash_is_busy()){
			break;
		}
	}
	mspi_high();
}

/**
 * @brief This function serves to erase a sector.
 * @param[in]   addr the start address of the sector needs to erase.
 * @return none
 */
_attribute_ram_code_ void flash_erase_sector(unsigned long addr){
	unsigned char r = irq_disable();

	wd_clear();

	flash_send_cmd(FLASH_WRITE_ENABLE_CMD);
	flash_send_cmd(FLASH_SECT_ERASE_CMD);
	flash_send_addr(addr);
	mspi_high();
	flash_wait_done();

	irq_restore(r);
}


/**
 * @brief This function writes the buffer's content to a page.
 * @param[in]   addr the start address of the page
 * @param[in]   len the length(in byte) of content needs to write into the page
 * @param[in]   buf the start address of the content needs to write into
 * @return none
 */
_attribute_ram_code_ void flash_write_page(unsigned long addr, unsigned long len, unsigned char *buf){
	unsigned char r = irq_disable();
	unsigned int ns = 256 - (addr&0xff);
	int nw = 0;
	do{
		nw = len > ns ? ns :len;
		// important:  buf must not reside at flash, such as constant string.  If that case, pls copy to memory first before write
		flash_send_cmd(FLASH_WRITE_ENABLE_CMD);
		flash_send_cmd(FLASH_WRITE_CMD);
		flash_send_addr(addr);

		unsigned int i;
		for(i = 0; i < nw; ++i){
			mspi_write(buf[i]);		/* write data */
			mspi_wait();
		}
		mspi_high();
		flash_wait_done();
		ns = 256;
		addr+=nw;
		buf+=nw;
		len-=nw;
	}while(len>0);

	irq_restore(r);
}

/**
 * @brief This function reads the content from a page to the buf.
 * @param[in]   addr the start address of the page
 * @param[in]   len the length(in byte) of content needs to read out from the page
 * @param[out]  buf the start address of the buffer
 * @return none
 */
_attribute_ram_code_ void flash_read_page(unsigned long addr, unsigned long len, unsigned char *buf){
	unsigned char r = irq_disable();


	flash_send_cmd(FLASH_READ_CMD);
	flash_send_addr(addr);

	mspi_write(0x00);		/* dummy,  to issue clock */
	mspi_wait();
	mspi_ctrl_write(0x0a);	/* auto mode */
	mspi_wait();
	/* get data */
	for(int i = 0; i < len; ++i){
		*buf++ = mspi_get();
		mspi_wait();
	}
	mspi_high();

	irq_restore(r);
}


#if FIRMWARES_SIGNATURE_ENABLE
/**
 * @brief	  MAC id. Before reading UID of flash, you must read MID of flash. and then you can
 *            look up the related table to select the idcmd and read UID of flash
 * @return    MID of the flash
 **/
_attribute_ram_code_ unsigned int flash_read_mid(void){
	unsigned char j = 0;
	unsigned int flash_mid = 0;
	unsigned char r = irq_disable();
	flash_send_cmd(FLASH_GET_JEDEC_ID);
	mspi_write(0x00);		/* dummy,  to issue clock */
	mspi_wait();
	mspi_ctrl_write(0x0a);	/* auto mode */
	mspi_wait();

	for(j = 0; j < 3; ++j){
		((unsigned char*)(&flash_mid))[j] = mspi_get();
		mspi_wait();
	}
	mspi_high();
	irq_restore(r);
	return flash_mid;
}

/**
 * @brief	  UID. Before reading UID of flash, you must read MID of flash. and then you can
 *            look up the related table to select the idcmd and read UID of flash
 * @param[in] idcmd - get this value to look up the table based on MID of flash
 * @param[in] buf   - store UID of flash
 * @param[in] uidtype - 1:16byte uid, 0:8byte uid
 * @return    none.
 */
_attribute_ram_code_ static void flash_read_uid(Flash_Uid_Cmddef_e idcmd, unsigned char *buf, Flash_Uid_Typedef_e uidtype)
{
	unsigned char j = 0;
	unsigned char r = irq_disable();
	flash_send_cmd(idcmd);
	/*
	 * If add flash type, should pay attention to the cmd of read UID.
	 */
	if(FLASH_UID_CMD_GD_PUYA==idcmd)				//< GD/puya
	{
		flash_send_addr(0x00);
		mspi_write(0x00);		/* dummy,  to issue clock */
		mspi_wait();
	}
	mspi_write(0x00);			/* dummy,  to issue clock */
	mspi_wait();
	mspi_ctrl_write(0x0a);		/* auto mode */
	mspi_wait();

	for(j = 0; j < (uidtype?16:8); ++j){
		*buf++ = mspi_get();
		mspi_wait();
	}
	mspi_high();
	irq_restore(r);
}

/**
 * @brief 		 This function serves to read flash mid and uid,and check the correctness of mid and uid.
 * @param[out]   flash_mid - Flash Manufacturer ID
 * @param[out]   flash_uid - Flash Unique ID
 * @return       0:error 1:ok
 */
_attribute_ram_code_ int flash_read_mid_uid_with_check( unsigned int *flash_mid ,unsigned char *flash_uid)
{
	 unsigned char no_uid[16]={0x51,0x01,0x51,0x01,0x51,0x01,0x51,0x01,0x51,0x01,0x51,0x01,0x51,0x01,0x51,0x01};
	 int i,f_cnt=0;
	 unsigned int mid;
	 unsigned char uid_8byte = 0;
	 mid = flash_read_mid();
	 *flash_mid  = mid;

	 /*
	  * If add flash type, need pay attention to the read uid cmd and the bir number of status register
		   Flash Type    CMD        MID      Company

		   MD25D40DGIG	 0x4b     0x134051     GD
		   GD25D10C      0x4b     0x1140C8     GD
		   GD25D10B      0x4b     0x1140C8     GD
		   ZB25WD40B	 0x4b  	  0x13325e     ZB
		   ZB25WD20A	 0x4b  	  0x12325e     ZB
	*/
	 if((mid == 0x134051)||(mid==0x1140C8)||(mid==0x12325e))
	 {
		 flash_read_uid(FLASH_READ_UID_CMD_GD_PUYA,(unsigned char *)flash_uid, FLASH_TYPE_16BYTE_UID);
	 }
	 else if(mid==0x13325e)
	 {
		 flash_read_uid(FLASH_READ_UID_CMD_GD_PUYA,(unsigned char *)flash_uid, FLASH_TYPE_8BYTE_UID);
		 uid_8byte = 1;
	 }
	 else{
		 return 0;
	 }
	 if(0 == uid_8byte){
		for(i=0;i<16;i++){
			if(flash_uid[i]==no_uid[i]){
				f_cnt++;
			}
		}
	 }
	 else{
		  memset(flash_uid+8,0,8);//Clear the last eight bytes of a 16 byte array when the length of uid is 8.
	 }

	if(f_cnt==16){//no uid flash
		return 0;

	}else{
		return  1;
	}
}
#endif

#if FLASH_LOCK_EN
/**
 * @brief This function write the status of flash.
 * @param[in]  data - the value of status
 * @return     status
 */
_attribute_ram_code_ void flash_write_status(Flash_Status_Typedef_e type , unsigned short data)
{
	unsigned char r = irq_disable();

	flash_send_cmd(FLASH_WRITE_ENABLE_CMD);
	flash_send_cmd(FLASH_WRITE_STATUS_CMD_LOWBYTE);
	if ((type == FLASH_TYPE_8BIT_STATUS)){
		mspi_write((unsigned char)data);   //8 bit status
	}else if(type == FLASH_TYPE_16BIT_STATUS_ONE_CMD){

		mspi_write((unsigned char)data);
		mspi_wait();
		mspi_write((unsigned char)(data>>8));//16bit status

	}else if(type == FLASH_TYPE_16BIT_STATUS_TWO_CMD){

		mspi_write((unsigned char)data);
		mspi_wait();
		flash_send_cmd(FLASH_WRITE_STATUS_CMD_HIGHBYTE);
		mspi_write((unsigned char)(data>>8));//16bit status

	}
	mspi_wait();
	mspi_high();
	flash_wait_done();
	sleep_us(100);
	mspi_high();
	irq_restore(r);
}

/**
 * @brief This function reads the status of flash.
 * @param[in]  cmd - the cmd of read status
 * @param[in]  none
 * @return none
 */
_attribute_ram_code_ unsigned char flash_read_status(unsigned char cmd)
{
	unsigned char r = irq_disable();
	unsigned char status =0;
	flash_send_cmd(cmd);
	/* get 8 bit status */
	status = mspi_read();
	mspi_high();
	irq_restore(r);
	return status;
}
#endif

