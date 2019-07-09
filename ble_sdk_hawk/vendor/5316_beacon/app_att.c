
/********************************************************************************************************
 * @file     app_att.c
 *
 * @brief    for TLSR chips
 *
 * @author   BLE Group
 * @date     May. 12, 2018
 *
 * @par      Copyright (c) Telink Semiconductor (Shanghai) Co., Ltd.
 *           All rights reserved.
 *
 *           The information contained herein is confidential and proprietary property of Telink
 *           Semiconductor (Shanghai) Co., Ltd. and is available under the terms
 *           of Commercial License Agreement between Telink Semiconductor (Shanghai)
 *           Co., Ltd. and the licensee in separate contract or the terms described here-in.
 *           This heading MUST NOT be removed from this file.
 *
 *           Licensees are granted free, non-transferable use of the information in this
 *           file under Mutual Non-Disclosure Agreement. NO WARRENTY of ANY KIND is provided.
 *
 *******************************************************************************************************/

#include "tl_common.h"
#include <stack/ble/ble.h>

//already do it in project exclude from build setting
//#if (__PROJECT_5316_BEACON__)

u8 beacon_resetCharater = 0; //reset Charateristic stored in EEPROM
extern u8 beacon_txPowerValue[];

const u16 beacon_deviceServiceUUID = SERVICE_UUID_DEVICE_INFORMATION;
const u16 beacon_gapServiceUUID = SERVICE_UUID_GENERIC_ACCESS;
const u16 beacon_batteryServiceUUID = SERVICE_UUID_BATTERY;

const u16 beacon_devicefwv = GATT_UUID_FIRMWARE_VER;
const u16 beacon_deviceName = GATT_UUID_DEVICE_NAME;
const u16 beacon_batteryLevelUUID = GATT_UUID_BATTERY_LEVEL;
u8 beacon_battery_level = 100;
const u8 fwm[]={'t','l','8','2','6','x','0','1'};
const u8 beacon_devName[12]= {'t','e','l','i','n','k','b','e','a','c','o','n'};
const u8 beacon_txPowerModeConfigUserDesc[] = "Tx Power Mode";
const u8 beacon_txPowerLevelReadUserDesc[] = "Read Adv Power Level";
const u8 beacon_advPeriodUserDesc[] = "Beacon Period";
const u8 beacon_resetCharacterUserDesc[] = "Reset Config";
const u8 beacon_proxiUUIDUserDesc[] = "Proximity UUID";
const u8 beacon_majorConfigUserDesc[] = "Major";
const u8 beacon_minorConfigUserDesc[] = "Minor";
const u8 beacon_eddystoneIDUserDesc[] = "Eddystone ID";
const u8 beacon_beaconModeUserDesc[] = "Beacon Mode";

const u16 beacon_primaryServiceUUID 	= GATT_UUID_PRIMARY_SERVICE;
const u16 beacon_characterUUID 	= GATT_UUID_CHARACTER;
const u16 beacon_characterUserDescriptionUUID = GATT_UUID_CHAR_USER_DESC;

const u8 beacon_confServiceUUID[16] = {0xd8,0x81,0xc9,0x1a,0xb9,0x99,0x96,0xab,0xba,0x40,0x86,0x87,0x80,0x20,0x0c,0xee};
const u8 beacon_advTxPowerConfUUID[16] = {0xd8,0x81,0xc9,0x1a,0xb9,0x99,0x96,0xab,0xba,0x40,0x86,0x87,0x86,0x20,0x0c,0xee};
const u8 beacon_txPowerModeConfUUID[16] = {0xd8,0x81,0xc9,0x1a,0xb9,0x99,0x96,0xab,0xba,0x40,0x86,0x87,0x88,0x20,0x0c,0xee};
const u8 beacon_proxiUUIDConfUUID[16] = {0xd8,0x81,0xc9,0x1a,0xb9,0x99,0x96,0xab,0xba,0x40,0x86,0x87,0x89,0x20,0x0c,0xee};
const u8 beacon_txPM_BeaconPeriodConfUUID[16] = {0xd8,0x81,0xc9,0x1a,0xb9,0x99,0x96,0xab,0xba,0x40,0x86,0x87,0x8A,0x20,0x0c,0xee};
const u8 beacon_BeaconCharacterResetUUID[16] = {0xd8,0x81,0xc9,0x1a,0xb9,0x99,0x96,0xab,0xba,0x40,0x86,0x87,0x83,0x20,0x0c,0xee};
const u8 beacon_majorConfUUID[16] = {0xd8,0x81,0xc9,0x1a,0xb9,0x99,0x96,0xab,0xba,0x40,0x86,0x87,0x8B,0x20,0x0c,0xee};
const u8 beacon_minorConfUUID[16] = {0xd8,0x81,0xc9,0x1a,0xb9,0x99,0x96,0xab,0xba,0x40,0x86,0x87,0x8C,0x20,0x0c,0xee};
const u8 beacon_eddystoneIDConfUUID[16] = {0xd8,0x81,0xc9,0x1a,0xb9,0x99,0x96,0xab,0xba,0x40,0x86,0x87,0x8D,0x20,0x0c,0xee};
const u8 beacon_workMode[16] = {0xd8,0x81,0xc9,0x1a,0xb9,0x99,0x96,0xab,0xba,0x40,0x86,0x87,0x8E,0x20,0x0c,0xee};
// Device Name Characteristic Properties
const u8 beacon_confCharacterProp = CHAR_PROP_READ | CHAR_PROP_WRITE_WITHOUT_RSP;
const u8 beacon_deviceInf  = CHAR_PROP_READ | CHAR_PROP_NOTIFY;

extern  telink_beacon_config_t telink_beacon_config;
extern ibeacon_adv_t ibeacon_tbl_adv;
extern eddystone_UID_t eddystone_UID_tbl_adv;
//extern eddystone_URL_t eddystone_URL_tbl_adv;
//extern eddystone_TLM_t eddystone_TLM_tbl_adv;

extern int setTxPowerMode(void* p);
//extern int setAdvTxLevel(void* p);
extern int setBeaconPeriod(void* p);
extern int setIbeaconProximityUUID(void* p);
extern int setBeaconMajor(void* p);
extern int setBeaconMinor(void* p);
extern int setEddystoneID(void* p);
extern int setBeaconMode(void* p);
extern int resetBeaconCharater(void * p);
// TM : to modify
const attribute_t beacon_Attributes[] = {
	{37,0,0,0,0,0},	//
	/*Handle 1~3, 0x180A, device info service*/
	// gatt
	{3,ATT_PERMISSIONS_READ,2,2,(u8*)(&beacon_primaryServiceUUID), 	(u8 *)(&beacon_deviceServiceUUID), 0},
	//firmware version
	{0,ATT_PERMISSIONS_READ,2,1,(u8*)(&beacon_characterUUID), 	(u8*)(&beacon_deviceInf), 0},
	{0,ATT_PERMISSIONS_READ,2,sizeof(fwm),(u8*)(&beacon_devicefwv),(u8*)fwm, 0},//characteristic value

    /*Handle 4~6, 0x1800, GAP Service*/
	{3,ATT_PERMISSIONS_READ,2,2,(u8*)(&beacon_primaryServiceUUID), (u8 *)(&beacon_gapServiceUUID), 0},
	//device name
	{0,ATT_PERMISSIONS_READ,2,1,(u8*)(&beacon_characterUUID), 	(u8*)(&beacon_deviceInf), 0},
	{0,ATT_PERMISSIONS_RDWR,2,sizeof(beacon_devName),(u8*)(&beacon_deviceName), (u8*)beacon_devName, 0},//characteristic value

    /*Handle 7~9, 0x180F, Battery Service*/
	//battery
	{3,ATT_PERMISSIONS_READ,2,2,(u8*)(&beacon_primaryServiceUUID), 	(u8*)(&beacon_batteryServiceUUID), 0},
	{0,ATT_PERMISSIONS_READ,2,1,(u8*)(&beacon_characterUUID), 		(u8*)(&beacon_deviceInf), 0},
	{0,ATT_PERMISSIONS_READ,2,1,(u8*)(&beacon_batteryLevelUUID), (u8*)(&beacon_battery_level), 0},

    /*Handle 0x0A~0x25, Beacon configuration Service, 0xEE0C.....D8*/
	// gatt
	{28,ATT_PERMISSIONS_READ,2,16,(u8*)(&beacon_primaryServiceUUID), 	(u8 *)(&beacon_confServiceUUID), 0},		//beacon conf service

	//tx power mode configure characteristic declare
	{0,ATT_PERMISSIONS_READ,2,1,(u8*)(&beacon_characterUUID), 		(u8*)(&beacon_confCharacterProp), 0},
	{0,ATT_PERMISSIONS_RDWR,16,1,(u8*)(&beacon_txPowerModeConfUUID), 	&(telink_beacon_config.txpower_mode), &setTxPowerMode},//characteristic value
	{0,ATT_PERMISSIONS_READ,2,sizeof(beacon_txPowerModeConfigUserDesc),(u8*)(&beacon_characterUserDescriptionUUID),(u8*)beacon_txPowerModeConfigUserDesc, 0},//characteristic user description

	//ADV Tx Power configuration characteristic declare
					//type							//prop
	{0,ATT_PERMISSIONS_READ,2,1,(u8*)(&beacon_characterUUID), 		(u8*)(&beacon_deviceInf), 0},
	{0,ATT_PERMISSIONS_READ,16, 4,(u8*)(&beacon_advTxPowerConfUUID), 	beacon_txPowerValue, 0},//characteristic value
	{0,ATT_PERMISSIONS_READ,2,sizeof(beacon_txPowerLevelReadUserDesc),(u8*)(&beacon_characterUserDescriptionUUID),(u8*)beacon_txPowerLevelReadUserDesc, 0},//characteristic user description

	//Beacon Period configuration characteristic declare
	{0,ATT_PERMISSIONS_READ,2,1,(u8*)(&beacon_characterUUID), 		(u8*)(&beacon_confCharacterProp), 0},
	{0,ATT_PERMISSIONS_RDWR,16, 2,(u8*)(&beacon_txPM_BeaconPeriodConfUUID), 	(u8 *)(&(telink_beacon_config.beacon_period)), &setBeaconPeriod},//characteristic value
	{0,ATT_PERMISSIONS_READ,2,sizeof(beacon_advPeriodUserDesc),(u8*)(&beacon_characterUserDescriptionUUID),(u8*)beacon_advPeriodUserDesc, 0},//characteristic user description

    //Beacon charateristic reset
	{0,ATT_PERMISSIONS_READ,2,1,(u8*)(&beacon_characterUUID), 		(u8*)(&beacon_confCharacterProp), 0},
	{0,ATT_PERMISSIONS_RDWR,16,1,(u8*)(&beacon_BeaconCharacterResetUUID), 	&beacon_resetCharater, &resetBeaconCharater},//characteristic value
	{0,ATT_PERMISSIONS_READ,2,sizeof(beacon_resetCharacterUserDesc),(u8*)(&beacon_characterUserDescriptionUUID),(u8*)beacon_resetCharacterUserDesc, 0},//characteristic user description


	//Company UUID configuration characteristic declare, 0x17
	{0,ATT_PERMISSIONS_READ,2,1,(u8*)(&beacon_characterUUID), 		(u8*)(&beacon_confCharacterProp), 0},
	{0,ATT_PERMISSIONS_RDWR,16,16,(u8*)(&beacon_proxiUUIDConfUUID), 	ibeacon_tbl_adv.proximity_UUID, &setIbeaconProximityUUID},//characteristic value
	{0,ATT_PERMISSIONS_READ,2,sizeof(beacon_proxiUUIDUserDesc),(u8*)(&beacon_characterUserDescriptionUUID),(u8*)beacon_proxiUUIDUserDesc, 0},//characteristic user description


	//Major configuration characteristic declare
	{0,ATT_PERMISSIONS_READ,2,1,(u8*)(&beacon_characterUUID), 		(u8*)(&beacon_confCharacterProp), 0},
	{0,ATT_PERMISSIONS_RDWR,16,2,(u8*)(&beacon_majorConfUUID), (u8*)(&(ibeacon_tbl_adv.major)), &setBeaconMajor},//characteristic value
	{0,ATT_PERMISSIONS_READ,2,sizeof(beacon_majorConfigUserDesc),(u8*)(&beacon_characterUserDescriptionUUID),(u8*)beacon_majorConfigUserDesc, 0},//characteristic user description


	//Minor configuration characteristic declare
	{0,ATT_PERMISSIONS_READ,2,1,(u8*)(&beacon_characterUUID), 		(u8*)(&beacon_confCharacterProp), 0},
	{0,ATT_PERMISSIONS_RDWR,16,2,(u8*)(&beacon_minorConfUUID), (u8*)(&(ibeacon_tbl_adv.minor)), &setBeaconMinor},//characteristic value
	{0,ATT_PERMISSIONS_READ,2,sizeof(beacon_minorConfigUserDesc),(u8*)(&beacon_characterUserDescriptionUUID),(u8*)beacon_minorConfigUserDesc, 0},//characteristic user description


	//Eddystone UUID configuration characteristic declare
	{0,ATT_PERMISSIONS_READ,2,1,(u8*)(&beacon_characterUUID), 		(u8*)(&beacon_confCharacterProp), 0},
	{0,ATT_PERMISSIONS_RDWR,16,16,(u8*)(&beacon_eddystoneIDConfUUID), 	eddystone_UID_tbl_adv.namespace_ID, &setEddystoneID},//characteristic value
	{0,ATT_PERMISSIONS_READ,2,sizeof(beacon_eddystoneIDUserDesc),(u8*)(&beacon_characterUserDescriptionUUID),(u8*)beacon_eddystoneIDUserDesc, 0},//characteristic user description


	//Beacon work mode
	{0,ATT_PERMISSIONS_READ,2,1,(u8*)(&beacon_characterUUID), 		(u8*)(&beacon_confCharacterProp), 0},
	{0,ATT_PERMISSIONS_RDWR,16,sizeof(beacon_adv_mode_config_t),(u8*)(&beacon_workMode), 	(u8*)(&(telink_beacon_config.beacon_mode)), &setBeaconMode},//characteristic value
	{0,ATT_PERMISSIONS_READ,2,sizeof(beacon_beaconModeUserDesc),(u8*)(&beacon_characterUserDescriptionUUID),(u8*)beacon_beaconModeUserDesc, 0},//characteristic user description
};

void my_att_init ()
{
	bls_att_setAttributeTable ((u8 *)beacon_Attributes);
	//blt_smp_func_init ();						//to enable security
}

//#endif // (__PROJECT_5316_BEACON__)


