#pragma once
#pragma once
#include"Input.h"

class IMU :public Input
{
public:
	void read(RovData& rov_data);
	void init();
private:
#define MAX_PACKET_LEN          (128)
	uint8_t ID; 
	int16_t AccRaw[3];  
	int16_t GyoRaw[3];
	int16_t MagRaw[3];
	float Eular[3];   
	int32_t Pressure;
	typedef struct
	{
		uint32_t ofs;
		uint8_t buf[MAX_PACKET_LEN];
		uint16_t payload_len;
		uint16_t len;
	}Packet_t;
	
	enum status
	{
		kStatus_Idle,
		kStatus_Cmd,
		kStatus_LenLow,
		kStatus_LenHigh,
		kStatus_CRCLow,
		kStatus_CRCHigh,
		kStatus_Data,
	};

	typedef enum
	{
		kItemKeyStatus = 0x80,   
		kItemID = 0x90,   
		kItemUID = 0x91,   
		kItemIPAdress = 0x92,   
		kItemAccRaw = 0xA0,   
		kItemAccRawFiltered = 0xA1,
		kItemAccLinear = 0xA2,
		kItemAccCalibrated = 0xA3,
		kItemGyoRaw = 0xB0,   
		kItemGyoRawFiltered = 0xB1,
		kItemGyoCalibrated = 0xB3,
		kItemMagRaw = 0xC0,   
		kItemMagRawFiltered = 0xC1,
		kItemMagCalibrated = 0xC3,
		kItemAtdE = 0xD0,   
		kItemAtdQ = 0xD1,   
		kItemTemp = 0xE0,
		kItemPressure = 0xF0,   
		kItemEnd = 0xFF,
	}ItemID_t;
	Packet_t RxPkt;
	static void crc16_update(uint16_t *currectCrc, const uint8_t *src, uint32_t lengthInBytes);
    uint32_t Packet_Decode(uint8_t c);
	void DispData(Packet_t *pkt);
};


