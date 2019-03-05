#include "IMU.h"
#include "RovData.h"
#include <stdlib.h>
#include <string.h>
#include <Arduino.h>




void IMU::crc16_update(uint16_t * currectCrc, const uint8_t * src, uint32_t lengthInBytes)
{
	uint32_t crc = *currectCrc;
	uint32_t j;
	for (j = 0; j < lengthInBytes; ++j)
	{
		uint32_t i;
		uint32_t byte = src[j];
		crc ^= byte << 8;
		for (i = 0; i < 8; ++i)
		{
			uint32_t temp = crc << 1;
			if (crc & 0x8000)
			{
				temp ^= 0x1021;
			}
			crc = temp;
		}
	}
	*currectCrc = crc;
}

uint32_t IMU::Packet_Decode(uint8_t c)
{
	static uint16_t CRCReceived = 0; /* CRC value received from a frame */
	static uint16_t CRCCalculated = 0;  /* CRC value caluated from a frame */
	static uint8_t status = kStatus_Idle; /* state machine */
	static uint8_t crc_header[4] = { 0x5A, 0xA5, 0x00, 0x00 };

	switch (status)
	{
	case kStatus_Idle:
		if (c == 0x5A)
			status = kStatus_Cmd;
		break;
	case kStatus_Cmd:
		if (c == 0xA5)
			status = kStatus_LenLow;
		break;
	case kStatus_LenLow:
		RxPkt.payload_len = c;
		crc_header[2] = c;
		status = kStatus_LenHigh;
		break;
	case kStatus_LenHigh:
		RxPkt.payload_len |= (c << 8);
		crc_header[3] = c;
		status = kStatus_CRCLow;
		break;
	case kStatus_CRCLow:
		CRCReceived = c;
		status = kStatus_CRCHigh;
		break;
	case kStatus_CRCHigh:
		CRCReceived |= (c << 8);
		RxPkt.ofs = 0;
		CRCCalculated = 0;
		status = kStatus_Data;
		break;
	case kStatus_Data:
		RxPkt.buf[RxPkt.ofs++] = c;
		if (RxPkt.ofs >= RxPkt.payload_len)
		{
			/* calculate CRC */
			crc16_update(&CRCCalculated, crc_header, 4);
			crc16_update(&CRCCalculated, RxPkt.buf, RxPkt.ofs);

			/* CRC match */
			if (CRCCalculated == CRCReceived)
			{
				/* ?????????????? */
				DispData(&RxPkt);
			}
			status = kStatus_Idle;
		}
		break;
	default:
		status = kStatus_Idle;
		break;
	}
}



void IMU::read(RovData & rov_data)
{
	if (Serial3.available())
	{
		uint8_t ch = Serial3.read();
		Packet_Decode(ch);
		rov_data.m_roll = Eular[0];
		rov_data.m_pitch = Eular[1];
		rov_data.m_yaw = Eular[2];
	}
}


void IMU::init()
{
	Serial3.begin(115200);
}

void IMU::DispData(Packet_t * pkt)
{

	if (pkt->buf[0] == kItemID) /* user ID */
	{
		ID = pkt->buf[1];
	}
	if (pkt->buf[2] == kItemAccRaw)  /* Acc raw value */
	{
		memcpy(AccRaw, (void*)pkt->buf[3], 6);
	}

	if (pkt->buf[9] == kItemGyoRaw)  /* gyro raw value */
	{
		memcpy(GyoRaw, (void*)pkt->buf[10], 6);
	}

	if (pkt->buf[16] == kItemMagRaw)  /* mag raw value */
	{
		memcpy(MagRaw, (void*)pkt->buf[17], 6);
	}
	if (pkt->buf[23] == kItemAtdE)  /* atd E */
	{
		Eular[0] = ((float)(int16_t)(pkt->buf[24] + (pkt->buf[25] << 8))) / 100;
		Eular[1] = ((float)(int16_t)(pkt->buf[26] + (pkt->buf[27] << 8))) / 100;
		Eular[2] = ((float)(int16_t)(pkt->buf[28] + (pkt->buf[29] << 8))) / 10;
	}

	/*Serial.print("Pitch:");
	Serial.print(Eular[0]);
	Serial.print("  ");
	Serial.print("Roll:");
	Serial.print(Eular[1]);
	Serial.print("  ");

	Serial.print("Yaw:");
	Serial.print(Eular[2]);
	Serial.print("  ");
	Serial.print("\r\n");*/
}

