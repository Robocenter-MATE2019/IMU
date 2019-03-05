/*
 Name:		IMUSensor.ino
 Created:	18.02.2019 19:40:08
 Author:	User
*/
#include"IMU.h"
#include"RovData.h"
IMU imu;
int time;
RovData data;
void setup() {
	imu.init();
	Serial.begin(115200);
	Serial.println("setup");
	data.m_pitch = 0;
	data.m_yaw = 0;
	data.m_roll = 0;
	time = millis();
}

void loop() {
	imu.read(data);
	if (millis() - time > 100) {
		Serial.println(data.m_roll);
		Serial.println(data.m_pitch);
		Serial.println(data.m_yaw);
		Serial.println("------");
		time = millis();
	}
	/*if (Serial3.available())
	{
		uint8_t ch = Serial3.read();
		imu.Packet_Decode(ch);
	}*/
}
