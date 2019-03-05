#pragma once
#include <stdint.h>

struct RovData
{
	int8_t m_axis_x;
	int8_t m_axis_y;
	int8_t m_axis_z;
	int8_t m_axis_w;
	int8_t m_micro_axis_x;
	int8_t m_micro_axis_y;
	int8_t m_micro_axis_z;
	uint8_t m_regulator_type;
	int8_t m_manipulator_grab;
	int8_t m_manipulator_rotate;
	int8_t m_rotary_camera_1;
	int8_t m_rotary_camera_2;
	int8_t m_yaw;
	float m_roll;
	float m_pitch;
	float m_depth;
	int8_t	m_temperature;
};