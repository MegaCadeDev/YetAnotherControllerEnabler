#ifndef YETANOTHERCONTROLLERENABLER_PAD_DATA_H
#define YETANOTHERCONTROLLERENABLER_PAD_DATA_H

#include <stddef.h>
#include <stdint.h>

/*
 * 120-byte controller sample accepted by scePadRead / scePadVirtualDeviceInsertData.
 * Field order matches the layout checked in public PS5 pad research:
 * buttons at 0x00, touch at 0x34, connected at 0x4C, timestamp at 0x50, size 120.
 */

#define PAD_BUTTON_CREATE    0x00000001u
#define PAD_BUTTON_L3        0x00000002u
#define PAD_BUTTON_R3        0x00000004u
#define PAD_BUTTON_OPTIONS   0x00000008u
#define PAD_BUTTON_UP        0x00000010u
#define PAD_BUTTON_RIGHT     0x00000020u
#define PAD_BUTTON_DOWN      0x00000040u
#define PAD_BUTTON_LEFT      0x00000080u
#define PAD_BUTTON_L2        0x00000100u
#define PAD_BUTTON_R2        0x00000200u
#define PAD_BUTTON_L1        0x00000400u
#define PAD_BUTTON_R1        0x00000800u
#define PAD_BUTTON_TRIANGLE  0x00001000u
#define PAD_BUTTON_CIRCLE    0x00002000u
#define PAD_BUTTON_CROSS     0x00004000u
#define PAD_BUTTON_SQUARE    0x00008000u
#define PAD_BUTTON_PS        0x00010000u
#define PAD_BUTTON_TOUCHPAD  0x00100000u

#define PAD_VIRT_TYPE_DUALSENSE 3

#define PAD_TRIGGER_DIGITAL_THRESHOLD 30

typedef struct PadTouch {
	uint16_t x;
	uint16_t y;
	uint8_t id;
	uint8_t reserved[3];
} PadTouch;

typedef struct PadData {
	uint32_t buttons;
	uint8_t left_x;
	uint8_t left_y;
	uint8_t right_x;
	uint8_t right_y;
	uint8_t l2;
	uint8_t r2;
	uint8_t analog_reserved[2];
	float orient_x;
	float orient_y;
	float orient_z;
	float orient_w;
	float accel_x;
	float accel_y;
	float accel_z;
	float angvel_x;
	float angvel_y;
	float angvel_z;
	uint8_t touch_count;
	uint8_t touch_reserved[7];
	PadTouch touch[2];
	int32_t connected;
	uint64_t timestamp_us;
	uint32_t extension_unit_id;
	uint8_t extension_reserved;
	uint8_t extension_length;
	uint8_t extension_data[10];
	uint8_t connected_count;
	uint8_t reserved[2];
	uint8_t device_unique_length;
	uint8_t device_unique_data[12];
} PadData;

typedef struct PadState {
	uint32_t buttons;
	uint8_t left_x;
	uint8_t left_y;
	uint8_t right_x;
	uint8_t right_y;
	uint8_t l2;
	uint8_t r2;
} PadState;

int scePadInit(void);
int scePadGetHandle(int user_id, int port_type, int index);
int scePadSetProcessPrivilege(int privilege);
int scePadVirtualDeviceAddDevice(void *param, int type);
int scePadVirtualDeviceInsertData(int handle, const PadData *data);
int scePadVirtualDeviceDeleteDevice(int handle);
uint64_t sceKernelGetProcessTime(void);

static inline void pad_state_neutral(PadState *state)
{
	state->buttons = 0;
	state->left_x = 128;
	state->left_y = 128;
	state->right_x = 128;
	state->right_y = 128;
	state->l2 = 0;
	state->r2 = 0;
}

static inline void pad_data_from_state(PadData *data, const PadState *state, uint64_t timestamp_us)
{
	*data = (PadData){0};
	data->buttons = state->buttons;
	data->left_x = state->left_x;
	data->left_y = state->left_y;
	data->right_x = state->right_x;
	data->right_y = state->right_y;
	data->l2 = state->l2;
	data->r2 = state->r2;
	if (state->l2 >= PAD_TRIGGER_DIGITAL_THRESHOLD)
		data->buttons |= PAD_BUTTON_L2;
	if (state->r2 >= PAD_TRIGGER_DIGITAL_THRESHOLD)
		data->buttons |= PAD_BUTTON_R2;
	data->orient_w = 1.0f;
	data->connected = 1;
	data->connected_count = 1;
	data->timestamp_us = timestamp_us;
}

#endif
