#include "hid_map.h"
#include "pad_data.h"

#include <stdio.h>
#include <string.h>

static int g_failed;

static void expect(int cond, const char *message)
{
	if (!cond) {
		fprintf(stderr, "FAIL %s\n", message);
		g_failed++;
	}
}

static void test_layout(void)
{
	expect(sizeof(PadData) == 120, "PadData is 120 bytes");
	expect(offsetof(PadData, touch_count) == 0x34, "touch offset");
	expect(offsetof(PadData, connected) == 0x4c, "connected offset");
	expect(offsetof(PadData, timestamp_us) == 0x50, "timestamp offset");
}

static void test_x360(void)
{
	uint8_t report[20] = {0};
	PadState state;
	int16_t lx = 0;
	int16_t ly = 32767;

	report[0] = 0x00;
	report[1] = 0x14;
	report[2] = 0x01 | 0x10;
	report[3] = 0x10;
	report[4] = 200;
	memcpy(report + 6, &lx, sizeof(lx));
	memcpy(report + 8, &ly, sizeof(ly));

	expect(hid_map_x360(report, sizeof(report), &state) == 0, "x360 parse");
	expect(state.buttons & PAD_BUTTON_UP, "x360 up");
	expect(state.buttons & PAD_BUTTON_OPTIONS, "x360 start -> options");
	expect(state.buttons & PAD_BUTTON_CROSS, "x360 A -> cross");
	expect(state.l2 == 200, "x360 left trigger");
	expect(state.left_x == 128, "x360 centered x");
	expect(state.left_y == 0, "x360 stick up is small y");
	report[3] |= 0x04;
	expect(hid_map_x360(report, sizeof(report), &state) == 0, "x360 guide parse");
	expect(state.buttons & PAD_BUTTON_PS, "x360 guide -> ps");

	{
		uint8_t burst[40];

		memset(burst, 0, sizeof(burst));
		burst[1] = 0x14;
		burst[3] = 0x10;
		burst[21] = 0x14;
		expect(hid_map_x360(burst, sizeof(burst), &state) == 0, "x360 burst parse");
		expect(state.buttons & PAD_BUTTON_CROSS, "x360 burst shows the press");
		expect(hid_map_x360_take_pending(&state) == 1, "x360 burst releases");
		expect((state.buttons & PAD_BUTTON_CROSS) == 0, "x360 burst ended");
		burst[3] = 0;
		expect(hid_map_x360(burst, 20, &state) == 0, "x360 release");
		expect((state.buttons & PAD_BUTTON_CROSS) == 0, "x360 release clears");
		expect(hid_map_x360_take_pending(&state) == 0, "x360 no pending hold");
	}
}

static void test_ps3(void)
{
	uint8_t report[49];
	PadState state;

	memset(report, 0, sizeof(report));
	report[0] = 0x01;
	report[2] = 0x01 | 0x10;
	report[3] = 0x40;
	report[4] = 0x01;
	report[6] = 128;
	report[7] = 0;
	report[18] = 200;

	expect(hid_map_ps3(report, sizeof(report), &state) == 0, "ps3 parse");
	expect(state.buttons & PAD_BUTTON_CREATE, "ps3 select -> create");
	expect(state.buttons & PAD_BUTTON_UP, "ps3 dpad up");
	expect(state.buttons & PAD_BUTTON_CROSS, "ps3 cross");
	expect(state.buttons & PAD_BUTTON_PS, "ps3 ps");
	expect(state.left_x == 128, "ps3 centered x");
	expect(state.left_y == 0, "ps3 stick up stays small");
	expect(state.l2 == 200, "ps3 left trigger");
	expect(hid_kind_for_ids(0x054c, 0x0268) == HID_KIND_PS3, "ps3 kind");
}

static void test_dinput(void)
{
	uint8_t adapter[10];
	uint8_t shanwan[19];
	PadState state;

	memset(adapter, 0, sizeof(adapter));
	adapter[0] = 0x01;
	adapter[1] = 128;
	adapter[2] = 0;
	adapter[3] = 128;
	adapter[4] = 128;
	adapter[5] = 0x08;
	adapter[6] = 0x01;
	adapter[7] = 0x04;
	adapter[8] = 180;
	expect(hid_map_dinput(adapter, sizeof(adapter), &state) == 0, "8bitdo dinput parse");
	expect(state.buttons & PAD_BUTTON_CROSS, "8bitdo south -> cross");
	expect(state.buttons & PAD_BUTTON_PS, "8bitdo guide -> ps");
	expect((state.buttons & PAD_BUTTON_UP) == 0, "8bitdo centered hat");
	expect(state.left_y == 0, "8bitdo stick up stays small");
	expect(state.l2 == 180, "8bitdo left trigger");

	memset(shanwan, 0, sizeof(shanwan));
	shanwan[0] = 0x02;
	shanwan[1] = 0x12;
	shanwan[2] = 0x02;
	shanwan[3] = 128;
	shanwan[4] = 128;
	shanwan[17] = 90;
	expect(hid_map_dinput(shanwan, sizeof(shanwan), &state) == 0, "shanwan dinput parse");
	expect(state.buttons & PAD_BUTTON_CROSS, "shanwan cross");
	expect(state.buttons & PAD_BUTTON_OPTIONS, "shanwan start -> options");
	expect(state.buttons & PAD_BUTTON_PS, "shanwan sys -> ps");
	expect(state.buttons & PAD_BUTTON_RIGHT, "shanwan hat right");
	expect(state.l2 == 90, "shanwan left trigger");
	expect(hid_kind_for_ids(0x2dc8, 0x3100) == HID_KIND_DINPUT, "adapter v1 kind");
	expect(hid_kind_for_ids(0x2dc8, 0x3105) == HID_KIND_DINPUT, "adapter v2 kind");
	expect(hid_kind_for_ids(0x2dc8, 0x3106) == HID_KIND_X360, "adapter xinput kind");
	expect(hid_is_xbox360(0x045e, 0x028e), "wired 360 id");
}

static void test_xboxone(void)
{
	uint8_t report[48];
	uint8_t guide[6];
	PadState state;
	int16_t lx = 0;
	int16_t ly = 32767;
	uint16_t trigger = 1023;

	memset(report, 0, sizeof(report));
	memset(guide, 0, sizeof(guide));
	hid_map_xboxone_reset();
	report[0] = 0x20;
	report[4] = 0x10;
	report[5] = 0x01;
	memcpy(report + 6, &trigger, sizeof(trigger));
	memcpy(report + 10, &lx, sizeof(lx));
	memcpy(report + 12, &ly, sizeof(ly));

	expect(hid_map_xboxone(report, 18, &state) == 0, "xboxone parse");
	expect(state.buttons & PAD_BUTTON_CROSS, "xboxone A -> cross");
	expect(state.buttons & PAD_BUTTON_UP, "xboxone dpad up");
	expect(state.l2 == 255, "xboxone left trigger");
	expect(state.left_x == 128, "xboxone centered x");
	expect(state.left_y == 0, "xboxone stick up is small y");
	expect((state.buttons & PAD_BUTTON_PS) == 0, "xboxone guide starts clear");
	report[4] = 0x12;
	expect(hid_map_xboxone(report, 18, &state) == 0, "xboxone keepalive parse");
	expect(state.buttons & PAD_BUTTON_CROSS, "xboxone keepalive keeps A");
	expect((state.buttons & PAD_BUTTON_PS) == 0, "xboxone keepalive is not ps");
	report[4] = 0x04;
	expect(hid_map_xboxone(report, 18, &state) == 0, "xboxone menu parse");
	expect(state.buttons & PAD_BUTTON_OPTIONS, "xboxone menu -> options");
	expect((state.buttons & PAD_BUTTON_CROSS) == 0, "xboxone menu clears A");
	report[4] = 0x08;
	expect(hid_map_xboxone(report, 18, &state) == 0, "xboxone view parse");
	expect(state.buttons & PAD_BUTTON_CREATE, "xboxone view -> create");
	expect((state.buttons & PAD_BUTTON_OPTIONS) == 0, "xboxone view clears menu");
	report[4] = 0x00;
	report[5] = 0x00;
	expect(hid_map_xboxone(report, 18, &state) == 0, "xboxone release");
	expect((state.buttons & PAD_BUTTON_UP) == 0, "xboxone dpad release");
	report[4] = 0x10;
	report[5] = 0x00;
	expect(hid_map_xboxone(report, 18, &state) == 0, "xboxone A again");

	guide[0] = 0x07;
	guide[4] = 0x01;
	expect(hid_map_xboxone(guide, sizeof(guide), &state) == 0, "xboxone guide");
	expect(state.buttons & PAD_BUTTON_PS, "xboxone guide is ps");
	expect(state.buttons & PAD_BUTTON_CROSS, "xboxone guide keeps face buttons");

	guide[4] = 0x00;
	expect(hid_map_xboxone(guide, sizeof(guide), &state) == 0, "xboxone guide release");
	expect((state.buttons & PAD_BUTTON_PS) == 0, "xboxone guide released");

	hid_map_xboxone_set_share(1);
	report[3] = 44;
	report[22] = 0x01;
	expect(hid_map_xboxone(report, sizeof(report), &state) == 0, "xboxone series parse");
	expect(state.buttons & PAD_BUTTON_TOUCHPAD, "xboxone share");
	{
		uint8_t burst[36];

		memset(burst, 0, sizeof(burst));
		burst[0] = 0x20;
		burst[3] = 14;
		burst[4] = 0x10;
		burst[18] = 0x20;
		burst[21] = 14;
		expect(hid_map_xboxone(burst, sizeof(burst), &state) == 0, "xboxone burst parse");
		expect(state.buttons & PAD_BUTTON_CROSS, "xboxone burst shows the press");
		memset(report, 0, sizeof(report));
		report[0] = 0x20;
		report[3] = 14;
		expect(hid_map_xboxone(report, 18, &state) == 0, "xboxone newer report wins");
		expect((state.buttons & PAD_BUTTON_CROSS) == 0, "xboxone newer report released");
		expect(hid_map_xboxone_take_pending(&state) == 0, "xboxone pending cleared");
		burst[4] = 0x10;
		expect(hid_map_xboxone(burst, sizeof(burst), &state) == 0, "xboxone burst again");
		expect(state.buttons & PAD_BUTTON_CROSS, "xboxone burst press again");
		expect(hid_map_xboxone_take_pending(&state) == 1, "xboxone burst releases");
		expect((state.buttons & PAD_BUTTON_CROSS) == 0, "xboxone burst ended");
	}
	expect(hid_kind_for_ids(0x045e, 0x0b12) == HID_KIND_XBOXONE, "series kind");
	expect(hid_kind_for_ids(0x045e, 0x028e) == HID_KIND_X360, "360 stays 360");
	hid_map_xboxone_reset();
}

static void test_ds4(void)
{
	uint8_t report[64] = {0};
	PadState state;

	report[0] = 0x01;
	report[1] = 10;
	report[2] = 20;
	report[3] = 30;
	report[4] = 40;
	report[5] = 0x20 | 0x02;
	report[6] = 0x20;
	report[7] = 0x03;
	report[8] = 255;
	report[9] = 1;

	expect(hid_map_ds4(report, sizeof(report), &state) == 0, "ds4 parse");
	expect(state.left_x == 10 && state.left_y == 20, "ds4 sticks");
	expect(state.buttons & PAD_BUTTON_CROSS, "ds4 cross");
	expect(state.buttons & PAD_BUTTON_RIGHT, "ds4 hat east");
	expect(state.buttons & PAD_BUTTON_OPTIONS, "ds4 options");
	expect(state.buttons & PAD_BUTTON_TOUCHPAD, "ds4 touch click");
	expect(state.buttons & PAD_BUTTON_PS, "ds4 ps");
	expect(state.l2 == 255 && state.r2 == 1, "ds4 analog triggers");
}

static void test_switch(void)
{
	uint8_t report[64] = {0};
	PadState state;

	report[0] = 0x30;
	report[3] = 0x04;
	report[4] = 0x12;
	report[5] = 0x02;
	report[6] = 0x00;
	report[7] = 0x00;
	report[8] = 0x80;

	expect(hid_map_switch_pro(report, sizeof(report), &state) == 0, "switch parse");
	expect(state.buttons & PAD_BUTTON_CROSS, "switch B -> cross");
	expect(state.buttons & PAD_BUTTON_OPTIONS, "switch plus -> options");
	expect(state.buttons & PAD_BUTTON_PS, "switch home -> ps");
	report[0] = 0x21;
	expect(hid_map_switch_pro(report, sizeof(report), &state) == 0, "switch reply parse");
	expect(state.buttons & PAD_BUTTON_CROSS, "switch reply keeps cross");
	expect(state.buttons & PAD_BUTTON_UP, "switch up");
	expect(state.left_y < 128, "switch stick up is small y");
}

static void test_switch2(void)
{
	uint8_t report[64] = {0};
	PadState state;

	report[0] = 0x09;
	report[3] = 0x01;
	report[4] = 0x01;
	report[5] = 0x01;
	report[6] = 0x00;
	report[7] = 0x00;
	report[8] = 0x80;

	expect(hid_map_switch2_pro(report, sizeof(report), &state) == 0, "switch2 parse");
	expect(state.buttons & PAD_BUTTON_CROSS, "switch2 south -> cross");
	expect(state.buttons & PAD_BUTTON_DOWN, "switch2 dpad down");
	expect(state.buttons & PAD_BUTTON_PS, "switch2 home -> ps");
	expect(state.left_y < 128, "switch2 stick up is small y");
	expect(hid_map_switch_pro(report, sizeof(report), &state) != 0, "switch pro ignores 0x09");

	memset(report, 0, sizeof(report));
	report[0] = 0x3f;
	report[1] = 0x01;
	report[3] = 0x02;
	expect(hid_map_switch_pro(report, 12, &state) == 0, "switch simple parse");
	expect(state.buttons & PAD_BUTTON_CROSS, "switch simple B -> cross");
	expect(state.buttons & PAD_BUTTON_RIGHT, "switch simple hat right");

	memset(report, 0, sizeof(report));
	report[4] = 0x04;
	report[10] = 0x00;
	report[11] = 0x08;
	report[12] = 0x80;
	report[13] = 0x00;
	report[14] = 0x08;
	report[15] = 0x80;
	expect(hid_map_switch2_ble(report, 16, &state) == 0, "switch2 ble parse");
	expect(state.buttons & PAD_BUTTON_CROSS, "switch2 ble south -> cross");
	expect(state.left_x == 128, "switch2 ble centered x");
}

static void test_digital_triggers(void)
{
	PadState state;
	PadData data;

	pad_state_neutral(&state);
	state.l2 = 40;
	pad_data_from_state(&data, &state, 5);
	expect(data.connected == 1, "connected");
	expect(data.buttons & PAD_BUTTON_L2, "analog l2 sets the digital bit");
	expect(data.timestamp_us == 5, "timestamp copied");
	expect(data.orient_w == 1.0f, "identity orientation");
}

int main(void)
{
	test_layout();
	test_x360();
	test_ps3();
	test_dinput();
	test_xboxone();
	test_ds4();
	test_switch();
	test_switch2();
	test_digital_triggers();
	if (g_failed) {
		fprintf(stderr, "%d checks failed\n", g_failed);
		return 1;
	}
	puts("ok");
	return 0;
}
