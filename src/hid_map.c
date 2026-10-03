#include "hid_map.h"

#include <string.h>

uint8_t hid_axis16_to_u8(int16_t axis, int invert_y)
{
	int value = ((int)axis + 32768) >> 8;
	if (value > 255)
		value = 255;
	if (invert_y)
		value = 255 - value;
	return (uint8_t)value;
}

uint8_t hid_axis12_to_u8(uint16_t axis, int invert_y)
{
	unsigned value = axis & 0x0fffu;
	value >>= 4;
	if (value > 255)
		value = 255;
	if (invert_y)
		value = 255 - value;
	return (uint8_t)value;
}

int hid_is_xboxone(uint16_t vendor_id, uint16_t product_id)
{
	if (vendor_id != HID_VID_MICROSOFT)
		return 0;
	switch (product_id) {
	case 0x02d1: /* Xbox One */
	case 0x02dd: /* Xbox One, 2015 firmware */
	case 0x02e3: /* Xbox One Elite */
	case 0x02ea: /* Xbox One S */
	case 0x0b00: /* Xbox Elite Series 2 */
	case 0x0b0a: /* Xbox Adaptive Controller */
	case 0x0b12: /* Xbox Series S|X */
		return 1;
	default:
		return 0;
	}
}

int hid_is_ps3(uint16_t vendor_id, uint16_t product_id)
{
	return vendor_id == HID_VID_SONY && product_id == HID_PID_DS3;
}

int hid_is_xbox360(uint16_t vendor_id, uint16_t product_id)
{
	return vendor_id == HID_VID_MICROSOFT &&
	       (product_id == 0x028eu || product_id == 0x028fu);
}

int hid_8bitdo_mode(uint16_t vendor_id, uint16_t product_id)
{
	if (vendor_id != HID_VID_8BITDO)
		return HID_8BITDO_NONE;
	switch (product_id) {
	case 0x3016: /* wired XInput */
	case 0x3106: /* Adapter 2, XInput */
	case 0x3107:
	case 0x3109:
	case 0x310a:
	case 0x310b:
		return HID_8BITDO_XINPUT;
	case 0x3100: /* USB Wireless Adapter */
	case 0x3101: /* receiver */
	case 0x3102:
	case 0x3103:
	case 0x3104:
	case 0x3105: /* USB Wireless Adapter 2 */
		return HID_8BITDO_DINPUT;
	default:
		return HID_8BITDO_NONE;
	}
}

enum HidKind hid_kind_for_ids(uint16_t vendor_id, uint16_t product_id)
{
	if (hid_is_ps3(vendor_id, product_id))
		return HID_KIND_PS3;
	if (vendor_id == HID_VID_SONY &&
	    (product_id == HID_PID_DS4_V1 || product_id == HID_PID_DS4_V2))
		return HID_KIND_DS4;
	if (vendor_id == HID_VID_NINTENDO && product_id == HID_PID_SWITCH_PRO)
		return HID_KIND_SWITCH_PRO;
	if (hid_is_xboxone(vendor_id, product_id))
		return HID_KIND_XBOXONE;
	if (hid_is_xbox360(vendor_id, product_id) ||
	    hid_8bitdo_mode(vendor_id, product_id) == HID_8BITDO_XINPUT)
		return HID_KIND_X360;
	if (hid_8bitdo_mode(vendor_id, product_id) == HID_8BITDO_DINPUT)
		return HID_KIND_DINPUT;
	if (vendor_id == HID_VID_MICROSOFT)
		return HID_KIND_X360;
	return HID_KIND_UNKNOWN;
}

static void apply_hat(PadState *out, unsigned hat)
{
	switch (hat) {
	case 0:
		out->buttons |= PAD_BUTTON_UP;
		break;
	case 1:
		out->buttons |= PAD_BUTTON_UP | PAD_BUTTON_RIGHT;
		break;
	case 2:
		out->buttons |= PAD_BUTTON_RIGHT;
		break;
	case 3:
		out->buttons |= PAD_BUTTON_DOWN | PAD_BUTTON_RIGHT;
		break;
	case 4:
		out->buttons |= PAD_BUTTON_DOWN;
		break;
	case 5:
		out->buttons |= PAD_BUTTON_DOWN | PAD_BUTTON_LEFT;
		break;
	case 6:
		out->buttons |= PAD_BUTTON_LEFT;
		break;
	case 7:
		out->buttons |= PAD_BUTTON_UP | PAD_BUTTON_LEFT;
		break;
	default:
		break;
	}
}

static int g_x360_defer_ticks;
static PadState g_x360_defer_release;

static int pad_level_down(const PadState *state)
{
	return state->buttons != 0 || state->l2 >= 30 || state->r2 >= 30;
}

static void x360_apply_frame(const uint8_t *report, PadState *out)
{
	int16_t axis;
	uint8_t b1;
	uint8_t b2;

	pad_state_neutral(out);
	b1 = report[2];
	b2 = report[3];
	/* A hat value of "every direction" is the neutral code on some clones. */
	if ((b1 & 0x0f) == 0x0f)
		b1 = (uint8_t)(b1 & ~0x0fu);

	if (b1 & 0x01) out->buttons |= PAD_BUTTON_UP;
	if (b1 & 0x02) out->buttons |= PAD_BUTTON_DOWN;
	if (b1 & 0x04) out->buttons |= PAD_BUTTON_LEFT;
	if (b1 & 0x08) out->buttons |= PAD_BUTTON_RIGHT;
	if (b1 & 0x10) out->buttons |= PAD_BUTTON_OPTIONS;
	if (b1 & 0x20) out->buttons |= PAD_BUTTON_CREATE;
	if (b1 & 0x40) out->buttons |= PAD_BUTTON_L3;
	if (b1 & 0x80) out->buttons |= PAD_BUTTON_R3;

	if (b2 & 0x01) out->buttons |= PAD_BUTTON_L1;
	if (b2 & 0x02) out->buttons |= PAD_BUTTON_R1;
	if (b2 & 0x04) out->buttons |= PAD_BUTTON_PS;
	if (b2 & 0x10) out->buttons |= PAD_BUTTON_CROSS;
	if (b2 & 0x20) out->buttons |= PAD_BUTTON_CIRCLE;
	if (b2 & 0x40) out->buttons |= PAD_BUTTON_SQUARE;
	if (b2 & 0x80) out->buttons |= PAD_BUTTON_TRIANGLE;

	out->l2 = report[4];
	out->r2 = report[5];

	memcpy(&axis, report + 6, sizeof(axis));
	out->left_x = hid_axis16_to_u8(axis, 0);
	memcpy(&axis, report + 8, sizeof(axis));
	out->left_y = hid_axis16_to_u8(axis, 1);
	memcpy(&axis, report + 10, sizeof(axis));
	out->right_x = hid_axis16_to_u8(axis, 0);
	memcpy(&axis, report + 12, sizeof(axis));
	out->right_y = hid_axis16_to_u8(axis, 1);
}

void hid_map_x360_reset(void)
{
	g_x360_defer_ticks = 0;
	pad_state_neutral(&g_x360_defer_release);
}

int hid_map_x360_take_pending(PadState *out)
{
	if (out == NULL || g_x360_defer_ticks <= 0)
		return 0;
	g_x360_defer_ticks = 0;
	*out = g_x360_defer_release;
	return 1;
}

int hid_map_x360(const uint8_t *report, size_t length, PadState *out)
{
	size_t offset = 0;
	int applied = -1;
	int saw_down = 0;
	PadState down;
	PadState final;

	if (report == NULL || out == NULL || length < 14)
		return -1;
	if (!(report[0] == 0x00 && report[1] == 0x14) &&
	    length >= 15 && report[1] == 0x00 && report[2] == 0x14)
		offset = 1;

	pad_state_neutral(&down);
	pad_state_neutral(&final);
	while (offset + 14 <= length) {
		if (report[offset] != 0x00 || report[offset + 1] != 0x14)
			break;
		x360_apply_frame(report + offset, &final);
		if (!saw_down && pad_level_down(&final)) {
			down = final;
			saw_down = 1;
		}
		applied = 0;
		if (offset + 20 + 14 > length)
			break;
		offset += 20;
	}
	if (applied != 0)
		return -1;

	/* Press and release in one read: show the press once, then the release. */
	if (saw_down && !pad_level_down(&final)) {
		g_x360_defer_release = final;
		g_x360_defer_ticks = 1;
		*out = down;
		return 0;
	}
	g_x360_defer_ticks = 0;
	*out = final;
	return 0;
}

static int g_xbox_share;
static int g_xbox_guide;
static int g_xbox_have;
static int g_xbox_defer_ticks;
static PadState g_xbox_last;
static PadState g_xbox_defer_press;
static PadState g_xbox_defer_release;

void hid_map_xboxone_set_share(int enabled)
{
	g_xbox_share = enabled ? 1 : 0;
}

void hid_map_xboxone_reset(void)
{
	g_xbox_guide = 0;
	g_xbox_have = 0;
	g_xbox_share = 0;
	g_xbox_defer_ticks = 0;
	pad_state_neutral(&g_xbox_last);
	pad_state_neutral(&g_xbox_defer_press);
	pad_state_neutral(&g_xbox_defer_release);
}

int hid_map_xboxone_take_pending(PadState *out)
{
	if (out == NULL || g_xbox_defer_ticks <= 0)
		return 0;
	if (--g_xbox_defer_ticks > 0)
		*out = g_xbox_defer_press;
	else
		*out = g_xbox_defer_release;
	g_xbox_last = *out;
	g_xbox_have = 1;
	return 1;
}

static int xbox_guide_pressed(const uint8_t *report, size_t length)
{
	if (length >= 6 && report[4] == 0x5b)
		return (report[5] & 0x01) != 0;
	if (length >= 5)
		return (report[4] & 0x03) != 0;
	return 0;
}

static void xbox_apply_guide(PadState *out)
{
	if (g_xbox_guide)
		out->buttons |= PAD_BUTTON_PS;
	else
		out->buttons &= ~PAD_BUTTON_PS;
}

static void xbox_note_guide(const uint8_t *report, size_t length)
{
	g_xbox_guide = xbox_guide_pressed(report, length);
	if (!g_xbox_have)
		pad_state_neutral(&g_xbox_last);
	xbox_apply_guide(&g_xbox_last);
}

static int xbox_map_input(const uint8_t *report, size_t length, size_t payload, PadState *out)
{
	uint16_t buttons;
	uint16_t trigger;
	int16_t axis;
	size_t share;

	if (payload < 0x0e || length < 18)
		return -1;

	pad_state_neutral(out);
	buttons = (uint16_t)(report[4] | ((uint16_t)report[5] << 8));
	/* Bit 0x0002 is the GIP keep-alive flag, not a button. */
	buttons &= (uint16_t)~0x0002u;
	if (buttons & 0x0004) out->buttons |= PAD_BUTTON_OPTIONS;
	if (buttons & 0x0008) out->buttons |= PAD_BUTTON_CREATE;
	if (buttons & 0x0010) out->buttons |= PAD_BUTTON_CROSS;
	if (buttons & 0x0020) out->buttons |= PAD_BUTTON_CIRCLE;
	if (buttons & 0x0040) out->buttons |= PAD_BUTTON_SQUARE;
	if (buttons & 0x0080) out->buttons |= PAD_BUTTON_TRIANGLE;
	if (buttons & 0x0100) out->buttons |= PAD_BUTTON_UP;
	if (buttons & 0x0200) out->buttons |= PAD_BUTTON_DOWN;
	if (buttons & 0x0400) out->buttons |= PAD_BUTTON_LEFT;
	if (buttons & 0x0800) out->buttons |= PAD_BUTTON_RIGHT;
	if (buttons & 0x1000) out->buttons |= PAD_BUTTON_L1;
	if (buttons & 0x2000) out->buttons |= PAD_BUTTON_R1;
	if (buttons & 0x4000) out->buttons |= PAD_BUTTON_L3;
	if (buttons & 0x8000) out->buttons |= PAD_BUTTON_R3;

	memcpy(&trigger, report + 6, sizeof(trigger));
	out->l2 = (uint8_t)(trigger > 1023 ? 255 : (trigger >> 2));
	memcpy(&trigger, report + 8, sizeof(trigger));
	out->r2 = (uint8_t)(trigger > 1023 ? 255 : (trigger >> 2));

	memcpy(&axis, report + 10, sizeof(axis));
	out->left_x = hid_axis16_to_u8(axis, 0);
	memcpy(&axis, report + 12, sizeof(axis));
	out->left_y = hid_axis16_to_u8(axis, 1);
	memcpy(&axis, report + 14, sizeof(axis));
	out->right_x = hid_axis16_to_u8(axis, 0);
	memcpy(&axis, report + 16, sizeof(axis));
	out->right_y = hid_axis16_to_u8(axis, 1);

	share = 0;
	if (g_xbox_share) {
		if (payload < 44 && payload > 14)
			share = 4 + 14;
		else if (payload == 44)
			share = 4 + 18;
		else if (payload == 46)
			share = 4 + 28;
		else if (payload >= 60)
			share = 4 + 42;
	}
	if (share != 0 && share < length && (report[share] & 0x01))
		out->buttons |= PAD_BUTTON_TOUCHPAD;

	xbox_apply_guide(out);
	g_xbox_last = *out;
	g_xbox_have = 1;
	return 0;
}

static int xbox_buttons_down(const PadState *state)
{
	return (state->buttons & ~PAD_BUTTON_PS) != 0 || state->l2 >= 30 || state->r2 >= 30;
}

int hid_map_xboxone(const uint8_t *report, size_t length, PadState *out)
{
	size_t offset = 0;
	int applied = -1;
	int saw_down = 0;
	PadState down;
	PadState final;

	if (report == NULL || out == NULL || length < 4)
		return -1;

	pad_state_neutral(&down);
	pad_state_neutral(&final);
	while (offset + 4 <= length) {
		const uint8_t *msg = report + offset;
		size_t payload = msg[3];
		size_t total;
		int chunked = (msg[1] & 0xc0) != 0;

		if (payload < 2 || payload > 60 || offset + 4 + payload > length) {
			if (offset > 0)
				break;
			if (chunked)
				return -1;
			if (msg[0] == 0x07 && length >= 5) {
				xbox_note_guide(msg, length);
				*out = g_xbox_last;
				g_xbox_defer_ticks = 0;
				return 0;
			}
			if (msg[0] == 0x20 && length >= 18) {
				g_xbox_defer_ticks = 0;
				return xbox_map_input(msg, length, length - 4, out);
			}
			return -1;
		}

		total = 4 + payload;
		if (!chunked && msg[0] == 0x07) {
			xbox_note_guide(msg, total);
			final = g_xbox_last;
			*out = final;
			applied = 0;
		} else if (!chunked && msg[0] == 0x20 &&
			   xbox_map_input(msg, total, payload, out) == 0) {
			if (!saw_down && xbox_buttons_down(out)) {
				down = *out;
				saw_down = 1;
			}
			final = *out;
			applied = 0;
		}
		offset += total;
	}
	if (applied != 0)
		return applied;

	/*
	 * Press and release in one packet: show the press for this sample only.
	 * The next idle poll returns the release. A newer report below clears
	 * that and uses the pad's current buttons, so nothing stays held.
	 */
	if (saw_down && !xbox_buttons_down(&final)) {
		g_xbox_defer_press = down;
		g_xbox_defer_release = final;
		g_xbox_defer_ticks = 1;
		*out = down;
		g_xbox_last = down;
		g_xbox_have = 1;
		return 0;
	}
	g_xbox_defer_ticks = 0;
	*out = final;
	g_xbox_last = final;
	g_xbox_have = 1;
	return 0;
}

int hid_map_ds4(const uint8_t *report, size_t length, PadState *out)
{
	const uint8_t *body;
	uint8_t buttons0;
	uint8_t buttons1;

	if (report == NULL || out == NULL || length < 10)
		return -1;

	body = report;
	if (report[0] == 0x01) {
		if (length < 11)
			return -1;
		body = report + 1;
	}

	pad_state_neutral(out);
	out->left_x = body[0];
	out->left_y = body[1];
	out->right_x = body[2];
	out->right_y = body[3];

	buttons0 = body[4];
	buttons1 = body[5];
	apply_hat(out, buttons0 & 0x0fu);
	if (buttons0 & 0x10) out->buttons |= PAD_BUTTON_SQUARE;
	if (buttons0 & 0x20) out->buttons |= PAD_BUTTON_CROSS;
	if (buttons0 & 0x40) out->buttons |= PAD_BUTTON_CIRCLE;
	if (buttons0 & 0x80) out->buttons |= PAD_BUTTON_TRIANGLE;
	if (buttons1 & 0x01) out->buttons |= PAD_BUTTON_L1;
	if (buttons1 & 0x02) out->buttons |= PAD_BUTTON_R1;
	if (buttons1 & 0x04) out->buttons |= PAD_BUTTON_L2;
	if (buttons1 & 0x08) out->buttons |= PAD_BUTTON_R2;
	if (buttons1 & 0x10) out->buttons |= PAD_BUTTON_CREATE;
	if (buttons1 & 0x20) out->buttons |= PAD_BUTTON_OPTIONS;
	if (buttons1 & 0x40) out->buttons |= PAD_BUTTON_L3;
	if (buttons1 & 0x80) out->buttons |= PAD_BUTTON_R3;

	if (length > (size_t)(body - report) + 8) {
		out->l2 = body[7];
		out->r2 = body[8];
	}
	if ((size_t)(body - report) + 6 < length && (body[6] & 0x01))
		out->buttons |= PAD_BUTTON_PS;
	if ((size_t)(body - report) + 6 < length && (body[6] & 0x02))
		out->buttons |= PAD_BUTTON_TOUCHPAD;
	return 0;
}

int hid_map_switch_pro(const uint8_t *report, size_t length, PadState *out)
{
	const uint8_t *body;
	uint16_t lx;
	uint16_t ly;
	uint16_t rx;
	uint16_t ry;
	uint8_t right;
	uint8_t shared;
	uint8_t left;

	if (report == NULL || out == NULL || length < 12)
		return -1;
	/* Simple Bluetooth report, before the full 0x30 mode is enabled. */
	if (report[0] == 0x3f && length >= 12) {
		int16_t axis;

		pad_state_neutral(out);
		if (report[1] & 0x01) out->buttons |= PAD_BUTTON_CROSS;
		if (report[1] & 0x02) out->buttons |= PAD_BUTTON_CIRCLE;
		if (report[1] & 0x04) out->buttons |= PAD_BUTTON_SQUARE;
		if (report[1] & 0x08) out->buttons |= PAD_BUTTON_TRIANGLE;
		if (report[1] & 0x10) out->buttons |= PAD_BUTTON_L1;
		if (report[1] & 0x20) out->buttons |= PAD_BUTTON_R1;
		if (report[1] & 0x40) out->l2 = 255;
		if (report[1] & 0x80) out->r2 = 255;
		if (report[2] & 0x01) out->buttons |= PAD_BUTTON_CREATE;
		if (report[2] & 0x02) out->buttons |= PAD_BUTTON_OPTIONS;
		if (report[2] & 0x04) out->buttons |= PAD_BUTTON_L3;
		if (report[2] & 0x08) out->buttons |= PAD_BUTTON_R3;
		if (report[2] & 0x10) out->buttons |= PAD_BUTTON_PS;
		apply_hat(out, report[3] & 0x0f);
		memcpy(&axis, report + 4, sizeof(axis));
		out->left_x = hid_axis16_to_u8(axis, 0);
		memcpy(&axis, report + 6, sizeof(axis));
		out->left_y = hid_axis16_to_u8(axis, 1);
		memcpy(&axis, report + 8, sizeof(axis));
		out->right_x = hid_axis16_to_u8(axis, 0);
		memcpy(&axis, report + 10, sizeof(axis));
		out->right_y = hid_axis16_to_u8(axis, 1);
		if (out->l2 >= 30)
			out->buttons |= PAD_BUTTON_L2;
		if (out->r2 >= 30)
			out->buttons |= PAD_BUTTON_R2;
		return 0;
	}
	if (length < 13)
		return -1;
	/* 0x21 replies and 0x30/0x31/0x32/0x33 input share the same button block. */
	if (report[0] != 0x21 && report[0] != 0x30 && report[0] != 0x31 &&
	    report[0] != 0x32 && report[0] != 0x33)
		return -1;

	body = report + 3;
	right = body[0];
	shared = body[1];
	left = body[2];

	pad_state_neutral(out);
	/* Western face map: B cross, A circle, Y square, X triangle. */
	if (right & 0x01) out->buttons |= PAD_BUTTON_SQUARE;
	if (right & 0x02) out->buttons |= PAD_BUTTON_TRIANGLE;
	if (right & 0x04) out->buttons |= PAD_BUTTON_CROSS;
	if (right & 0x08) out->buttons |= PAD_BUTTON_CIRCLE;
	if (right & 0x40) out->buttons |= PAD_BUTTON_R1;
	if (right & 0x80) out->buttons |= PAD_BUTTON_R2;

	if (shared & 0x01) out->buttons |= PAD_BUTTON_CREATE;
	if (shared & 0x02) out->buttons |= PAD_BUTTON_OPTIONS;
	if (shared & 0x04) out->buttons |= PAD_BUTTON_R3;
	if (shared & 0x08) out->buttons |= PAD_BUTTON_L3;
	if (shared & 0x10) out->buttons |= PAD_BUTTON_PS;

	if (left & 0x01) out->buttons |= PAD_BUTTON_DOWN;
	if (left & 0x02) out->buttons |= PAD_BUTTON_UP;
	if (left & 0x04) out->buttons |= PAD_BUTTON_RIGHT;
	if (left & 0x08) out->buttons |= PAD_BUTTON_LEFT;
	if (left & 0x40) out->buttons |= PAD_BUTTON_L1;
	if (left & 0x80) out->buttons |= PAD_BUTTON_L2;

	lx = (uint16_t)(body[3] | ((body[4] & 0x0f) << 8));
	ly = (uint16_t)((body[4] >> 4) | (body[5] << 4));
	rx = (uint16_t)(body[6] | ((body[7] & 0x0f) << 8));
	ry = (uint16_t)((body[7] >> 4) | (body[8] << 4));
	out->left_x = hid_axis12_to_u8(lx, 0);
	out->left_y = hid_axis12_to_u8(ly, 1);
	out->right_x = hid_axis12_to_u8(rx, 0);
	out->right_y = hid_axis12_to_u8(ry, 1);

	if (out->buttons & PAD_BUTTON_L2)
		out->l2 = 255;
	if (out->buttons & PAD_BUTTON_R2)
		out->r2 = 255;
	return 0;
}

int hid_map_switch2_pro(const uint8_t *report, size_t length, PadState *out)
{
	const uint8_t *body;
	uint16_t lx;
	uint16_t ly;
	uint16_t rx;
	uint16_t ry;
	uint8_t right;
	uint8_t shared;
	uint8_t left;

	if (report == NULL || out == NULL || length < 12)
		return -1;
	if (report[0] != 0x09 && report[0] != 0x30 && report[0] != 0x31)
		return -1;

	/* Same 12-bit sticks as the original Pro, starting after id, timer, and battery. */
	body = report + 3;
	right = body[0];
	shared = body[1];
	left = body[2];
	pad_state_neutral(out);

	/* Face order is B A Y X. South is Cross, east Circle, west Square, north Triangle. */
	if (right & 0x01) out->buttons |= PAD_BUTTON_CROSS;
	if (right & 0x02) out->buttons |= PAD_BUTTON_CIRCLE;
	if (right & 0x04) out->buttons |= PAD_BUTTON_SQUARE;
	if (right & 0x08) out->buttons |= PAD_BUTTON_TRIANGLE;
	if (right & 0x10) out->buttons |= PAD_BUTTON_R1;
	if (right & 0x20) out->r2 = 255;
	if (right & 0x40) out->buttons |= PAD_BUTTON_OPTIONS;
	if (right & 0x80) out->buttons |= PAD_BUTTON_L3;

	if (shared & 0x01) out->buttons |= PAD_BUTTON_DOWN;
	if (shared & 0x02) out->buttons |= PAD_BUTTON_RIGHT;
	if (shared & 0x04) out->buttons |= PAD_BUTTON_LEFT;
	if (shared & 0x08) out->buttons |= PAD_BUTTON_UP;
	if (shared & 0x10) out->buttons |= PAD_BUTTON_L1;
	if (shared & 0x20) out->l2 = 255;
	if (shared & 0x40) out->buttons |= PAD_BUTTON_CREATE;
	if (shared & 0x80) out->buttons |= PAD_BUTTON_R3;

	if (left & 0x01) out->buttons |= PAD_BUTTON_PS;

	lx = (uint16_t)(body[3] | ((body[4] & 0x0f) << 8));
	ly = (uint16_t)((body[4] >> 4) | (body[5] << 4));
	rx = (uint16_t)(body[6] | ((body[7] & 0x0f) << 8));
	ry = (uint16_t)((body[7] >> 4) | (body[8] << 4));
	out->left_x = hid_axis12_to_u8(lx, 0);
	out->left_y = hid_axis12_to_u8(ly, 1);
	out->right_x = hid_axis12_to_u8(rx, 0);
	out->right_y = hid_axis12_to_u8(ry, 1);
	if (out->l2 >= 30)
		out->buttons |= PAD_BUTTON_L2;
	if (out->r2 >= 30)
		out->buttons |= PAD_BUTTON_R2;
	return 0;
}

static void switch2_stick12(const uint8_t *packed, uint8_t *x, uint8_t *y)
{
	uint32_t value = (uint32_t)packed[0] | ((uint32_t)packed[1] << 8) | ((uint32_t)packed[2] << 16);

	*x = hid_axis12_to_u8((uint16_t)(value & 0x0fff), 0);
	*y = hid_axis12_to_u8((uint16_t)((value >> 12) & 0x0fff), 1);
}

int hid_map_switch2_ble(const uint8_t *report, size_t length, PadState *out)
{
	uint32_t buttons;

	if (report == NULL || out == NULL || length < 16)
		return -1;
	if (report[0] == 0x09 || report[0] == 0x30 || report[0] == 0x31)
		return hid_map_switch2_pro(report, length, out);

	pad_state_neutral(out);
	memcpy(&buttons, report + 4, sizeof(buttons));
	buttons &= 0x03ffffffu;
	if (buttons & 0x04) out->buttons |= PAD_BUTTON_CROSS;
	if (buttons & 0x08) out->buttons |= PAD_BUTTON_CIRCLE;
	if (buttons & 0x02) out->buttons |= PAD_BUTTON_SQUARE;
	if (buttons & 0x01) out->buttons |= PAD_BUTTON_TRIANGLE;
	if (buttons & 0x40) out->buttons |= PAD_BUTTON_R1;
	if (buttons & 0x80) out->r2 = 255;
	if (buttons & 0x400000) out->buttons |= PAD_BUTTON_L1;
	if (buttons & 0x800000) out->l2 = 255;
	if (buttons & 0x10000) out->buttons |= PAD_BUTTON_DOWN;
	if (buttons & 0x20000) out->buttons |= PAD_BUTTON_UP;
	if (buttons & 0x40000) out->buttons |= PAD_BUTTON_RIGHT;
	if (buttons & 0x80000) out->buttons |= PAD_BUTTON_LEFT;
	if (buttons & 0x100) out->buttons |= PAD_BUTTON_CREATE;
	if (buttons & 0x200) out->buttons |= PAD_BUTTON_OPTIONS;
	if (buttons & 0x400) out->buttons |= PAD_BUTTON_R3;
	if (buttons & 0x800) out->buttons |= PAD_BUTTON_L3;
	if (buttons & 0x001000) out->buttons |= PAD_BUTTON_PS;
	if (buttons & 0x100000) out->buttons |= PAD_BUTTON_PS;
	if (length >= 16) {
		switch2_stick12(report + 10, &out->left_x, &out->left_y);
		switch2_stick12(report + 13, &out->right_x, &out->right_y);
	}
	if (out->l2 >= 30)
		out->buttons |= PAD_BUTTON_L2;
	if (out->r2 >= 30)
		out->buttons |= PAD_BUTTON_R2;
	return 0;
}

int hid_map_ps3(const uint8_t *report, size_t length, PadState *out)
{
	uint8_t buttons0;
	uint8_t buttons1;
	uint8_t buttons2;

	if (report == NULL || out == NULL || length < 10 || report[0] != 0x01)
		return -1;

	pad_state_neutral(out);
	buttons0 = report[2];
	buttons1 = report[3];
	buttons2 = report[4];

	if (buttons0 & 0x01) out->buttons |= PAD_BUTTON_CREATE;
	if (buttons0 & 0x02) out->buttons |= PAD_BUTTON_L3;
	if (buttons0 & 0x04) out->buttons |= PAD_BUTTON_R3;
	if (buttons0 & 0x08) out->buttons |= PAD_BUTTON_OPTIONS;
	if (buttons0 & 0x10) out->buttons |= PAD_BUTTON_UP;
	if (buttons0 & 0x20) out->buttons |= PAD_BUTTON_RIGHT;
	if (buttons0 & 0x40) out->buttons |= PAD_BUTTON_DOWN;
	if (buttons0 & 0x80) out->buttons |= PAD_BUTTON_LEFT;

	if (buttons1 & 0x01) out->buttons |= PAD_BUTTON_L2;
	if (buttons1 & 0x02) out->buttons |= PAD_BUTTON_R2;
	if (buttons1 & 0x04) out->buttons |= PAD_BUTTON_L1;
	if (buttons1 & 0x08) out->buttons |= PAD_BUTTON_R1;
	if (buttons1 & 0x10) out->buttons |= PAD_BUTTON_TRIANGLE;
	if (buttons1 & 0x20) out->buttons |= PAD_BUTTON_CIRCLE;
	if (buttons1 & 0x40) out->buttons |= PAD_BUTTON_CROSS;
	if (buttons1 & 0x80) out->buttons |= PAD_BUTTON_SQUARE;
	if (buttons2 & 0x01) out->buttons |= PAD_BUTTON_PS;

	out->left_x = report[6];
	out->left_y = report[7];
	out->right_x = report[8];
	out->right_y = report[9];
	if (length > 19) {
		out->l2 = report[18];
		out->r2 = report[19];
	}
	if (out->l2 == 0 && (buttons1 & 0x01))
		out->l2 = 255;
	if (out->r2 == 0 && (buttons1 & 0x02))
		out->r2 = 255;
	return 0;
}

static int dinput_hat_byte(uint8_t hat)
{
	return hat <= 8 || hat == 0x0f;
}

static void dinput_map_shanwan(const uint8_t *report, size_t length, PadState *out)
{
	pad_state_neutral(out);
	if (report[0] & 0x01) out->buttons |= PAD_BUTTON_SQUARE;
	if (report[0] & 0x02) out->buttons |= PAD_BUTTON_CROSS;
	if (report[0] & 0x04) out->buttons |= PAD_BUTTON_CIRCLE;
	if (report[0] & 0x08) out->buttons |= PAD_BUTTON_TRIANGLE;
	if (report[0] & 0x10) out->buttons |= PAD_BUTTON_L1;
	if (report[0] & 0x20) out->buttons |= PAD_BUTTON_R1;
	if (report[0] & 0x40) out->buttons |= PAD_BUTTON_L2;
	if (report[0] & 0x80) out->buttons |= PAD_BUTTON_R2;
	if (length > 1) {
		if (report[1] & 0x01) out->buttons |= PAD_BUTTON_CREATE;
		if (report[1] & 0x02) out->buttons |= PAD_BUTTON_OPTIONS;
		if (report[1] & 0x04) out->buttons |= PAD_BUTTON_L3;
		if (report[1] & 0x08) out->buttons |= PAD_BUTTON_R3;
		if (report[1] & 0x10) out->buttons |= PAD_BUTTON_PS;
		if (report[1] & 0x20) out->buttons |= PAD_BUTTON_TOUCHPAD;
	}
	if (length > 2)
		apply_hat(out, report[2] & 0x0f);
	if (length > 6) {
		out->left_x = report[3];
		out->left_y = report[4];
		out->right_x = report[5];
		out->right_y = report[6];
	}
	if (length >= 19) {
		out->l2 = report[17];
		out->r2 = report[18];
	}
	if (out->l2 == 0 && (report[0] & 0x40))
		out->l2 = 255;
	if (out->r2 == 0 && (report[0] & 0x80))
		out->r2 = 255;
}

/* Sticks, then hat, then buttons in south/east/west/north order. */
static void dinput_map_axes_first(const uint8_t *body, size_t length, PadState *out)
{
	uint8_t buttons0 = length > 5 ? body[5] : 0;
	uint8_t buttons1 = length > 6 ? body[6] : 0;

	pad_state_neutral(out);
	if (length > 4)
		apply_hat(out, body[4] & 0x0f);
	if (buttons0 & 0x01) out->buttons |= PAD_BUTTON_CROSS;
	if (buttons0 & 0x02) out->buttons |= PAD_BUTTON_CIRCLE;
	if (buttons0 & 0x04) out->buttons |= PAD_BUTTON_SQUARE;
	if (buttons0 & 0x08) out->buttons |= PAD_BUTTON_TRIANGLE;
	if (buttons0 & 0x10) out->buttons |= PAD_BUTTON_L1;
	if (buttons0 & 0x20) out->buttons |= PAD_BUTTON_R1;
	if (buttons0 & 0x40) out->buttons |= PAD_BUTTON_CREATE;
	if (buttons0 & 0x80) out->buttons |= PAD_BUTTON_OPTIONS;
	if (buttons1 & 0x01) out->buttons |= PAD_BUTTON_L3;
	if (buttons1 & 0x02) out->buttons |= PAD_BUTTON_R3;
	if (buttons1 & 0x04) out->buttons |= PAD_BUTTON_PS;
	if (length > 3) {
		out->left_x = body[0];
		out->left_y = body[1];
		out->right_x = body[2];
		out->right_y = body[3];
	}
	if (length > 8) {
		out->l2 = body[7];
		out->r2 = body[8];
	}
}

int hid_map_dinput(const uint8_t *report, size_t length, PadState *out)
{
	if (report == NULL || out == NULL || length < 7)
		return -1;

	/*
	 * 8BitDo D-input adapters send four axes, a hat, then buttons, usually
	 * behind report id 0x01. A 19-byte Shanwan-style report starts with the
	 * button bytes, so a stick byte of 0x80 must not be treated as a hat.
	 */
	if (report[0] == 0x01 && length >= 8 && dinput_hat_byte(report[5])) {
		dinput_map_axes_first(report + 1, length - 1, out);
		return 0;
	}
	if (length <= 12 && report[0] >= 0x20 && dinput_hat_byte(report[4])) {
		dinput_map_axes_first(report, length, out);
		return 0;
	}
	dinput_map_shanwan(report, length, out);
	return 0;
}

int hid_map_report(enum HidKind kind, const uint8_t *report, size_t length, PadState *out)
{
	if (report == NULL || length == 0)
		return -1;

	if (kind == HID_KIND_UNKNOWN) {
		if (report[0] == 0x00 && length >= 14 && report[1] == 0x14)
			kind = HID_KIND_X360;
		else if (report[0] == 0x09)
			kind = HID_KIND_SWITCH2_PRO;
		else if (report[0] == 0x21 || report[0] == 0x30 || report[0] == 0x31 ||
			 report[0] == 0x32 || report[0] == 0x33)
			kind = HID_KIND_SWITCH_PRO;
		else if (report[0] == 0x01)
			kind = HID_KIND_DS4;
	}

	switch (kind) {
	case HID_KIND_X360:
		return hid_map_x360(report, length, out);
	case HID_KIND_XBOXONE:
		return hid_map_xboxone(report, length, out);
	case HID_KIND_DS4:
		return hid_map_ds4(report, length, out);
	case HID_KIND_PS3:
		return hid_map_ps3(report, length, out);
	case HID_KIND_DINPUT:
		return hid_map_dinput(report, length, out);
	case HID_KIND_SWITCH_PRO:
		return hid_map_switch_pro(report, length, out);
	case HID_KIND_SWITCH2_PRO:
		return hid_map_switch2_pro(report, length, out);
	default:
		return -1;
	}
}
