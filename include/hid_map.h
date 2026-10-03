#ifndef YETANOTHERCONTROLLERENABLER_HID_MAP_H
#define YETANOTHERCONTROLLERENABLER_HID_MAP_H

#include "pad_data.h"

#include <stddef.h>
#include <stdint.h>

#define HID_VID_MICROSOFT 0x045eu
#define HID_VID_SONY      0x054cu
#define HID_VID_NINTENDO  0x057eu
#define HID_VID_8BITDO    0x2dc8u

#define HID_PID_DS3       0x0268u
#define HID_PID_DS4_V1    0x05c4u
#define HID_PID_DS4_V2    0x09ccu
#define HID_PID_SWITCH_PRO 0x2009u
#define HID_PID_SWITCH2_PRO 0x2069u

enum HidKind {
	HID_KIND_UNKNOWN = 0,
	HID_KIND_X360,
	HID_KIND_DS4,
	HID_KIND_SWITCH_PRO,
	HID_KIND_SWITCH2_PRO,
	HID_KIND_XBOXONE,
	HID_KIND_PS3,
	HID_KIND_DINPUT
};

/* 8BitDo USB adapter firmware presents one of these two report shapes. */
enum {
	HID_8BITDO_NONE = 0,
	HID_8BITDO_XINPUT = 1,
	HID_8BITDO_DINPUT = 2
};

/* Classify a USB device. Report shape still decides the parser. */
enum HidKind hid_kind_for_ids(uint16_t vendor_id, uint16_t product_id);

/*
 * Translate one interrupt-IN report into a DualSense-style PadState.
 * Returns 0 when the report was recognized.
 */
int hid_map_report(enum HidKind kind, const uint8_t *report, size_t length, PadState *out);

/* First-party wired Xbox One and Series pads (GIP), not Xbox 360. */
int hid_is_xboxone(uint16_t vendor_id, uint16_t product_id);

/* Wired DualShock 3 / Sixaxis. */
int hid_is_ps3(uint16_t vendor_id, uint16_t product_id);

/* Wired Xbox 360 pads, including an 8BitDo dongle that spoofs 045e:028e. */
int hid_is_xbox360(uint16_t vendor_id, uint16_t product_id);

/* HID_8BITDO_XINPUT, HID_8BITDO_DINPUT, or HID_8BITDO_NONE. */
int hid_8bitdo_mode(uint16_t vendor_id, uint16_t product_id);

/* Series pads put Share at the end of the GIP input report. */
void hid_map_xboxone_set_share(int enabled);
void hid_map_xboxone_reset(void);
void hid_map_x360_reset(void);

/*
 * A tap can arrive as press and release in one USB packet. The press is
 * returned immediately; call this on the next idle poll to apply the release.
 * A newer report always wins, so this does not keep a button down.
 * Returns 1 when out was updated.
 */
int hid_map_xboxone_take_pending(PadState *out);
int hid_map_x360_take_pending(PadState *out);

int hid_map_x360(const uint8_t *report, size_t length, PadState *out);
int hid_map_xboxone(const uint8_t *report, size_t length, PadState *out);
int hid_map_ds4(const uint8_t *report, size_t length, PadState *out);
int hid_map_ps3(const uint8_t *report, size_t length, PadState *out);
int hid_map_dinput(const uint8_t *report, size_t length, PadState *out);
int hid_map_switch_pro(const uint8_t *report, size_t length, PadState *out);
int hid_map_switch2_pro(const uint8_t *report, size_t length, PadState *out);
int hid_map_switch2_ble(const uint8_t *report, size_t length, PadState *out);

/* 12-bit Switch axis (0..4095, center near 2048) to an 8-bit PS axis. */
uint8_t hid_axis12_to_u8(uint16_t axis, int invert_y);

/* Signed Xbox axis to an 8-bit PS axis. invert_y flips the PS "up is smaller" axis. */
uint8_t hid_axis16_to_u8(int16_t axis, int invert_y);

#endif
