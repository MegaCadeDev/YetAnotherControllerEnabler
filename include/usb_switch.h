#ifndef YETANOTHERCONTROLLERENABLER_USB_SWITCH_H
#define YETANOTHERCONTROLLERENABLER_USB_SWITCH_H

#include <stddef.h>
#include <stdint.h>

/* Open the Switch Pro and start the same USB init sequence OGX Mini uses. */
int usb_switch_open(void);

/* True after the controller has answered with a standard input report. */
int usb_switch_is_ready(void);

/* True when the opened device is a Switch 2 Pro (057e:2069). */
int usb_switch_is_switch2(void);

/* True when the opened device is a DualShock 4. */
int usb_switch_is_ds4(void);

/* True when the opened device is a wired Xbox One or Series pad. */
int usb_switch_is_xbox(void);

/* True when the opened device is a wired DualShock 3. */
int usb_switch_is_ps3(void);

/* True when the opened device speaks the Xbox 360 interrupt protocol. */
int usb_switch_is_x360(void);

/* True when the opened device uses a DirectInput-style report. */
int usb_switch_is_dinput(void);

/* True when the opened device is an 8BitDo adapter. */
int usb_switch_is_8bitdo(void);

/* Series pads report a Share button in the GIP input packet. */
int usb_switch_xbox_share(void);

/* Short name of the opened controller, for toasts. */
const char *usb_switch_name(void);

/* Short reason from the last failed open, for the on-screen notice. */
const char *usb_switch_status(void);

void usb_switch_close(void);

/* True while the opened USB device still answers. */
int usb_switch_present(void);

/*
 * Read one interrupt report.
 * Returns the byte count, 0 when no report is waiting, or -1 if the device is gone.
 */
int usb_switch_read(uint8_t *buffer, size_t length);

/* True when a follow-up report was already captured and will not block. */
int usb_switch_input_queued(void);

#endif
