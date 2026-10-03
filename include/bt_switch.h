#ifndef YETANOTHERCONTROLLERENABLER_BT_SWITCH_H
#define YETANOTHERCONTROLLERENABLER_BT_SWITCH_H

#include <stddef.h>
#include <stdint.h>

/* Open the console's second Bluetooth controller. The DualSense radio is left alone. */
int bt_switch_start(void);

/*
 * While hold is set, a wired pad owns the virtual controller, so wireless
 * pairing waits.
 */
void bt_switch_set_hold(int hold);

/* Advance the radio. Returns a report length, 0 when none is waiting, or -1 if a live pad left. */
int bt_switch_read(uint8_t *report, size_t length);

int bt_switch_is_ready(void);
int bt_switch_is_switch2(void);

/* "Switch Pro" or "Switch 2 Pro" once a pad has been chosen. */
const char *bt_switch_name(void);

/* Switch 2 drops the link unless rumble is refreshed about every 5 ms. */
int bt_switch_fast_poll(void);

/* End the current pad and look for another. The radio stays open. */
void bt_switch_drop(void);

#endif
