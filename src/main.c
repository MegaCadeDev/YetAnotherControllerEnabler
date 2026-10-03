#include "bt_switch.h"
#include "hid_map.h"
#include "pad_assign.h"
#include "pad_data.h"
#include "usb_switch.h"

#include <errno.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <time.h>
#include <unistd.h>

typedef struct notify_request {
	char unused[45];
	char message[3075];
} notify_request_t;

int sceKernelSendNotificationRequest(int device, notify_request_t *request, size_t size, int flags);
int sceSysmoduleLoadModuleInternal(uint32_t module_id);
int sceUserServiceInitialize(void *parameters);

static int g_pad_load;
static int g_pad_user;
static int g_pad_init;
static int g_pad_add;
static void button_names(uint32_t buttons, char *out, size_t size);

static uint64_t monotonic_us(void);

static uint32_t g_pad_buttons;
static uint32_t g_click_logged;
static uint64_t g_click_until[32];
static uint32_t g_dpad_phys;
static uint32_t g_dpad_pulse;
static uint64_t g_dpad_sample_us;
static uint64_t g_dpad_pulse_until;

/* Directions repeat in the PS5 UI, so a tap is a short pulse and a missed
 * release cannot keep the last direction down. Other buttons stay visible
 * briefly so a one-packet tap still registers. */
#define XINPUT_DPAD (PAD_BUTTON_UP | PAD_BUTTON_DOWN | PAD_BUTTON_LEFT | PAD_BUTTON_RIGHT)

static void xinput_click(PadState *state, uint32_t seen, int have_sample)
{
	uint64_t now = monotonic_us();
	uint32_t physical;
	uint32_t rose;
	uint32_t shown;
	uint32_t dpad;
	int i;

	if (have_sample) {
		uint32_t rose_dpad;

		physical = state->buttons;
		rose_dpad = (physical & XINPUT_DPAD) & ~g_dpad_phys;
		g_dpad_phys = physical & XINPUT_DPAD;
		g_dpad_sample_us = now;
		if (rose_dpad != 0) {
			g_dpad_pulse |= rose_dpad;
			g_dpad_pulse_until = now + 32000ull;
		}
		rose = (seen & ~g_pad_buttons) & ~XINPUT_DPAD;
		g_pad_buttons = physical & ~XINPUT_DPAD;
		for (i = 0; i < 32; i++) {
			if (rose & (1u << i))
				g_click_until[i] = now + 100000ull;
		}
	} else {
		physical = g_pad_buttons;
		if (g_dpad_sample_us == 0 || now - g_dpad_sample_us > 50000ull)
			g_dpad_phys = 0;
	}
	if (g_dpad_pulse_until != 0 && now >= g_dpad_pulse_until) {
		g_dpad_pulse = 0;
		g_dpad_pulse_until = 0;
	}
	dpad = g_dpad_phys | g_dpad_pulse;
	shown = physical;
	for (i = 0; i < 32; i++) {
		uint32_t bit = 1u << i;

		if (shown & bit)
			continue;
		if (bit & XINPUT_DPAD) {
			g_click_until[i] = 0;
			continue;
		}
		if (g_click_until[i] != 0 && now < g_click_until[i])
			shown |= bit;
		else
			g_click_until[i] = 0;
	}
	shown = (shown & ~XINPUT_DPAD) | dpad;
	if ((shown & ~physical) != 0 && (shown & ~physical) != g_click_logged) {
		char names[160];

		button_names(shown & ~physical, names, sizeof(names));
		printf("YetAnotherControllerEnabler: xinput click %s\n", names);
	}
	g_click_logged = shown & ~physical;
	state->buttons = shown;
}

static void xinput_click_reset(void)
{
	memset(g_click_until, 0, sizeof(g_click_until));
	g_pad_buttons = 0;
	g_click_logged = 0;
	g_dpad_phys = 0;
	g_dpad_pulse = 0;
	g_dpad_sample_us = 0;
	g_dpad_pulse_until = 0;
}

static void notify(const char *message)
{
	notify_request_t request;

	memset(&request, 0, sizeof(request));
	snprintf(request.message, sizeof(request.message), "%s", message);
	sceKernelSendNotificationRequest(0, &request, sizeof(request), 0);
	printf("YetAnotherControllerEnabler: %s\n", message);
}

#define LISTEN_PORT 9045
#define FRAME_SIZE 16
#define INJECT_HZ 60

static uint64_t monotonic_us(void)
{
	struct timespec ts;

	clock_gettime(CLOCK_MONOTONIC, &ts);
	return (uint64_t)ts.tv_sec * 1000000ull + (uint64_t)ts.tv_nsec / 1000ull;
}

static void sleep_until_next_tick(uint64_t *next_us)
{
	uint64_t now = monotonic_us();
	struct timespec req;

	if (*next_us < now)
		*next_us = now;
	req.tv_sec = (time_t)((*next_us - now) / 1000000ull);
	req.tv_nsec = (long)(((*next_us - now) % 1000000ull) * 1000ull);
	nanosleep(&req, NULL);
	*next_us += 1000000ull / INJECT_HZ;
}

static int parse_frame(const uint8_t frame[FRAME_SIZE], PadState *out)
{
	uint32_t buttons;

	memcpy(&buttons, frame, sizeof(buttons));
	pad_state_neutral(out);
	out->buttons = buttons;
	out->left_x = frame[4];
	out->left_y = frame[5];
	out->right_x = frame[6];
	out->right_y = frame[7];
	out->l2 = frame[8];
	out->r2 = frame[9];
	return 0;
}

static int listen_frames(void)
{
	int fd;
	int on = 1;
	struct sockaddr_in addr;

	fd = socket(AF_INET, SOCK_STREAM, 0);
	if (fd < 0) {
		perror("socket");
		return -1;
	}
	setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &on, sizeof(on));
	memset(&addr, 0, sizeof(addr));
	addr.sin_family = AF_INET;
	addr.sin_port = htons(LISTEN_PORT);
	addr.sin_addr.s_addr = htonl(INADDR_ANY);
	if (bind(fd, (struct sockaddr *)&addr, sizeof(addr)) != 0) {
		printf("YetAnotherControllerEnabler: another YetAnotherControllerEnabler is already running\n");
		perror("bind");
		close(fd);
		return -1;
	}
	if (listen(fd, 1) != 0) {
		perror("listen");
		close(fd);
		return -1;
	}
	return fd;
}

static int open_virtual_pad(void)
{
	static int logged_fail;
	int handle;

	g_pad_load = sceSysmoduleLoadModuleInternal(0x80000024);
	g_pad_user = sceUserServiceInitialize(NULL);
	g_pad_init = scePadInit();
	scePadSetProcessPrivilege(1);
	handle = pad_assign_from_shell();
	g_pad_add = handle;
	if (handle > 0)
		logged_fail = 0;
	if (handle > 0 || !logged_fail) {
		if (handle <= 0)
			logged_fail = 1;
		printf("YetAnotherControllerEnabler: pad load 0x%08x user 0x%08x init 0x%08x add 0x%08x\n",
		       (unsigned)g_pad_load, (unsigned)g_pad_user, (unsigned)g_pad_init,
		       (unsigned)handle);
	}
	return handle;
}

static void button_names(uint32_t buttons, char *out, size_t size)
{
	static const struct {
		uint32_t bit;
		const char *name;
	} table[] = {
		{ PAD_BUTTON_UP, "up" },
		{ PAD_BUTTON_DOWN, "down" },
		{ PAD_BUTTON_LEFT, "left" },
		{ PAD_BUTTON_RIGHT, "right" },
		{ PAD_BUTTON_CROSS, "cross" },
		{ PAD_BUTTON_CIRCLE, "circle" },
		{ PAD_BUTTON_SQUARE, "square" },
		{ PAD_BUTTON_TRIANGLE, "triangle" },
		{ PAD_BUTTON_L1, "l1" },
		{ PAD_BUTTON_R1, "r1" },
		{ PAD_BUTTON_L2, "l2" },
		{ PAD_BUTTON_R2, "r2" },
		{ PAD_BUTTON_L3, "l3" },
		{ PAD_BUTTON_R3, "r3" },
		{ PAD_BUTTON_CREATE, "create" },
		{ PAD_BUTTON_OPTIONS, "options" },
		{ PAD_BUTTON_PS, "ps" },
		{ PAD_BUTTON_TOUCHPAD, "touch" },
	};
	size_t used = 0;
	size_t i;
	int any = 0;

	if (size == 0)
		return;
	out[0] = '\0';
	for (i = 0; i < sizeof(table) / sizeof(table[0]); i++) {
		int wrote;

		if ((buttons & table[i].bit) == 0)
			continue;
		wrote = snprintf(out + used, size - used, "%s%s", any ? " " : "", table[i].name);
		if (wrote < 0 || (size_t)wrote >= size - used)
			return;
		used += (size_t)wrote;
		any = 1;
	}
	if (!any)
		snprintf(out, size, "none");
}

static void note_xinput(const char *tag, const uint8_t *report, int length, const PadState *state)
{
	static uint8_t last_sig[8];
	static int last_sig_n = -1;
	static uint32_t last_buttons = 0xffffffffu;
	static int last_l2;
	static int last_r2;
	uint8_t sig[8];
	char hex[80];
	char names[160];
	int sig_n = 0;
	int xbox = usb_switch_is_xbox();
	int digital_l2 = state != NULL && state->l2 >= 30;
	int digital_r2 = state != NULL && state->r2 >= 30;
	int show;
	int i;
	int pos = 0;

	if (state == NULL)
		return;
	memset(sig, 0, sizeof(sig));
	if (report != NULL && length > 0) {
		if (xbox && report[0] == 0x20 && length >= 6) {
			sig_n = 2;
			memcpy(sig, report + 4, 2);
		} else if (xbox && report[0] == 0x07 && length >= 5) {
			sig_n = length < 6 ? length : 6;
			memcpy(sig, report, (size_t)sig_n);
		} else if (!xbox && length >= 6) {
			sig_n = 4;
			memcpy(sig, report + 2, 4);
		}
	}
	if (strcmp(tag, "report") == 0 && sig_n == last_sig_n &&
	    (sig_n == 0 || memcmp(sig, last_sig, (size_t)sig_n) == 0) &&
	    state->buttons == last_buttons && digital_l2 == last_l2 && digital_r2 == last_r2)
		return;
	if (strcmp(tag, "report") == 0) {
		last_sig_n = sig_n;
		memcpy(last_sig, sig, sizeof(last_sig));
		last_buttons = state->buttons;
		last_l2 = digital_l2;
		last_r2 = digital_r2;
	}
	hex[0] = '\0';
	show = report == NULL ? 0 : length;
	if (show > 20)
		show = 20;
	for (i = 0; i < show && pos < (int)sizeof(hex) - 4; i++)
		pos += snprintf(hex + pos, sizeof(hex) - (size_t)pos, "%02x%s",
				report[i], i + 1 == show ? "" : " ");
	button_names(state->buttons, names, sizeof(names));
	if (show > 0)
		printf("YetAnotherControllerEnabler: %s %s raw %s -> %s%s%s\n",
		       xbox ? "xbox" : "360", tag, hex, names,
		       digital_l2 ? " L2" : "", digital_r2 ? " R2" : "");
	else
		printf("YetAnotherControllerEnabler: %s %s -> %s%s%s\n",
		       xbox ? "xbox" : "360", tag, names,
		       digital_l2 ? " L2" : "", digital_r2 ? " R2" : "");
}

static void note_xinput_ignored(const uint8_t *report, int length)
{
	static uint8_t last[8];
	static int logs;
	char hex[80];
	int n = length < 8 ? length : 8;
	int show = length > 20 ? 20 : length;
	int i;
	int pos = 0;

	if (report == NULL || length <= 0 || logs >= 40)
		return;
	if (n > 0 && memcmp(report, last, (size_t)n) == 0)
		return;
	memcpy(last, report, (size_t)n);
	logs++;
	for (i = 0; i < show && pos < (int)sizeof(hex) - 4; i++)
		pos += snprintf(hex + pos, sizeof(hex) - (size_t)pos, "%02x%s",
				report[i], i + 1 == show ? "" : " ");
	printf("YetAnotherControllerEnabler: %s ignored %d bytes %s\n",
	       usb_switch_is_xbox() ? "xbox" : "360", length, hex);
}

static int inject_once(int handle, const PadState *state)
{
	PadData data;
	uint64_t now = sceKernelGetProcessTime();

	/* The pad service drops samples whose time is not process time. */
	if (now > 1000)
		now -= 1000;
	pad_data_from_state(&data, state, now);
	return scePadVirtualDeviceInsertData(handle, &data);
}

static void forget_controller(int *handle, int *usb_open, int *wireless, int *told_connected,
			      unsigned *quiet_reads, PadState *state, uint64_t *next_scan)
{
	char message[96];
	char label[32];

	snprintf(label, sizeof(label), "%s", *wireless ? bt_switch_name() : usb_switch_name());
	snprintf(message, sizeof(message), "Controller %s Disconnected", label);
	if (*wireless)
		bt_switch_drop();
	else
		usb_switch_close();
	*wireless = 0;
	*usb_open = 0;
	*told_connected = 0;
	*quiet_reads = 0;
	pad_state_neutral(state);
	pad_assign_release();
	hid_map_xboxone_reset();
	hid_map_x360_reset();
	xinput_click_reset();
	*handle = -1;
	*next_scan = monotonic_us() + 300000ull;
	notify(message);
	notify("Searching for Controllers...");
}

int main(void)
{
	int handle = -1;
	int listen_fd = -1;
	int client = -1;
	int usb_open = 0;
	int wireless = 0;
	int told_connected = 0;
	int told_inject = 0;
	unsigned quiet_reads = 0;
	uint64_t next_pad = 0;
	uint64_t next_scan = 0;
	char shown_status[160] = "";
	uint8_t report[128];
	uint8_t frame[FRAME_SIZE];
	size_t frame_fill = 0;
	PadState state;
	uint64_t next_tick;
	unsigned missing_logs = 0;

	setvbuf(stdout, NULL, _IOLBF, 0);
	pad_state_neutral(&state);
	printf("YetAnotherControllerEnabler: started\n");
	printf("YetAnotherControllerEnabler: bluetooth scan off\n");
	notify("Searching for Controllers...");
	{
		int removed = pad_assign_release();
		if (removed > 0)
			printf("YetAnotherControllerEnabler: removed %d leftover controllers\n", removed);
	}

	listen_fd = listen_frames();
	if (listen_fd >= 0) {
		int flags = fcntl(listen_fd, F_GETFL, 0);
		fcntl(listen_fd, F_SETFL, flags | O_NONBLOCK);
		printf("YetAnotherControllerEnabler: frame listener on tcp/%d "
		       "(16 bytes: u32 buttons, lx ly rx ry l2 r2, 6 reserved)\n",
		       LISTEN_PORT);
	}

	next_tick = monotonic_us();
	for (;;) {
		ssize_t n;
		ssize_t bt_n = -2;

		if (!usb_open && !wireless && monotonic_us() >= next_scan) {
			int found = usb_switch_open();
			const char *status = usb_switch_status();

			next_scan = monotonic_us() + (found == 0 ? 0 : 500000ull);
			if (found == 0) {
				char message[96];

				usb_open = 1;
				quiet_reads = 0;
				missing_logs = 0;
				told_connected = 0;
				shown_status[0] = '\0';
				pad_assign_release();

				if (usb_switch_is_xbox())
					hid_map_xboxone_set_share(usb_switch_xbox_share());
				snprintf(message, sizeof(message), "Found %s Controller or Adapter...",
					 usb_switch_name());
				notify(message);
			} else if (strcmp(shown_status, status) != 0) {
				snprintf(shown_status, sizeof(shown_status), "%s", status);
				printf("YetAnotherControllerEnabler: %s\n", status);
			}
		}

		if ((usb_open || wireless) &&
		    (wireless ? bt_switch_is_ready() : usb_switch_is_ready()) &&
		    handle < 0 && monotonic_us() >= next_pad) {
			int opened = open_virtual_pad();
			if (opened > 0) {
				handle = opened;
				told_inject = 0;
			} else if (!told_inject) {
				printf("YetAnotherControllerEnabler: %s\n", pad_assign_status());
				printf("YetAnotherControllerEnabler: console refused another controller, so buttons have nowhere to go\n");
				told_inject = 1;
			}
			next_pad = monotonic_us() + 2000000ull;
		}

		if (client < 0 && listen_fd >= 0) {
			client = accept(listen_fd, NULL, NULL);
			if (client >= 0) {
				int flags = fcntl(client, F_GETFL, 0);
				fcntl(client, F_SETFL, flags | O_NONBLOCK);
				frame_fill = 0;
				printf("YetAnotherControllerEnabler: test host connected\n");
			}
		}

		if (client >= 0) {
			n = read(client, frame + frame_fill, FRAME_SIZE - frame_fill);
			if (n == 0) {
				close(client);
				client = -1;
				pad_state_neutral(&state);
			} else if (n > 0) {
				frame_fill += (size_t)n;
				if (frame_fill == FRAME_SIZE) {
					parse_frame(frame, &state);
					frame_fill = 0;
				}
			}
		}

		if ((usb_open || wireless) && !told_connected && handle >= 0 &&
		    (wireless ? bt_switch_is_ready() : usb_switch_is_ready())) {
			told_connected = 1;
			{
				char message[64];
				snprintf(message, sizeof(message), "Controller %s Connected",
					 wireless ? bt_switch_name() : usb_switch_name());
				notify(message);
			}
		}

		if (usb_open || wireless) {
			int samples = 0;
			uint32_t seen_buttons = 0;
			PadState latest;
			int xinput = !wireless && (usb_switch_is_xbox() || usb_switch_is_x360());

			pad_state_neutral(&latest);
			if (wireless)
				n = bt_n == -2 ? bt_switch_read(report, sizeof(report)) : bt_n;
			else
				n = usb_switch_read(report, sizeof(report));
			while (n >= 0) {
				if (n > 0) {
					PadState mapped;
					enum HidKind kind = HID_KIND_SWITCH_PRO;
					quiet_reads = 0;
					if (wireless && bt_switch_is_switch2())
						kind = HID_KIND_SWITCH2_PRO;
					else if (wireless)
						kind = HID_KIND_SWITCH_PRO;
					else if (usb_switch_is_ds4())
						kind = HID_KIND_DS4;
					else if (usb_switch_is_switch2())
						kind = HID_KIND_SWITCH2_PRO;
					else if (usb_switch_is_xbox())
						kind = HID_KIND_XBOXONE;
					else if (usb_switch_is_ps3())
						kind = HID_KIND_PS3;
					else if (usb_switch_is_x360())
						kind = HID_KIND_X360;
					else if (usb_switch_is_dinput())
						kind = HID_KIND_DINPUT;
					if (wireless && bt_switch_is_switch2() &&
					    hid_map_switch2_ble(report, (size_t)n, &mapped) == 0) {
						latest = mapped;
						samples++;
						missing_logs = 0;
					} else if (hid_map_report(kind, report, (size_t)n, &mapped) == 0) {
						if (xinput)
							note_xinput("report", report, (int)n, &mapped);
						seen_buttons |= mapped.buttons;
						latest = mapped;
						samples++;
						missing_logs = 0;
					} else if (xinput) {
						note_xinput_ignored(report, (int)n);
					} else if (report[0] != 0x21 && missing_logs < 4) {
						printf("YetAnotherControllerEnabler: unrecognized %zd-byte report, first byte 0x%02x\n",
						       n, report[0]);
						missing_logs++;
					}
				}
				if (wireless || !usb_switch_input_queued())
					break;
				n = usb_switch_read(report, sizeof(report));
			}
			if (n < 0) {
				forget_controller(&handle, &usb_open, &wireless, &told_connected, &quiet_reads,
						  &state, &next_scan);
			} else if (samples > 0) {
				state = latest;
				if (xinput)
					xinput_click(&state, seen_buttons, 1);
			} else if (xinput) {
				PadState pending;
				int fresh = 0;

				if (usb_switch_is_xbox() && hid_map_xboxone_take_pending(&pending)) {
					state = pending;
					fresh = 1;
					quiet_reads = 0;
					note_xinput("idle", NULL, 0, &state);
				} else if (usb_switch_is_x360() && hid_map_x360_take_pending(&pending)) {
					state = pending;
					fresh = 1;
					quiet_reads = 0;
					note_xinput("idle", NULL, 0, &state);
				}
				xinput_click(&state, fresh ? state.buttons : 0, fresh);
			}
			if (xinput && usb_open) {
				static uint64_t hold_log_us;
				uint64_t now = monotonic_us();

				if (state.buttons == 0) {
					hold_log_us = 0;
				} else if (hold_log_us == 0) {
					hold_log_us = now;
				} else if (now - hold_log_us >= 500000ull) {
					char names[160];

					button_names(state.buttons, names, sizeof(names));
					printf("YetAnotherControllerEnabler: %s still holding %s\n",
					       usb_switch_is_xbox() ? "xbox" : "360", names);
					hold_log_us = now;
				}
			}
			if (n == 0 && usb_switch_is_ready() && ++quiet_reads >= 40) {
				quiet_reads = 0;
				if (wireless || !usb_switch_present()) {
					forget_controller(&handle, &usb_open, &wireless, &told_connected,
							  &quiet_reads, &state, &next_scan);
				}
			}
		}

		if (bt_switch_fast_poll()) {
			struct timespec req;

			req.tv_sec = 0;
			req.tv_nsec = 4000000L;
			nanosleep(&req, NULL);
		} else {
			sleep_until_next_tick(&next_tick);
		}
		if (handle >= 0 && (!bt_switch_fast_poll() || monotonic_us() >= next_tick)) {
			int inserted = inject_once(handle, &state);
			if (bt_switch_fast_poll())
				next_tick += 1000000ull / INJECT_HZ;
			if (inserted < 0 && !told_inject) {
				printf("YetAnotherControllerEnabler: pad input rejected %08x\n", (unsigned)inserted);
				told_inject = 1;
			}
		}
	}
}
