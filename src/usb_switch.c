#include "usb_switch.h"
#include "hid_map.h"

#include <dev/usb/usb_ioctl.h>
#include <dev/usb/usbdi.h>

#include <dirent.h>
#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/select.h>
#include <sys/time.h>
#include <unistd.h>

#define SWITCH_VENDOR 0x057eu
#define SWITCH_PRO 0x2009u
#define SWITCH2_PRO 0x2069u
#define PACKET_SIZE 64
#define EP_SLOTS 4

struct switch_dev {
	int ctrl_fd;
	int in_fd;
	int out_fd;
	int use_fs;
	uint8_t in_index;
	uint8_t out_index;
	uint8_t bulk_out_index;
	uint8_t bulk_in_index;
	uint8_t has_bulk_in;
	uint8_t has_hid_out;
	uint8_t counter;
	char path[64];
	struct usb_fs_endpoint endpoints[EP_SLOTS];
};

static struct switch_dev g_dev = {
	.ctrl_fd = -1,
	.in_fd = -1,
	.out_fd = -1,
};

static int g_switch2;
static int g_ds4;
static int g_xbox;
static int g_ps3;
static int g_x360;
static int g_dinput;
static int g_bitdo;
static uint16_t g_product;
static uint8_t g_gip_seq;
static char g_status[160] = "Switch controller not found";
static char g_fail[80];
static char g_seen[96];
static int g_seen_count;

const char *usb_switch_status(void)
{
	return g_status;
}

static int usb_has_x360_iface(int fd);

static uint16_t read_uword(const uWord word)
{
	return (uint16_t)(word[0] | ((uint16_t)word[1] << 8));
}

static void remember_device(uint16_t vendor, uint16_t product)
{
	char item[16];
	size_t used;

	g_seen_count++;
	used = strlen(g_seen);
	snprintf(item, sizeof(item), "%04x:%04x ", vendor, product);
	if (used + strlen(item) < sizeof(g_seen))
		memcpy(g_seen + used, item, strlen(item) + 1);
}

static void fs_stop(uint8_t ep_index)
{
	struct usb_fs_stop stop;
	struct usb_fs_complete complete;

	memset(&stop, 0, sizeof(stop));
	stop.ep_index = ep_index;
	ioctl(g_dev.ctrl_fd, USB_FS_STOP, &stop);
	memset(&complete, 0, sizeof(complete));
	ioctl(g_dev.ctrl_fd, USB_FS_COMPLETE, &complete);
}

/*
 * USB_FS_COMPLETE returns EBUSY until the transfer is queued as finished.
 * Cancelling an input transfer stalls the pipe, so input waits return 1
 * and leave the transfer running.
 */
static int fs_wait_complete(uint8_t ep_index, int timeout_ms, int cancel_on_timeout)
{
	int spins;

	spins = timeout_ms / 5 + 1;
	while (spins-- > 0) {
		struct usb_fs_complete complete;
		struct timeval wait;
		fd_set read_set;
		fd_set write_set;

		memset(&complete, 0, sizeof(complete));
		complete.ep_index = ep_index;
		if (ioctl(g_dev.ctrl_fd, USB_FS_COMPLETE, &complete) == 0)
			return 0;
		if (errno != EBUSY) {
			snprintf(g_fail, sizeof(g_fail), "complete errno %d", errno);
			return -1;
		}
		FD_ZERO(&read_set);
		FD_ZERO(&write_set);
		FD_SET(g_dev.ctrl_fd, &read_set);
		FD_SET(g_dev.ctrl_fd, &write_set);
		wait.tv_sec = 0;
		wait.tv_usec = 5000;
		if (select(g_dev.ctrl_fd + 1, &read_set, &write_set, NULL, &wait) > 0)
			usleep(1000);
		else
			usleep(5000);
	}
	if (cancel_on_timeout)
		fs_stop(ep_index);
	return 1;
}

static uint8_t g_ep_pending[EP_SLOTS];
static void *g_fs_frame[EP_SLOTS];
static uint32_t g_fs_length[EP_SLOTS];

static int fs_transfer(uint8_t ep_index, uint8_t *buffer, uint32_t length, int timeout_ms, int cancel_on_timeout)
{
	struct usb_fs_endpoint *endpoint;
	struct usb_fs_start start;
	int waited;

	if (ep_index >= EP_SLOTS)
		return -1;
	endpoint = &g_dev.endpoints[ep_index];
	if (length > PACKET_SIZE)
		length = PACKET_SIZE;
	/*
	 * The kernel reads these pointers again when the transfer completes.
	 * They have to outlive this call, including when the first wait times out
	 * and the next call collects the packet.
	 */
	g_fs_frame[ep_index] = buffer;
	g_fs_length[ep_index] = length;
	endpoint->ppBuffer = &g_fs_frame[ep_index];
	endpoint->pLength = &g_fs_length[ep_index];
	endpoint->nFrames = 1;
	endpoint->flags = USB_FS_FLAG_SINGLE_SHORT_OK;
	endpoint->timeout = timeout_ms < 0 ? 1000 : (uint16_t)timeout_ms;

	/* Start the URB and return. The next read collects it. */
	if (timeout_ms < 0) {
		if (g_ep_pending[ep_index])
			return 0;
		start.ep_index = ep_index;
		if (ioctl(g_dev.ctrl_fd, USB_FS_START, &start) != 0) {
			if (errno == EBUSY) {
				g_ep_pending[ep_index] = 1;
				return 0;
			}
			return -1;
		}
		g_ep_pending[ep_index] = 1;
		return 0;
	}

	if (ep_index < EP_SLOTS && g_ep_pending[ep_index]) {
		waited = fs_wait_complete(ep_index, timeout_ms + 30, 0);
		if (waited != 0) {
			/* Stopping an IN transfer stalls the pipe, so a short wait leaves it queued. */
			if (waited > 0 && cancel_on_timeout) {
				fs_stop(ep_index);
				g_ep_pending[ep_index] = 0;
			}
			return waited < 0 ? -1 : 0;
		}
		g_ep_pending[ep_index] = 0;
		goto finished;
	}

	start.ep_index = ep_index;
	if (ioctl(g_dev.ctrl_fd, USB_FS_START, &start) != 0) {
		if (errno == EBUSY && ep_index < EP_SLOTS) {
			g_ep_pending[ep_index] = 1;
			return 0;
		}
		snprintf(g_fail, sizeof(g_fail), "start errno %d", errno);
		return -1;
	}
	if (ep_index < EP_SLOTS)
		g_ep_pending[ep_index] = 1;
	waited = fs_wait_complete(ep_index, timeout_ms + 30, cancel_on_timeout);
	if (waited < 0) {
		if (ep_index < EP_SLOTS)
			g_ep_pending[ep_index] = 0;
		return -1;
	}
	if (waited > 0) {
		if (cancel_on_timeout && ep_index < EP_SLOTS)
			g_ep_pending[ep_index] = 0;
		return 0;
	}
	if (ep_index < EP_SLOTS)
		g_ep_pending[ep_index] = 0;
finished:
	if (endpoint->status != USB_ERR_NORMAL_COMPLETION &&
	    endpoint->status != USB_ERR_SHORT_XFER) {
		if (endpoint->status == USB_ERR_TIMEOUT || endpoint->status == USB_ERR_CANCELLED)
			return 0;
		snprintf(g_fail, sizeof(g_fail), "xfer status %d", endpoint->status);
		errno = EIO;
		return -1;
	}
	return (int)g_fs_length[ep_index];
}

static void fs_clear_pending(uint8_t ep_index)
{
	if (ep_index < EP_SLOTS && g_ep_pending[ep_index]) {
		fs_stop(ep_index);
		g_ep_pending[ep_index] = 0;
	}
}

static int send_out(const uint8_t *packet, size_t length)
{
	uint8_t copy[PACKET_SIZE];

	if (!g_dev.use_fs || length == 0 || length > PACKET_SIZE)
		return -1;
	/* An input transfer left running steals the next completion. */
	fs_clear_pending(g_dev.in_index);
	memset(copy, 0, sizeof(copy));
	memcpy(copy, packet, length);
	if (fs_transfer(g_dev.out_index, copy, PACKET_SIZE, 200, 1) != PACKET_SIZE) {
		if (g_fail[0] == '\0')
			snprintf(g_fail, sizeof(g_fail), "out timeout");
		return -1;
	}
	return 0;
}

enum {
	STEP_MAC = 0,
	STEP_HS1,
	STEP_BAUD,
	STEP_HS2,
	STEP_DISABLE,
	STEP_LED,
	STEP_LED_HOME,
	STEP_MODE,
	STEP_GYRO,
	STEP_READY
};

static int g_step = STEP_MAC;
static int g_ready;
static int g_init_done;

static void fill_neutral_rumble(uint8_t *packet)
{
	packet[2] = 0x00;
	packet[3] = 0x01;
	packet[4] = 0x40;
	packet[5] = 0x40;
	packet[6] = 0x00;
	packet[7] = 0x01;
	packet[8] = 0x40;
	packet[9] = 0x40;
}

static int send_subcommand(uint8_t subcommand, const uint8_t *args, int arg_count, size_t length)
{
	uint8_t packet[PACKET_SIZE];

	memset(packet, 0, sizeof(packet));
	packet[0] = 0x01;
	packet[1] = g_dev.counter;
	g_dev.counter = (uint8_t)((g_dev.counter + 1) & 0x0f);
	fill_neutral_rumble(packet);
	packet[10] = subcommand;
	if (args != NULL && arg_count > 0)
		memcpy(packet + 11, args, (size_t)arg_count);
	(void)length;
	return send_out(packet, 64);
}

static int send_rumble(void)
{
	uint8_t packet[PACKET_SIZE];

	memset(packet, 0, sizeof(packet));
	packet[0] = 0x10;
	packet[1] = g_dev.counter;
	g_dev.counter = (uint8_t)((g_dev.counter + 1) & 0x0f);
	fill_neutral_rumble(packet);
	return send_out(packet, 64);
}

static int send_usb_command(uint8_t command)
{
	uint8_t packet[PACKET_SIZE];

	memset(packet, 0, sizeof(packet));
	packet[0] = 0x80;
	packet[1] = command;
	return send_out(packet, 64);
}

static int is_switch_pro(int fd)
{
	struct usb_device_descriptor descriptor;
	uint16_t vendor;
	uint16_t product;

	memset(&descriptor, 0, sizeof(descriptor));
	if (ioctl(fd, USB_GET_DEVICE_DESC, &descriptor) != 0)
		return 0;
	vendor = read_uword(descriptor.idVendor);
	product = read_uword(descriptor.idProduct);
	remember_device(vendor, product);
	g_switch2 = 0;
	g_ds4 = 0;
	g_xbox = 0;
	g_ps3 = 0;
	g_x360 = 0;
	g_dinput = 0;
	g_bitdo = 0;
	g_product = product;
	if (vendor == SWITCH_VENDOR && product == SWITCH_PRO)
		return 1;
	if (vendor == SWITCH_VENDOR && product == SWITCH2_PRO) {
		g_switch2 = 1;
		return 1;
	}
	if (vendor == 0x054cu && (product == 0x05c4u || product == 0x09ccu)) {
		g_ds4 = 1;
		return 1;
	}
	if (hid_is_xboxone(vendor, product)) {
		g_xbox = 1;
		return 1;
	}
	if (hid_is_ps3(vendor, product)) {
		g_ps3 = 1;
		return 1;
	}
	if (hid_is_xbox360(vendor, product) || hid_8bitdo_mode(vendor, product) != HID_8BITDO_NONE) {
		int iface = usb_has_x360_iface(fd);
		int xinput = hid_8bitdo_mode(vendor, product) == HID_8BITDO_XINPUT ||
			     hid_is_xbox360(vendor, product);

		if (iface == 1)
			xinput = 1;
		else if (iface == 0 && !hid_is_xbox360(vendor, product))
			xinput = 0;
		g_bitdo = vendor == HID_VID_8BITDO;
		if (xinput)
			g_x360 = 1;
		else
			g_dinput = 1;
		return 1;
	}
	return 0;
}

static int open_existing(const char *path)
{
	int fd = open(path, O_RDONLY | O_NONBLOCK);
	if (fd < 0)
		return -1;
	if (!is_switch_pro(fd)) {
		close(fd);
		return -1;
	}
	snprintf(g_dev.path, sizeof(g_dev.path), "%s", path);
	printf("YetAnotherControllerEnabler: %s at %s\n", usb_switch_name(), path);
	return fd;
}

static int open_from_directory(void)
{
	static const char *dirs[] = {"/dev", "/dev/usb", NULL};
	int dir_index;

	for (dir_index = 0; dirs[dir_index] != NULL; dir_index++) {
		DIR *dir = opendir(dirs[dir_index]);
		struct dirent *entry;

		if (dir == NULL)
			continue;
		while ((entry = readdir(dir)) != NULL) {
			int dots = 0;
			char path[128];
			const char *name = entry->d_name;
			const char *scan;
			int fd;

			if (strncmp(name, "ugen", 4) != 0 && strncmp(name, "usb", 3) != 0)
				continue;
			for (scan = name; *scan != '\0'; scan++) {
				if (*scan == '.')
					dots++;
			}
			if (dots != 1)
				continue;
			snprintf(path, sizeof(path), "%s/%s", dirs[dir_index], name);
			fd = open_existing(path);
			if (fd >= 0) {
				closedir(dir);
				return fd;
			}
		}
		closedir(dir);
	}
	return -1;
}

static int open_from_usbctl(void)
{
	static const char *controls[] = {"/dev/usbctl", "/dev/usb/usbctl", "/dev/usb0", NULL};
	int control_index;

	for (control_index = 0; controls[control_index] != NULL; control_index++) {
		struct usb_device_info list[48];
		struct usb_read_dir directory;
		int fd;
		int count;
		int index;

		fd = open(controls[control_index], O_RDONLY | O_NONBLOCK);
		if (fd < 0)
			continue;
		memset(&directory, 0, sizeof(directory));
		memset(list, 0, sizeof(list));
		directory.urd_data = list;
		directory.urd_startentry = 0;
		directory.urd_maxlen = sizeof(list);
		if (ioctl(fd, USB_READ_DIR, &directory) != 0) {
			close(fd);
			continue;
		}
		close(fd);
		count = (int)(directory.urd_maxlen / sizeof(struct usb_device_info));
		if (count > (int)(sizeof(list) / sizeof(list[0])))
			count = (int)(sizeof(list) / sizeof(list[0]));
		for (index = 0; index < count; index++) {
			char path[64];
			int device;

			if (list[index].udi_vendorNo == 0)
				continue;
			snprintf(path, sizeof(path), "/dev/ugen%u.%u", list[index].udi_bus, list[index].udi_addr);
			device = open_existing(path);
			if (device >= 0)
				return device;
			snprintf(path, sizeof(path), "/dev/ugen%u.%u", list[index].udi_bus, list[index].udi_index);
			device = open_existing(path);
			if (device >= 0)
				return device;
		}
	}
	return -1;
}

static int open_numbered(void)
{
	int bus;
	int address;

	for (bus = 0; bus <= 3; bus++) {
		for (address = 0; address <= 16; address++) {
			char path[64];
			int fd;

			snprintf(path, sizeof(path), "/dev/ugen%d.%d", bus, address);
			fd = open_existing(path);
			if (fd >= 0)
				return fd;
		}
	}
	return -1;
}

static int fs_open_endpoint(int fd, uint8_t index, uint8_t address)
{
	struct usb_fs_open open_request;

	memset(&open_request, 0, sizeof(open_request));
	open_request.max_bufsize = PACKET_SIZE;
	open_request.max_frames = 1;
	open_request.ep_index = index;
	open_request.ep_no = address;
	return ioctl(fd, USB_FS_OPEN, &open_request);
}

static void read_bulk_iface1(int fd, uint8_t *out_address, uint8_t *in_address)
{
	struct usb_gen_descriptor descriptor;
	uint8_t raw[512];
	int offset = 0;
	int length;
	int iface = -1;

	*out_address = 0;
	*in_address = 0;
	memset(&descriptor, 0, sizeof(descriptor));
	memset(raw, 0, sizeof(raw));
	descriptor.ugd_data = raw;
	descriptor.ugd_maxlen = sizeof(raw);
	if (ioctl(fd, USB_GET_FULL_DESC, &descriptor) != 0)
		return;
	length = descriptor.ugd_actlen;
	if (length <= 0 || length > (int)sizeof(raw))
		length = (int)sizeof(raw);
	while (offset + 2 <= length) {
		int entry = raw[offset];
		uint8_t address;

		if (entry < 2 || offset + entry > length)
			break;
		if (raw[offset + 1] == 0x04 && entry >= 3)
			iface = raw[offset + 2];
		address = raw[offset + 2];
		if (iface == 1 && raw[offset + 1] == 0x05 && entry >= 5 &&
		    (raw[offset + 3] & 0x03) == 0x02) {
			if ((address & 0x80) != 0 && *in_address == 0)
				*in_address = address;
			else if (address != 0 && *out_address == 0)
				*out_address = address;
		}
		offset += entry;
	}
}

/* 1 when an Xbox 360 interface is present, 0 when the descriptor says otherwise, -1 if unread. */
static int usb_has_x360_iface(int fd)
{
	struct usb_gen_descriptor descriptor;
	uint8_t raw[512];
	int offset = 0;
	int length;
	int saw_iface = 0;

	memset(&descriptor, 0, sizeof(descriptor));
	memset(raw, 0, sizeof(raw));
	descriptor.ugd_data = raw;
	descriptor.ugd_maxlen = sizeof(raw);
	if (ioctl(fd, USB_GET_FULL_DESC, &descriptor) != 0)
		return -1;
	length = descriptor.ugd_actlen;
	if (length <= 0 || length > (int)sizeof(raw))
		length = (int)sizeof(raw);
	while (offset + 2 <= length) {
		int entry = raw[offset];

		if (entry < 2 || offset + entry > length)
			break;
		if (raw[offset + 1] == 0x04 && entry >= 9) {
			saw_iface = 1;
			if (raw[offset + 5] == 0xff && raw[offset + 6] == 0x5d && raw[offset + 7] == 0x01)
				return 1;
		}
		offset += entry;
	}
	return saw_iface ? 0 : -1;
}

static void read_interrupt_addresses(int fd, uint8_t *outs, int *out_count, uint8_t *ins, int *in_count)
{
	struct usb_gen_descriptor descriptor;
	uint8_t raw[512];
	int offset = 0;
	int length;

	*out_count = 0;
	*in_count = 0;
	memset(&descriptor, 0, sizeof(descriptor));
	memset(raw, 0, sizeof(raw));
	descriptor.ugd_data = raw;
	descriptor.ugd_maxlen = sizeof(raw);
	if (ioctl(fd, USB_GET_FULL_DESC, &descriptor) != 0)
		return;
	length = descriptor.ugd_actlen;
	if (length <= 0 || length > (int)sizeof(raw))
		length = (int)sizeof(raw);
	while (offset + 2 <= length) {
		int entry = raw[offset];
		uint8_t address;

		if (entry < 2 || offset + entry > length)
			break;
		address = raw[offset + 2];
		if (raw[offset + 1] == 0x05 && entry >= 5 && (raw[offset + 3] & 0x03) == 0x03) {
			if ((address & 0x80) != 0 && *in_count < 8)
				ins[(*in_count)++] = address;
			else if (address != 0 && *out_count < 8)
				outs[(*out_count)++] = address;
		}
		offset += entry;
	}
}

static int open_first_endpoint(int fd, uint8_t index, const uint8_t *addresses, int count)
{
	int i;

	for (i = 0; i < count; i++) {
		if (fs_open_endpoint(fd, index, addresses[i]) == 0)
			return addresses[i];
	}
	return -1;
}

/* GIP data lives on interrupt 0x02/0x82. Try that pair before any other interrupt. */
static int open_gip_endpoint(int fd, uint8_t index, const uint8_t *addresses, int count, uint8_t prefer)
{
	uint8_t ordered[9];
	int n = 0;
	int i;

	ordered[n++] = prefer;
	for (i = 0; i < count && n < (int)sizeof(ordered); i++) {
		if (addresses[i] != prefer)
			ordered[n++] = addresses[i];
	}
	return open_first_endpoint(fd, index, ordered, n);
}

static int fs_setup(int fd)
{
	struct usb_fs_init init;
	uint8_t outs[8];
	uint8_t ins[8];
	uint8_t fallback[15];
	int iface = 0;
	int config = 0;
	int out_count = 0;
	int in_count = 0;
	int out_address;
	int i;

	/* Claim only. Detach is a port reset, which is the disconnect. */
	ioctl(fd, USB_CLAIM_INTERFACE, &iface);
	if (g_switch2) {
		iface = 1;
		ioctl(fd, USB_CLAIM_INTERFACE, &iface);
	}
	if (ioctl(fd, USB_GET_CONFIG, &config) == 0 && (config & 0xff) == 0xff) {
		config = 0;
		if (ioctl(fd, USB_SET_CONFIG, &config) != 0) {
			snprintf(g_fail, sizeof(g_fail), "set config errno %d", errno);
			return -1;
		}
	}
	read_interrupt_addresses(fd, outs, &out_count, ins, &in_count);

	memset(g_dev.endpoints, 0, sizeof(g_dev.endpoints));
	memset(&init, 0, sizeof(init));
	init.pEndpoints = g_dev.endpoints;
	init.ep_index_max = g_switch2 ? EP_SLOTS : 2;
	if (ioctl(fd, USB_FS_INIT, &init) != 0) {
		snprintf(g_fail, sizeof(g_fail), "fs init errno %d", errno);
		return -1;
	}
	g_dev.in_index = 0;
	g_dev.out_index = 1;
	if (g_switch2) {
		uint8_t bulk_out = 0;
		uint8_t bulk_in = 0;
		int opened;

		read_bulk_iface1(fd, &bulk_out, &bulk_in);
		g_dev.bulk_out_index = 2;
		g_dev.bulk_in_index = 3;
		g_dev.has_bulk_in = 0;
		if (bulk_out == 0) {
			snprintf(g_fail, sizeof(g_fail), "switch2 bulk out missing");
			return -1;
		}
		if (open_first_endpoint(fd, g_dev.bulk_out_index, &bulk_out, 1) < 0) {
			snprintf(g_fail, sizeof(g_fail), "switch2 bulk out %02x", bulk_out);
			return -1;
		}
		if (bulk_in != 0 && open_first_endpoint(fd, g_dev.bulk_in_index, &bulk_in, 1) == 0)
			g_dev.has_bulk_in = 1;
		opened = open_first_endpoint(fd, g_dev.in_index, ins, in_count);
		if (opened < 0) {
			uint8_t in_fallback[2] = {0x81, 0x82};
			opened = open_first_endpoint(fd, g_dev.in_index, in_fallback, 2);
		}
		if (opened < 0) {
			snprintf(g_fail, sizeof(g_fail), "switch2 hid in missing");
			return -1;
		}
		g_dev.has_hid_out = open_first_endpoint(fd, g_dev.out_index, outs, out_count) >= 0;
		g_dev.use_fs = 1;
		printf("YetAnotherControllerEnabler: Switch 2 Pro bulk out 0x%02x in 0x%02x\n", bulk_out, bulk_in);
		return 0;
	}
	if (g_xbox) {
		out_address = open_gip_endpoint(fd, g_dev.out_index, outs, out_count, 0x02);
		if (out_address < 0) {
			snprintf(g_fail, sizeof(g_fail), "xbox out missing");
			return -1;
		}
		if (open_gip_endpoint(fd, g_dev.in_index, ins, in_count, 0x82) < 0) {
			snprintf(g_fail, sizeof(g_fail), "xbox in missing");
			return -1;
		}
		g_dev.has_hid_out = 1;
		g_dev.use_fs = 1;
		printf("YetAnotherControllerEnabler: Xbox out endpoint 0x%02x\n", out_address);
		return 0;
	}
	if (g_x360) {
		out_address = open_first_endpoint(fd, g_dev.out_index, outs, out_count);
		if (out_address < 0) {
			snprintf(g_fail, sizeof(g_fail), "360 out missing");
			return -1;
		}
		if (open_first_endpoint(fd, g_dev.in_index, ins, in_count) < 0) {
			snprintf(g_fail, sizeof(g_fail), "360 in missing");
			return -1;
		}
		g_dev.has_hid_out = 1;
		g_dev.use_fs = 1;
		printf("YetAnotherControllerEnabler: %s out endpoint 0x%02x\n", usb_switch_name(), out_address);
		return 0;
	}
	out_address = open_first_endpoint(fd, g_dev.out_index, outs, out_count);
	if (out_address < 0) {
		for (i = 0; i < 15; i++)
			fallback[i] = (uint8_t)(i + 1);
		out_address = open_first_endpoint(fd, g_dev.out_index, fallback, 15);
	}
	if (out_address < 0 && !g_ds4 && !g_ps3 && !g_dinput) {
		if (out_count > 0)
			snprintf(g_fail, sizeof(g_fail), "out ep 22 cfg %d saw %02x", config, outs[0]);
		else
			snprintf(g_fail, sizeof(g_fail), "out ep 22 cfg %d no ep", config);
		return -1;
	}
	if (open_first_endpoint(fd, g_dev.in_index, ins, in_count) < 0) {
		uint8_t in_fallback[2] = {0x81, 0x82};
		open_first_endpoint(fd, g_dev.in_index, in_fallback, 2);
	}
	g_dev.has_hid_out = out_address >= 0;
	g_dev.use_fs = 1;
	printf("YetAnotherControllerEnabler: %s out endpoint 0x%02x\n", usb_switch_name(), out_address);
	return 0;
}

static int open_hid_node(void)
{
	DIR *dir = opendir("/dev");
	struct dirent *entry;

	if (dir == NULL)
		return -1;
	while ((entry = readdir(dir)) != NULL) {
		char path[128];
		int fd;

		if (strncmp(entry->d_name, "uhid", 4) != 0)
			continue;
		snprintf(path, sizeof(path), "/dev/%s", entry->d_name);
		fd = open_existing(path);
		if (fd >= 0) {
			closedir(dir);
			return fd;
		}
	}
	closedir(dir);
	return -1;
}

int usb_switch_present(void)
{
	struct usb_device_descriptor descriptor;

	if (g_dev.ctrl_fd < 0)
		return 0;
	memset(&descriptor, 0, sizeof(descriptor));
	return ioctl(g_dev.ctrl_fd, USB_GET_DEVICE_DESC, &descriptor) == 0;
}

#define IN_QUEUE 8

static uint8_t g_in_q[IN_QUEUE][PACKET_SIZE];
static int g_in_q_len[IN_QUEUE];
static int g_in_q_r;
static int g_in_q_w;
static int g_in_q_n;

static void queue_clear(void)
{
	g_in_q_r = 0;
	g_in_q_w = 0;
	g_in_q_n = 0;
}

static void queue_push(const uint8_t *data, int len)
{
	if (data == NULL || len <= 0 || g_in_q_n >= IN_QUEUE)
		return;
	if (len > PACKET_SIZE)
		len = PACKET_SIZE;
	memcpy(g_in_q[g_in_q_w], data, (size_t)len);
	if (len < PACKET_SIZE)
		memset(g_in_q[g_in_q_w] + len, 0, (size_t)(PACKET_SIZE - len));
	g_in_q_len[g_in_q_w] = len;
	g_in_q_w = (g_in_q_w + 1) % IN_QUEUE;
	g_in_q_n++;
}

static int queue_pop(uint8_t *buffer, size_t length)
{
	int len;

	if (g_in_q_n <= 0)
		return 0;
	len = g_in_q_len[g_in_q_r];
	if ((size_t)len > length)
		len = (int)length;
	memcpy(buffer, g_in_q[g_in_q_r], (size_t)len);
	g_in_q_r = (g_in_q_r + 1) % IN_QUEUE;
	g_in_q_n--;
	return len;
}

int usb_switch_input_queued(void)
{
	return g_in_q_n > 0;
}

void usb_switch_close(void)
{
	if (g_dev.use_fs && g_dev.ctrl_fd >= 0) {
		struct usb_fs_uninit uninit;
		memset(&uninit, 0, sizeof(uninit));
		ioctl(g_dev.ctrl_fd, USB_FS_UNINIT, &uninit);
	}
	if (g_dev.in_fd >= 0)
		close(g_dev.in_fd);
	if (g_dev.out_fd >= 0)
		close(g_dev.out_fd);
	if (g_dev.ctrl_fd >= 0)
		close(g_dev.ctrl_fd);
	memset(&g_dev, 0, sizeof(g_dev));
	g_dev.ctrl_fd = -1;
	g_dev.in_fd = -1;
	g_dev.out_fd = -1;
	memset(g_ep_pending, 0, sizeof(g_ep_pending));
	queue_clear();
	g_switch2 = 0;
	g_ds4 = 0;
	g_xbox = 0;
	g_ps3 = 0;
	g_x360 = 0;
	g_dinput = 0;
	g_bitdo = 0;
	g_product = 0;
	g_gip_seq = 0;
	g_ready = 0;
	g_step = STEP_MAC;
	g_init_done = 0;
}

static int input_report(uint8_t id);

enum {
	INIT_SETTLE = 0,
	INIT_SEND_HS,
	INIT_WAIT_HS,
	INIT_SEND_BAUD,
	INIT_WAIT_BAUD,
	INIT_SEND_HS2,
	INIT_WAIT_HS2,
	INIT_SEND_DISABLE,
	INIT_SEND_MODE,
	INIT_WAIT_MODE,
	INIT_SEND_LED,
	INIT_SEND_HOME,
	INIT_DONE,
	INIT_S2_SETTLE,
	INIT_S2_SEND,
	INIT_S2_GAP,
	INIT_S2_DISABLE,
	INIT_DS4_SETTLE,
	INIT_DS4_LED,
	INIT_XB_SETTLE,
	INIT_XB_POWER,
	INIT_XB_SINIT,
	INIT_XB_LED,
	INIT_XB_EXTRA,
	INIT_SIMPLE_SETTLE,
	INIT_PS3_SETTLE,
	INIT_PS3_ENABLE,
	INIT_X360_SETTLE,
	INIT_X360_LED
};

static const uint8_t S2_PKT_0[] = {0x03,0x91,0x00,0x0d,0x00,0x08,0x00,0x00,0x01,0x00,0xff,0xff,0xff,0xff,0xff,0xff};
static const uint8_t S2_PKT_1[] = {0x07,0x91,0x00,0x01,0x00,0x00,0x00,0x00};
static const uint8_t S2_PKT_2[] = {0x16,0x91,0x00,0x01,0x00,0x00,0x00,0x00};
static const uint8_t S2_PKT_3[] = {0x15,0x91,0x00,0x01,0x00,0x0e,0x00,0x00,0x00,0x02,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff};
static const uint8_t S2_PKT_4[] = {0x15,0x91,0x00,0x02,0x00,0x11,0x00,0x00,0x00,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff};
static const uint8_t S2_PKT_5[] = {0x15,0x91,0x00,0x03,0x00,0x01,0x00,0x00,0x00};
static const uint8_t S2_PKT_6[] = {0x09,0x91,0x00,0x07,0x00,0x08,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
static const uint8_t S2_PKT_7[] = {0x0c,0x91,0x00,0x02,0x00,0x04,0x00,0x00,0x27,0x00,0x00,0x00};
static const uint8_t S2_PKT_8[] = {0x11,0x91,0x00,0x03,0x00,0x00,0x00,0x00};
static const uint8_t S2_PKT_9[] = {0x0a,0x91,0x00,0x08,0x00,0x14,0x00,0x00,0x01,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0x35,0x00,0x46,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
static const uint8_t S2_PKT_10[] = {0x0c,0x91,0x00,0x04,0x00,0x04,0x00,0x00,0x27,0x00,0x00,0x00};
static const uint8_t S2_PKT_11[] = {0x03,0x91,0x00,0x0a,0x00,0x04,0x00,0x00,0x09,0x00,0x00,0x00};
static const uint8_t S2_PKT_12[] = {0x10,0x91,0x00,0x01,0x00,0x00,0x00,0x00};
static const uint8_t S2_PKT_13[] = {0x01,0x91,0x00,0x0c,0x00,0x00,0x00,0x00};
static const uint8_t S2_PKT_14[] = {0x03,0x91,0x00,0x01,0x00,0x00,0x00};
static const uint8_t S2_PKT_15[] = {0x0a,0x91,0x00,0x02,0x00,0x04,0x00,0x00,0x03,0x00,0x00};
static const uint8_t S2_PKT_16[] = {0x09,0x91,0x00,0x07,0x00,0x08,0x00,0x00,0x01,0x00,0x00,0x00,0x00,0x00,0x00,0x00};

static const struct {
	const uint8_t *data;
	uint8_t len;
} s2_packets[] = {
	{S2_PKT_0, sizeof(S2_PKT_0)}, {S2_PKT_1, sizeof(S2_PKT_1)},
	{S2_PKT_2, sizeof(S2_PKT_2)}, {S2_PKT_3, sizeof(S2_PKT_3)},
	{S2_PKT_4, sizeof(S2_PKT_4)}, {S2_PKT_5, sizeof(S2_PKT_5)},
	{S2_PKT_6, sizeof(S2_PKT_6)}, {S2_PKT_7, sizeof(S2_PKT_7)},
	{S2_PKT_8, sizeof(S2_PKT_8)}, {S2_PKT_9, sizeof(S2_PKT_9)},
	{S2_PKT_10, sizeof(S2_PKT_10)}, {S2_PKT_11, sizeof(S2_PKT_11)},
	{S2_PKT_12, sizeof(S2_PKT_12)}, {S2_PKT_13, sizeof(S2_PKT_13)},
	{S2_PKT_14, sizeof(S2_PKT_14)}, {S2_PKT_15, sizeof(S2_PKT_15)},
	{S2_PKT_16, sizeof(S2_PKT_16)}
};

static int g_s2_index;
static uint64_t g_s2_keepalive_ms;

static int g_init_state;
static int g_init_tries;
static uint64_t g_init_due;
static uint64_t g_x360_led_ms;
static uint64_t g_ps3_keep_ms;
static uint8_t g_ps3_led[48];

static uint64_t mono_ms(void)
{
	struct timeval tv;

	gettimeofday(&tv, NULL);
	return (uint64_t)tv.tv_sec * 1000ull + (uint64_t)tv.tv_usec / 1000ull;
}

static int send_exact(uint8_t index, const uint8_t *packet, size_t length, int timeout_ms)
{
	uint8_t copy[PACKET_SIZE];
	int sent;

	if (!g_dev.use_fs || length == 0 || length > PACKET_SIZE)
		return -1;
	fs_clear_pending(g_dev.in_index);
	if (g_dev.has_bulk_in)
		fs_clear_pending(g_dev.bulk_in_index);
	memset(copy, 0, sizeof(copy));
	memcpy(copy, packet, length);
	sent = fs_transfer(index, copy, (uint32_t)length, timeout_ms, 1);
	return sent == (int)length ? 0 : -1;
}

static int send_switch2_keepalive(void)
{
	return send_exact(g_dev.bulk_out_index, S2_PKT_14, sizeof(S2_PKT_14), 40);
}

static int usb_set_report(int fd, uint8_t type, uint8_t id, void *data, uint16_t length)
{
	struct usb_ctl_request request;

	memset(&request, 0, sizeof(request));
	request.ucr_data = data;
	request.ucr_flags = USB_SHORT_XFER_OK;
	request.ucr_request.bmRequestType = 0x21;
	request.ucr_request.bRequest = 0x09;
	request.ucr_request.wValue[0] = id;
	request.ucr_request.wValue[1] = type;
	request.ucr_request.wLength[0] = (uint8_t)length;
	request.ucr_request.wLength[1] = (uint8_t)(length >> 8);
	return ioctl(fd, USB_DO_REQUEST, &request) == 0 ? 0 : -1;
}

static void ps3_fill_led(void)
{
	int slot;

	memset(g_ps3_led, 0, sizeof(g_ps3_led));
	g_ps3_led[9] = 0x02;
	for (slot = 0; slot < 4; slot++) {
		uint8_t *led = g_ps3_led + 10 + slot * 5;

		led[0] = 0xff;
		led[1] = 0x27;
		led[2] = 0x10;
		led[4] = 0x32;
	}
}

static void ps3_keepalive(void)
{
	if (g_dev.ctrl_fd < 0)
		return;
	if (g_ps3_keep_ms != 0 && mono_ms() - g_ps3_keep_ms < 1000)
		return;
	g_ps3_keep_ms = mono_ms();
	usb_set_report(g_dev.ctrl_fd, 0x02, 0x01, g_ps3_led, sizeof(g_ps3_led));
}

static void x360_led(void)
{
	uint8_t led[3] = {0x01, 0x03, 0x06};

	if (!g_dev.has_hid_out)
		return;
	if (g_x360_led_ms != 0 && mono_ms() - g_x360_led_ms < 1000)
		return;
	g_x360_led_ms = mono_ms();
	send_exact(g_dev.out_index, led, sizeof(led), 40);
}

static void init_reset(void)
{
	int quick = g_xbox || g_x360 || g_ps3 || g_dinput;

	g_init_state = g_xbox ? INIT_XB_SETTLE :
		       g_x360 ? INIT_X360_SETTLE :
		       g_ps3 ? INIT_PS3_SETTLE :
		       g_dinput ? INIT_SIMPLE_SETTLE :
		       g_ds4 ? INIT_DS4_SETTLE :
		       g_switch2 ? INIT_S2_SETTLE : INIT_SETTLE;
	g_init_tries = 0;
	g_s2_index = 0;
	g_gip_seq = 0;
	g_init_done = 0;
	g_x360_led_ms = 0;
	g_ps3_keep_ms = 0;
	g_init_due = mono_ms() + (quick ? 50 : 250);
	g_s2_keepalive_ms = mono_ms();
}

static int read_in(uint8_t *buffer, uint32_t length)
{
	return fs_transfer(g_dev.in_index, buffer, length, 15, 0);
}

static int wait_timed_out(void)
{
	if (mono_ms() < g_init_due)
		return 0;
	if (++g_init_tries < 5)
		return 1;
	g_init_tries = 0;
	return 2;
}

static int init_ds4(void)
{
	uint8_t packet[32];

	if (g_init_state == INIT_DS4_SETTLE) {
		if (mono_ms() < g_init_due)
			return 0;
		g_init_state = INIT_DS4_LED;
	}
	memset(packet, 0, sizeof(packet));
	packet[0] = 0x05;
	packet[1] = 0x02;
	packet[8] = 0x40;
	if (g_dev.has_hid_out)
		send_exact(g_dev.out_index, packet, sizeof(packet), 80);
	g_init_state = INIT_DONE;
	g_init_done = 1;
	g_ready = 1;
	g_step = STEP_READY;
	printf("YetAnotherControllerEnabler: DualShock 4 ready\n");
	return 0;
}

static int send_gip(const uint8_t *packet, size_t length)
{
	uint8_t copy[PACKET_SIZE];

	if (length < 3 || length > sizeof(copy))
		return -1;
	memcpy(copy, packet, length);
	copy[2] = g_gip_seq++;
	return send_exact(g_dev.out_index, copy, length, 80);
}

static int xbox_step(const uint8_t *packet, size_t length, int next_state)
{
	if (send_gip(packet, length) != 0) {
		if (++g_init_tries < 4)
			return 0;
		return -1;
	}
	g_init_tries = 0;
	g_init_state = next_state;
	return 0;
}

static int init_xbox(void)
{
	static const uint8_t power_on[] = {0x05, 0x20, 0x00, 0x01, 0x00};
	static const uint8_t s_init[] = {
		0x05, 0x20, 0x00, 0x0f, 0x06, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x55, 0x53, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
	};
	static const uint8_t led_on[] = {0x0a, 0x20, 0x00, 0x03, 0x00, 0x01, 0x14};
	static const uint8_t extra[] = {0x4d, 0x10, 0x00, 0x02, 0x07, 0x00};

	if (g_init_state == INIT_XB_SETTLE) {
		if (mono_ms() < g_init_due)
			return 0;
		g_init_state = INIT_XB_POWER;
		g_init_tries = 0;
	}
	if (g_init_state == INIT_XB_POWER)
		return xbox_step(power_on, sizeof(power_on), INIT_XB_SINIT);
	if (g_init_state == INIT_XB_SINIT)
		return xbox_step(s_init, sizeof(s_init), INIT_XB_LED);
	if (g_init_state == INIT_XB_LED)
		return xbox_step(led_on, sizeof(led_on), g_product == 0x0b00 ? INIT_XB_EXTRA : INIT_DONE);
	if (g_init_state == INIT_XB_EXTRA) {
		int sent = xbox_step(extra, sizeof(extra), INIT_DONE);
		if (g_init_state != INIT_DONE)
			return sent;
	}
	if (g_init_state == INIT_DONE) {
		g_init_done = 1;
		g_ready = 1;
		g_step = STEP_READY;
		printf("YetAnotherControllerEnabler: Xbox ready\n");
	}
	return 0;
}

static int init_switch2(uint8_t *buffer, uint32_t length)
{
	(void)buffer;
	(void)length;
	if (g_init_state == INIT_S2_SETTLE) {
		if (mono_ms() < g_init_due)
			return 0;
		g_init_state = INIT_S2_SEND;
		g_s2_index = 0;
	}
	if (g_init_state == INIT_S2_SEND) {
		const uint8_t *packet = s2_packets[g_s2_index].data;
		uint8_t len = s2_packets[g_s2_index].len;

		if (send_exact(g_dev.bulk_out_index, packet, len, 80) != 0) {
			if (++g_init_tries < 4)
				return 0;
			return -1;
		}
		g_init_tries = 0;
		if (g_dev.has_bulk_in) {
			uint8_t junk[PACKET_SIZE];
			fs_transfer(g_dev.bulk_in_index, junk, sizeof(junk), 8, 1);
		}
		g_init_state = INIT_S2_GAP;
		g_init_due = mono_ms() + 12;
		return 0;
	}
	if (g_init_state == INIT_S2_GAP) {
		if (mono_ms() < g_init_due)
			return 0;
		g_s2_index++;
		if (g_s2_index < (int)(sizeof(s2_packets) / sizeof(s2_packets[0]))) {
			g_init_state = INIT_S2_SEND;
			return 0;
		}
		g_init_state = INIT_S2_DISABLE;
	}
	if (g_init_state == INIT_S2_DISABLE) {
		if (g_dev.has_hid_out)
			send_usb_command(0x04);
		send_switch2_keepalive();
		g_s2_keepalive_ms = mono_ms();
		g_init_state = INIT_DONE;
		g_init_done = 1;
		g_ready = 1;
		g_step = STEP_READY;
		printf("YetAnotherControllerEnabler: Switch 2 Pro ready\n");
	}
	return 0;
}

static int mark_ready(const char *name)
{
	g_init_state = INIT_DONE;
	g_init_done = 1;
	g_ready = 1;
	g_step = STEP_READY;
	printf("YetAnotherControllerEnabler: %s ready\n", name);
	return 0;
}

static int init_simple(void)
{
	if (mono_ms() < g_init_due)
		return 0;
	return mark_ready(usb_switch_name());
}

static int init_ps3(void)
{
	static uint8_t magic[4] = {0x42, 0x0c, 0x00, 0x00};

	if (g_init_state == INIT_PS3_SETTLE) {
		if (mono_ms() < g_init_due)
			return 0;
		g_init_state = INIT_PS3_ENABLE;
	}
	ps3_fill_led();
	if (usb_set_report(g_dev.ctrl_fd, 0x03, 0xf4, magic, sizeof(magic)) != 0)
		printf("YetAnotherControllerEnabler: DualShock 3 enable errno %d\n", errno);
	if (usb_set_report(g_dev.ctrl_fd, 0x02, 0x01, g_ps3_led, sizeof(g_ps3_led)) != 0)
		printf("YetAnotherControllerEnabler: DualShock 3 led errno %d\n", errno);
	g_ps3_keep_ms = mono_ms();
	return mark_ready("DualShock 3");
}

static int init_x360(void)
{
	if (g_init_state == INIT_X360_SETTLE) {
		if (mono_ms() < g_init_due)
			return 0;
		g_init_state = INIT_X360_LED;
		g_init_tries = 0;
	}
	if (g_init_state == INIT_X360_LED) {
		uint8_t led[3] = {0x01, 0x03, 0x06};

		if (send_exact(g_dev.out_index, led, sizeof(led), 80) != 0 && ++g_init_tries < 4)
			return 0;
		g_x360_led_ms = mono_ms();
	}
	return mark_ready(usb_switch_name());
}

/* One handshake step per call so a late plug-in can answer before the LEDs are sent. */
static int init_poll(uint8_t *buffer, uint32_t length)
{
	if (g_xbox)
		return init_xbox();
	if (g_x360)
		return init_x360();
	if (g_ps3)
		return init_ps3();
	if (g_dinput)
		return init_simple();
	if (g_ds4)
		return init_ds4();
	if (g_switch2)
		return init_switch2(buffer, length);
	uint8_t mode = 0x30;
	uint8_t led = 0x01;
	uint8_t home[3] = {0x0f, 0xf0, 0xf0};
	int got = 0;

	if (g_init_state == INIT_DONE)
		return 0;
	if (g_init_state == INIT_SETTLE) {
		if (mono_ms() < g_init_due)
			return 0;
		g_init_state = INIT_SEND_HS;
		g_init_tries = 0;
	}

	switch (g_init_state) {
	case INIT_SEND_HS:
		if (send_usb_command(0x02) != 0)
			return -1;
		g_init_state = INIT_WAIT_HS;
		g_init_due = mono_ms() + 250;
		return 0;
	case INIT_WAIT_HS:
		got = read_in(buffer, length);
		if (got >= 2 && buffer[0] == 0x81 && buffer[1] == 0x02) {
			g_init_state = INIT_SEND_BAUD;
			g_init_tries = 0;
			break;
		}
		{
			int timed = wait_timed_out();
			if (timed == 1)
				g_init_state = INIT_SEND_HS;
			else if (timed == 2)
				g_init_state = INIT_SEND_BAUD;
		}
		break;
	case INIT_SEND_BAUD:
		if (send_usb_command(0x03) != 0)
			return -1;
		g_init_state = INIT_WAIT_BAUD;
		g_init_due = mono_ms() + 250;
		return 0;
	case INIT_WAIT_BAUD:
		got = read_in(buffer, length);
		if (got >= 2 && buffer[0] == 0x81 && buffer[1] == 0x03) {
			g_init_state = INIT_SEND_HS2;
			g_init_tries = 0;
			break;
		}
		{
			int timed = wait_timed_out();
			if (timed == 1)
				g_init_state = INIT_SEND_BAUD;
			else if (timed == 2)
				g_init_state = INIT_SEND_HS2;
		}
		break;
	case INIT_SEND_HS2:
		if (send_usb_command(0x02) != 0)
			return -1;
		g_init_state = INIT_WAIT_HS2;
		g_init_due = mono_ms() + 250;
		return 0;
	case INIT_WAIT_HS2:
		got = read_in(buffer, length);
		if (got >= 2 && buffer[0] == 0x81 && buffer[1] == 0x02) {
			g_init_state = INIT_SEND_DISABLE;
			g_init_tries = 0;
			break;
		}
		{
			int timed = wait_timed_out();
			if (timed == 1)
				g_init_state = INIT_SEND_HS2;
			else if (timed == 2)
				g_init_state = INIT_SEND_DISABLE;
		}
		break;
	case INIT_SEND_DISABLE:
		if (send_usb_command(0x04) != 0)
			return -1;
		g_init_state = INIT_SEND_MODE;
		return 0;
	case INIT_SEND_MODE:
		if (send_subcommand(0x03, &mode, 1, 12) != 0)
			return -1;
		g_init_state = INIT_WAIT_MODE;
		g_init_due = mono_ms() + 250;
		g_init_tries = 0;
		return 0;
	case INIT_WAIT_MODE:
		got = read_in(buffer, length);
		if ((got >= 15 && buffer[0] == 0x21 && buffer[14] == 0x03) ||
		    (got >= 1 && (buffer[0] == 0x30 || buffer[0] == 0x31))) {
			g_init_state = INIT_SEND_LED;
			g_init_tries = 0;
			break;
		}
		{
			int timed = wait_timed_out();
			if (timed == 1)
				g_init_state = INIT_SEND_MODE;
			else if (timed == 2)
				g_init_state = INIT_SEND_LED;
		}
		break;
	case INIT_SEND_LED:
		if (send_subcommand(0x30, &led, 1, 12) != 0)
			return -1;
		g_init_state = INIT_SEND_HOME;
		return 0;
	case INIT_SEND_HOME:
		if (send_subcommand(0x38, home, 3, 14) != 0)
			return -1;
		g_init_state = INIT_DONE;
		g_init_done = 1;
		g_ready = 1;
		g_step = STEP_READY;
		printf("YetAnotherControllerEnabler: Switch Pro LEDs on\n");
		return 0;
	default:
		return 0;
	}
	if (got < 0)
		return -1;
	if (got > 0 && (buffer[0] == 0x30 || buffer[0] == 0x31) &&
	    g_init_state < INIT_SEND_LED) {
		g_init_state = INIT_SEND_LED;
		g_init_tries = 0;
	}
	if (got > 0 && input_report(buffer[0]))
		return got;
	return 0;
}

static int input_report(uint8_t id)
{
	if (g_ps3)
		return id == 0x01;
	if (g_dinput)
		return 1;
	if (g_x360)
		return id == 0x00;
	if (g_xbox)
		return id == 0x20 || id == 0x07;
	if (g_ds4 && id == 0x01)
		return 1;
	if (g_switch2 && id == 0x09)
		return 1;
	return id == 0x21 || id == 0x30 || id == 0x31 || id == 0x32 || id == 0x33;
}

/*
 * Xbox One keeps resending the same button packet until this matches the
 * xone acknowledge: client id with the internal flag, and a 16-bit length.
 * Echoing the original flag byte leaves the press latched.
 */
static void ack_gip(const uint8_t *report, int count)
{
	size_t offset = 0;

	if (!g_xbox || report == NULL || count < 4)
		return;
	while (offset + 4 <= (size_t)count) {
		const uint8_t *msg = report + offset;
		size_t payload = msg[3];
		uint8_t client;
		uint8_t ack[13];

		if ((msg[3] & 0x80) || payload < 2 || payload > 60 ||
		    offset + 4 + payload > (size_t)count)
			break;
		offset += 4 + payload;
		if ((msg[1] & 0x10) == 0)
			continue;
		client = (uint8_t)((msg[1] & 0x0f) | 0x20);
		memset(ack, 0, sizeof(ack));
		ack[0] = 0x01;
		ack[1] = client;
		ack[2] = msg[2];
		ack[3] = 0x09;
		ack[5] = msg[0];
		ack[6] = client;
		ack[7] = (uint8_t)payload;
		send_exact(g_dev.out_index, ack, sizeof(ack), 40);
	}
}

int usb_switch_is_switch2(void)
{
	return g_switch2;
}

int usb_switch_is_ds4(void)
{
	return g_ds4;
}

int usb_switch_is_xbox(void)
{
	return g_xbox;
}

int usb_switch_is_ps3(void)
{
	return g_ps3;
}

int usb_switch_is_x360(void)
{
	return g_x360;
}

int usb_switch_is_dinput(void)
{
	return g_dinput;
}

int usb_switch_is_8bitdo(void)
{
	return g_bitdo;
}

int usb_switch_xbox_share(void)
{
	return g_xbox && g_product == 0x0b12;
}

const char *usb_switch_name(void)
{
	if (g_ps3)
		return "DualShock 3";
	if (g_bitdo)
		return "8BitDo";
	if (g_x360)
		return "Xbox 360";
	if (g_xbox)
		return "Xbox";
	if (g_ds4)
		return "DualShock 4";
	if (g_switch2)
		return "Switch 2 Pro";
	return "Switch Pro";
}

static int take_report(uint8_t *buffer, int count)
{
	if (count >= 1 && input_report(buffer[0])) {
		g_ready = 1;
		g_step = STEP_READY;
		return count;
	}
	return 0;
}

int usb_switch_is_ready(void)
{
	return g_ready;
}

static int g_capturing;

/* Every IN transfer for a live pad uses this buffer, including one left queued across the inject sleep. */
static uint8_t g_rx[PACKET_SIZE];

static int in_transfer(uint8_t *dst, uint32_t dst_cap, int timeout_ms)
{
	int got;

	if (g_dev.in_index >= EP_SLOTS)
		return -1;
	got = fs_transfer(g_dev.in_index, g_rx, PACKET_SIZE, timeout_ms, 0);
	if (got > 0 && dst != NULL && dst != g_rx) {
		uint32_t n = (uint32_t)got;

		if (n > dst_cap)
			n = dst_cap;
		memcpy(dst, g_rx, n);
		return (int)n;
	}
	return got;
}

/* Leave an IN transfer queued so a packet that arrives during the inject sleep is kept. */
static void arm_input(void)
{
	if (!g_init_done || (!g_xbox && !g_x360) || g_dev.in_index >= EP_SLOTS)
		return;
	if (g_ep_pending[g_dev.in_index])
		return;
	fs_transfer(g_dev.in_index, g_rx, PACKET_SIZE, -1, 0);
}

/* Keep the IN pipe armed and hold a release that arrives during an acknowledge. */
static void capture_followup(void)
{
	int i;

	if (g_capturing || !g_init_done || (!g_xbox && !g_x360))
		return;
	g_capturing = 1;
	for (i = 0; i < IN_QUEUE && g_in_q_n < IN_QUEUE; i++) {
		uint8_t extra[PACKET_SIZE];
		int got;

		memset(extra, 0, sizeof(extra));
		got = in_transfer(extra, PACKET_SIZE, 2);
		if (got <= 0)
			break;
		if (g_xbox)
			ack_gip(extra, got);
		if (g_xbox && extra[0] != 0x20 && extra[0] != 0x07)
			continue;
		if (g_x360 && extra[0] != 0x00)
			continue;
		queue_push(extra, got);
	}
	arm_input();
	g_capturing = 0;
}

int usb_switch_read(uint8_t *buffer, size_t length)
{
	ssize_t count;
	int queued;

	if (buffer == NULL || length == 0)
		return -1;

	queued = queue_pop(buffer, length);
	if (queued > 0) {
		arm_input();
		return take_report(buffer, queued);
	}

	if (g_dev.in_fd >= 0) {
		count = read(g_dev.in_fd, buffer, length);
		if (count > 0)
			return take_report(buffer, (int)count);
		return 0;
	}
	if (g_dev.use_fs && !g_init_done) {
		int got = init_poll(buffer, (uint32_t)length);
		if (got < 0)
			return -1;
		if (got > 0)
			return take_report(buffer, got);
		return 0;
	}
	if (g_dev.use_fs) {
		static unsigned keepalives;
		int transferred;
		uint32_t ask = (uint32_t)length;

		if (g_x360)
			x360_led();
		if (g_ps3)
			ps3_keepalive();
		if (ask > PACKET_SIZE)
			ask = PACKET_SIZE;
		memset(buffer, 0, ask);
		transferred = in_transfer(buffer, ask, 20);
		if (transferred > 0 && g_xbox)
			ack_gip(buffer, transferred);
		if (transferred > 0 && (g_xbox || g_x360))
			capture_followup();
		else if (g_xbox || g_x360)
			arm_input();
		if (transferred > 0 && input_report(buffer[0])) {
			if (g_switch2) {
				if (mono_ms() - g_s2_keepalive_ms >= 2000) {
					g_s2_keepalive_ms = mono_ms();
					send_switch2_keepalive();
				}
			} else if (!g_ds4 && !g_xbox && !g_x360 && !g_ps3 && !g_dinput) {
				keepalives++;
				if (keepalives >= 8 && g_dev.has_hid_out && !g_ep_pending[g_dev.out_index]) {
					keepalives = 0;
					send_rumble();
				}
			}
			return take_report(buffer, transferred);
		}
		if (transferred > 0)
			return take_report(buffer, transferred);
		if (transferred < 0 && (errno == ENODEV || errno == ENXIO || errno == EBADF))
			return -1;
		return 0;
	}
	return 0;
}

int usb_switch_open(void)
{
	int fd;

	if (g_dev.ctrl_fd >= 0)
		return 0;

	g_seen[0] = '\0';
	g_seen_count = 0;
	fd = open_hid_node();
	if (fd < 0)
		fd = open_from_usbctl();
	if (fd < 0)
		fd = open_from_directory();
	if (fd < 0)
		fd = open_numbered();
	if (fd < 0) {
		if (g_seen_count == 0)
			snprintf(g_status, sizeof(g_status), "No USB devices opened");
		else
			snprintf(g_status, sizeof(g_status), "No supported controller. Saw %s", g_seen);
		return -1;
	}

	{
		int writable = open(g_dev.path, O_RDWR);

		if (writable < 0) {
			snprintf(g_status, sizeof(g_status), "Switch Pro open errno %d", errno);
			close(fd);
			return -2;
		}
		close(fd);
		fd = writable;
	}
	g_dev.ctrl_fd = fd;
	g_ready = 0;
	g_init_done = 0;
	g_step = STEP_MAC;
	g_fail[0] = '\0';
	if (fs_setup(fd) != 0) {
		printf("YetAnotherControllerEnabler: Switch Pro output report failed\n");
		usb_switch_close();
		snprintf(g_status, sizeof(g_status), "USB setup failed: %s",
			 g_fail[0] != '\0' ? g_fail : "unknown");
		return -2;
	}
	init_reset();
	printf("YetAnotherControllerEnabler: Switch Pro init started\n");
	return 0;
}
