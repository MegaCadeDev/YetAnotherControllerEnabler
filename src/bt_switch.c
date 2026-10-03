#include "bt_switch.h"

#include <dev/usb/usb_ioctl.h>
#include <dev/usb/usbdi.h>

#include <dirent.h>
#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <string.h>
#include <sys/select.h>
#include <sys/time.h>
#include <unistd.h>
#include <sys/ioctl.h>

/*
 * Controller 0 on the internal Bluetooth chip. Controller 1 is DualSense:
 * its endpoints (0x04 / 0x84 / 0x85) are never opened, and HCI Reset is never sent.
 */
#define EP_EVT 0
#define EP_ACL_IN 1
#define EP_ACL_OUT 2
#define EP_COUNT 3

#define CID_SIGNAL 0x0001
#define CID_ATT 0x0004
#define CID_SMP 0x0006
#define CID_HID_CONTROL 0x0040
#define CID_HID_INTERRUPT 0x0041
#define PSM_HID_CONTROL 0x0011
#define PSM_HID_INTERRUPT 0x0013

enum {
	PH_CLOSED = 0,
	PH_SETUP,
	PH_SCAN_BR,
	PH_SCAN_LE,
	PH_NAME,
	PH_CONNECT,
	PH_LE_CONNECT,
	PH_SECURITY,
	PH_L2CAP,
	PH_SW1,
	PH_SW2,
	PH_READY
};

struct link_key {
	uint8_t addr[6];
	uint8_t key[16];
	int used;
};

static int g_fd = -1;
static struct usb_fs_endpoint g_ep[EP_COUNT];
static void *g_ptr[EP_COUNT];
static uint32_t g_len[EP_COUNT];
static uint8_t g_pending[EP_COUNT];
static uint16_t g_room[EP_COUNT];
static uint8_t g_evt[64];
static uint8_t g_acl[512];

static int g_phase = PH_CLOSED;
static int g_setup_step;
static int g_hold;
static int g_ready;
static int g_switch2;
static int g_lost;
static int g_needs_pair;
static uint16_t g_wait_op;
static int g_cmd_done;
static int g_cmd_status;
static uint8_t g_acl_credits = 1;
static uint8_t g_acl_inflight;
static uint16_t g_handle;
static uint8_t g_peer[6];
static uint8_t g_peer_type;
static uint8_t g_page_rep;
static uint16_t g_clock;
static uint8_t g_local[6];
static int g_have_local;
static uint16_t g_dcid_control;
static uint16_t g_dcid_interrupt;
static int g_control_up;
static int g_interrupt_up;
static int g_control_cfg;
static int g_interrupt_cfg;
static uint8_t g_sig_id = 1;
static uint8_t g_hid_seq;
static int g_sw_step;
static uint64_t g_due;
static uint64_t g_keep_ms;
static uint8_t g_vib_seq;
static uint8_t g_report[128];
static int g_report_len;
static uint8_t g_l2cap[1024];
static int g_l2cap_len;
static uint16_t g_l2cap_expect;
static int g_l2cap_open;
static struct link_key g_keys[4];
static uint8_t g_preq[7];
static uint8_t g_pres[7];
static uint8_t g_mrand[16];
static uint8_t g_srand[16];
static uint8_t g_mconfirm[16];
static int g_smp;
static int g_smp_step;
static int g_scan_step;
static int g_le_on;
static int g_inquiry_on;
static int g_scan_enable;

static const uint8_t g_ltk1[17] = {
	0x00, 0xea, 0xbd, 0x47, 0x13, 0x89, 0x35, 0x42,
	0xc6, 0x79, 0xee, 0x07, 0xf2, 0x53, 0x2c, 0x6c, 0x31
};
static const uint8_t g_ltk2[17] = {
	0x00, 0x40, 0xb0, 0x8a, 0x5f, 0xcd, 0x1f, 0x9b,
	0x41, 0x12, 0x5c, 0xac, 0xc6, 0x3f, 0x38, 0xa0, 0x73
};

static uint64_t mono_ms(void)
{
	struct timeval tv;

	gettimeofday(&tv, NULL);
	return (uint64_t)tv.tv_sec * 1000ull + (uint64_t)tv.tv_usec / 1000ull;
}

static void put_le16(uint8_t *p, uint16_t value)
{
	p[0] = (uint8_t)value;
	p[1] = (uint8_t)(value >> 8);
}

static uint16_t get_le16(const uint8_t *p)
{
	return (uint16_t)(p[0] | ((uint16_t)p[1] << 8));
}

static void reverse_bytes(const uint8_t *src, uint8_t *dst, int n)
{
	int i;

	for (i = 0; i < n; i++)
		dst[i] = src[n - 1 - i];
}

static int fs_wait(uint8_t index, int timeout_ms)
{
	int spins = timeout_ms / 5 + 1;

	while (spins-- > 0) {
		struct usb_fs_complete complete;
		struct timeval wait;
		fd_set reads;
		fd_set writes;

		memset(&complete, 0, sizeof(complete));
		complete.ep_index = index;
		if (ioctl(g_fd, USB_FS_COMPLETE, &complete) == 0) {
			if (complete.ep_index < EP_COUNT)
				g_pending[complete.ep_index] = 0;
			if (complete.ep_index == index)
				return 0;
			continue;
		}
		if (errno != EBUSY)
			return -1;
		FD_ZERO(&reads);
		FD_ZERO(&writes);
		FD_SET(g_fd, &reads);
		FD_SET(g_fd, &writes);
		wait.tv_sec = 0;
		wait.tv_usec = 5000;
		if (select(g_fd + 1, &reads, &writes, NULL, &wait) > 0)
			usleep(1000);
		else
			usleep(5000);
	}
	return 1;
}

static int fs_xfer(uint8_t index, uint8_t *buffer, uint32_t length, int timeout_ms)
{
	struct usb_fs_start start;
	int waited;

	if (g_room[index] > 0 && length > g_room[index])
		length = g_room[index];
	if (length == 0)
		return 0;
	g_ptr[index] = buffer;
	g_len[index] = length;
	g_ep[index].ppBuffer = &g_ptr[index];
	g_ep[index].pLength = &g_len[index];
	g_ep[index].nFrames = 1;
	g_ep[index].flags = USB_FS_FLAG_SINGLE_SHORT_OK;
	/*
	 * IN transfers stay armed until the chip answers. A one-second timeout
	 * was firing during the USB scan and dropping the command reply.
	 */
	g_ep[index].timeout = index == EP_ACL_OUT ? (uint16_t)timeout_ms : 0;
	if (!g_pending[index]) {
		start.ep_index = index;
		if (ioctl(g_fd, USB_FS_START, &start) != 0) {
			if (errno != EBUSY)
				return -1;
		}
		g_pending[index] = 1;
	}
	waited = fs_wait(index, timeout_ms);
	if (waited != 0)
		return waited < 0 ? -1 : 0;
	g_pending[index] = 0;
	if (g_ep[index].status != USB_ERR_NORMAL_COMPLETION &&
	    g_ep[index].status != USB_ERR_SHORT_XFER)
		return 0;
	if (g_len[index] > length)
		return 0;
	return (int)g_len[index];
}

static int hci_control(const uint8_t *data, uint16_t length)
{
	struct usb_ctl_request request;

	memset(&request, 0, sizeof(request));
	request.ucr_data = (void *)data;
	request.ucr_flags = USB_SHORT_XFER_OK;
	request.ucr_request.bmRequestType = 0x20;
	request.ucr_request.bRequest = 0;
	request.ucr_request.wLength[0] = (uint8_t)length;
	request.ucr_request.wLength[1] = (uint8_t)(length >> 8);
	return ioctl(g_fd, USB_DO_REQUEST, &request) == 0 ? 0 : -1;
}

static void queue_report(const uint8_t *data, int length)
{
	if (length <= 0 || length > (int)sizeof(g_report))
		return;
	memcpy(g_report, data, (size_t)length);
	g_report_len = length;
}

static int name_is_switch(const char *name)
{
	char lower[80];
	size_t i;

	for (i = 0; i < sizeof(lower) - 1 && name[i] != '\0'; i++) {
		char c = name[i];
		if (c >= 'A' && c <= 'Z')
			c = (char)(c - 'A' + 'a');
		lower[i] = c;
	}
	lower[i] = '\0';
	if (strstr(lower, "dualsense") != NULL || strstr(lower, "sony") != NULL)
		return 0;
	if (strstr(lower, "pro controller") != NULL || strstr(lower, "joy-con") != NULL)
		return 1;
	return 0;
}

static int class_is_gamepad(const uint8_t cod[3])
{
	unsigned minor = (cod[0] >> 2) & 0x3f;
	unsigned major = cod[1] & 0x1f;

	return minor == 0x02 && major == 0x05;
}

static struct link_key *key_for(const uint8_t addr[6])
{
	int i;

	for (i = 0; i < 4; i++) {
		if (g_keys[i].used && memcmp(g_keys[i].addr, addr, 6) == 0)
			return &g_keys[i];
	}
	return NULL;
}

static void remember_key(const uint8_t addr[6], const uint8_t key[16])
{
	struct link_key *slot = key_for(addr);
	int i;

	if (slot == NULL) {
		for (i = 0; i < 4; i++) {
			if (!g_keys[i].used) {
				slot = &g_keys[i];
				break;
			}
		}
	}
	if (slot == NULL)
		slot = &g_keys[0];
	memcpy(slot->addr, addr, 6);
	memcpy(slot->key, key, 16);
	slot->used = 1;
}

static int hci_send(uint16_t opcode, const uint8_t *param, uint8_t n)
{
	uint8_t packet[260];

	if (g_wait_op != 0)
		return -1;
	packet[0] = (uint8_t)opcode;
	packet[1] = (uint8_t)(opcode >> 8);
	packet[2] = n;
	if (n > 0)
		memcpy(packet + 3, param, n);
	if (hci_control(packet, (uint16_t)(n + 3)) != 0) {
		printf("YetAnotherControllerEnabler: bluetooth command %04x errno %d\n", opcode, errno);
		return -1;
	}
	g_wait_op = opcode;
	g_cmd_done = 0;
	g_cmd_status = 0;
	return 0;
}

static int acl_send(uint16_t cid, const uint8_t *payload, uint16_t length)
{
	uint8_t packet[700];
	uint16_t total = (uint16_t)(length + 4);
	int sent;

	if (g_handle == 0 || g_acl_credits == 0 || g_acl_inflight >= 3 || total + 4 > sizeof(packet))
		return -1;
	put_le16(packet, (uint16_t)(g_handle | 0x2000));
	put_le16(packet + 2, total);
	put_le16(packet + 4, length);
	put_le16(packet + 6, cid);
	memcpy(packet + 8, payload, length);
	sent = fs_xfer(EP_ACL_OUT, packet, (uint32_t)(total + 4), 40);
	if (sent <= 0)
		return -1;
	if (g_acl_credits > 0)
		g_acl_credits--;
	g_acl_inflight++;
	return 0;
}

static void signal_send(const uint8_t *payload, uint16_t length)
{
	acl_send(CID_SIGNAL, payload, length);
}

static void l2cap_connect(uint16_t psm, uint16_t scid)
{
	uint8_t body[8];

	body[0] = 0x02;
	body[1] = g_sig_id++;
	if (g_sig_id == 0)
		g_sig_id = 1;
	put_le16(body + 2, 4);
	put_le16(body + 4, psm);
	put_le16(body + 6, scid);
	signal_send(body, sizeof(body));
}

static void l2cap_config(uint16_t dcid)
{
	uint8_t body[12];

	body[0] = 0x04;
	body[1] = g_sig_id++;
	if (g_sig_id == 0)
		g_sig_id = 1;
	put_le16(body + 2, 8);
	put_le16(body + 4, dcid);
	put_le16(body + 6, 0);
	body[8] = 0x01;
	body[9] = 0x02;
	put_le16(body + 10, 0x0040);
	signal_send(body, sizeof(body));
}

static void hid_output(const uint8_t *report, int length)
{
	uint8_t body[80];

	if (length + 1 > (int)sizeof(body) || g_dcid_interrupt == 0)
		return;
	body[0] = 0xa2;
	memcpy(body + 1, report, (size_t)length);
	acl_send(g_dcid_interrupt, body, (uint16_t)(length + 1));
}

static void sw1_subcmd(uint8_t sub, const uint8_t *arg, int n)
{
	uint8_t report[16];

	memset(report, 0, sizeof(report));
	report[0] = 0x01;
	report[1] = g_hid_seq;
	g_hid_seq = (uint8_t)((g_hid_seq + 1) & 0x0f);
	report[3] = 0x01;
	report[4] = 0x40;
	report[5] = 0x40;
	report[7] = 0x01;
	report[8] = 0x40;
	report[9] = 0x40;
	report[10] = sub;
	if (arg != NULL && n > 0)
		memcpy(report + 11, arg, (size_t)n);
	hid_output(report, 11 + (n > 0 ? n : 0));
}

static void sw1_rumble(void)
{
	uint8_t report[10];

	memset(report, 0, sizeof(report));
	report[0] = 0x10;
	report[1] = g_hid_seq;
	g_hid_seq = (uint8_t)((g_hid_seq + 1) & 0x0f);
	report[3] = 0x01;
	report[4] = 0x40;
	report[5] = 0x40;
	report[7] = 0x01;
	report[8] = 0x40;
	report[9] = 0x40;
	hid_output(report, sizeof(report));
}

static void att_write(uint16_t handle, const uint8_t *value, int n, int command)
{
	uint8_t body[80];

	body[0] = command ? 0x52 : 0x12;
	put_le16(body + 1, handle);
	if (n > 0)
		memcpy(body + 3, value, (size_t)n);
	acl_send(CID_ATT, body, (uint16_t)(3 + n));
}

static void sw2_cmd(uint8_t cmd, uint8_t sub, const uint8_t *data, uint8_t n)
{
	uint8_t body[48];

	body[0] = cmd;
	body[1] = 0x91;
	body[2] = 0x01;
	body[3] = sub;
	body[4] = 0x00;
	body[5] = n;
	body[6] = 0x00;
	body[7] = 0x00;
	if (n > 0 && data != NULL)
		memcpy(body + 8, data, n);
	att_write(0x0014, body, 8 + n, 1);
}

static void sw2_keepalive(void)
{
	uint8_t payload[33];
	uint8_t block[16];
	uint8_t motor[5];

	if (mono_ms() - g_keep_ms < 5)
		return;
	g_keep_ms = mono_ms();
	memset(payload, 0, sizeof(payload));
	memset(motor, 0, sizeof(motor));
	memset(block, 0, sizeof(block));
	block[0] = (uint8_t)(0x50 + (g_vib_seq & 0x0f));
	g_vib_seq++;
	memcpy(block + 1, motor, 5);
	memcpy(block + 6, motor, 5);
	memcpy(block + 11, motor, 5);
	payload[0] = 0x00;
	memcpy(payload + 1, block, 16);
	memcpy(payload + 17, block, 16);
	att_write(0x0016, payload, sizeof(payload), 1);
}

static const uint8_t sbox[256] = {
	0x63,0x7c,0x77,0x7b,0xf2,0x6b,0x6f,0xc5,0x30,0x01,0x67,0x2b,0xfe,0xd7,0xab,0x76,
	0xca,0x82,0xc9,0x7d,0xfa,0x59,0x47,0xf0,0xad,0xd4,0xa2,0xaf,0x9c,0xa4,0x72,0xc0,
	0xb7,0xfd,0x93,0x26,0x36,0x3f,0xf7,0xcc,0x34,0xa5,0xe5,0xf1,0x71,0xd8,0x31,0x15,
	0x04,0xc7,0x23,0xc3,0x18,0x96,0x05,0x9a,0x07,0x12,0x80,0xe2,0xeb,0x27,0xb2,0x75,
	0x09,0x83,0x2c,0x1a,0x1b,0x6e,0x5a,0xa0,0x52,0x3b,0xd6,0xb3,0x29,0xe3,0x2f,0x84,
	0x53,0xd1,0x00,0xed,0x20,0xfc,0xb1,0x5b,0x6a,0xcb,0xbe,0x39,0x4a,0x4c,0x58,0xcf,
	0xd0,0xef,0xaa,0xfb,0x43,0x4d,0x33,0x85,0x45,0xf9,0x02,0x7f,0x50,0x3c,0x9f,0xa8,
	0x51,0xa3,0x40,0x8f,0x92,0x9d,0x38,0xf5,0xbc,0xb6,0xda,0x21,0x10,0xff,0xf3,0xd2,
	0xcd,0x0c,0x13,0xec,0x5f,0x97,0x44,0x17,0xc4,0xa7,0x7e,0x3d,0x64,0x5d,0x19,0x73,
	0x60,0x81,0x4f,0xdc,0x22,0x2a,0x90,0x88,0x46,0xee,0xb8,0x14,0xde,0x5e,0x0b,0xdb,
	0xb0,0xe0,0x32,0x3a,0x0a,0x49,0x06,0x24,0x5c,0xc2,0xd3,0xac,0x62,0x91,0x95,0xe4,
	0x79,0xe7,0xc8,0x37,0x6d,0x8d,0xd5,0x4e,0xa9,0x6c,0x56,0xf4,0xea,0x65,0x7a,0xae,
	0x08,0xba,0x78,0x25,0x2e,0x1c,0xa6,0xb4,0xc6,0xe8,0xdd,0x74,0x1f,0x4b,0xbd,0x8b,
	0x8a,0x70,0x3e,0xb5,0x66,0x48,0x03,0xf6,0x0e,0x61,0x35,0x57,0xb9,0x86,0xc1,0x1d,
	0x9e,0xe1,0xf8,0x98,0x11,0x69,0xd9,0x8e,0x94,0x9b,0x1e,0x87,0xe9,0xce,0x55,0x28,
	0xdf,0x8c,0xa1,0x89,0x0d,0xbf,0xe6,0x42,0x68,0x41,0x99,0x2d,0x0f,0xb0,0x54,0xbb
};
static const uint8_t rcon[11] = {0x00,0x01,0x02,0x04,0x08,0x10,0x20,0x40,0x80,0x1b,0x36};

static uint8_t xtime(uint8_t x)
{
	return (uint8_t)((x << 1) ^ ((x & 0x80) ? 0x1b : 0));
}

static void aes_encrypt(const uint8_t key[16], const uint8_t in[16], uint8_t out[16])
{
	uint8_t round_key[176];
	uint8_t state[16];
	int i;
	int round;

	memcpy(round_key, key, 16);
	for (i = 4; i < 44; i++) {
		uint8_t temp[4];
		int k = (i - 1) * 4;

		temp[0] = round_key[k];
		temp[1] = round_key[k + 1];
		temp[2] = round_key[k + 2];
		temp[3] = round_key[k + 3];
		if ((i % 4) == 0) {
			uint8_t t = temp[0];
			temp[0] = (uint8_t)(sbox[temp[1]] ^ rcon[i / 4]);
			temp[1] = sbox[temp[2]];
			temp[2] = sbox[temp[3]];
			temp[3] = sbox[t];
		}
		k = (i - 4) * 4;
		round_key[i * 4] = (uint8_t)(round_key[k] ^ temp[0]);
		round_key[i * 4 + 1] = (uint8_t)(round_key[k + 1] ^ temp[1]);
		round_key[i * 4 + 2] = (uint8_t)(round_key[k + 2] ^ temp[2]);
		round_key[i * 4 + 3] = (uint8_t)(round_key[k + 3] ^ temp[3]);
	}
	memcpy(state, in, 16);
	for (i = 0; i < 16; i++)
		state[i] ^= round_key[i];
	for (round = 1; round <= 10; round++) {
		uint8_t tmp[16];
		int row;
		int col;

		for (i = 0; i < 16; i++)
			state[i] = sbox[state[i]];
		memcpy(tmp, state, 16);
		for (row = 0; row < 4; row++) {
			for (col = 0; col < 4; col++)
				state[row + 4 * col] = tmp[row + 4 * ((col + row) & 3)];
		}
		if (round < 10) {
			for (col = 0; col < 4; col++) {
				uint8_t a = state[4 * col];
				uint8_t b = state[4 * col + 1];
				uint8_t c = state[4 * col + 2];
				uint8_t d = state[4 * col + 3];
				uint8_t mix = (uint8_t)(a ^ b ^ c ^ d);

				state[4 * col] = (uint8_t)(a ^ mix ^ xtime((uint8_t)(a ^ b)));
				state[4 * col + 1] = (uint8_t)(b ^ mix ^ xtime((uint8_t)(b ^ c)));
				state[4 * col + 2] = (uint8_t)(c ^ mix ^ xtime((uint8_t)(c ^ d)));
				state[4 * col + 3] = (uint8_t)(d ^ mix ^ xtime((uint8_t)(d ^ a)));
			}
		}
		for (i = 0; i < 16; i++)
			state[i] ^= round_key[round * 16 + i];
	}
	memcpy(out, state, 16);
}

static void smp_c1(const uint8_t r[16], uint8_t *confirm)
{
	uint8_t p1[16];
	uint8_t p2[16];
	uint8_t t[16];
	uint8_t tk[16];
	uint8_t ia[6];
	uint8_t ra[6];
	int i;

	memset(tk, 0, sizeof(tk));
	reverse_bytes(g_pres, p1, 7);
	reverse_bytes(g_preq, p1 + 7, 7);
	p1[14] = g_peer_type;
	p1[15] = 0;
	reverse_bytes(g_local, ia, 6);
	reverse_bytes(g_peer, ra, 6);
	memset(p2, 0, sizeof(p2));
	memcpy(p2 + 4, ia, 6);
	memcpy(p2 + 10, ra, 6);
	for (i = 0; i < 16; i++)
		t[i] = (uint8_t)(r[i] ^ p1[i]);
	aes_encrypt(tk, t, t);
	for (i = 0; i < 16; i++)
		t[i] ^= p2[i];
	aes_encrypt(tk, t, confirm);
}

static void smp_send_confirm(void)
{
	uint8_t body[17];
	uint8_t wire[16];
	int fd;
	ssize_t n;

	fd = open("/dev/urandom", O_RDONLY);
	if (fd >= 0) {
		n = read(fd, g_mrand, sizeof(g_mrand));
		close(fd);
		if (n != (ssize_t)sizeof(g_mrand))
			memset(g_mrand, 0x5a, sizeof(g_mrand));
	} else {
		memset(g_mrand, 0x5a, sizeof(g_mrand));
	}
	smp_c1(g_mrand, g_mconfirm);
	body[0] = 0x03;
	reverse_bytes(g_mconfirm, wire, 16);
	memcpy(body + 1, wire, 16);
	acl_send(CID_SMP, body, sizeof(body));
	g_smp_step = 2;
}

static void smp_start_encrypt(void)
{
	uint8_t rprime[16];
	uint8_t stk[16];
	uint8_t flipped[16];
	uint8_t param[28];
	uint8_t tk[16];

	memset(tk, 0, sizeof(tk));
	memcpy(rprime + 8, g_mrand + 8, 8);
	memcpy(rprime, g_srand + 8, 8);
	aes_encrypt(tk, rprime, stk);
	reverse_bytes(stk, flipped, 16);
	put_le16(param, g_handle);
	memset(param + 2, 0, 10);
	memcpy(param + 12, flipped, 16);
	hci_send(0x2019, param, sizeof(param));
	g_smp_step = 4;
}

static int adv_name(const uint8_t *data, int length, char *out, int outlen)
{
	int i = 0;

	if (outlen <= 0)
		return 0;
	out[0] = '\0';
	while (i + 1 < length) {
		int item = data[i];
		int n;

		if (item == 0 || i + 1 + item > length)
			break;
		n = item - 1;
		if ((data[i + 1] == 0x08 || data[i + 1] == 0x09) && n > 0) {
			if (n >= outlen)
				n = outlen - 1;
			memcpy(out, data + i + 2, (size_t)n);
			out[n] = '\0';
			return 1;
		}
		i += item + 1;
	}
	return 0;
}

static int sw2_mfg(const uint8_t *data, int length)
{
	int i = 0;

	while (i + 1 < length) {
		int item = data[i];
		const uint8_t *payload;
		int n;

		if (item == 0 || i + 1 + item > length)
			break;
		n = item - 1;
		payload = data + i + 2;
		if (data[i + 1] == 0xff && n >= 18 && get_le16(payload) == 0x0553 &&
		    get_le16(payload + 5) == 0x057e) {
			uint16_t pid = get_le16(payload + 7);
			if (pid == 0x2069 || pid == 0x2066 || pid == 0x2067) {
				g_needs_pair = 1;
				if (n >= 18) {
					int z;
					g_needs_pair = 1;
					for (z = 0; z < 6; z++) {
						if (payload[12 + z] != 0)
							g_needs_pair = 0;
					}
				}
				return 1;
			}
		}
		i += item + 1;
	}
	return 0;
}

static void begin_inquiry(void);

static void reset_link(void)
{
	g_handle = 0;
	g_ready = 0;
	g_switch2 = 0;
	g_control_up = 0;
	g_interrupt_up = 0;
	g_control_cfg = 0;
	g_interrupt_cfg = 0;
	g_dcid_control = 0;
	g_dcid_interrupt = 0;
	g_l2cap_open = 0;
	g_l2cap_len = 0;
	g_smp = 0;
	g_smp_step = 0;
	g_sw_step = 0;
	g_acl_inflight = 0;
}

static void on_command_done(uint16_t opcode, uint8_t status)
{
	if (opcode != g_wait_op)
		return;
	g_cmd_done = 1;
	g_cmd_status = status;
	g_wait_op = 0;
	if (g_phase == PH_SETUP && status == 0)
		g_setup_step++;
	if (status == 0 && opcode == 0x0401)
		g_inquiry_on = 1;
	if (opcode == 0x200c && (status == 0 || status == 0x0c)) {
		/* 0x0c means the scan was already in the requested state. */
		g_le_on = g_scan_enable;
		if (g_scan_step == 1) {
			g_phase = PH_SCAN_BR;
			g_scan_step = 2;
			g_due = mono_ms();
		} else if (g_scan_step == 8) {
			g_scan_step = 0;
			g_phase = PH_SCAN_LE;
			g_due = mono_ms() + 8000;
			printf("YetAnotherControllerEnabler: scanning for Switch 2 Pro.\n");
		}
	} else if (opcode == 0x200b && (status == 0 || status == 0x0c)) {
		if (status == 0x0c)
			g_le_on = 1;
		if (g_scan_step == 6) {
			g_scan_step = g_le_on ? 0 : 7;
			g_phase = PH_SCAN_LE;
			g_due = g_le_on ? mono_ms() + 8000 : mono_ms();
			if (g_le_on)
				printf("YetAnotherControllerEnabler: scanning for Switch 2 Pro.\n");
		}
	} else if (opcode == 0x0402 && (status == 0 || status == 0x0c)) {
		g_inquiry_on = 0;
		if (g_scan_step == 4) {
			g_phase = PH_SCAN_LE;
			g_scan_step = 5;
			g_due = mono_ms();
		}
	} else if (status != 0 && status != 0x0c) {
		printf("YetAnotherControllerEnabler: bluetooth command %04x status %02x\n", opcode, status);
	}
}

static int first_sight(const uint8_t addr[6])
{
	static uint8_t seen[16][6];
	static int count;
	int i;

	for (i = 0; i < count; i++) {
		if (memcmp(seen[i], addr, 6) == 0)
			return 0;
	}
	if (count < 16)
		memcpy(seen[count++], addr, 6);
	return 1;
}

static void on_inquiry_result(const uint8_t *addr, const uint8_t cod[3], uint8_t rep, uint16_t clock)
{
	uint8_t param[10];
	int gamepad = class_is_gamepad(cod);

	if (first_sight(addr)) {
		printf("YetAnotherControllerEnabler: heard %02x:%02x:%02x:%02x:%02x:%02x class %02x%02x%02x%s\n",
		       addr[5], addr[4], addr[3], addr[2], addr[1], addr[0],
		       cod[2], cod[1], cod[0], gamepad ? " gamepad" : "");
	}
	if (g_hold || g_phase == PH_CONNECT || g_phase == PH_READY || g_handle != 0)
		return;
	if (!gamepad)
		return;
	memcpy(g_peer, addr, 6);
	g_page_rep = rep;
	g_clock = (uint16_t)(clock | 0x8000);
	g_switch2 = 0;
	g_phase = PH_NAME;
	memcpy(param, addr, 6);
	param[6] = rep;
	param[7] = 0;
	put_le16(param + 8, clock);
	hci_send(0x0419, param, sizeof(param));
	printf("YetAnotherControllerEnabler: asking that gamepad for its name\n");
}

static void connect_classic(void)
{
	uint8_t param[13];

	g_phase = PH_CONNECT;
	g_switch2 = 0;
	g_wait_op = 0;
	memcpy(param, g_peer, 6);
	put_le16(param + 6, 0xcc18);
	param[8] = g_page_rep;
	param[9] = 0;
	put_le16(param + 10, g_clock);
	param[12] = 0x01;
	hci_send(0x0402, NULL, 0);
	g_due = mono_ms() + 200;
	printf("YetAnotherControllerEnabler: Switch Pro connecting\n");
}

static void connect_le(const uint8_t *addr, uint8_t type)
{
	uint8_t param[25];

	if (g_phase == PH_LE_CONNECT || g_phase == PH_SW2 || g_handle != 0)
		return;
	memcpy(g_peer, addr, 6);
	g_peer_type = type;
	g_switch2 = 1;
	g_phase = PH_LE_CONNECT;
	g_wait_op = 0;
	hci_send(0x200c, (const uint8_t *)"\x00\x00", 2);
	memset(param, 0, sizeof(param));
	put_le16(param, 0x0060);
	put_le16(param + 2, 0x0030);
	param[6] = type;
	memcpy(param + 7, addr, 6);
	put_le16(param + 14, 6);
	put_le16(param + 16, 6);
	put_le16(param + 20, 400);
	g_due = mono_ms() + 250;
	printf("YetAnotherControllerEnabler: Switch 2 Pro advertising, connecting\n");
}

static void security_reply(const uint8_t *event, int length)
{
	uint8_t param[32];
	struct link_key *key;

	if (length < 6)
		return;
	if (event[0] == 0x31) {
		memcpy(param, event + 2, 6);
		param[6] = 0x03;
		param[7] = 0x00;
		param[8] = 0x03;
		hci_send(0x042b, param, 9);
	} else if (event[0] == 0x33) {
		memcpy(param, event + 2, 6);
		hci_send(0x042c, param, 6);
	} else if (event[0] == 0x17) {
		key = key_for(event + 2);
		if (key != NULL) {
			memcpy(param, event + 2, 6);
			memcpy(param + 6, key->key, 16);
			hci_send(0x040b, param, 22);
		} else {
			memcpy(param, event + 2, 6);
			hci_send(0x040c, param, 6);
		}
	} else if (event[0] == 0x16) {
		memcpy(param, event + 2, 6);
		hci_send(0x040e, param, 6);
	} else if (event[0] == 0x18 && length >= 23) {
		remember_key(event + 2, event + 8);
	}
}

static void l2cap_signal(const uint8_t *data, int length)
{
	int offset = 0;

	while (offset + 4 <= length) {
		uint8_t code = data[offset];
		uint8_t id = data[offset + 1];
		uint16_t len = get_le16(data + offset + 2);
		const uint8_t *body = data + offset + 4;
		uint8_t reply[16];

		if (offset + 4 + len > length)
			break;
		if (code == 0x02 && len >= 4) {
			uint16_t psm = get_le16(body);
			uint16_t scid = get_le16(body + 2);
			uint16_t dcid = psm == PSM_HID_INTERRUPT ? CID_HID_INTERRUPT : CID_HID_CONTROL;

			if (psm == PSM_HID_INTERRUPT)
				g_dcid_interrupt = scid;
			else
				g_dcid_control = scid;
			reply[0] = 0x03;
			reply[1] = id;
			put_le16(reply + 2, 8);
			put_le16(reply + 4, dcid);
			put_le16(reply + 6, scid);
			put_le16(reply + 8, 0);
			put_le16(reply + 10, 0);
			signal_send(reply, 12);
		} else if (code == 0x03 && len >= 8) {
			uint16_t dcid = get_le16(body);
			uint16_t scid = get_le16(body + 2);
			uint16_t result = get_le16(body + 4);

			if (result == 0) {
				if (scid == CID_HID_CONTROL) {
					g_dcid_control = dcid;
					g_control_up = 1;
					l2cap_config(dcid);
				} else if (scid == CID_HID_INTERRUPT) {
					g_dcid_interrupt = dcid;
					g_interrupt_up = 1;
					l2cap_config(dcid);
				}
			}
		} else if (code == 0x04 && len >= 4) {
			reply[0] = 0x05;
			reply[1] = id;
			put_le16(reply + 2, 6);
			put_le16(reply + 4, get_le16(body));
			put_le16(reply + 6, 0);
			put_le16(reply + 8, 0);
			signal_send(reply, 10);
			if (get_le16(body) == g_dcid_control || get_le16(body) == CID_HID_CONTROL)
				g_control_cfg = 1;
			if (get_le16(body) == g_dcid_interrupt || get_le16(body) == CID_HID_INTERRUPT)
				g_interrupt_cfg = 1;
		} else if (code == 0x05) {
			if (g_control_up)
				g_control_cfg = 1;
			if (g_interrupt_up)
				g_interrupt_cfg = 1;
		}
		offset += 4 + len;
	}
	if (g_phase == PH_L2CAP && g_control_up && !g_interrupt_up && g_control_cfg)
		l2cap_connect(PSM_HID_INTERRUPT, CID_HID_INTERRUPT);
	if (g_control_up && g_interrupt_up && g_control_cfg && g_interrupt_cfg && g_phase == PH_L2CAP) {
		g_phase = PH_SW1;
		g_sw_step = 0;
		g_due = mono_ms();
		printf("YetAnotherControllerEnabler: Switch Pro channels open\n");
	}
}

static void smp_input(const uint8_t *data, int length)
{
	if (length < 1)
		return;
	if (data[0] == 0x02 && length >= 7) {
		memcpy(g_pres, data, 7);
		smp_send_confirm();
	} else if (data[0] == 0x03 && length >= 17) {
		reverse_bytes(data + 1, g_mconfirm, 16);
		{
			uint8_t body[17];
			uint8_t wire[16];

			body[0] = 0x04;
			reverse_bytes(g_mrand, wire, 16);
			memcpy(body + 1, wire, 16);
			acl_send(CID_SMP, body, sizeof(body));
		}
		g_smp_step = 3;
	} else if (data[0] == 0x04 && length >= 17) {
		uint8_t check[16];

		reverse_bytes(data + 1, g_srand, 16);
		smp_c1(g_srand, check);
		if (memcmp(check, g_mconfirm, 16) != 0)
			printf("YetAnotherControllerEnabler: Switch 2 pairing check failed\n");
		smp_start_encrypt();
	} else if (data[0] == 0x05) {
		printf("YetAnotherControllerEnabler: Switch 2 pairing rejected\n");
		g_smp = 0;
	}
}

static void att_input(const uint8_t *data, int length)
{
	uint16_t handle;

	if (length < 1)
		return;
	if (data[0] == 0x01 && length >= 4) {
		if (data[3] == 0x05 && !g_smp) {
			g_smp = 1;
			g_smp_step = 1;
			g_preq[0] = 0x01;
			g_preq[1] = 0x03;
			g_preq[2] = 0x00;
			g_preq[3] = 0x00;
			g_preq[4] = 16;
			g_preq[5] = 0x01;
			g_preq[6] = 0x01;
			acl_send(CID_SMP, g_preq, 7);
			printf("YetAnotherControllerEnabler: Switch 2 pairing\n");
		} else if (g_phase == PH_SW2) {
			g_due = mono_ms();
		}
		return;
	}
	if (data[0] == 0x13 && g_phase == PH_SW2) {
		g_due = mono_ms();
		return;
	}
	if (data[0] == 0x1b && length >= 3) {
		handle = get_le16(data + 1);
		if (handle == 0x001a && length >= 5 && data[4] == 0x01) {
			g_due = mono_ms();
		} else if (handle == 0x000a || handle == 0x000e) {
			queue_report(data + 3, length - 3);
			g_ready = 1;
			g_phase = PH_READY;
		}
	}
}

static void acl_payload(uint16_t cid, const uint8_t *data, int length)
{
	if (cid == CID_SIGNAL)
		l2cap_signal(data, length);
	else if (cid == CID_ATT)
		att_input(data, length);
	else if (cid == CID_SMP)
		smp_input(data, length);
	else if (cid == g_dcid_interrupt || cid == CID_HID_INTERRUPT) {
		if (length >= 2 && data[0] == 0xa1)
			queue_report(data + 1, length - 1);
		else if (length >= 1)
			queue_report(data, length);
	}
}

static void acl_feed(const uint8_t *packet, int length)
{
	uint16_t handle_flags;
	uint16_t dlen;
	uint16_t cid;
	uint16_t plen;
	int start;

	if (length < 4)
		return;
	handle_flags = get_le16(packet);
	dlen = get_le16(packet + 2);
	if ((handle_flags & 0x0fff) != g_handle || 4 + dlen > length)
		return;
	start = (handle_flags & 0x3000) != 0;
	if (start) {
		if (dlen < 4)
			return;
		plen = get_le16(packet + 4);
		cid = get_le16(packet + 6);
		if (4 + plen <= dlen) {
			acl_payload(cid, packet + 8, plen);
			g_l2cap_open = 0;
			return;
		}
		g_l2cap_expect = plen;
		g_l2cap_len = dlen - 4;
		if (g_l2cap_len > (int)sizeof(g_l2cap))
			g_l2cap_len = (int)sizeof(g_l2cap);
		memcpy(g_l2cap, packet + 4, (size_t)g_l2cap_len);
		g_l2cap_open = 1;
	} else if (g_l2cap_open && g_l2cap_len + dlen <= (int)sizeof(g_l2cap)) {
		memcpy(g_l2cap + g_l2cap_len, packet + 4, dlen);
		g_l2cap_len += dlen;
		if (g_l2cap_len >= 4 + g_l2cap_expect) {
			acl_payload(get_le16(g_l2cap + 2), g_l2cap + 4, g_l2cap_expect);
			g_l2cap_open = 0;
		}
	}
}

static void hci_event(const uint8_t *event, int length)
{
	uint8_t code;
	uint8_t n;
	const uint8_t *p;
	int i;

	if (length < 2)
		return;
	code = event[0];
	n = event[1];
	p = event + 2;
	if (2 + n > length)
		n = (uint8_t)(length - 2);
	if (code == 0x0e && n >= 3) {
		on_command_done(get_le16(p + 1), n >= 4 ? p[3] : 0);
		if (get_le16(p + 1) == 0x1005 && n >= 8 && p[3] == 0) {
			uint16_t packets = get_le16(p + 6);
			if (packets > 0 && packets < 32)
				g_acl_credits = (uint8_t)packets;
		}
		if (get_le16(p + 1) == 0x1009 && n >= 10 && p[3] == 0) {
			memcpy(g_local, p + 4, 6);
			g_have_local = 1;
			printf("YetAnotherControllerEnabler: bluetooth address %02x:%02x:%02x:%02x:%02x:%02x\n",
			       g_local[5], g_local[4], g_local[3], g_local[2], g_local[1], g_local[0]);
		}
	} else if (code == 0x0f && n >= 4) {
		on_command_done(get_le16(p + 2), p[0]);
	} else if (code == 0x02 && n >= 1) {
		int count = p[0];
		const uint8_t *item = p + 1;
		for (i = 0; i < count && item + 14 <= p + n; i++) {
			on_inquiry_result(item, item + 9, item[6], get_le16(item + 12));
			item += 14;
		}
	} else if (code == 0x22 && n >= 1) {
		int count = p[0];
		const uint8_t *item = p + 1;
		for (i = 0; i < count && item + 14 <= p + n; i++) {
			on_inquiry_result(item, item + 8, item[6], get_le16(item + 11));
			item += 14;
		}
	} else if (code == 0x2f && n >= 15) {
		const uint8_t *eir = p + 15;
		int eir_len = n - 15;
		int at = 0;
		char name[64];
		int namelen = 0;

		on_inquiry_result(p + 1, p + 9, p[7], get_le16(p + 12));
		name[0] = '\0';
		while (at + 1 < eir_len) {
			int elen = eir[at];
			if (elen == 0 || at + 1 + elen > eir_len)
				break;
			if ((eir[at + 1] == 0x08 || eir[at + 1] == 0x09) && elen > 1) {
				namelen = elen - 1;
				if (namelen > 63)
					namelen = 63;
				memcpy(name, eir + at + 2, (size_t)namelen);
				name[namelen] = '\0';
			}
			at += elen + 1;
		}
		if (name[0] != '\0')
			printf("YetAnotherControllerEnabler: bluetooth name '%s'\n", name);
		if (name_is_switch(name) && g_handle == 0 && g_phase != PH_CONNECT &&
		    g_phase != PH_READY) {
			if (g_phase != PH_NAME) {
				memcpy(g_peer, p + 1, 6);
				g_page_rep = p[7];
				g_clock = (uint16_t)(get_le16(p + 12) | 0x8000);
			}
			connect_classic();
		}
	} else if (code == 0x07 && n >= 7) {
		char name[64];
		int namelen = n - 7;

		if (namelen > 63)
			namelen = 63;
		memcpy(name, p + 7, (size_t)namelen);
		name[namelen] = '\0';
		printf("YetAnotherControllerEnabler: bluetooth name '%s'\n", name);
		if (p[0] == 0 && name_is_switch(name))
			connect_classic();
		else if (g_phase == PH_NAME) {
			printf("YetAnotherControllerEnabler: that name is not a Switch controller\n");
			g_phase = PH_SCAN_BR;
		}
	} else if (code == 0x03 && n >= 11) {
		if (p[0] == 0) {
			g_handle = get_le16(p + 1);
			g_phase = (n >= 11 && p[10]) ? PH_L2CAP : PH_SECURITY;
			if (g_phase == PH_L2CAP) {
				l2cap_connect(PSM_HID_CONTROL, CID_HID_CONTROL);
			} else {
				uint8_t param[2];
				put_le16(param, g_handle);
				hci_send(0x0411, param, 2);
			}
			printf("YetAnotherControllerEnabler: bluetooth link %04x\n", g_handle);
		} else {
			printf("YetAnotherControllerEnabler: bluetooth connect status %02x\n", p[0]);
			g_phase = PH_SCAN_BR;
		}
	} else if (code == 0x04 && n >= 10) {
		uint8_t param[7];
		if (!g_hold && class_is_gamepad(p + 6)) {
			memcpy(g_peer, p, 6);
			memcpy(param, p, 6);
			param[6] = 0x00;
			hci_send(0x0409, param, 7);
			g_phase = PH_CONNECT;
			printf("YetAnotherControllerEnabler: gamepad requested a connection\n");
		}
	} else if (code == 0x01 && n >= 1) {
		/* Inquiry finished on its own. Linux waits for this instead of cancelling. */
		g_inquiry_on = 0;
		if (g_phase == PH_SCAN_BR) {
			g_phase = PH_SCAN_LE;
			g_scan_step = 5;
			g_due = mono_ms();
		}
	} else if (code == 0x05 && n >= 4) {
		uint16_t handle = get_le16(p + 1);
		if (g_handle != 0 && handle == g_handle) {
			if (g_ready)
				g_lost = 1;
			printf("YetAnotherControllerEnabler: bluetooth disconnected reason %02x\n", p[3]);
			reset_link();
			g_phase = PH_SCAN_LE;
			g_scan_step = 0;
			g_le_on = 1;
			g_due = mono_ms() + 1000;
		}
	} else if (code == 0x08 && n >= 4) {
		if (p[0] == 0 && p[3] != 0 && g_phase == PH_SECURITY) {
			g_phase = g_switch2 ? PH_SW2 : PH_L2CAP;
			g_sw_step = 0;
			g_due = mono_ms() + (g_switch2 ? 500 : 0);
			if (!g_switch2)
				l2cap_connect(PSM_HID_CONTROL, CID_HID_CONTROL);
			printf("YetAnotherControllerEnabler: bluetooth encrypted\n");
		}
	} else if (code == 0x13 && n >= 1) {
		int handles = p[0];
		const uint8_t *item = p + 1;
		for (i = 0; i < handles && item + 4 <= p + n; i++) {
			uint16_t done = get_le16(item + 2);
			g_acl_credits = (uint8_t)(g_acl_credits + done);
			if (g_acl_inflight >= done)
				g_acl_inflight = (uint8_t)(g_acl_inflight - done);
			else
				g_acl_inflight = 0;
			item += 4;
		}
	} else if (code == 0x17 || code == 0x18 || code == 0x31 || code == 0x33 || code == 0x16) {
		security_reply(event, length);
	} else if (code == 0x3e && n >= 1 && p[0] == 0x02) {
		int count = p[1];
		const uint8_t *item = p + 2;
		for (i = 0; i < count && item + 10 < p + n; i++) {
			uint8_t dlen = item[8];
			char name[40];
			if (item + 10 + dlen > p + n)
				break;
			if (first_sight(item + 2)) {
				if (adv_name(item + 9, dlen, name, sizeof(name)))
					printf("YetAnotherControllerEnabler: heard LE %02x:%02x:%02x:%02x:%02x:%02x '%s'\n",
					       item[7], item[6], item[5], item[4], item[3], item[2], name);
				else
					printf("YetAnotherControllerEnabler: heard LE %02x:%02x:%02x:%02x:%02x:%02x\n",
					       item[7], item[6], item[5], item[4], item[3], item[2]);
			}
			if (!g_hold && (sw2_mfg(item + 9, dlen) ||
			    (adv_name(item + 9, dlen, name, sizeof(name)) && name_is_switch(name))))
				connect_le(item + 2, item[1]);
			item += 10 + dlen;
		}
	} else if (code == 0x3e && n >= 1 && p[0] == 0x0d) {
		int count = p[1];
		const uint8_t *item = p + 2;

		for (i = 0; i < count && item + 24 <= p + n; i++) {
			uint8_t dlen = item[23];
			char name[40];
			if (item + 24 + dlen > p + n)
				break;
			if (first_sight(item + 3)) {
				if (adv_name(item + 24, dlen, name, sizeof(name)))
					printf("YetAnotherControllerEnabler: heard LE %02x:%02x:%02x:%02x:%02x:%02x '%s'\n",
					       item[8], item[7], item[6], item[5], item[4], item[3], name);
				else
					printf("YetAnotherControllerEnabler: heard LE %02x:%02x:%02x:%02x:%02x:%02x\n",
					       item[8], item[7], item[6], item[5], item[4], item[3]);
			}
			if (!g_hold && (sw2_mfg(item + 24, dlen) ||
			    (adv_name(item + 24, dlen, name, sizeof(name)) && name_is_switch(name))))
				connect_le(item + 3, item[2]);
			item += 24 + dlen;
		}
	} else if (code == 0x3e && n >= 12 && p[0] == 0x01) {
		if (p[1] == 0) {
			g_handle = get_le16(p + 2);
			g_switch2 = 1;
			g_phase = PH_SW2;
			g_sw_step = 0;
			g_due = mono_ms() + 500;
			printf("YetAnotherControllerEnabler: Switch 2 Pro link %04x\n", g_handle);
		} else {
			printf("YetAnotherControllerEnabler: Switch 2 connect status %02x\n", p[1]);
			g_phase = PH_SCAN_LE;
		}
	}
}

static int g_evt_ep = EP_EVT;
static int g_acl_ep = EP_ACL_IN;
static int g_evt_locked;
static uint8_t g_ep_no[EP_COUNT] = {0x81, 0x82, 0x01};

static int looks_like_event(const uint8_t *data, int n)
{
	uint8_t code;

	if (n < 2)
		return 0;
	code = data[0] == 0x04 && n >= 3 ? data[1] : data[0];
	return code == 0x0e || code == 0x0f || code == 0x02 || code == 0x03 ||
	    code == 0x04 || code == 0x05 || code == 0x07 || code == 0x08 ||
	    code == 0x13 || code == 0x22 || code == 0x3e;
}

static uint8_t g_asm[260];
static int g_asm_len;

/* The event pipe is 16 bytes. Inquiry and advert reports are longer than that. */
static void feed_event(const uint8_t *data, int n)
{
	int need;

	if (n <= 0)
		return;
	/* A short packet is its own event. Do not glue it onto a fragment that never finished. */
	if (g_asm_len >= 2 && n < 16 && g_asm_len + n < 2 + g_asm[1])
		g_asm_len = 0;
	if (g_asm_len == 0 && data[0] == 0x04) {
		data++;
		n--;
		if (n <= 0)
			return;
	}
	if (g_asm_len + n > (int)sizeof(g_asm))
		g_asm_len = 0;
	memcpy(g_asm + g_asm_len, data, (size_t)n);
	g_asm_len += n;
	while (g_asm_len >= 2) {
		need = 2 + g_asm[1];
		if (need < 2 || need > (int)sizeof(g_asm)) {
			g_asm_len = 0;
			return;
		}
		if (g_asm_len < need)
			return;
		hci_event(g_asm, need);
		if (g_asm_len == need) {
			g_asm_len = 0;
			return;
		}
		memmove(g_asm, g_asm + need, (size_t)(g_asm_len - need));
		g_asm_len -= need;
	}
}

static void pump_usb(void)
{
	int n;
	int guard;

	for (guard = 0; guard < 24; guard++) {
		n = fs_xfer((uint8_t)g_evt_ep, g_evt,
			    g_room[g_evt_ep] ? g_room[g_evt_ep] : 16,
			    g_wait_op != 0 ? 40 : 8);
		if (n < 0) {
			static int told_read;

			if (told_read < 3) {
				printf("YetAnotherControllerEnabler: bluetooth event read errno %d\n", errno);
				told_read++;
			}
			break;
		}
		if (n >= 2) {
			g_evt_locked = 1;
			feed_event(g_evt, n);
			continue;
		}
		break;
	}
	if (!g_evt_locked) {
		int other = g_evt_ep == EP_EVT ? EP_ACL_IN : EP_EVT;

		n = fs_xfer((uint8_t)other, g_acl,
			    g_room[other] > sizeof(g_acl) ? (uint32_t)sizeof(g_acl) : g_room[other],
			    8);
		if (n >= 2 && looks_like_event(g_acl, n)) {
			g_evt_ep = other;
			g_acl_ep = other == EP_EVT ? EP_ACL_IN : EP_EVT;
			g_evt_locked = 1;
			printf("YetAnotherControllerEnabler: bluetooth events on 0x%02x\n", g_ep_no[g_evt_ep]);
			feed_event(g_acl, n);
		}
	}
	if (g_handle != 0) {
		n = fs_xfer((uint8_t)g_acl_ep, g_acl,
			    g_room[g_acl_ep] ? g_room[g_acl_ep] : 64, 8);
		if (n > 4)
			acl_feed(g_acl, n);
	}
}

static void begin_inquiry(void)
{
	uint8_t param[5] = {0x33, 0x8b, 0x9e, 0x06, 0x00};

	if (g_wait_op != 0)
		return;
	g_phase = PH_SCAN_BR;
	g_due = mono_ms() + 8000;
	hci_send(0x0401, param, sizeof(param));
	printf("YetAnotherControllerEnabler: scanning for Switch Pro. Hold the sync button until the lights blink.\n");
}

static void le_scan_params(void)
{
	uint8_t param[7];

	memset(param, 0, sizeof(param));
	param[0] = 0x01;
	put_le16(param + 1, 0x0060);
	put_le16(param + 3, 0x0030);
	hci_send(0x200b, param, sizeof(param));
}

static void drive(void)
{
	static const uint8_t mask[8] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x3f};
	static const uint8_t led = 0x01;
	static const uint8_t mode = 0x30;
	static const uint8_t init_p03[] = {0x01, 0x00, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff};
	static const uint8_t init_p15[] = {0x00};
	static const uint8_t init_p0c[] = {0x2f, 0x00, 0x00, 0x00};
	static const uint8_t init_p0a[] = {
		0x01, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
		0x35, 0x00, 0x46, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
	};
	static const uint8_t init_p03b[] = {0x09, 0x00, 0x00, 0x00};
	static const uint8_t init_p01[] = {0x00, 0x00, 0x00, 0x00};
	static const uint8_t init_led[] = {0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
	static const uint8_t notify_on[2] = {0x01, 0x00};

	if (g_fd < 0 || g_wait_op != 0)
		return;
	if (g_hold && g_phase != PH_CLOSED && g_phase != PH_SETUP && g_handle != 0) {
		uint8_t param[3];
		put_le16(param, g_handle);
		param[2] = 0x13;
		hci_send(0x0406, param, 3);
		reset_link();
		g_phase = PH_SCAN_BR;
		g_due = mono_ms() + 1000;
		return;
	}
	if (g_phase == PH_SETUP) {
		if (g_setup_step == 0)
			hci_send(0x1005, NULL, 0);
		else if (g_setup_step == 1)
			hci_send(0x1009, NULL, 0);
		else if (g_setup_step == 2)
			hci_send(0x0c56, (const uint8_t *)"\x01", 1);
		else if (g_setup_step == 3)
			hci_send(0x0c45, (const uint8_t *)"\x02", 1);
		else if (g_setup_step == 4)
			hci_send(0x0c01, mask, sizeof(mask));
		else if (g_setup_step == 5)
			hci_send(0x2001, mask, 8);
		else if (g_setup_step == 6)
			hci_send(0x0c1a, (const uint8_t *)"\x02", 1);
		else {
			/* Linux leaves an active scan alone until inquiry needs the radio. */
			g_phase = PH_SCAN_LE;
			g_scan_step = 0;
			g_le_on = 1;
			g_due = mono_ms() + 8000;
			printf("YetAnotherControllerEnabler: bluetooth radio ready\n");
			printf("YetAnotherControllerEnabler: scanning for Switch 2 Pro.\n");
		}
		return;
	}
	if (g_hold)
		return;
	/*
	 * Same order as Linux interleaved discovery:
	 * listen, stop LE scan, inquiry, then scan parameters and scan enable.
	 * A command is sent only after the previous one finishes.
	 */
	if (g_phase == PH_SCAN_LE && g_scan_step == 0 && mono_ms() >= g_due) {
		g_scan_enable = 0;
		hci_send(0x200c, (const uint8_t *)"\x00\x00", 2);
		g_scan_step = 1;
		g_due = mono_ms() + 100;
		return;
	}
	if (g_scan_step == 1 && g_wait_op == 0 && mono_ms() >= g_due) {
		g_le_on = 0;
		g_phase = PH_SCAN_BR;
		g_scan_step = 2;
		g_due = mono_ms();
	}
	if (g_phase == PH_SCAN_BR && g_scan_step == 2 && mono_ms() >= g_due) {
		begin_inquiry();
		g_scan_step = 3;
		return;
	}
	if (g_phase == PH_SCAN_BR && g_scan_step == 3 && mono_ms() >= g_due) {
		if (g_inquiry_on) {
			hci_send(0x0402, NULL, 0);
			g_scan_step = 4;
			g_due = mono_ms() + 100;
		} else {
			g_phase = PH_SCAN_LE;
			g_scan_step = 5;
			g_due = mono_ms();
		}
		return;
	}
	if (g_scan_step == 4 && g_wait_op == 0 && mono_ms() >= g_due) {
		g_inquiry_on = 0;
		g_phase = PH_SCAN_LE;
		g_scan_step = 5;
		g_due = mono_ms();
	}
	if (g_phase == PH_SCAN_LE && g_scan_step == 5 && mono_ms() >= g_due) {
		if (g_le_on) {
			g_scan_step = 0;
			g_due = mono_ms() + 8000;
			printf("YetAnotherControllerEnabler: scanning for Switch 2 Pro.\n");
			return;
		}
		le_scan_params();
		g_scan_step = 6;
		g_due = mono_ms() + 100;
		return;
	}
	if (g_scan_step == 6 && g_wait_op == 0 && mono_ms() >= g_due) {
		g_scan_step = 7;
		g_due = mono_ms();
	}
	if (g_phase == PH_SCAN_LE && g_scan_step == 7 && mono_ms() >= g_due) {
		g_scan_enable = 1;
		hci_send(0x200c, (const uint8_t *)"\x01\x00", 2);
		g_scan_step = 8;
		g_due = mono_ms() + 100;
		return;
	}
	if (g_scan_step == 8 && g_wait_op == 0 && mono_ms() >= g_due) {
		g_le_on = 1;
		g_phase = PH_SCAN_LE;
		g_scan_step = 0;
		g_due = mono_ms() + 8000;
		printf("YetAnotherControllerEnabler: scanning for Switch 2 Pro.\n");
		return;
	}
	if (g_phase == PH_CONNECT && g_wait_op == 0 && mono_ms() >= g_due) {
		uint8_t param[13];
		memcpy(param, g_peer, 6);
		put_le16(param + 6, 0xcc18);
		param[8] = g_page_rep;
		param[9] = 0;
		put_le16(param + 10, g_clock);
		param[12] = 0x01;
		hci_send(0x0405, param, sizeof(param));
		g_due = mono_ms() + 20000;
	}
	if (g_phase == PH_LE_CONNECT && g_wait_op == 0 && mono_ms() >= g_due) {
		uint8_t param[25];
		memset(param, 0, sizeof(param));
		put_le16(param, 0x0060);
		put_le16(param + 2, 0x0030);
		param[6] = g_peer_type;
		memcpy(param + 7, g_peer, 6);
		put_le16(param + 14, 6);
		put_le16(param + 16, 6);
		put_le16(param + 20, 400);
		hci_send(0x200d, param, sizeof(param));
		g_due = mono_ms() + 20000;
	}
	if (g_phase == PH_SW1 && mono_ms() >= g_due) {
		if (g_sw_step == 0)
			sw1_subcmd(0x02, NULL, 0);
		else if (g_sw_step == 1)
			sw1_subcmd(0x03, &mode, 1);
		else if (g_sw_step == 2)
			sw1_subcmd(0x30, &led, 1);
		else {
			g_phase = PH_READY;
			g_ready = 1;
			g_switch2 = 0;
			printf("YetAnotherControllerEnabler: Switch Pro wireless ready\n");
			return;
		}
		g_sw_step++;
		g_due = mono_ms() + 80;
	}
	if (g_phase == PH_SW2 && mono_ms() >= g_due) {
		uint8_t pair[14];
		if (g_sw_step == 0) {
			att_write(0x0004, notify_on, 2, 0);
		} else if (g_sw_step == 1) {
			att_write(0x001b, notify_on, 2, 0);
		} else if (g_sw_step == 2) {
			sw2_cmd(0x03, 0x0d, init_p03, sizeof(init_p03));
		} else if (g_sw_step == 3) {
			sw2_cmd(0x07, 0x01, NULL, 0);
		} else if (g_sw_step == 4) {
			sw2_cmd(0x16, 0x01, NULL, 0);
		} else if (g_sw_step == 5) {
			sw2_cmd(0x15, 0x03, init_p15, sizeof(init_p15));
		} else if (g_sw_step == 6) {
			sw2_cmd(0x0c, 0x02, init_p0c, sizeof(init_p0c));
		} else if (g_sw_step == 7) {
			sw2_cmd(0x11, 0x03, NULL, 0);
		} else if (g_sw_step == 8) {
			sw2_cmd(0x0a, 0x08, init_p0a, sizeof(init_p0a));
		} else if (g_sw_step == 9) {
			sw2_cmd(0x0c, 0x04, init_p0c, sizeof(init_p0c));
		} else if (g_sw_step == 10) {
			sw2_cmd(0x03, 0x0a, init_p03b, sizeof(init_p03b));
		} else if (g_sw_step == 11) {
			sw2_cmd(0x10, 0x01, NULL, 0);
		} else if (g_sw_step == 12) {
			sw2_cmd(0x01, 0x0c, NULL, 0);
		} else if (g_sw_step == 13) {
			sw2_cmd(0x01, 0x01, init_p01, sizeof(init_p01));
		} else if (g_sw_step == 14) {
			sw2_cmd(0x09, 0x07, init_led, sizeof(init_led));
		} else if (g_sw_step == 15 && g_needs_pair && g_have_local) {
			pair[0] = 0x00;
			pair[1] = 0x02;
			memcpy(pair + 2, g_local, 6);
			memcpy(pair + 8, g_local, 6);
			sw2_cmd(0x15, 0x01, pair, sizeof(pair));
		} else if (g_sw_step == 16 && g_needs_pair) {
			sw2_cmd(0x15, 0x04, g_ltk1, sizeof(g_ltk1));
		} else if (g_sw_step == 17 && g_needs_pair) {
			sw2_cmd(0x15, 0x02, g_ltk2, sizeof(g_ltk2));
		} else if (g_sw_step == 18 && g_needs_pair) {
			sw2_cmd(0x15, 0x03, (const uint8_t *)"\x00", 1);
		} else {
			att_write(0x000b, notify_on, 2, 0);
			g_phase = PH_READY;
			g_ready = 1;
			g_switch2 = 1;
			g_keep_ms = 0;
			printf("YetAnotherControllerEnabler: Switch 2 Pro wireless ready\n");
			return;
		}
		g_sw_step++;
		g_due = mono_ms() + (g_sw_step < 3 ? 200 : 40);
	}
	if (g_phase == PH_READY && g_switch2)
		sw2_keepalive();
	if (g_phase == PH_READY && !g_switch2 && g_report_len > 0) {
		static int reports;
		reports++;
		if (reports >= 8) {
			reports = 0;
			sw1_rumble();
		}
	}
}

static int endpoint_present(const uint8_t *raw, int length, uint8_t address)
{
	int offset = 0;

	while (offset + 2 <= length) {
		int entry = raw[offset];
		if (entry < 2 || offset + entry > length)
			break;
		if (raw[offset + 1] == 0x05 && entry >= 3 && raw[offset + 2] == address)
			return 1;
		offset += entry;
	}
	return 0;
}

static int radio_endpoints(const uint8_t *raw, int length)
{
	return endpoint_present(raw, length, 0x82) &&
	    endpoint_present(raw, length, 0x81) &&
	    endpoint_present(raw, length, 0x01);
}

/* The HCI pipes live on an alternate setting. Alt 0 is idle and never returns events. */
static void select_radio_alt(int fd, const uint8_t *raw, int length)
{
	struct usb_alt_interface alt;
	int offset = 0;
	int iface_index = -1;
	int last_number = -1;
	int current_alt = 0;
	int event_iface = -1;
	int event_alt = 0;
	int dual_iface = -1;

	while (offset + 2 <= length) {
		int entry = raw[offset];

		if (entry < 2 || offset + entry > length)
			break;
		if (raw[offset + 1] == 0x04 && entry >= 4) {
			int number = raw[offset + 2];

			if (number != last_number) {
				iface_index++;
				last_number = number;
			}
			current_alt = raw[offset + 3];
		} else if (raw[offset + 1] == 0x05 && entry >= 3 && iface_index >= 0) {
			if (raw[offset + 2] == 0x82) {
				event_iface = iface_index;
				event_alt = current_alt;
			} else if (raw[offset + 2] == 0x85) {
				dual_iface = iface_index;
			}
		}
		offset += entry;
	}
	if (event_iface < 0 || event_iface == dual_iface)
		return;
	memset(&alt, 0, sizeof(alt));
	alt.uai_interface_index = (uint8_t)event_iface;
	if (ioctl(fd, USB_GET_ALTINTERFACE, &alt) != 0 || alt.uai_alt_index != event_alt) {
		alt.uai_interface_index = (uint8_t)event_iface;
		alt.uai_alt_index = (uint8_t)event_alt;
		if (ioctl(fd, USB_SET_ALTINTERFACE, &alt) != 0)
			printf("YetAnotherControllerEnabler: bluetooth alt %d errno %d\n", event_alt, errno);
	}
	/*
	 * The system driver keeps a read queued on this interface and takes every
	 * event. Release that interface only. The DualSense interface stays attached.
	 */
	if (ioctl(fd, USB_IFACE_DRIVER_DETACH, &event_iface) != 0)
		printf("YetAnotherControllerEnabler: bluetooth interface %d release errno %d\n", event_iface, errno);
	else
		printf("YetAnotherControllerEnabler: bluetooth interface %d released\n", event_iface);
}

static int open_radio_fd(int fd, int known_node)
{
	struct usb_fs_init init;
	struct usb_fs_uninit uninit;
	struct usb_fs_open open_request;
	uint8_t raw[4096];
	struct usb_gen_descriptor descriptor;
	int desc_length = 0;
	/* 0x81 is the 16-byte interrupt pipe. 0x82 is ACL in and stays empty until a link exists. */
	uint8_t addresses[3] = {0x81, 0x82, 0x01};
	uint32_t sizes[3] = {64, 512, 512};
	int i;

	memset(&descriptor, 0, sizeof(descriptor));
	memset(raw, 0, sizeof(raw));
	descriptor.ugd_data = raw;
	descriptor.ugd_maxlen = sizeof(raw);
	if (ioctl(fd, USB_GET_FULL_DESC, &descriptor) != 0) {
		if (known_node)
			printf("YetAnotherControllerEnabler: bluetooth descriptor errno %d\n", errno);
		if (!known_node)
			return -1;
	} else {
		desc_length = descriptor.ugd_actlen > 0 ? descriptor.ugd_actlen : (int)sizeof(raw);
		if (desc_length > (int)sizeof(raw))
			desc_length = (int)sizeof(raw);
		/* Function 1 is DualSense. Its event pipe is 0x85, and it stays closed. */
		if (endpoint_present(raw, desc_length, 0x85) &&
		    !endpoint_present(raw, desc_length, 0x82))
			return -1;
		if (!radio_endpoints(raw, desc_length) && !known_node)
			return -1;
		select_radio_alt(fd, raw, desc_length);
	}
	memset(g_ep, 0, sizeof(g_ep));
	memset(&init, 0, sizeof(init));
	init.pEndpoints = g_ep;
	init.ep_index_max = EP_COUNT;
	if (ioctl(fd, USB_FS_INIT, &init) != 0) {
		if (known_node)
			printf("YetAnotherControllerEnabler: bluetooth fs init errno %d\n", errno);
		return -1;
	}
	for (i = 0; i < 3; i++) {
		memset(&open_request, 0, sizeof(open_request));
		open_request.max_bufsize = sizes[i];
		open_request.max_frames = 1;
		open_request.ep_index = (uint8_t)i;
		open_request.ep_no = addresses[i];
		if (ioctl(fd, USB_FS_OPEN, &open_request) != 0) {
			if (known_node)
				printf("YetAnotherControllerEnabler: bluetooth ep 0x%02x errno %d\n", addresses[i], errno);
			memset(&uninit, 0, sizeof(uninit));
			ioctl(fd, USB_FS_UNINIT, &uninit);
			return -1;
		}
		g_room[i] = open_request.max_packet_length;
		if (open_request.max_bufsize > 0 &&
		    (g_room[i] == 0 || open_request.max_bufsize < g_room[i]))
			g_room[i] = (uint16_t)open_request.max_bufsize;
		if (g_room[i] == 0 || g_room[i] > sizes[i])
			g_room[i] = (uint16_t)sizes[i];
		if (i == EP_EVT && g_room[i] > sizeof(g_evt))
			g_room[i] = (uint16_t)sizeof(g_evt);
		g_ep_no[i] = addresses[i];
		if (known_node)
			printf("YetAnotherControllerEnabler: bluetooth ep 0x%02x packet %u\n",
			       addresses[i], open_request.max_packet_length);
	}
	g_fd = fd;
	return 0;
}

static int open_radio(void)
{
	const char *first = "/dev/ugen0.2";
	int fd;
	DIR *dir;
	struct dirent *entry;

	fd = open(first, O_RDWR);
	if (fd < 0)
		printf("YetAnotherControllerEnabler: bluetooth %s open errno %d\n", first, errno);
	else if (open_radio_fd(fd, 1) == 0)
		return 0;
	else
		close(fd);
	dir = opendir("/dev");
	if (dir == NULL)
		return -1;
	while ((entry = readdir(dir)) != NULL) {
		char path[64];
		int dots = 0;
		size_t i;

		if (strncmp(entry->d_name, "ugen", 4) != 0)
			continue;
		for (i = 0; entry->d_name[i] != '\0'; i++) {
			if (entry->d_name[i] == '.')
				dots++;
		}
		if (dots != 1)
			continue;
		snprintf(path, sizeof(path), "/dev/%s", entry->d_name);
		if (strcmp(path, first) == 0)
			continue;
		fd = open(path, O_RDWR);
		if (fd < 0)
			continue;
		if (open_radio_fd(fd, 0) == 0) {
			closedir(dir);
			printf("YetAnotherControllerEnabler: bluetooth radio %s\n", path);
			return 0;
		}
		close(fd);
	}
	closedir(dir);
	return -1;
}

int bt_switch_start(void)
{
	if (g_fd >= 0)
		return 0;
	if (open_radio() != 0) {
		printf("YetAnotherControllerEnabler: bluetooth radio not opened\n");
		return -1;
	}
	g_phase = PH_SETUP;
	g_setup_step = 0;
	g_due = 0;
	printf("YetAnotherControllerEnabler: bluetooth radio opened\n");
	return 0;
}

void bt_switch_set_hold(int hold)
{
	g_hold = hold;
}

int bt_switch_is_ready(void)
{
	return g_ready;
}

int bt_switch_is_switch2(void)
{
	return g_switch2;
}

const char *bt_switch_name(void)
{
	return g_switch2 ? "Switch 2 Pro" : "Switch Pro";
}

int bt_switch_fast_poll(void)
{
	return g_ready && g_switch2;
}

void bt_switch_drop(void)
{
	if (g_handle != 0 && g_wait_op == 0) {
		uint8_t param[3];
		put_le16(param, g_handle);
		param[2] = 0x13;
		hci_send(0x0406, param, 3);
	}
	reset_link();
	g_lost = 0;
	g_phase = g_fd >= 0 ? PH_SCAN_BR : PH_CLOSED;
	g_due = 0;
}

int bt_switch_read(uint8_t *report, size_t length)
{
	int n;

	if (g_fd < 0 || report == NULL || length == 0)
		return 0;
	pump_usb();
	if (g_wait_op != 0) {
		static uint16_t wait_op;
		static uint64_t wait_since;

		if (g_wait_op != wait_op) {
			wait_op = g_wait_op;
			wait_since = mono_ms();
		} else if (mono_ms() - wait_since > 4000) {
			printf("YetAnotherControllerEnabler: bluetooth command %04x had no reply\n", g_wait_op);
			if (g_phase == PH_SETUP)
				g_setup_step++;
			g_wait_op = 0;
			wait_op = 0;
		}
	}
	if (g_cmd_done)
		g_cmd_done = 0;
	drive();
	pump_usb();
	if (g_lost) {
		g_lost = 0;
		return -1;
	}
	if (!g_ready || g_report_len <= 0)
		return 0;
	n = g_report_len;
	if ((size_t)n > length)
		n = (int)length;
	memcpy(report, g_report, (size_t)n);
	g_report_len = 0;
	return n;
}
