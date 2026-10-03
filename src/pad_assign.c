#include "pad_assign.h"
#include "pad_data.h"

#include <errno.h>
#include <fcntl.h>
#include <signal.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include <machine/reg.h>
#include <ps5/kernel.h>
#include <ps5/nid.h>
#include <sys/mman.h>
#include <sys/ptrace.h>
#include <sys/syscall.h>
#include <sys/sysctl.h>
#include <sys/wait.h>

int sceSysmoduleLoadModuleInternal(uint32_t module_id);
int sceUserServiceInitialize(void *parameters);
int sceUserServiceGetForegroundUser(int *user_id);

static char g_status[120] = "No virtual pad yet";
static int g_shell_done;
static int g_code = -1;
static int g_bind = -1;
static int g_klog_handle = -1;
static uint64_t g_device;
static uint64_t g_devices[8];
static int g_device_count;

static void elevate(void);

static void remember_device(uint64_t id)
{
	int i;

	if (id == 0)
		return;
	for (i = 0; i < g_device_count; i++) {
		if (g_devices[i] == id)
			return;
	}
	if (g_device_count < 8)
		g_devices[g_device_count++] = id;
	g_device = id;
}

static void pad_ready(void)
{
	static int ready;

	if (ready)
		return;
	elevate();
	sceSysmoduleLoadModuleInternal(0x80000024);
	sceUserServiceInitialize(NULL);
	scePadInit();
	scePadSetProcessPrivilege(1);
	ready = 1;
}

static int plausible_handle(int handle)
{
	return handle > 0 && handle < 64;
}

static int sys_ptrace(int request, pid_t pid, caddr_t addr, int data)
{
	return (int)__syscall(SYS_ptrace, request, pid, addr, data);
}

static void elevate(void)
{
	uint8_t caps[16];
	pid_t pid = getpid();

	memset(caps, 0xff, sizeof(caps));
	kernel_set_ucred_authid(pid, 0x4800000000010003ull);
	kernel_set_ucred_caps(pid, caps);
}

static pid_t find_pid(const char *name)
{
	int mib[4] = {1, 14, 8, 0};
	size_t size = 0;
	uint8_t *buf;
	uint8_t *ptr;
	pid_t found = -1;

	if (sysctl(mib, 4, NULL, &size, NULL, 0) != 0 || size == 0)
		return -1;
	buf = malloc(size);
	if (!buf)
		return -1;
	if (sysctl(mib, 4, buf, &size, NULL, 0) != 0) {
		free(buf);
		return -1;
	}
	ptr = buf;
	while (ptr + 8 < buf + size) {
		int stride = *(int *)ptr;
		pid_t pid;
		const char *tdname;

		if (stride < 64 || stride > 4096)
			break;
		if (ptr + stride > buf + size)
			break;
		pid = *(pid_t *)(ptr + 72);
		tdname = (const char *)(ptr + 447);
		if (pid > 0 && strcmp(tdname, name) == 0 && (found < 0 || pid < found))
			found = pid;
		ptr += stride;
	}
	free(buf);
	return found;
}

static int pt_io(pid_t pid, int op, intptr_t remote, void *local, size_t len)
{
	struct ptrace_io_desc iod;

	iod.piod_op = op;
	iod.piod_offs = (void *)remote;
	iod.piod_addr = local;
	iod.piod_len = len;
	return sys_ptrace(PT_IO, pid, (caddr_t)&iod, 0);
}

static intptr_t resolve_sym(pid_t pid, const char *lib, const char *sym)
{
	uint32_t handle = 0;
	intptr_t addr;
	char sprx[64];
	char nid[12];

	if (kernel_dynlib_handle(pid, lib, &handle) != 0 || handle == 0) {
		snprintf(sprx, sizeof(sprx), "%s.sprx", lib);
		if (kernel_dynlib_handle(pid, sprx, &handle) != 0)
			return 0;
	}
	addr = kernel_dynlib_dlsym(pid, handle, sym);
	if (!addr) {
		nid_encode(sym, nid);
		addr = kernel_dynlib_resolve(pid, handle, nid);
	}
	return addr;
}

static intptr_t code_cave(pid_t pid, const char *lib)
{
	uint32_t handle = 0;
	intptr_t addr;
	char sprx[64];

	if (kernel_dynlib_handle(pid, lib, &handle) != 0 || handle == 0) {
		snprintf(sprx, sizeof(sprx), "%s.sprx", lib);
		if (kernel_dynlib_handle(pid, sprx, &handle) != 0 || handle == 0)
			return 0;
	}
	addr = kernel_dynlib_init_addr(pid, handle);
	if (!addr)
		addr = kernel_dynlib_fini_addr(pid, handle);
	return addr;
}

/* Call fn(a1, a2, a3) inside an already stopped process. Restores registers. */
static int64_t pt_call3(pid_t pid, intptr_t fn, intptr_t trap, uint64_t a1, uint64_t a2, uint64_t a3)
{
	struct reg regs;
	struct reg saved;
	int status;
	int waited = 0;
	int trapped = 0;
	intptr_t new_rsp;

	if (sys_ptrace(PT_GETREGS, pid, (caddr_t)&regs, 0) != 0)
		return -1;
	saved = regs;
	new_rsp = (regs.r_rsp - 256) & ~(intptr_t)0xf;
	if (pt_io(pid, PIOD_WRITE_D, new_rsp, &trap, sizeof(trap)) != 0)
		return -1;
	regs.r_rsp = new_rsp;
	regs.r_rip = fn;
	regs.r_rdi = (int64_t)a1;
	regs.r_rsi = (int64_t)a2;
	regs.r_rdx = (int64_t)a3;
	regs.r_rcx = 0;
	regs.r_r8 = 0;
	regs.r_r9 = 0;
	if (sys_ptrace(PT_SETREGS, pid, (caddr_t)&regs, 0) != 0)
		return -1;
	if (sys_ptrace(PT_CONTINUE, pid, (caddr_t)1, 0) != 0)
		return -1;
	while (waited < 2000) {
		int got = waitpid(pid, &status, WNOHANG);
		int sig;

		if (got < 0)
			break;
		if (got == 0) {
			usleep(1000);
			waited++;
			continue;
		}
		if (!WIFSTOPPED(status)) {
			sys_ptrace(PT_SETREGS, pid, (caddr_t)&saved, 0);
			return -1;
		}
		sig = WSTOPSIG(status);
		if (sig == SIGTRAP) {
			trapped = 1;
			break;
		}
		sys_ptrace(PT_CONTINUE, pid, (caddr_t)1, sig == SIGCHLD ? 0 : sig);
	}
	if (!trapped) {
		sys_ptrace(PT_SETREGS, pid, (caddr_t)&saved, 0);
		return -1;
	}
	if (sys_ptrace(PT_GETREGS, pid, (caddr_t)&regs, 0) != 0)
		return -1;
	sys_ptrace(PT_SETREGS, pid, (caddr_t)&saved, 0);
	return (int64_t)regs.r_rax;
}

static int call_remote(const char *process, const char *lib, const char *sym,
		       uint64_t a1, uint64_t a2, int64_t *out)
{
	pid_t pid;
	intptr_t fn;
	intptr_t trap;
	uint8_t original = 0;
	uint8_t breakpoint = 0xcc;
	int attached = 0;

	pid = find_pid(process);
	if (pid < 0)
		return -1;
	if (sys_ptrace(PT_ATTACH, pid, 0, 0) != 0)
		return -errno;
	attached = 1;
	if (waitpid(pid, NULL, 0) < 0) {
		sys_ptrace(PT_DETACH, pid, (caddr_t)1, 0);
		return -1;
	}
	fn = resolve_sym(pid, lib, sym);
	trap = code_cave(pid, lib);
	if (!fn || !trap) {
		sys_ptrace(PT_DETACH, pid, (caddr_t)1, 0);
		return -2;
	}
	kernel_set_vmem_protection(pid, trap, 16, PROT_READ | PROT_WRITE | PROT_EXEC);
	if (pt_io(pid, PIOD_READ_D, trap, &original, 1) != 0)
		original = 0x90;
	pt_io(pid, PIOD_WRITE_D, trap, &breakpoint, 1);
	*out = pt_call3(pid, fn, trap, a1, a2, 0);
	pt_io(pid, PIOD_WRITE_D, trap, &original, 1);
	sys_ptrace(PT_DETACH, pid, (caddr_t)1, 0);
	attached = 0;
	(void)attached;
	return 0;
}

static int shellcore_vda(int *code)
{
	pid_t pid;
	struct reg regs;
	struct {
		int32_t size;
		int32_t user_id;
		int32_t pad[6];
	} param;
	struct {
		int32_t size;
		int32_t user_id;
		int32_t pad[6];
	} out;
	intptr_t param_addr;
	intptr_t fn;
	intptr_t trap;
	uint8_t original = 0x90;
	uint8_t breakpoint = 0xcc;
	int64_t result;
	int i;
	const int32_t sentinel = 0x7eadbeef;

	*code = -1;
	pid = find_pid("SceShellCore");
	if (pid < 0) {
		*code = -1;
		return -1;
	}
	if (sys_ptrace(PT_ATTACH, pid, 0, 0) != 0) {
		*code = -errno;
		return -1;
	}
	if (waitpid(pid, NULL, 0) < 0) {
		sys_ptrace(PT_DETACH, pid, (caddr_t)1, 0);
		return -1;
	}
	fn = resolve_sym(pid, "libScePad", "scePadVirtualDeviceAddDevice");
	trap = code_cave(pid, "libScePad");
	if (!fn || !trap || sys_ptrace(PT_GETREGS, pid, (caddr_t)&regs, 0) != 0) {
		sys_ptrace(PT_DETACH, pid, (caddr_t)1, 0);
		return -2;
	}
	memset(&param, 0, sizeof(param));
	param.size = (int32_t)sizeof(param);
	param.user_id = 1;
	for (i = 0; i < 6; i++)
		param.pad[i] = sentinel;
	param_addr = (regs.r_rsp - 4096) & ~(intptr_t)0xf;
	kernel_set_vmem_protection(pid, trap, 16, PROT_READ | PROT_WRITE | PROT_EXEC);
	pt_io(pid, PIOD_READ_D, trap, &original, 1);
	pt_io(pid, PIOD_WRITE_D, trap, &breakpoint, 1);
	pt_io(pid, PIOD_WRITE_D, param_addr, &param, sizeof(param));
	result = pt_call3(pid, fn, trap, (uint64_t)param_addr, 3, 0);
	memset(&out, 0, sizeof(out));
	pt_io(pid, PIOD_READ_D, param_addr, &out, sizeof(out));
	pt_io(pid, PIOD_WRITE_D, trap, &original, 1);
	sys_ptrace(PT_DETACH, pid, (caddr_t)1, 0);
	*code = (int)result;
	if (plausible_handle((int)result))
		return (int)result;
	for (i = 0; i < 6; i++) {
		if (out.pad[i] != sentinel && plausible_handle(out.pad[i]))
			return out.pad[i];
	}
	return -1;
}

static uint64_t parse_hex(const char *text)
{
	uint64_t value = 0;

	while (*text) {
		char c = *text++;
		if (c >= '0' && c <= '9')
			value = (value << 4) | (uint64_t)(c - '0');
		else if (c >= 'a' && c <= 'f')
			value = (value << 4) | (uint64_t)(c - 'a' + 10);
		else if (c >= 'A' && c <= 'F')
			value = (value << 4) | (uint64_t)(c - 'A' + 10);
		else
			break;
	}
	return value;
}

static uint64_t device_from_line(const char *line)
{
	const char *mark;

	if (!strstr(line, "DEVICE_ADDED") && !strstr(line, "DeviceAdded"))
		return 0;
	if (!strstr(line, "subType:22") && !strstr(line, "subType=22") &&
	    !strstr(line, "subtype:22") && !strstr(line, "subtype=22"))
		return 0;
	mark = strstr(line, "DeviceId:0x");
	if (!mark)
		mark = strstr(line, "deviceId=0x");
	if (!mark)
		return 0;
	return parse_hex(mark + 11);
}

static uint64_t read_klog_device(int fd, int wait_ms)
{
	char chunk[512];
	char line[1024];
	size_t fill = 0;
	uint64_t found = 0;
	int waited = 0;

	while (waited <= wait_ms) {
		ssize_t n = read(fd, chunk, sizeof(chunk));
		if (n < 0) {
			if (errno == EAGAIN || errno == EWOULDBLOCK) {
				if (wait_ms == 0)
					break;
				usleep(20000);
				waited += 20;
				continue;
			}
			break;
		}
		if (n == 0)
			break;
		for (ssize_t i = 0; i < n; i++) {
			if (chunk[i] == '\n' || fill + 1 >= sizeof(line)) {
				uint64_t id;
				const char *ret;
				line[fill] = '\0';
				id = device_from_line(line);
				if (id) {
					found = id;
					remember_device(id);
				}
				ret = strstr(line, "Open Pad");
				if (ret && (ret = strstr(line, "ret=")) != NULL) {
					int opened = (int)strtol(ret + 4, NULL, 0);
					if (plausible_handle(opened))
						g_klog_handle = opened;
				}
				fill = 0;
			} else {
				line[fill++] = chunk[i];
			}
		}
	}
	return found;
}

static int local_vda(int *code)
{
	struct {
		int32_t size;
		int32_t user_id;
		int32_t pad[6];
	} param;
	int handle;
	int i;
	const int32_t sentinel = 0x7eadbeef;

	memset(&param, 0, sizeof(param));
	param.size = (int32_t)sizeof(param);
	param.user_id = 1;
	for (i = 0; i < 6; i++)
		param.pad[i] = sentinel;
	handle = scePadVirtualDeviceAddDevice(&param, PAD_VIRT_TYPE_DUALSENSE);
	*code = handle;
	if (!plausible_handle(handle)) {
		for (i = 0; i < 6; i++) {
			if (param.pad[i] != sentinel && plausible_handle(param.pad[i]))
				return param.pad[i];
		}
		return -1;
	}
	return handle;
}

/* The player prompt only accepts the new controller's PS button.
 * Press PS inside SceShellUI so the account list opens, and leave Cross
 * alone so the player can pick the account. */
static int shellui_press_ps(int known_handle)
{
	pid_t pid;
	intptr_t fn_get;
	intptr_t fn_vdi;
	intptr_t trap;
	intptr_t data_addr;
	struct reg regs;
	uint8_t original = 0x90;
	uint8_t breakpoint = 0xcc;
	int users[3];
	int user_count = 0;
	int foreground = 0;
	int handle = known_handle;
	int type;
	int index;
	int user;
	int frame;
	PadState state;
	PadData data;

	if (sceUserServiceGetForegroundUser(&foreground) == 0 && foreground > 0)
		users[user_count++] = foreground;
	users[user_count++] = -1;
	users[user_count++] = 1;

	pid = find_pid("SceShellUI");
	if (pid < 0)
		return -1;
	if (sys_ptrace(PT_ATTACH, pid, 0, 0) != 0)
		return -1;
	if (waitpid(pid, NULL, 0) < 0) {
		sys_ptrace(PT_DETACH, pid, (caddr_t)1, 0);
		return -1;
	}
	fn_get = resolve_sym(pid, "libScePad", "scePadGetHandle");
	fn_vdi = resolve_sym(pid, "libScePad", "scePadVirtualDeviceInsertData");
	trap = code_cave(pid, "libScePad");
	if (!fn_vdi || !trap || sys_ptrace(PT_GETREGS, pid, (caddr_t)&regs, 0) != 0) {
		sys_ptrace(PT_DETACH, pid, (caddr_t)1, 0);
		return -1;
	}
	kernel_set_vmem_protection(pid, trap, 16, PROT_READ | PROT_WRITE | PROT_EXEC);
	pt_io(pid, PIOD_READ_D, trap, &original, 1);
	pt_io(pid, PIOD_WRITE_D, trap, &breakpoint, 1);

	if (handle <= 0 && fn_get) {
		for (type = 0; type < 2 && handle <= 0; type++) {
			int port = type == 0 ? PAD_VIRT_TYPE_DUALSENSE : 0;
			int first = port == 0 ? 1 : 0;
			for (user = 0; user < user_count && handle <= 0; user++) {
				for (index = first; index < 4 && handle <= 0; index++) {
					int64_t got = pt_call3(pid, fn_get, trap,
							       (uint64_t)(uint32_t)users[user],
							       (uint64_t)port, (uint64_t)index);
					if (plausible_handle((int32_t)got))
						handle = (int32_t)got;
				}
			}
		}
	}
	if (handle <= 0) {
		pt_io(pid, PIOD_WRITE_D, trap, &original, 1);
		sys_ptrace(PT_DETACH, pid, (caddr_t)1, 0);
		return -1;
	}

	pad_state_neutral(&state);
	state.buttons = PAD_BUTTON_PS;
	pad_data_from_state(&data, &state, sceKernelGetProcessTime());
	data_addr = (regs.r_rsp - 4096) & ~(intptr_t)0xf;
	pt_io(pid, PIOD_WRITE_D, data_addr, &data, sizeof(data));
	for (frame = 0; frame < 12; frame++)
		pt_call3(pid, fn_vdi, trap, (uint64_t)handle, (uint64_t)data_addr, 0);
	state.buttons = 0;
	pad_data_from_state(&data, &state, sceKernelGetProcessTime());
	pt_io(pid, PIOD_WRITE_D, data_addr, &data, sizeof(data));
	pt_call3(pid, fn_vdi, trap, (uint64_t)handle, (uint64_t)data_addr, 0);
	pt_io(pid, PIOD_WRITE_D, trap, &original, 1);
	sys_ptrace(PT_DETACH, pid, (caddr_t)1, 0);
	return handle;
}

static int scan_handle(void)
{
	int users[3];
	int count = 0;
	int foreground = 0;
	int i;
	int index;

	if (sceUserServiceGetForegroundUser(&foreground) == 0 && foreground > 0)
		users[count++] = foreground;
	users[count++] = 1;
	users[count++] = 0x10000000;
	for (i = 0; i < count; i++) {
		for (index = 0; index < 4; index++) {
			int handle = scePadGetHandle(users[i], PAD_VIRT_TYPE_DUALSENSE, index);
			if (plausible_handle(handle))
				return handle;
		}
	}
	return -1;
}

static void set_status(int handle)
{
	if (handle > 0)
		snprintf(g_status, sizeof(g_status), "Virtual pad %d", handle);
	else
		snprintf(g_status, sizeof(g_status), "No virtual pad, shell %08x bind %d",
			 (unsigned)g_code, g_bind);
}

int pad_assign_from_shell(void)
{
	int handle;
	uint64_t device = 0;
	int klog_fd;
	int user = 0x10000000;
	int64_t bound = -1;

	if (g_shell_done) {
		handle = pad_assign_device_handle();
		if (handle <= 0)
			handle = scan_handle();
		if (handle > 0)
			set_status(handle);
		return handle;
	}
	g_shell_done = 1;
	elevate();
	scePadSetProcessPrivilege(1);

	klog_fd = open("/dev/klog", O_RDONLY | O_NONBLOCK);
	if (klog_fd >= 0)
		read_klog_device(klog_fd, 0);

	handle = local_vda(&g_code);
	if (handle <= 0 && (unsigned)g_code != 0x803b0006u)
		handle = shellcore_vda(&g_code);
	if (klog_fd >= 0) {
		/* A refused add will not print a device id. Waiting here freezes USB reads. */
		device = read_klog_device(klog_fd, handle > 0 ? 800 : 0);
		close(klog_fd);
	}
	if (sceUserServiceGetForegroundUser(&user) != 0 || user <= 0)
		user = 0x10000000;
	if (device) {
		if (call_remote("SceShellUI", "libSceMbus", "sceMbusBindDeviceWithUserId",
				device, (uint64_t)(uint32_t)user, &bound) != 0)
			call_remote("SceShellCore", "libSceMbus", "sceMbusBindDeviceWithUserId",
				    device, (uint64_t)(uint32_t)user, &bound);
		g_bind = (int)bound;
	}
	g_device = device;
	remember_device(device);
	if (!plausible_handle(handle) && g_klog_handle > 0)
		handle = g_klog_handle;
	if (!plausible_handle(handle))
		handle = scan_handle();
	/* The system log's device id is the handle InsertData accepts. */
	if (device > 0 && device <= 0x7fffffff)
		handle = (int)device;
	if (handle > 0)
		shellui_press_ps(handle);
	set_status(handle);
	printf("YetAnotherControllerEnabler: %s device 0x%llx bind %d\n", g_status,
	       (unsigned long long)device, g_bind);
	return handle;
}

int pad_assign_device_handle(void)
{
	if (g_device > 0 && g_device <= 0x7fffffff)
		return (int)g_device;
	return -1;
}

int pad_assign_release(void)
{
	int removed = 0;
	int i;
	int fd;

	pad_ready();
	fd = open("/dev/klog", O_RDONLY | O_NONBLOCK);
	if (fd >= 0) {
		read_klog_device(fd, 200);
		close(fd);
	}
	for (i = 0; i < g_device_count; i++) {
		int64_t result = -1;

		if (call_remote("SceShellUI", "libSceMbus", "sceMbusDisconnectDevice",
				g_devices[i], 0, &result) != 0)
			call_remote("SceShellCore", "libSceMbus", "sceMbusDisconnectDevice",
				    g_devices[i], 0, &result);
		if (result == 0)
			removed++;
		if (g_devices[i] <= 0x7fffffff &&
		    scePadVirtualDeviceDeleteDevice((int)g_devices[i]) == 0)
			removed++;
	}
	for (i = 0; i < 64; i++) {
		if (scePadVirtualDeviceDeleteDevice(i) == 0)
			removed++;
	}
	g_device_count = 0;
	g_device = 0;
	g_klog_handle = -1;
	g_shell_done = 0;
	g_code = -1;
	g_bind = -1;
	printf("YetAnotherControllerEnabler: removed %d virtual controllers\n", removed);
	return removed;
}

const char *pad_assign_status(void)
{
	return g_status;
}
