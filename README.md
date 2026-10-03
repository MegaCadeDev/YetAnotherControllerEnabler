# YetAnotherControllerEnabler

YetAnotherControllerEnabler is a homebrew payload for a PS5. It lets a controller from another console play on the PS5 by publishing a virtual DualSense and filling that pad with the other controller's buttons and sticks. The console treats the virtual pad like a DualSense: it can be assigned to a player, and games read it as a normal controller.

Load the ELF and leave it running. It watches for a supported controller, brings that pad up, and posts an on-screen notice when it is searching, when it finds one, and when that controller connects or disconnects. When the console asks which account is using the new pad, you choose the account yourself.

## Supported controllers

- Switch 1 Pro over USB
- Switch 2 Pro over USB
- DualShock 3 (PS3) over USB
- 8BitDo V1 and V2 wireless adapters. A controller on one of these works because that adapter is already translating it.

A DualShock 4 works as a PS5 controller only when it is connected through an 8BitDo adapter. Plugged in on its own, the PS5 takes over the driver and treats it as a normal PS4 controller.

## Build

Install the [PS5 payload SDK](https://github.com/ps5-payload-dev/sdk) by following that repository's instructions, including the Clang version it asks for. `make` uses the SDK's `prospero-clang`, so a normal host compiler is not enough to produce the payload.

Point `PS5_PAYLOAD_SDK` at the SDK directory that contains `toolchain/prospero.mk`:

```sh
export PS5_PAYLOAD_SDK=/path/to/ps5-payload-sdk
make
make host-test
```

`make` writes `YetAnotherControllerEnabler.elf`. `make host-test` compiles the report parsers with the system C compiler (`cc`) and runs them on the build machine. It does not need the payload SDK.

## Load

With elfldr already listening on the console:

```sh
export PS5_HOST=192.168.1.50
export PS5_PORT=9021
make test
```

The payload logs to stdout. `scePadVirtualDeviceAddDevice` can return `0x803B0006` until the console assignment screen is accepted. The payload retries every two seconds.

## Known issues

Xbox One over USB is still in development. The stick axes come through, but the other buttons do not yet translate reliably. The d-pad is the worst of it: one press is sometimes ignored, sometimes correct, and sometimes held down after you let go. Face buttons have also been missed.

## Future improvements

The supported controllers work over USB, or through an 8BitDo adapter. The goal is to pair them wirelessly with the PS5's internal antenna as well, and still publish a virtual DualSense that translates their input.

## Requesting more controllers

Support is limited to controllers I can test on. I will not buy more controllers to add support. A [Ko-fi donation](https://ko-fi.com/megacadedev) earmarked for a specific controller can be considered toward buying that pad. A donation does not guarantee that support will be added.

Xbox One Wireless adapters are out of scope. Their firmware has to be reverse engineered before they can be supported, and that work is not part of this project.

## LAN frame check

The payload also listens on TCP port 9045. Each frame is 16 little-endian bytes: `uint32` buttons, then `lx ly rx ry l2 r2`, then 6 unused bytes. Sticks use 0–255 with 128 at center.

```sh
python3 tools/send_frame.py 192.168.1.50 cross
```

That holds Cross on the virtual pad so you can confirm injection before the USB node is known.

## License

Copyright (C) 2026 MegaCadeDev.

Licensed under the GNU General Public License v3.0 or later. See [LICENSE](LICENSE).
