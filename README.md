# Weeb Labs USBrx — S/PDIF → USB audio capture (RP2040)

> **This branch (`i2s-input`, fork totochi-2022/USBrx): I2S ADC input instead of S/PDIF.**
> It turns a second Pico into the recording half of a full-duplex setup with
> [DSPi](https://github.com/WeebLabs/DSPi): DSPi plays, this recorder captures an
> I2S ADC, and **both run on DSPi's I2S clocks** (24-bit, 44.1/48/88.2/96 kHz).
> The rest of this README describes the original S/PDIF firmware; the
> differences are listed in this section.

### I2S variant: what changed

- The S/PDIF receiver (`pico_spdif_rx`, submodule removed) is replaced by
  `src/i2s_rx.c` + `src/i2s_rx.pio`: DSPi's framing-checked I2S **slave** RX
  program (`audio_i2s_rx_slave_checked`) feeding an IRQ-less DMA ring
  (2048 stereo frames).
- The sample rate is measured from LRCLK (DMA ring advance over 32 ms windows)
  and snapped to 44.1 / 48 / 88.2 / 96 kHz; lock needs two agreeing windows.
- Framing slips (the master restarted or glitched, e.g. DSPi resyncing its
  outputs) are flagged by the PIO program and the receiver restarts cleanly.
  Clock loss (no LRCLK for 5 ms) streams silence until the clocks return.
- The servo starts exactly at the target fill, and the target is **adaptive**
  (see below) instead of a fixed 256 frames.
- USB: product string "USBrx I2S", input terminal type *line connector*
  (0x0603), bcdDevice 0x0200.
- Builds for Pico (RP2040) and Pico 2 (RP2350): `-DPICO_BOARD=pico2`.

### I2S variant: latency auto-tuning

The ring set-point is the recorder's buffering latency. It is tuned while
streaming, like an adaptive jitter buffer (`ADAPT_*` in `src/main.c`):

| Setting | Default | Meaning |
|---|---|---|
| `ADAPT_START_MS` | 3.0 | Initial target (5.3 ms fixed before) |
| `ADAPT_MIN_MS` / `ADAPT_MAX_MS` | 1.5 / 12.0 | Bounds |
| `ADAPT_NEAR_MS` | 0.25 | Headroom left after a packet below this = near underrun |
| `ADAPT_STEP_UP_MS` / `ADAPT_UNDERRUN_UP_MS` | 1.0 / 2.0 | Raise at once on a near underrun / real underrun |
| `ADAPT_CALM_PACKETS` | 30000 | ~30 s of comfortable headroom before stepping down |
| `ADAPT_STEP_DOWN_MS` | 0.25 | Step down size |
| `ADAPT_COOLDOWN_PACKETS` | 500 | Ignore near events while the servo follows a raise |

- Fast up, slow down. A step down happens only if the whole calm window kept
  enough headroom that the smaller target still clears the margin.
- Changes never click: the servo moves the fill by at most 2 frames per ms
  (a tiny, inaudible time-warp).
- Only events while locked and streaming count. Clock loss and slips re-prime
  instead of raising the target. The learned target is kept across streams
  until power-off.
- Each change is logged on the UART: `latency target X ms (underruns N, near M)`.

### I2S variant: wiring

| Signal | Recorder Pico | Connects to |
|---|---|---|
| BCK   | GP10 (input) | DSPi BCK (default GPIO 14) and the ADC BCK |
| LRCLK | GP11 (input) | DSPi LRCLK (default GPIO 15) and the ADC LRCK |
| DATA  | GP12 (input) | ADC DOUT |
| GND   | GND | DSPi GND and ADC GND |
| (MCK) | — | DSPi MCK (default GPIO 13) → ADC SCKI, if the ADC needs it |
| Rate LEDs | GP2..GP5 | high when locked at 44.1 / 48 / 88.2 / 96 kHz |
| Debug | GP0/GP1 | UART0 115200 |

- DSPi must be **driving BCK/LRCLK**: at least one DSPi output slot set to I2S
  (and MCK enabled if the ADC needs a system clock).
- The ADC must be in **slave mode, I2S (Philips) format, 24-bit**
  (e.g. PCM1808: MD0 = MD1 = low, FMT = low, SCKI = DSPi MCK at 256 fs).
- Pins are the `I2S_*_PIN` defines at the top of `src/main.c`.
- The host sees DSPi (playback) and USBrx I2S (recording) as two devices; use
  PipeWire, a macOS aggregate device, or ASIO4ALL to treat them as one. Both
  run on DSPi's crystal, so no drift accumulates between them.
- Select the same rate on the host as DSPi is running at (the recorder
  follows LRCLK and does not resample).

### I2S variant: build

```sh
git clone -b i2s-input https://github.com/totochi-2022/USBrx.git
cd USBrx
mkdir build && cd build
cmake -DCMAKE_BUILD_TYPE=Release ..                 # Pico (RP2040)
# cmake -DCMAKE_BUILD_TYPE=Release -DPICO_BOARD=pico2 ..   # Pico 2 (RP2350)
cmake --build . -j4                                  # -> build/usbrx.uf2
```

Status (2026-11-07): written, **not yet built or tested on hardware**.

---

Firmware for the Raspberry Pi Pico / RP2040 that receives a stereo **S/PDIF**
stream on a GPIO and presents it to a USB host as a **class-compliant USB Audio
Class 1.0 recording device** ("Weeb Labs USBrx"). No drivers required on
macOS, Windows, or Linux.

The S/PDIF source is the master clock. A rate-matching servo continuously adapts
the USB data rate to the incoming stream's clock by varying the isochronous
packet size each frame (implicit feedback), so the audio is captured
sample-accurately **without any resampling**.

```
 S/PDIF in (GP15) ──[PIO bi-phase decode + DMA]──► FIFO ring buffer (1536 stereo frames)
                                                       │
                                 rate-matching servo, once per USB frame (1 ms)
                                                       │
                          UAC1 isochronous async IN endpoint 0x81 ──► USB host
```

---

## Features

- **Class-compliant UAC1 capture device** — plug-and-play, no host drivers.
- **24-bit stereo PCM** at **44.1 / 48 / 88.2 / 96 kHz**.
- **Sample rate follows the S/PDIF source** — detected automatically by the
  receiver; no resampling, ever.
- **Clock-drift rate matching** — a software servo keeps the capture buffer
  centered by nudging the USB packet size ±a sample or two per frame, absorbing
  the small (ppm-level) difference between the S/PDIF and USB host clocks.
- **Graceful with no signal** — streams silence until a stable S/PDIF lock is
  acquired, then begins capturing.
- **Onboard-LED status** and **UART debug log** for easy bring-up.
- **GPIO status outputs** - simple status outputs to drive LEDs
- Self-contained CMake build on top of the Raspberry Pi Pico SDK + TinyUSB.

---

## How it works

### Signal chain

1. **S/PDIF receive** — the [`pico_spdif_rx`](https://github.com/elehobica/pico_spdif_rx)
   library (PIO + DMA) decodes the bi-phase-mark-coded S/PDIF bitstream on a
   single GPIO, recovers the sample rate, and fills an internal FIFO. The DMA
   delivers data in blocks of 192 stereo frames. This FIFO is the input ring
   buffer.
2. **Format conversion** — each S/PDIF sub-frame word carries the 24-bit sample
   in bits `[27:4]`; the firmware extracts it as little-endian 24-bit PCM.
3. **USB transmit** — a custom UAC1 class driver feeds the isochronous IN
   endpoint. The host polls it once per 1 ms frame; each frame the servo decides
   how many stereo frames to send.

### Rate matching (implicit feedback)

Because the device is the clock master (driven by S/PDIF) and the endpoint is an
**asynchronous IN** endpoint, the standard USB Audio rate-feedback mechanism is
*implicit*: the device simply varies how many samples it puts in each packet, and
the host accepts whatever size arrives. There is **no separate feedback
endpoint** — that only exists for asynchronous *OUT* (playback) endpoints.

Each frame the servo computes:

```
samples_this_frame = nominal + correction
  nominal     = actual S/PDIF rate ÷ 1000      (feed-forward; e.g. ~48.0 @ 48 kHz)
  correction  = clamp(Kp · (fifo_level − target), ±2 samples)
```

A fractional-sample accumulator turns the floating-point result into an integer
count per frame (so 44.1/88.2 kHz average out correctly). The feed-forward term
uses the *measured* S/PDIF frequency, so the proportional correction only ever
has to absorb tiny clock jitter — the loop is gentle and never audibly
time-warps the audio. The buffer parks at `TARGET_FILL_FRAMES` (256 ≈ 5.3 ms
@ 48 kHz), comfortably clear of underflow and overflow. *(I2S variant: the
set-point is adaptive instead; see "Latency auto-tuning" at the top.)*

### Why a custom UAC1 class driver?

TinyUSB's built-in audio class driver is **UAC2-only** (`audiod_open()` rejects
any AudioControl interface whose `bInterfaceProtocol` isn't UAC2). This firmware
therefore sets `CFG_TUD_AUDIO = 0` and registers its own minimal **UAC1** class
driver via `usbd_app_driver_get_cb()`, with an **Interface Association
Descriptor** so the host (and TinyUSB's config parser) bind both the
AudioControl and AudioStreaming interfaces to it. The driver handles
`SET/GET_INTERFACE`, the UAC1 endpoint **sampling-frequency control**, and drives
the IN endpoint from the rate-matching servo.

---

## Supported formats & the full-speed bandwidth limit

The RP2040's USB is **full-speed only** (12 Mbit/s), capping a single
isochronous endpoint at 1023 bytes per 1 ms frame. At 24-bit stereo that limits
the achievable sample rate:

| Rate      | Bytes / frame (24-bit stereo) | Fits in full-speed? |
|-----------|-------------------------------|---------------------|
| 44.1 kHz  | ~265                          | ✅                  |
| 48 kHz    | 288                           | ✅                  |
| 88.2 kHz  | ~529                          | ✅                  |
| 96 kHz    | 576                           | ✅                  |
| 176.4 kHz | ~1058                         | ❌ (exceeds 1023)   |
| 192 kHz   | 1152                          | ❌ (exceeds 1023)   |

So the device advertises 44.1 / 48 / 88.2 / 96 kHz. 176.4 / 192 kHz S/PDIF
sources are not supported at 24-bit on full-speed USB. The endpoint reserves a
600-byte max packet (100 frames) to give the servo headroom at 96 kHz.

---

## Hardware

### Wiring

| Pico pin | GPIO | Function |
|----------|------|----------|
| 20       | GP15 | S/PDIF data in |
| 1        | GP0  | UART0 TX (debug log, optional) |
| 2        | GP1  | UART0 RX (debug log, optional) |
| 4        | GP2  | Output, high when S/PDIF locked to 44.1kHz |
| 5        | GP3  | Output, high when S/PDIF locked to 48kHz |
| 6        | GP4  | Output, high when S/PDIF locked to 88.2kHz |
| 7        | GP5  | Output, high when S/PDIF locked to 96kHZ |
| 38 etc.  | GND  | ground |

You need a coaxial or TOSLINK S/PDIF receiver module (e.g. a DLR1160 or
equivalent) to convert the S/PDIF signal to 3.3 V logic feeding GP15. For
TOSLINK, the receiver module's logic output can drive GP15 directly. See the
upstream [schematic](https://github.com/elehobica/pico_spdif_rx/blob/main/doc/SPDIF_Rx_Schematic.png).

The input GPIO is the `SPDIF_DATA_PIN` define at the top of `src/main.c`.

A set of GPIO pins provide status outputs that can be used for simple input lock reporting,
possibly driving additional LEDs.

### Onboard-LED status

| LED               | Meaning                                            |
|-------------------|----------------------------------------------------|
| Fast blink (~10 Hz) | Running, but the host has not enumerated it yet   |
| Slow blink (~2 Hz)  | Enumerated/mounted by the host, not yet recording |
| Solid on          | Host has the stream open (recording)               |

> Note: on a **Pico W**, the onboard LED is wired to the wireless chip (not
> GP25), so this indicator won't light there even though USB works fine.

### Debug log

`printf` output goes to **UART0 @ 115200** (GP0/GP1). It reports startup,
S/PDIF lock/loss, and the detected sample rate. USB is left entirely for audio.

---

## Building

### Prerequisites

- **CMake ≥ 3.13** and the **`arm-none-eabi-gcc`** toolchain.
- The **Raspberry Pi Pico SDK** (this project is verified against **2.1.1**),
  with its TinyUSB submodule initialized.

### Get the sources

```sh
git clone --recurse-submodules https://github.com/WeebLabs/USBrx.git
cd USBrx
# If you cloned without --recurse-submodules:
git submodule update --init
```

This pulls in `lib/pico_spdif_rx`. The Pico SDK is **not** vendored in the repo —
provide it one of two ways:

- **Environment variable (standard):**
  ```sh
  export PICO_SDK_PATH=/path/to/pico-sdk     # must have its tinyusb submodule initialized
  ```
- **Or place/symlink it in-tree** as `./pico-sdk` (the build uses it automatically
  if present):
  ```sh
  git clone -b 2.1.1 https://github.com/raspberrypi/pico-sdk.git
  git -C pico-sdk submodule update --init lib/tinyusb
  ```

### Build

```sh
mkdir build && cd build
cmake -DCMAKE_BUILD_TYPE=Release ..
cmake --build . -j4
```

Output: **`build/usbrx.uf2`**.

> **Raspberry Pi Pico 2 (RP2350):** the S/PDIF library also supports it (≤ 96 kHz).
> Configure with `cmake -DPICO_BOARD=pico2 -DPICO_PLATFORM=rp2350 ..`.

---

## Flashing

1. Hold **BOOTSEL** and plug the Pico into USB — it mounts as the `RPI-RP2`
   drive.
2. Copy `build/usbrx.uf2` onto that drive.
3. The board reboots and enumerates as **Weeb Labs USBrx**.

---

## Using it

The device appears as a standard 2-channel, 24-bit audio **input**:

- **macOS** — *System Settings → Sound → Input*, or *Audio MIDI Setup*. It shows
  up even with no S/PDIF source connected (it streams silence until locked).
- **Windows** — *Settings → System → Sound → Input* (or Sound Control Panel →
  Recording).
- **Linux** — `arecord -l` / PipeWire / PulseAudio; it's a standard USB-audio
  capture device.

> **Important — match the sample rate.** The actual stream rate is whatever the
> S/PDIF source is. Set the host's capture rate to match the source (e.g. 48 kHz
> for most digital sources, 44.1 kHz for CD). If the host's rate doesn't match,
> the buffer servo still prevents drift, but the audio will play back at the
> wrong pitch/speed — by design, since there is no resampling.

---

## Project layout

```
src/main.c             firmware: S/PDIF setup, rate-matching servo, custom UAC1 class driver
src/usb_descriptors.c  hand-rolled UAC1 descriptor set (IAD + AC/AS interfaces + iso IN EP)
src/tusb_config.h      TinyUSB configuration (built-in audio driver disabled; format constants)
CMakeLists.txt         build (finds the Pico SDK, builds pico_spdif_rx, links TinyUSB device)
pico_sdk_import.cmake  standard Pico SDK locator shim
lib/pico_spdif_rx/     S/PDIF receiver library (git submodule, pinned)
```

### Tuning

All in `src/main.c`:

| Define               | Default | Effect |
|----------------------|---------|--------|
| `SPDIF_DATA_PIN`     | 15      | S/PDIF input GPIO *(I2S variant: `I2S_*_PIN`)* |
| `TARGET_FILL_FRAMES` | 256     | Buffer set-point → capture latency (≈ 5.3 ms @ 48 kHz). Lower = less latency, less jitter margin. *(I2S variant: replaced by the adaptive `ADAPT_*` settings.)* |
| `SERVO_KP`           | 0.008   | Servo proportional gain. |
| `SERVO_MAX_CORR`     | 2.0     | Max packet-size deviation (samples/frame). |

---

## Troubleshooting

- **No USB device appears at all** — confirm the firmware is flashed (after the
  `.uf2` copy the `RPI-RP2` drive ejects and the board reboots), and use a USB
  **data** cable (many cables are charge-only) directly into the host. The
  onboard LED should blink (except on Pico W).
- **Enumerates but isn't listed as an audio input** — make sure you flashed the
  current build. (Hosts like macOS won't create the audio device unless the
  UAC1 sampling-frequency control and interface binding are correct, which this
  firmware handles.)
- **Wrong pitch / speed** — the host's selected sample rate doesn't match the
  S/PDIF source. Set them equal (see "Using it").
- **Periodic dropouts / glitches** — check the S/PDIF wiring/levels; the UART log
  reports lock/loss and parity. A flaky source can cause re-locks.
- **Inspect enumeration** — on macOS: `system_profiler SPUSBDataType` (USB) and
  `system_profiler SPAudioDataType` (audio); on Linux: `lsusb -v` and
  `arecord -l`.

---

## Acknowledgements

- [`pico_spdif_rx`](https://github.com/elehobica/pico_spdif_rx) by **Elehobica** —
  the PIO-based S/PDIF receiver (BSD-2-Clause).
- [TinyUSB](https://github.com/hathach/tinyusb) and the
  [Raspberry Pi Pico SDK](https://github.com/raspberrypi/pico-sdk).

The custom-UAC1-class-driver approach (disabling TinyUSB's UAC2-only built-in
driver and registering a UAC1 driver with an IAD) follows the pattern proven in
the DSPi firmware.

---

## License

This project's own code is licensed under the **GNU General Public License v3.0
or later** — see [`LICENSE`](LICENSE).

Bundled/linked dependencies retain their own licenses: the Raspberry Pi Pico SDK
and TinyUSB (BSD-3-Clause) and `pico_spdif_rx` (BSD-2-Clause).
