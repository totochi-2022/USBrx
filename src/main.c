/* SPDX-License-Identifier: GPL-3.0-or-later */
/*
 * I2S (ADC) -> USB UAC1 stereo recording device for the RP2040 / RP2350.
 * Copyright (C) 2026 Weeb Labs (S/PDIF original); I2S input variant.
 *
 *   I2S ADC data (GPIO) --[PIO slave RX + DMA ring, clocks from DSPi]--> ring
 *                                                          |
 *                              rate-matching servo (per USB frame)
 *                                                          |
 *                         UAC1 isochronous async IN endpoint --> USB host
 *
 * BCK/LRCLK are driven by an external I2S master (normally the DSPi that
 * plays the same session), so recording and playback share one clock.  The
 * receiver measures the LRCLK rate and fills a ring.  The USB host polls the
 * isochronous IN endpoint once per (1 ms) frame.  Each frame we decide how many
 * stereo frames to send: a feed-forward term equal to the measured I2S rate
 * (samples per millisecond) plus a small proportional correction that keeps
 * the ring near a target fill level.  Varying the packet size frame-by-frame
 * is the implicit feedback that matches the USB data rate to the I2S clock
 * (no resampling).
 *
 * The sample rate is dictated by the I2S master. The host must select the
 * matching rate; if it does not, audio streams at the wrong pitch (by design).
 *
 * TinyUSB's built-in audio driver is UAC2-only, so the audio function is
 * implemented here as a custom UAC1 class driver registered through
 * usbd_app_driver_get_cb() (pattern proven in the DSPi UAC1 firmware).
 */

#include <stdio.h>
#include <string.h>

#include "pico/stdlib.h"
#include "tusb.h"
#include "device/usbd_pvt.h"      /* usbd_class_driver_t, usbd_edpt_* */
#include "class/audio/audio.h"    /* AUDIO_* constants (not pulled in when CFG_TUD_AUDIO=0) */

#include "i2s_rx.h"
#include "tusb_config.h"

/* -------------------------------------------------------------------------- */
/* Configuration                                                               */
/* -------------------------------------------------------------------------- */
#define I2S_DATA_PIN          12     /* I2S data in (ADC DOUT) */
#define I2S_BCK_PIN           10     /* I2S bit clock in (from the master) */
#define I2S_LRCLK_PIN         11     /* I2S word clock in (from the master) */
#define LOCK_STATUS_44100_PIN 2      /* high when locked to 44.1 kHz */
#define LOCK_STATUS_48000_PIN 3      /* high when locked to 48 kHz */
#define LOCK_STATUS_88200_PIN 4      /* high when locked to 88.2 kHz */
#define LOCK_STATUS_96000_PIN 5      /* high when locked to 96 kHz */

#define PIN_DCDC_PSM_CTRL     23     /* Pico SMPS mode pin: drive high for PWM (less audio noise) */

#define AUDIO_IN_EP           0x81   /* isochronous IN endpoint address (must match descriptor) */

/* UAC1 control request codes (direction is encoded in bRequest, unlike UAC2). */
#define UAC1_REQ_SET_CUR      0x01
#define UAC1_REQ_GET_CUR      0x81

/* Target number of stereo frames buffered in the I2S ring. 256 frames keeps
 * latency low (~5.3 ms @ 48 kHz) while leaving ample margin against
 * under/overflow (ring holds 2048 frames). */
#define TARGET_FILL_FRAMES    256

/* Proportional servo gain and clamp. The I2S and USB clocks differ only by a
 * few ppm, so only a tiny correction is ever needed in steady state; the clamp
 * also bounds the pull-in rate so the audio is never noticeably time-warped. */
#define SERVO_KP              0.008f   /* (samples/frame) per (frame) of fill error */
#define SERVO_MAX_CORR        2.0f     /* max |correction| in samples/frame */

/* -------------------------------------------------------------------------- */
/* State (all USB callbacks below run in tud_task() context on core 0)         */
/* -------------------------------------------------------------------------- */
static volatile uint32_t g_usb_sample_rate = 48000;   /* host-selected rate (echoed on GET_CUR) */
static uint32_t          g_silence_frames  = 48;      /* silence packet size = host_rate / 1000  */

static uint32_t             g_last_rate    = 0;       /* last locked I2S rate seen by the servo */
static float                g_nominal_fpf  = 48.0f;   /* nominal stereo frames per USB frame */
static float                g_acc          = 0.0f;    /* fractional-sample accumulator */

static bool g_streaming = false;   /* AS alt setting 1 active */
static bool g_primed    = false;   /* FIFO has reached target at least once */

/* IN packet scratch (RAM). Only touched while no transfer is in flight. */
static uint8_t __attribute__((aligned(4))) g_pkt[USBRX_EP_IN_SIZE];

/* -------------------------------------------------------------------------- */
/* Rate-matching servo                                                         */
/* -------------------------------------------------------------------------- */
static void update_nominal_from_i2s(void)
{
    /* Use the measured frequency so the feed-forward includes the source's real
     * ppm offset; the servo then only has to absorb USB-vs-I2S frame jitter. */
    float actual = i2s_rx_get_rate_actual();
    if (actual < 1000.0f) {
        actual = (float) g_usb_sample_rate;   /* fallback before a measurement exists */
    }
    g_nominal_fpf = actual / 1000.0f;
}

static inline uint16_t silence_bytes(void)
{
    return (uint16_t)(g_silence_frames * USBRX_BYTES_PER_FRAME);
}

/*
 * Fill 'buf' with the next isochronous IN packet and return its size in bytes.
 * This is the rate-matching servo; it runs once per USB frame (each time the
 * previous IN packet completes).
 */
static uint16_t fill_audio_packet(uint8_t *buf)
{
    /* No valid I2S clocks/rate: emit silence at the host's cadence, stay unprimed. */
    if (!i2s_rx_locked()) {
        g_primed = false;
        g_acc    = 0.0f;
        uint16_t n = silence_bytes();
        memset(buf, 0, n);
        return n;
    }

    /* Refresh the feed-forward rate if the detected I2S rate changed; while
     * locked, also track the smoothed measurement. */
    uint32_t rate = i2s_rx_get_rate();
    if (rate != g_last_rate) {
        g_last_rate = rate;
        g_acc = 0.0f;
    }
    update_nominal_from_i2s();

    uint32_t avail = i2s_rx_frames_available();   /* stereo frames available */

    /* Prime: wait until the FIFO has filled to the target before streaming audio. */
    if (!g_primed) {
        if (avail >= (uint32_t) TARGET_FILL_FRAMES) {
            /* Start exactly at the target fill: audio that piled up while
             * acquiring would otherwise take the clamped servo a long, pitch-
             * shifted time to drain. */
            i2s_rx_trim(TARGET_FILL_FRAMES);
            avail    = TARGET_FILL_FRAMES;
            g_primed = true;
            g_acc    = 0.0f;
        } else {
            uint16_t n = silence_bytes();
            memset(buf, 0, n);
            return n;
        }
    }

    /* ---- Rate-matching servo: nominal feed-forward + clamped P correction ---- */
    int   err  = (int) avail - TARGET_FILL_FRAMES;
    float corr = SERVO_KP * (float) err;
    if (corr >  SERVO_MAX_CORR) corr =  SERVO_MAX_CORR;
    if (corr < -SERVO_MAX_CORR) corr = -SERVO_MAX_CORR;

    g_acc += g_nominal_fpf + corr;
    int n = (int) g_acc;           /* floor; g_acc is always >= 0 */
    g_acc -= (float) n;

    if (n < 0) n = 0;
    if (n > USBRX_MAX_FRAMES_PER_PACKET) n = USBRX_MAX_FRAMES_PER_PACKET;
    if ((uint32_t) n > avail) n = (int) avail;

    if (n == 0) {
        /* Real underrun: keep the stream alive and re-prime. */
        g_primed = false;
        uint16_t z = silence_bytes();
        memset(buf, 0, z);
        return z;
    }

    /* ---- Copy + convert n stereo frames to 24-bit LE ---- */
    uint32_t got = i2s_rx_read_s24le(buf, (uint32_t) n);
    return (uint16_t)(got * USBRX_BYTES_PER_FRAME);
}

/* ========================================================================== */
/* Custom UAC1 class driver (TinyUSB's built-in audio driver is UAC2-only)     */
/* ========================================================================== */

static struct {
    uint8_t        ac_itf;       /* AudioControl interface number */
    uint8_t        as_itf;       /* AudioStreaming interface number */
    uint8_t        cur_alt;      /* current AS alternate setting */
    bool           ep_open;      /* iso IN endpoint open */
    const uint8_t *ep_desc;      /* pointer to the iso IN endpoint descriptor */
    uint8_t        pending_cs;   /* deferred SET_CUR control selector */
    uint8_t        pending_len;  /* deferred SET_CUR data length */
} uac1;

static uint8_t uac1_ctrl_buf[8];  /* SET_CUR data-stage scratch */

/* Fill the scratch packet via the servo and queue it on the IN endpoint. */
static void uac1_arm_in(uint8_t rhport)
{
    uint16_t len = fill_audio_packet(g_pkt);
    usbd_edpt_xfer(rhport, AUDIO_IN_EP, g_pkt, len);
}

/* Apply an AudioStreaming alternate setting (0 = idle, 1 = streaming). */
static bool uac1_apply_alt(uint8_t rhport, uint8_t alt)
{
    if (alt > 1) return false;
    if (alt == uac1.cur_alt) return true;   /* idempotent SET_INTERFACE */
    uac1.cur_alt = alt;

    if (alt == 1) {
#ifdef TUP_DCD_EDPT_ISO_ALLOC
        TU_ASSERT(usbd_edpt_iso_activate(rhport, (tusb_desc_endpoint_t const *) uac1.ep_desc));
#else
        TU_ASSERT(usbd_edpt_open(rhport, (tusb_desc_endpoint_t const *) uac1.ep_desc));
#endif
        /* Clear any stale iso EP state (AVAIL/busy) left from a previous
         * alt cycle; stall+clear flushes both the stack and hardware flags. */
        usbd_edpt_stall(rhport, AUDIO_IN_EP);
        usbd_edpt_clear_stall(rhport, AUDIO_IN_EP);

        uac1.ep_open = true;
        g_streaming  = true;
        g_primed     = false;
        g_acc        = 0.0f;
        i2s_rx_flush();        /* start capture at "now" */

        uac1_arm_in(rhport);   /* queue the first packet */
    } else {
        if (uac1.ep_open) {
            usbd_edpt_close(rhport, AUDIO_IN_EP);
            uac1.ep_open = false;
        }
        g_streaming = false;
    }
    return true;
}

static void uac1_driver_init(void)
{
    memset(&uac1, 0, sizeof(uac1));
}

static bool uac1_driver_deinit(void)
{
    return true;
}

static void uac1_driver_reset(uint8_t rhport)
{
    (void) rhport;
    uac1.cur_alt = 0;
    uac1.ep_open = false;
    g_streaming  = false;
    g_primed     = false;
}

/* Claim the AC + AS interfaces and reserve the iso IN endpoint. */
static uint16_t uac1_driver_open(uint8_t rhport, tusb_desc_interface_t const *itf_desc, uint16_t max_len)
{
    TU_VERIFY(itf_desc->bInterfaceClass == TUSB_CLASS_AUDIO);
    TU_VERIFY(itf_desc->bInterfaceSubClass == AUDIO_SUBCLASS_CONTROL);
    TU_VERIFY(itf_desc->bAlternateSetting == 0);

    uac1.ac_itf = itf_desc->bInterfaceNumber;

    const uint8_t *p   = (const uint8_t *) itf_desc;
    const uint8_t *end = p + max_len;
    uint16_t drv_len = 0;

    /* AC standard interface descriptor */
    drv_len += tu_desc_len(p);
    p       += tu_desc_len(p);

    /* AC class-specific descriptors (header, terminals, ...) */
    while (p < end && tu_desc_type(p) == TUSB_DESC_CS_INTERFACE) {
        drv_len += tu_desc_len(p);
        p       += tu_desc_len(p);
    }

    /* AS interfaces (alt 0 + alt 1) and their class-specific + endpoint descriptors */
    while (p < end && tu_desc_type(p) == TUSB_DESC_INTERFACE) {
        tusb_desc_interface_t const *as = (tusb_desc_interface_t const *) p;
        if (as->bInterfaceClass != TUSB_CLASS_AUDIO ||
            as->bInterfaceSubClass != AUDIO_SUBCLASS_STREAMING) {
            break;
        }
        uac1.as_itf = as->bInterfaceNumber;

        drv_len += tu_desc_len(p);
        p       += tu_desc_len(p);

        while (p < end && tu_desc_type(p) != TUSB_DESC_INTERFACE) {
            if (tu_desc_type(p) == TUSB_DESC_ENDPOINT) {
                tusb_desc_endpoint_t const *ep = (tusb_desc_endpoint_t const *) p;
                if (ep->bmAttributes.xfer == TUSB_XFER_ISOCHRONOUS &&
                    tu_edpt_dir(ep->bEndpointAddress) == TUSB_DIR_IN) {
                    uac1.ep_desc = p;   /* remember for iso (de)activation */
#ifdef TUP_DCD_EDPT_ISO_ALLOC
                    usbd_edpt_iso_alloc(rhport, ep->bEndpointAddress, USBRX_EP_IN_SIZE);
#endif
                }
            }
            drv_len += tu_desc_len(p);
            p       += tu_desc_len(p);
        }
    }

    return drv_len;
}

static bool uac1_driver_control_xfer_cb(uint8_t rhport, uint8_t stage, tusb_control_request_t const *req)
{
    if (stage == CONTROL_STAGE_SETUP) {
        /* ----- Standard requests on our interfaces ----- */
        if (req->bmRequestType_bit.type == TUSB_REQ_TYPE_STANDARD) {
            uint8_t itf = TU_U16_LOW(req->wIndex);

            if (req->bRequest == TUSB_REQ_SET_INTERFACE) {
                uint8_t alt = TU_U16_LOW(req->wValue);
                if (itf == uac1.ac_itf) {
                    return (alt == 0) ? tud_control_status(rhport, req) : false;
                }
                if (itf == uac1.as_itf) {
                    return uac1_apply_alt(rhport, alt) ? tud_control_status(rhport, req) : false;
                }
                return false;
            }
            if (req->bRequest == TUSB_REQ_GET_INTERFACE) {
                static uint8_t alt_resp;
                if (itf == uac1.ac_itf)      alt_resp = 0;
                else if (itf == uac1.as_itf) alt_resp = uac1.cur_alt;
                else                         return false;
                return tud_control_xfer(rhport, req, &alt_resp, 1);
            }
            return false;
        }

        /* ----- Class requests (sampling-frequency control on the endpoint) ----- */
        if (req->bmRequestType_bit.type == TUSB_REQ_TYPE_CLASS &&
            req->bmRequestType_bit.recipient == TUSB_REQ_RCPT_ENDPOINT) {

            if (TU_U16_LOW(req->wIndex) != AUDIO_IN_EP) return false;
            uint8_t cs = TU_U16_HIGH(req->wValue);

            if (req->bmRequestType_bit.direction == TUSB_DIR_IN) {
                /* GET_CUR sampling frequency */
                if (req->bRequest == UAC1_REQ_GET_CUR && cs == AUDIO_CS_CTRL_SAM_FREQ) {
                    static uint8_t freq[3];
                    uint32_t f = g_usb_sample_rate;
                    freq[0] = (uint8_t)(f & 0xFF);
                    freq[1] = (uint8_t)((f >> 8) & 0xFF);
                    freq[2] = (uint8_t)((f >> 16) & 0xFF);
                    return tud_control_xfer(rhport, req, freq, 3);
                }
                return false;
            } else {
                /* SET_CUR sampling frequency: defer to the data stage */
                if (req->bRequest == UAC1_REQ_SET_CUR && cs == AUDIO_CS_CTRL_SAM_FREQ) {
                    uint16_t len = req->wLength;
                    if (len == 0 || len > sizeof(uac1_ctrl_buf)) return false;
                    uac1.pending_cs  = cs;
                    uac1.pending_len = (uint8_t) len;
                    return tud_control_xfer(rhport, req, uac1_ctrl_buf, len);
                }
                return false;
            }
        }
        return false;
    }

    if (stage == CONTROL_STAGE_DATA) {
        if (req->bmRequestType_bit.type == TUSB_REQ_TYPE_CLASS &&
            uac1.pending_cs == AUDIO_CS_CTRL_SAM_FREQ) {
            uint32_t f = (uint32_t) uac1_ctrl_buf[0]
                       | ((uint32_t) uac1_ctrl_buf[1] << 8)
                       | ((uint32_t) uac1_ctrl_buf[2] << 16);
            if (f == 44100 || f == 48000 || f == 88200 || f == 96000) {
                g_usb_sample_rate = f;
                g_silence_frames  = f / 1000;
            }
            uac1.pending_cs = 0;
        }
        return true;
    }

    return true;   /* CONTROL_STAGE_ACK */
}

static bool uac1_driver_xfer_cb(uint8_t rhport, uint8_t ep_addr, xfer_result_t result, uint32_t xferred_bytes)
{
    (void) result;
    (void) xferred_bytes;
    if (ep_addr == AUDIO_IN_EP) {
        if (uac1.ep_open) uac1_arm_in(rhport);   /* refill + queue next frame */
        return true;
    }
    return false;
}

static const usbd_class_driver_t uac1_driver = {
    .name            = "usbrx_uac1",
    .init            = uac1_driver_init,
    .deinit          = uac1_driver_deinit,
    .reset           = uac1_driver_reset,
    .open            = uac1_driver_open,
    .control_xfer_cb = uac1_driver_control_xfer_cb,
    .xfer_cb         = uac1_driver_xfer_cb,
    .sof             = NULL,
};

/* TinyUSB calls this weak symbol at tud_init() to discover app class drivers. */
usbd_class_driver_t const *usbd_app_driver_get_cb(uint8_t *driver_count)
{
    *driver_count = 1;
    return &uac1_driver;
}

/* -------------------------------------------------------------------------- */
/* Onboard-LED status indicator (diagnostic; no-op if board has no LED pin)     */
/*   fast blink (100 ms) : running, USB NOT enumerated                          */
/*   slow blink (500 ms) : enumerated/mounted, not streaming                    */
/*   solid on            : host is recording                                   */
/* -------------------------------------------------------------------------- */
static inline void led_init(void)
{
#ifdef PICO_DEFAULT_LED_PIN
    gpio_init(PICO_DEFAULT_LED_PIN);
    gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);
#endif
}

/* -------------------------------------------------------------------------- */
/* GPIO output status indicators                                              */
/* one each for locked to 44.1, 48, 88.2 and 96kHz I2S input                  */
/* -------------------------------------------------------------------------- */
static inline void usbrx_gpio_init(void)
{
    gpio_init(LOCK_STATUS_44100_PIN);
    gpio_init(LOCK_STATUS_48000_PIN);
    gpio_init(LOCK_STATUS_88200_PIN);
    gpio_init(LOCK_STATUS_96000_PIN);

    gpio_set_dir(LOCK_STATUS_44100_PIN, GPIO_OUT);
    gpio_set_dir(LOCK_STATUS_48000_PIN, GPIO_OUT);
    gpio_set_dir(LOCK_STATUS_88200_PIN, GPIO_OUT);
    gpio_set_dir(LOCK_STATUS_96000_PIN, GPIO_OUT);
}

static inline void led_set(bool on)
{
#ifdef PICO_DEFAULT_LED_PIN
    gpio_put(PICO_DEFAULT_LED_PIN, on);
#else
    (void) on;
#endif
}

static void led_task(void)
{
    static uint32_t last = 0;
    static bool     on   = false;

    if (g_streaming) {
        led_set(true);
        on = true;
        return;
    }

    uint32_t now      = to_ms_since_boot(get_absolute_time());
    uint32_t interval = tud_mounted() ? 500 : 100;
    if (now - last >= interval) {
        last = now;
        on = !on;
        led_set(on);
    }
}

static void gpio_task(void)
{
    if (i2s_rx_locked())
    {
        uint32_t r = i2s_rx_get_rate();
        gpio_put(LOCK_STATUS_44100_PIN, r == 44100 ? 1 : 0);
        gpio_put(LOCK_STATUS_48000_PIN, r == 48000 ? 1 : 0);
        gpio_put(LOCK_STATUS_88200_PIN, r == 88200 ? 1 : 0);
        gpio_put(LOCK_STATUS_96000_PIN, r == 96000 ? 1 : 0);
    }
    else
    {
        gpio_put(LOCK_STATUS_44100_PIN, 0);
        gpio_put(LOCK_STATUS_48000_PIN, 0);
        gpio_put(LOCK_STATUS_88200_PIN, 0);
        gpio_put(LOCK_STATUS_96000_PIN, 0);
    }
}

/* -------------------------------------------------------------------------- */
/* main                                                                        */
/* -------------------------------------------------------------------------- */
int main(void)
{
    /* Default clocks are fine: the receiver is wait-driven at divider 1.0
     * (>= 125 MHz sys gives ~20 PIO cycles per BCK period even at 96 kHz). */
    stdio_init_all();
    led_init();
    usbrx_gpio_init();

    /* Pico SMPS into PWM mode for lower audio noise (harmless on other boards). */
    gpio_init(PIN_DCDC_PSM_CTRL);
    gpio_set_dir(PIN_DCDC_PSM_CTRL, GPIO_OUT);
    gpio_put(PIN_DCDC_PSM_CTRL, 1);

    printf("\nI2S -> USB UAC1 recorder starting\n");

    /* Start the USB device stack FIRST so the device always enumerates,
     * independent of whether I2S clocks are present. */
    tusb_init();

    /* Start the I2S slave receiver (waits for the master's clocks). */
    i2s_rx_init(I2S_DATA_PIN, I2S_BCK_PIN, I2S_LRCLK_PIN);

    i2s_rx_state_t last_state = I2S_RX_NO_CLOCK;
    while (true) {
        tud_task();
        i2s_rx_poll();
        led_task();
        gpio_task();

        i2s_rx_state_t st = i2s_rx_get_state();
        if (st != last_state) {
            if (st == I2S_RX_LOCKED) {
                printf("I2S locked: %lu Hz (slips %lu)\n",
                       (unsigned long) i2s_rx_get_rate(),
                       (unsigned long) i2s_rx_get_slip_count());
            } else if (st == I2S_RX_NO_CLOCK) {
                printf("I2S clocks lost\n");
            }
            last_state = st;
        }
    }
    return 0;
}
