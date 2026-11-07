/* SPDX-License-Identifier: GPL-3.0-or-later */
/*
 * I2S slave receiver for USBrx (I2S build).  See i2s_rx.h.
 *
 * Ring capture mirrors DSPi's i2s_input.c: one PIO state machine pushes one
 * 32-bit word per channel (audio in bits [31:8]); an IRQ-less self-retriggering
 * DMA pair copies the RX FIFO into a power-of-two ring forever.  Because every
 * (re)start begins with a LEFT word at the ring base, the ring holds an even
 * number of words, and the PIO program keeps strict L,R alternation even
 * through slips, a LEFT word always sits at an even index.  The software read
 * index is therefore always kept even.
 */

#include <string.h>
#include <stdio.h>

#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "hardware/dma.h"
#include "hardware/gpio.h"

#include "i2s_rx.h"
#include "i2s_rx.pio.h"

/* -------------------------------------------------------------------------- */
/* Resources                                                                   */
/* -------------------------------------------------------------------------- */
#define RX_PIO          pio0
#define RX_SM           0u
#define RX_SLIP_IRQ     7u       /* raised by the checked program on a slip */

/* 4096 words = 2048 stereo frames: ~21 ms @ 96 kHz, ~46 ms @ 44.1 kHz. */
#define RING_WORDS      4096u
#define RING_BITS       14u      /* log2(ring bytes) for the DMA address wrap */
#define RING_BYTES      (RING_WORDS * 4u)

/* Keep at most this much buffered when nobody is draining (host not
 * recording): older audio is dropped so the writer never laps the reader. */
#define RING_HIGH_WATER_WORDS  (RING_WORDS * 3u / 4u)
#define RING_KEEP_WORDS        (512u * 2u)

static uint32_t __attribute__((aligned(RING_BYTES))) ring[RING_WORDS];
static uintptr_t ring_base;      /* stable source word for the reload channel */

static uint     dma_data;
static uint     dma_reload;
static int      prog_offset = -1;
static uint8_t  pin_data, pin_bck, pin_lrclk;

/* Patched RAM copy of the checked slave program (BCK/LRCLK pins are runtime). */
#define PROG_LEN (sizeof(audio_i2s_rx_slave_checked_program_instructions) / sizeof(uint16_t))
static uint16_t prog_ram[PROG_LEN];
static struct pio_program prog = { .instructions = prog_ram, .length = 0, .origin = -1 };

static uint32_t rd_word;         /* software read index (always even) */

/* -------------------------------------------------------------------------- */
/* Lock / rate measurement                                                     */
/* -------------------------------------------------------------------------- */
#define WINDOW_US            32000u   /* rate measurement window */
#define CLOCK_TIMEOUT_US     5000u    /* no words this long = clocks gone */
#define LOCK_WINDOWS         2        /* agreeing windows needed to lock */
#define POLL_GAP_US          10000u   /* a longer main-loop gap may have lapped the ring */
#define SNAP_TOLERANCE       0.005f   /* +/-0.5 % around a nominal rate */

static i2s_rx_state_t state = I2S_RX_NO_CLOCK;
static uint32_t locked_rate;
static float    rate_ema;

static uint32_t last_wr;
static uint64_t last_advance_us;
static uint64_t last_poll_us;
static uint64_t win_start_us;
static uint64_t win_words;
static uint32_t candidate;
static uint8_t  agree;

static uint32_t slip_count;
static uint32_t overflow_count;

/* -------------------------------------------------------------------------- */
/* Helpers                                                                     */
/* -------------------------------------------------------------------------- */
static inline uint32_t write_word(void) {
    return (uint32_t)((dma_hw->ch[dma_data].write_addr - (uint32_t)ring_base) / 4u) % RING_WORDS;
}

static inline uint32_t words_available(void) {
    return (write_word() - rd_word) % RING_WORDS;
}

/* Patch the `wait gpio` instructions: placeholder 0 = BCK, 1 = LRCLK. */
static void load_program(void) {
    memcpy(prog_ram, audio_i2s_rx_slave_checked_program_instructions, sizeof(prog_ram));
    for (uint i = 0; i < PROG_LEN; i++) {
        uint16_t instr = prog_ram[i];
        if ((instr >> 13) != 0x1u) continue;          /* not a WAIT */
        if (((instr >> 5) & 0x3u) != 0u) continue;    /* WAIT source not GPIO */
        uint8_t pin = (instr & 0x1Fu) ? pin_lrclk : pin_bck;
        prog_ram[i] = (uint16_t)((instr & ~0x1Fu) | (pin & 0x1Fu));
    }
    prog.length = (uint8_t)PROG_LEN;
    prog_offset = pio_add_program(RX_PIO, &prog);
}

static void start_dma_ring(void) {
    /* Reload channel: rewrites the data channel's write address (trigger
     * alias) with the ring base each time the data channel completes. */
    dma_channel_config cb = dma_channel_get_default_config(dma_reload);
    channel_config_set_transfer_data_size(&cb, DMA_SIZE_32);
    channel_config_set_read_increment(&cb, false);
    channel_config_set_write_increment(&cb, false);
    channel_config_set_chain_to(&cb, dma_reload);
    dma_channel_configure(dma_reload, &cb,
                          &dma_hw->ch[dma_data].al2_write_addr_trig,
                          &ring_base, 1, false);

    /* Data channel: PIO RX FIFO -> ring with write-address wrap. */
    dma_channel_config ca = dma_channel_get_default_config(dma_data);
    channel_config_set_transfer_data_size(&ca, DMA_SIZE_32);
    channel_config_set_read_increment(&ca, false);
    channel_config_set_write_increment(&ca, true);
    channel_config_set_ring(&ca, true, RING_BITS);
    channel_config_set_dreq(&ca, pio_get_dreq(RX_PIO, RX_SM, false));
    channel_config_set_chain_to(&ca, dma_reload);
    dma_channel_configure(dma_data, &ca, ring, &RX_PIO->rxf[RX_SM], RING_WORDS, true);
}

/* Race-free teardown of the self-retriggering pair (see DSPi
 * stop_all_dma_rings): disarm the chain via the non-triggering CTRL alias,
 * then abort both channels in one write. */
static void stop_dma_ring(void) {
    hw_write_masked(&dma_hw->ch[dma_data].al1_ctrl,
                    (uint32_t)dma_data << DMA_CH0_CTRL_TRIG_CHAIN_TO_LSB,
                    DMA_CH0_CTRL_TRIG_CHAIN_TO_BITS);
    dma_hw->abort = (1u << dma_data) | (1u << dma_reload);
    for (uint32_t guard = 1000000u; guard; guard--) {
        if (!(dma_hw->ch[dma_data].ctrl_trig & DMA_CH0_CTRL_TRIG_BUSY_BITS) &&
            !(dma_hw->ch[dma_reload].ctrl_trig & DMA_CH0_CTRL_TRIG_BUSY_BITS)) break;
        tight_loop_contents();
    }
}

static void reset_measurement(uint64_t now) {
    last_wr         = write_word();
    last_advance_us = now;
    win_start_us    = now;
    win_words       = 0;
    candidate       = 0;
    agree           = 0;
}

/* (Re)start the receiver from a clean state: first word at ring[0] = LEFT. */
static void rx_restart(void) {
    pio_sm_set_enabled(RX_PIO, RX_SM, false);
    stop_dma_ring();

    audio_i2s_rx_slave_checked_program_init(RX_PIO, RX_SM, (uint)prog_offset,
                                            pin_data, pin_lrclk);
    pio_sm_set_clkdiv_int_frac(RX_PIO, RX_SM, 1, 0);   /* wait-driven */
    pio_interrupt_clear(RX_PIO, RX_SLIP_IRQ);

    rd_word = 0;
    start_dma_ring();
    pio_sm_set_enabled(RX_PIO, RX_SM, true);

    reset_measurement(time_us_64());
}

static uint32_t snap_rate(float hz) {
    static const uint32_t rates[] = { 44100u, 48000u, 88200u, 96000u };
    for (unsigned i = 0; i < sizeof(rates) / sizeof(rates[0]); i++) {
        float r = (float)rates[i];
        if (hz > r * (1.0f - SNAP_TOLERANCE) && hz < r * (1.0f + SNAP_TOLERANCE)) return rates[i];
    }
    return 0;
}

static void set_state(i2s_rx_state_t s) {
    if (s == state) return;
    state = s;
    if (s != I2S_RX_LOCKED) locked_rate = 0;
}

/* -------------------------------------------------------------------------- */
/* Public API                                                                  */
/* -------------------------------------------------------------------------- */
void i2s_rx_init(uint8_t data_pin, uint8_t bck_pin, uint8_t lrclk_pin) {
    pin_data  = data_pin;
    pin_bck   = bck_pin;
    pin_lrclk = lrclk_pin;
    ring_base = (uintptr_t)ring;

    /* Clock pads: plain inputs (SIO function, also clears the RP2350 pad
     * isolation latch); `wait gpio` / `jmp pin` read the pad directly. */
    gpio_init(pin_bck);
    gpio_init(pin_lrclk);
    gpio_set_input_enabled(pin_bck, true);
    gpio_set_input_enabled(pin_lrclk, true);
    /* Data pad: PIO input. */
    pio_gpio_init(RX_PIO, pin_data);

    pio_sm_claim(RX_PIO, RX_SM);
    dma_data   = (uint)dma_claim_unused_channel(true);
    dma_reload = (uint)dma_claim_unused_channel(true);

    load_program();
    rx_restart();
    last_poll_us = time_us_64();

    printf("I2S RX: data GP%u, BCK GP%u, LRCLK GP%u (slave)\n",
           pin_data, pin_bck, pin_lrclk);
}

void i2s_rx_poll(void) {
    uint64_t now = time_us_64();

    /* Framing slip flagged by the PIO program: the master restarted or
     * glitched.  Restart cleanly; the rate is re-measured. */
    if (pio_interrupt_get(RX_PIO, RX_SLIP_IRQ)) {
        slip_count++;
        rx_restart();
        set_state(I2S_RX_ACQUIRING);
        last_poll_us = now;
        return;
    }

    /* A long main-loop stall could have lapped the ring between polls:
     * re-anchor the measurement instead of measuring through it. */
    if (now - last_poll_us > POLL_GAP_US) {
        reset_measurement(now);
    }
    last_poll_us = now;

    uint32_t wr  = write_word();
    uint32_t adv = (wr - last_wr) % RING_WORDS;
    last_wr = wr;

    if (adv) {
        if (state == I2S_RX_NO_CLOCK) {
            /* Clocks are back.  The SM may have stalled mid-word when they
             * stopped, so re-frame from scratch. */
            rx_restart();
            set_state(I2S_RX_ACQUIRING);
            return;
        }
        last_advance_us = now;
        win_words += adv;
    } else if (now - last_advance_us > CLOCK_TIMEOUT_US) {
        if (state != I2S_RX_NO_CLOCK) printf("I2S RX: clocks lost\n");
        set_state(I2S_RX_NO_CLOCK);
        rate_ema = 0.0f;
        return;
    }

    /* Drop old audio when nobody drains the ring (host not recording), so
     * the DMA writer never laps the read index.  Keep the newest audio. */
    uint32_t avail = (wr - rd_word) % RING_WORDS;
    if (avail > RING_HIGH_WATER_WORDS) {
        overflow_count++;
        rd_word = (wr - RING_KEEP_WORDS) % RING_WORDS & ~1u;
    }

    /* Rate measurement window. */
    uint64_t dt = now - win_start_us;
    if (dt >= WINDOW_US) {
        float hz = (float)win_words * 0.5f * 1e6f / (float)dt;
        win_start_us = now;
        win_words    = 0;

        uint32_t snap = snap_rate(hz);
        if (snap && snap == candidate) {
            if (agree < 255) agree++;
        } else {
            candidate = snap;
            agree     = snap ? 1 : 0;
        }

        if (state == I2S_RX_LOCKED) {
            if (snap != locked_rate) {
                printf("I2S RX: rate change / unstable (%.1f Hz)\n", (double)hz);
                set_state(I2S_RX_ACQUIRING);
            } else {
                rate_ema += 0.05f * (hz - rate_ema);
            }
        } else if (agree >= LOCK_WINDOWS) {
            locked_rate = candidate;
            rate_ema    = hz;
            state       = I2S_RX_LOCKED;
            printf("I2S RX: locked %lu Hz (%.1f Hz measured)\n",
                   (unsigned long)locked_rate, (double)hz);
        } else {
            set_state(I2S_RX_ACQUIRING);
        }
    }
}

i2s_rx_state_t i2s_rx_get_state(void) { return state; }
uint32_t i2s_rx_get_rate(void)        { return locked_rate; }
float    i2s_rx_get_rate_actual(void) { return rate_ema; }
uint32_t i2s_rx_get_slip_count(void)     { return slip_count; }
uint32_t i2s_rx_get_overflow_count(void) { return overflow_count; }

uint32_t i2s_rx_frames_available(void) {
    return words_available() / 2u;
}

void i2s_rx_flush(void) {
    rd_word = write_word() & ~1u;
}

void i2s_rx_trim(uint32_t keep_frames) {
    uint32_t avail = i2s_rx_frames_available();
    if (avail > keep_frames)
        rd_word = (rd_word + (avail - keep_frames) * 2u) % RING_WORDS;
}

uint32_t i2s_rx_read_s24le(uint8_t *dst, uint32_t n) {
    uint32_t avail = i2s_rx_frames_available();
    if (n > avail) n = avail;
    uint32_t idx = rd_word;
    for (uint32_t i = 0; i < n * 2u; i++) {
        /* 24-bit audio in bits [31:8]. */
        uint32_t s = ring[idx] >> 8;
        idx = (idx + 1u) % RING_WORDS;
        *dst++ = (uint8_t)(s & 0xFF);
        *dst++ = (uint8_t)((s >> 8) & 0xFF);
        *dst++ = (uint8_t)((s >> 16) & 0xFF);
    }
    rd_word = idx;
    return n;
}
