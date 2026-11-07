/* SPDX-License-Identifier: GPL-3.0-or-later */
/*
 * I2S slave receiver for USBrx (I2S build).
 *
 *   ADC data pin ──[PIO: audio_i2s_rx_slave_checked]──[IRQ-less DMA ring]──► ring
 *
 * BCK/LRCLK are inputs driven by an external I2S master (normally DSPi), so
 * the recorder runs on the master's clock.  The sample rate is measured from
 * how fast the DMA ring fills (2 words per LRCLK frame) and snapped to
 * 44.1/48/88.2/96 kHz.  Framing slips (master restarts / glitches) are flagged
 * by the PIO program and handled by restarting the receiver.
 *
 * All functions are main-loop context (tud_task runs on the same thread).
 */
#ifndef I2S_RX_H
#define I2S_RX_H

#include <stdbool.h>
#include <stdint.h>

typedef enum {
    I2S_RX_NO_CLOCK  = 0,   /* no BCK/LRCLK activity */
    I2S_RX_ACQUIRING = 1,   /* clocks present, measuring the rate */
    I2S_RX_LOCKED    = 2,   /* rate known and stable; audio valid */
} i2s_rx_state_t;

/* Claim PIO/DMA, configure pins and start receiving. */
void i2s_rx_init(uint8_t data_pin, uint8_t bck_pin, uint8_t lrclk_pin);

/* Main-loop service: rate measurement, lock state, slip/overflow handling.
 * Call every loop iteration (must run more often than the ring fill time,
 * ~21 ms at 96 kHz). */
void i2s_rx_poll(void);

i2s_rx_state_t i2s_rx_get_state(void);
static inline bool i2s_rx_locked(void) { return i2s_rx_get_state() == I2S_RX_LOCKED; }

/* Snapped rate in Hz (0 unless LOCKED). */
uint32_t i2s_rx_get_rate(void);
/* Smoothed measured rate in Hz (servo feed-forward; 0 if unknown). */
float i2s_rx_get_rate_actual(void);

/* Stereo frames waiting in the ring. */
uint32_t i2s_rx_frames_available(void);
/* Discard everything buffered (stream (re)start). */
void i2s_rx_flush(void);
/* Discard the oldest frames so at most keep_frames remain. */
void i2s_rx_trim(uint32_t keep_frames);
/* Read up to n stereo frames as 24-bit little-endian L,R (6 bytes/frame).
 * Returns frames read. */
uint32_t i2s_rx_read_s24le(uint8_t *dst, uint32_t n);

/* Diagnostics (cumulative since boot). */
uint32_t i2s_rx_get_slip_count(void);
uint32_t i2s_rx_get_overflow_count(void);

#endif /* I2S_RX_H */
