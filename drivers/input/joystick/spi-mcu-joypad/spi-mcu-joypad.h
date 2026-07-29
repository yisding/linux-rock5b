/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * SPI MCU Joypad driver
 *
 * Analog joystick + RGB LED controller behind an SPI-connected MCU.
 * A single 64-byte full-duplex transfer carries joystick state (RX)
 * and LED configuration (TX) simultaneously.
 */

#ifndef _SPI_MCU_JOYPAD_H
#define _SPI_MCU_JOYPAD_H

#include <linux/delay.h>
#include <linux/input.h>
#include <linux/led-class-multicolor.h>
#include <linux/mutex.h>
#include <linux/spi/spi.h>
#include <linux/workqueue.h>

/* Protocol constants derived from MCU communication analysis. */
#define MCU_BUF_LEN		0x40	/* 64-byte SPI transfer */
#define MCU_REPLY_OFFSET	0x20	/* RX offset where "Z:" reply starts */
#define MCU_REPLY_LEN		0x10	/* 16-byte reply window */
#define MCU_PAYLOAD_BYTES	12		/* 6 x le16 axis values */

/* TX buffer offsets for LED control bytes. */
#define MCU_TX_LED_MODE		0x07
#define MCU_TX_LED_SWITCH		0x08
#define MCU_TX_LED_LEVEL		0x09
#define MCU_TX_LED_SPEED		0x0A
#define MCU_TX_LED_RGB_BASE	0x0B	/* First RGB triplet */
#define MCU_TX_LED_SEQ			0x3C	/* Sequence counter (in CRC range) */
#define MCU_TX_LED_CRC			0x3D	/* CRC-16/XMODEM, 2 bytes big-endian */

/* CRC-16/XMODEM covers tx[0x07..0x3C] inclusive (54 bytes). */
#define MCU_TX_CRC_START	0x07
#define MCU_TX_CRC_LEN		54

/*
 * 16 TX RGB triplet slots: 8 per joystick ring.
 * Slots 0-7 = left stick, slots 8-15 = right stick.
 */
#define MCU_LED_TX_SLOTS	16	/* Total TX triplet slots */
#define MCU_LED_COLORS		3	/* R, G, B per slot */
#define MCU_LEDS_PER_RING	8	/* Physical LEDs per stick */
#define MCU_LED_RINGS		2	/* Left + right stick */

/* Number of analog axes reported by the MCU. */
#define MCU_NUM_AXES		6

/* Default polling interval in ms (~200 Hz). */
#define MCU_DEFAULT_POLL_MS	5

/*
 * Calibrated signed output range, centered at 0.
 */
#define MCU_AXIS_MIN		(-32767)
#define MCU_AXIS_MAX		32767

/*
 * Raw axis range after the MCU's << 3 expansion.
 * ~13-bit ADC shifted left 3 gives 0–65528 in practice; we use 0–65535
 * as the default min/max since the calibration will handle the actual
 * center offset.
 */
#define MCU_RAW_MIN		0
#define MCU_RAW_MAX		65535

/* Auto-calibration: number of SPI frames averaged to find stick center. */
#define MCU_CAL_FRAMES		50

/* Default radial deadzone (in calibrated axis units, 0–32767 scale). */
#define MCU_DEADZONE_DEFAULT	3000

/*
 * Per-stick calibration data.
 * x/y_zero is auto-calibrated at probe; min/max come from DT or defaults.
 */
struct mcu_stick_cal {
	s32	x_min, x_max, x_zero;
	s32	y_min, y_max, y_zero;
	bool	has_dt_range;
};

struct mcu_joypad {
	struct spi_device	*spi;
	struct device		*dev;

	/* Optional MCU power/enable GPIO. */
	struct gpio_desc	*enable_gpio;

	/* SPI transaction buffers. */
	u8	tx_buf[MCU_BUF_LEN] ____cacheline_aligned;
	u8	rx_buf[MCU_BUF_LEN] ____cacheline_aligned;

	/*
	 * Protects tx_buf from concurrent writes (LED brightness_set vs poll).
	 * spi_sync sleeps, so a mutex is correct here.
	 */
	struct mutex		tx_lock;

	/* Core polling that runs independently of input open/close. */
	struct delayed_work	poll_work;
	unsigned int		poll_interval_ms;

	/* Input subsystem state. */
	struct input_dev	*input;
	bool			input_opened;

	/* Stick calibration with center auto-detection, range from DT. */
	struct mcu_stick_cal	left_cal;
	struct mcu_stick_cal	right_cal;
	s32			trig_l_zero;
	s32			trig_r_zero;
	bool			calibrated;

	/* Radial deadzone per stick pair and linear deadzone for triggers. */
	u32			deadzone_left;
	u32			deadzone_right;
	u32			deadzone_triggers;

	/* LED subsystem state — two rings (left/right stick). */
	struct led_classdev_mc	mc_leds[MCU_LED_RINGS];
	struct mc_subled	subled_info[MCU_LED_RINGS][MCU_LED_COLORS];
	u8			led_mode;
	u8			led_switch;
	u8			led_speed;
	u8			led_seq;	/* TX sequence counter */
	bool			led_dirty;
};

int mcu_parse_rx(const u8 *rx, u16 axes[MCU_NUM_AXES]);
void mcu_joypad_encode_leds(struct mcu_joypad *mjp);

int mcu_joypad_input_register(struct mcu_joypad *mjp);
void mcu_joypad_input_report(struct mcu_joypad *mjp, const u16 raw[MCU_NUM_AXES]);
int mcu_joypad_calibrate(struct mcu_joypad *mjp);

int mcu_joypad_leds_register(struct mcu_joypad *mjp);
void mcu_joypad_leds_unregister(struct mcu_joypad *mjp);
void mcu_joypad_leds_suspend(struct mcu_joypad *mjp);

#endif /* _SPI_MCU_JOYPAD_H */
