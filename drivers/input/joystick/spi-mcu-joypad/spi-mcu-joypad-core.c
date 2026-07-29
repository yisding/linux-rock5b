// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * SPI MCU Joypad — core driver
 *
 * Manages the SPI transport, periodic polling via delayed_work, and
 * RX parsing.  Delegates input event reporting and LED control to
 * companion translation units compiled into the same module.
 *
 */

#include <linux/gpio/consumer.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/pm.h>
#include <linux/unaligned.h>

#include "spi-mcu-joypad.h"

/*
 * The MCU expects this ASCII header at the start of every TX frame.
 * Derived from tracing the original kernel module's probe sequence.
 */
static const u8 mcu_base_cmd[] = { 'Z', ':', 'f', 'f', 0x00, '\r', '\n' };

static void mcu_init_tx(struct mcu_joypad *mjp)
{
	memset(mjp->tx_buf, 0, MCU_BUF_LEN);
	memcpy(mjp->tx_buf, mcu_base_cmd, sizeof(mcu_base_cmd));
}

/*
 * Brightness is expected as pre-scaled RGB values by the MCU.
 */
static inline u8 scale_component(u8 val, u8 level)
{
	return (u8)(((u16)val * (u16)level) / 255u);
}

/*
 * CRC-16/XMODEM — polynomial 0x1021, init 0x0000, no reflection/XOR.
 * The MCU validates this before applying LED commands.
 */
static u16 mcu_crc16_xmodem(const u8 *data, size_t len)
{
	u16 crc = 0x0000;
	size_t i;
	int j;

	for (i = 0; i < len; i++) {
		crc ^= (u16)data[i] << 8;
		for (j = 0; j < 8; j++) {
			if (crc & 0x8000)
				crc = (crc << 1) ^ 0x1021;
			else
				crc <<= 1;
		}
	}
	return crc;
}

/*
 * Encode current LED state into tx_buf and stamp the CRC.
 * Caller must hold tx_lock.
 * Each ring (left/right stick) is exposed as a single multicolor LED.
 */
void mcu_joypad_encode_leds(struct mcu_joypad *mjp)
{
	u8 level;
	u16 crc;
	int ring, slot;

	mjp->tx_buf[MCU_TX_LED_MODE]   = mjp->led_mode;
	mjp->tx_buf[MCU_TX_LED_SWITCH] = mjp->led_switch;
	mjp->tx_buf[MCU_TX_LED_SPEED]  = mjp->led_speed;

	level = mjp->mc_leds[0].led_cdev.brightness;
	mjp->tx_buf[MCU_TX_LED_LEVEL] = level;

	for (ring = 0; ring < MCU_LED_RINGS; ring++) {
		struct mc_subled *sub = mjp->mc_leds[ring].subled_info;
		u8 r = scale_component(sub[0].intensity, level);
		u8 g = scale_component(sub[1].intensity, level);
		u8 b = scale_component(sub[2].intensity, level);

		for (slot = 0; slot < MCU_LEDS_PER_RING; slot++) {
			unsigned int off = MCU_TX_LED_RGB_BASE +
				((ring * MCU_LEDS_PER_RING + slot) *
				 MCU_LED_COLORS);

			mjp->tx_buf[off + 0] = r;
			mjp->tx_buf[off + 1] = g;
			mjp->tx_buf[off + 2] = b;
		}
	}

	/* Stamp sequence counter and CRC-16/XMODEM. */
	mjp->tx_buf[MCU_TX_LED_SEQ] = mjp->led_seq++;
	crc = mcu_crc16_xmodem(&mjp->tx_buf[MCU_TX_CRC_START],
			       MCU_TX_CRC_LEN);
	mjp->tx_buf[MCU_TX_LED_CRC]     = crc >> 8;
	mjp->tx_buf[MCU_TX_LED_CRC + 1] = crc & 0xff;

	mjp->led_dirty = false;
}

/* ---- RX parsing --------------------------------------------------------- */

/*
 * Validate the MCU reply at rx_buf + MCU_REPLY_OFFSET before
 * extracting axis values.
 * 
 * Axis output order (after the non-linear remapping from the MCU):
 *   axes[0] = lx, axes[1] = ly, axes[2] = rx,
 *   axes[3] = ry, axes[4] = lt, axes[5] = rt
 *
 * Each value is left-shifted by 3 to expand the MCU's ~13-bit ADC
 * samples into a 16-bit range.
 */
int mcu_parse_rx(const u8 *rx, u16 axes[MCU_NUM_AXES])
{
	const u8 *r = rx + MCU_REPLY_OFFSET;
	u16 sum = 0;
	u16 raw[6];
	int i;

	/* MCU header */
	if (r[0] != 'Z' || r[1] != ':')
		return -EBADMSG;

	/* Checksum over the 12 payload bytes to validate. */
	for (i = 2; i <= 13; i++)
		sum += r[i];

	if ((sum & 0xff) != r[14] || (sum >> 8) != r[15])
		return -EBADMSG;

	/* Extract 6 little-endian u16 values from the payload. */
	for (i = 0; i < 6; i++)
		raw[i] = get_unaligned_le16(&r[2 + i * 2]);

	/*
	 * Non-linear remapping: the MCU sends raw0/raw1 for the physical
	 * right stick and raw2/raw3 for the physical left stick, with
	 * X and Y swapped within each pair.  Left-shift by 3 to expand
	 * ADC range.
	 */
	axes[0] = raw[3] << 3;		/* lx (physical left stick) */
	axes[1] = raw[2] << 3;		/* ly */
	axes[2] = raw[1] << 3;		/* rx (physical right stick) */
	axes[3] = raw[0] << 3;		/* ry */
	axes[4] = raw[4] << 3;		/* lt */
	axes[5] = raw[5] << 3;		/* rt */

	return 0;
}

static void mcu_joypad_poll(struct work_struct *work)
{
	struct mcu_joypad *mjp = container_of(work, struct mcu_joypad,
					      poll_work.work);
	struct spi_transfer xfer = {
		.tx_buf = mjp->tx_buf,
		.rx_buf = mjp->rx_buf,
		.len    = MCU_BUF_LEN,
	};
	u16 axes[MCU_NUM_AXES];
	int err;

	mutex_lock(&mjp->tx_lock);
	if (mjp->led_dirty)
		mcu_joypad_encode_leds(mjp);
	mutex_unlock(&mjp->tx_lock);

	err = spi_sync_transfer(mjp->spi, &xfer, 1);
	if (err) {
		dev_err_ratelimited(mjp->dev, "SPI transfer failed: %d\n", err);
		goto reschedule;
	}

	err = mcu_parse_rx(mjp->rx_buf, axes);
	if (err) {
		dev_dbg_ratelimited(mjp->dev, "RX parse failed (bad header/checksum)\n");
		goto reschedule;
	}

	if (mjp->input_opened)
		mcu_joypad_input_report(mjp, axes);

reschedule:
	schedule_delayed_work(&mjp->poll_work,
			      msecs_to_jiffies(mjp->poll_interval_ms));
}

static int mcu_joypad_probe(struct spi_device *spi)
{
	struct mcu_joypad *mjp;
	u32 val;
	int err;

	mjp = devm_kzalloc(&spi->dev, sizeof(*mjp), GFP_KERNEL);
	if (!mjp)
		return -ENOMEM;

	mjp->spi = spi;
	mjp->dev = &spi->dev;
	spi_set_drvdata(spi, mjp);

	mutex_init(&mjp->tx_lock);
	INIT_DELAYED_WORK(&mjp->poll_work, mcu_joypad_poll);

	/* Optional enable/power GPIO for the MCU. */
	mjp->enable_gpio = devm_gpiod_get_optional(&spi->dev, "enable",
						   GPIOD_OUT_HIGH);
	if (IS_ERR(mjp->enable_gpio))
		return dev_err_probe(&spi->dev, PTR_ERR(mjp->enable_gpio),
				     "failed to get enable GPIO\n");
	if (mjp->enable_gpio)
		msleep(20);

	spi->mode = SPI_MODE_0;
	spi->bits_per_word = 8;
	err = spi_setup(spi);
	if (err) {
		dev_err(&spi->dev, "SPI setup failed: %d\n", err);
		return err;
	}

	mjp->poll_interval_ms = MCU_DEFAULT_POLL_MS;
	if (!device_property_read_u32(&spi->dev, "poll-interval-ms", &val))
		mjp->poll_interval_ms = val;

	mcu_init_tx(mjp);

	err = mcu_joypad_input_register(mjp);
	if (err)
		return err;

	/*
	 * Auto-calibrate stick centers. This runs MCU_CAL_FRAMES SPI
	 * transfers with sticks at rest to determine the zero position.
	 */
	err = mcu_joypad_calibrate(mjp);
	if (err)
		dev_warn(&spi->dev, "auto-calibration failed (%d), using defaults\n", err);

	err = mcu_joypad_leds_register(mjp);
	if (err)
		return err;

	schedule_delayed_work(&mjp->poll_work,
			      msecs_to_jiffies(mjp->poll_interval_ms));

	dev_info(&spi->dev, "MCU joypad ready (poll every %u ms)\n",
		 mjp->poll_interval_ms);
	return 0;
}

static void mcu_joypad_remove(struct spi_device *spi)
{
	struct mcu_joypad *mjp = spi_get_drvdata(spi);

	cancel_delayed_work_sync(&mjp->poll_work);
	mcu_joypad_leds_unregister(mjp);
}

/* ---- PM ops ------------------------------------------------------------- */

static int mcu_joypad_suspend(struct device *dev)
{
	struct spi_device *spi = to_spi_device(dev);
	struct mcu_joypad *mjp = spi_get_drvdata(spi);

	cancel_delayed_work_sync(&mjp->poll_work);
	mcu_joypad_leds_suspend(mjp);

	return 0;
}

static int mcu_joypad_resume(struct device *dev)
{
	struct spi_device *spi = to_spi_device(dev);
	struct mcu_joypad *mjp = spi_get_drvdata(spi);

	mutex_lock(&mjp->tx_lock);
	mcu_init_tx(mjp);
	mjp->led_switch = 1;
	mjp->led_dirty = true;
	mutex_unlock(&mjp->tx_lock);

	schedule_delayed_work(&mjp->poll_work,
			      msecs_to_jiffies(mjp->poll_interval_ms));

	return 0;
}

static DEFINE_SIMPLE_DEV_PM_OPS(mcu_joypad_pm, mcu_joypad_suspend,
				mcu_joypad_resume);

static const struct of_device_id mcu_joypad_of_match[] = {
	{ .compatible = "anbernic,spi-mcu-joypad" },
	{ }
};
MODULE_DEVICE_TABLE(of, mcu_joypad_of_match);

static const struct spi_device_id mcu_joypad_spi_id[] = {
	{ "spi-mcu-joypad", 0 },
	{ }
};
MODULE_DEVICE_TABLE(spi, mcu_joypad_spi_id);

static struct spi_driver mcu_joypad_driver = {
	.driver = {
		.name		= "spi-mcu-joypad",
		.of_match_table	= mcu_joypad_of_match,
		.pm		= pm_sleep_ptr(&mcu_joypad_pm),
	},
	.id_table	= mcu_joypad_spi_id,
	.probe		= mcu_joypad_probe,
	.remove		= mcu_joypad_remove,
};
module_spi_driver(mcu_joypad_driver);

MODULE_AUTHOR("ROCKNIX");
MODULE_DESCRIPTION("SPI MCU Joypad — analog sticks and RGB LEDs");
MODULE_LICENSE("GPL");
