/*
 * Copyright (c) 2018 Léonard Bise
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/** @file
 * @brief I2C bus driver for Atmel SAM0 MCU family.
 *
 * Limitations:
 * - Only I2C Master Mode with 7 bit addressing is currently supported.
 * - No reentrancy support.
 */

#include <i2c.h>
#include <dt-bindings/i2c/i2c.h>
#include <board.h>
#include <device.h>
#include <soc.h>
#include <errno.h>

#define SYS_LOG_DOMAIN "dev/i2c_sam0"
#define SYS_LOG_LEVEL CONFIG_SYS_LOG_I2C_LEVEL
#include <logging/sys_log.h>

#define I2C_SAM0_START_COND_CMD 0x1
#define I2C_SAM0_READ_CMD 0x2
#define I2C_SAM0_STOP_COND_CMD 0x3

#define I2C_SAM0_MSG_READ 1
#define I2C_SAM0_MSG_WRITE 0

#define I2C_SAM0_ACK_ACTION 0
#define I2C_SAM0_NACK_ACTION 1

#define I2C_SAM0_BUSSTATE_UNKNOWN 0
#define I2C_SAM0_BUSSTATE_IDLE 1
#define I2C_SAM0_BUSSTATE_OWNER 2
#define I2C_SAM0_BUSSTATE_BUSY 3

#define I2C_SAM0_SPEED_SM_FM 0x0
#define I2C_SAM0_SPEED_FM_PLUS 0x1
#define I2C_SAM0_SPEED_HS 0x2

/* SDA & SCL Rise Time in s as specified in
 * SAM D21 data sheet DS40001882C page 1027 */
#define I2C_SAM0_SM_FM_RISE_TIME 350
#define I2C_SAM0_FM_PLUS_RISE_TIME 100
#define I2C_SAM0_HS_RISE_TIME 60

/* Device constant configuration parameters */
struct i2c_sam0_dev_cfg {
	SercomI2cm *regs;
	u32_t pm_apbcmask;
	u16_t gclk_clkctrl_id;
	void (*irq_config_func)(struct device *dev);
};

struct sam0_msg {
	/* Buffer containing data to read or write */
	u8_t *buf;
	/* Length of the buffer */
	u32_t len;
	/* Index of the next byte to be read/written from/to the buffer */
	u32_t idx;
	/* Value of STATUS register at the end of the message */
	u32_t sam0_sts;
	/* Transfer flags as defined in the i2c.h file */
	u8_t flags;
};

/* Device run time data */
struct i2c_sam0_data {
	struct k_sem sem;
	struct sam0_msg msg;
};

#define DEV_CFG(dev) \
	((const struct i2c_sam0_dev_cfg *const)(dev)->config->config_info)

static void wait_synchronization(SercomI2cm *const regs)
{
#if defined(SERCOM_I2CS_SYNCBUSY_MASK)
	/* SYNCBUSY is a register */
	while ((regs->SYNCBUSY.reg & SERCOM_I2CS_SYNCBUSY_MASK) != 0) {
	}
#elif defined(SERCOM_I2C_STATUS_SYNCBUSY)
	/* SYNCBUSY is a bit */
	while ((regs->STATUS.reg & SERCOM_I2C_STATUS_SYNCBUSY) != 0) {
	}
#else
#error Unsupported device
#endif
}

static int i2c_sam0_set_bitrate(SercomI2cm *const i2cm, u32_t bitrate, u32_t rise_time_ns)
{
	i2cm->BAUD.bit.BAUD = CONFIG_SYS_CLOCK_HW_CYCLES_PER_SEC /
			      ((2 * bitrate) - 5 - (((CONFIG_SYS_CLOCK_HW_CYCLES_PER_SEC / 1000000)
						     * rise_time_ns) / (2 * 1000)));
	return 0;
}

static int i2c_sam0_configure(struct device *dev, u32_t config)
{
	const struct i2c_sam0_dev_cfg *const cfg = DEV_CFG(dev);
	SercomI2cm *const i2c = cfg->regs;
	SERCOM_I2CM_CTRLA_Type ctrla = {.reg = 0};
	SERCOM_I2CM_CTRLB_Type ctrlb = {.reg = 0};
	SERCOM_I2CS_ADDR_Type addr = {.reg = 0};
	u32_t bitrate;
	u32_t rise_time;

	if (!(config & I2C_MODE_MASTER)) {
		SYS_LOG_ERR("Slave mode is not supported");
		return -ENOTSUP;
	}

	if (config & I2C_ADDR_10_BITS) {
		/* Support 10 bits addresses */
		addr.bit.ADDR = 1;
	}

	/* Enable Master Mode */
	ctrla.bit.MODE = SERCOM_I2CM_CTRLA_MODE_I2C_MASTER_Val;

	/* Enable device */
	ctrla.bit.ENABLE = 1;

	/* Configure clock */
	switch (I2C_SPEED_GET(config)) {
	case I2C_SPEED_STANDARD:
		ctrla.bit.SPEED = I2C_SAM0_SPEED_SM_FM;
		bitrate = I2C_BITRATE_STANDARD;
		rise_time = I2C_SAM0_SM_FM_RISE_TIME;
		break;
	case I2C_SPEED_FAST:
		ctrla.bit.SPEED = I2C_SAM0_SPEED_SM_FM;
		bitrate = I2C_BITRATE_FAST;
		rise_time = I2C_SAM0_SM_FM_RISE_TIME;
		break;
	default:
		SYS_LOG_ERR("Unsupported I2C speed value");
		return -ENOTSUP;
	}

	/* Enable Smart Mode - acknowledge action automatically sent when reading DATA.DATA */
	ctrlb.bit.SMEN = 1;

	/* Update configuration - CTRLA, CTRLB and BAUD can only be updated when device is disabled */
	i2c->CTRLA.bit.ENABLE = 0;
	wait_synchronization(i2c);

	/* Configure baud rate */
	i2c_sam0_set_bitrate(i2c, bitrate, rise_time);
	/* Update control registers */
	i2c->CTRLB.reg = ctrlb.reg;
	wait_synchronization(i2c);
	i2c->CTRLA.reg = ctrla.reg;
	wait_synchronization(i2c);

	/* Set bus state to idle */
	i2c->STATUS.bit.BUSSTATE = I2C_SAM0_BUSSTATE_IDLE;
	wait_synchronization(i2c);

	return 0;
}

static void i2c_sam0_isr(void *arg)
{
	struct device *dev = arg;
	const struct i2c_sam0_dev_cfg *const cfg = DEV_CFG(dev);
	SercomI2cm *const i2c = cfg->regs;
	struct i2c_sam0_data *dev_data = dev->driver_data;
	bool transfer_completed = false;

	if (i2c->STATUS.bit.RXNACK) {
		SYS_LOG_ERR("I2C Slave answered with NACK");
		dev_data->msg.sam0_sts |= i2c->STATUS.bit.RXNACK;
		transfer_completed = true;
		SYS_LOG_DBG("RXNACK STOP");
		i2c->CTRLB.bit.CMD = I2C_SAM0_STOP_COND_CMD;
		wait_synchronization(i2c);
	}

	/* Slave on bus - Byte successfully received */
	if (i2c->INTFLAG.bit.SB) {
		if (dev_data->msg.idx + 1 == dev_data->msg.len) {
			/* last byte received - send NACK */
			i2c->CTRLB.bit.ACKACT = I2C_SAM0_NACK_ACTION;
			wait_synchronization(i2c);
			SYS_LOG_DBG("SB NACK");
			/* Send stop if requested */
			if (dev_data->msg.flags & I2C_MSG_STOP) {
				SYS_LOG_DBG("SB STOP");
				i2c->CTRLB.bit.CMD = I2C_SAM0_STOP_COND_CMD;
				wait_synchronization(i2c);
			}
			transfer_completed = true;
		} else {
			/* send ACK */
			i2c->CTRLB.bit.ACKACT = I2C_SAM0_ACK_ACTION;
			wait_synchronization(i2c);
			SYS_LOG_DBG("SB ACK");
		}
		/* Read byte */
		dev_data->msg.buf[dev_data->msg.idx++] = i2c->DATA.bit.DATA;
		wait_synchronization(i2c);
		SYS_LOG_DBG("R: 0x%X", dev_data->msg.buf[dev_data->msg.idx - 1]);
	}

	/* Master on bus - Byte transmitted */
	if (i2c->INTFLAG.bit.MB) {
		if (dev_data->msg.idx == dev_data->msg.len) {
			if (dev_data->msg.flags & I2C_MSG_STOP) {
				/* Send STOP condition */
				i2c->CTRLB.bit.CMD = I2C_SAM0_STOP_COND_CMD;
				wait_synchronization(i2c);
				SYS_LOG_DBG("MB STOP");
			}
			transfer_completed = true;
		} else {
			/* Transmit next byte */
			i2c->DATA.bit.DATA = dev_data->msg.buf[dev_data->msg.idx++];
			wait_synchronization(i2c);
			SYS_LOG_DBG("W: 0x%X", dev_data->msg.buf[dev_data->msg.idx - 1]);
		}
	}

	/* Error */
	if (i2c->INTFLAG.bit.ERROR) {
		SYS_LOG_ERR("Error on bus\n");
		dev_data->msg.sam0_sts |= i2c->STATUS.reg & (SERCOM_I2CM_STATUS_LENERR |
				SERCOM_I2CM_STATUS_SEXTTOUT | SERCOM_I2CM_STATUS_MEXTTOUT |
				SERCOM_I2CM_STATUS_LOWTOUT | SERCOM_I2CM_STATUS_ARBLOST |
				SERCOM_I2CM_STATUS_BUSERR);
	}

	if (transfer_completed) {
		/* Disable interrupts */
		i2c->INTENCLR.reg = SERCOM_I2CM_INTENSET_ERROR | SERCOM_I2CM_INTENSET_MB | SERCOM_I2CM_INTENSET_SB;
		/* We are done */
		k_sem_give(&dev_data->sem);
	}
}

static void i2c_sam0_msg_start(SercomI2cm *const i2c, u16_t addr, bool read) {
	/* Wait for IDLE or OWNER state */
	while(i2c->STATUS.bit.BUSSTATE != I2C_SAM0_BUSSTATE_IDLE && i2c->STATUS.bit.BUSSTATE != I2C_SAM0_BUSSTATE_OWNER) {
	}
	SYS_LOG_DBG("START addr=0x%X", addr);
	/* Set ACK action */
	i2c->CTRLB.bit.ACKACT = I2C_SAM0_ACK_ACTION;
	wait_synchronization(i2c);
	/* Set address and r/w bit - Start condition and address automatically emitted
	 * When slave ACKS MB IRQ is serviced */
	i2c->ADDR.bit.ADDR = (addr << 1) | (read ? I2C_SAM0_MSG_READ : I2C_SAM0_MSG_WRITE);
	/* Enable Error, Slave on Bus and Master on Bus interrupts */
	i2c->INTENSET.reg = SERCOM_I2CM_INTENSET_ERROR | SERCOM_I2CM_INTENSET_MB | SERCOM_I2CM_INTENSET_SB;
}

static int i2c_sam0_transfer(struct device *dev, struct i2c_msg *msgs,
				u8_t num_msgs, u16_t addr)
{
	const struct i2c_sam0_dev_cfg *const cfg = DEV_CFG(dev);
	struct i2c_sam0_data *dev_data = dev->driver_data;
	SercomI2cm *const i2c = cfg->regs;

	if (!num_msgs) {
		return 0;
	}

	/* Clear pending interrupts */
	i2c->INTFLAG.reg = SERCOM_I2CM_INTFLAG_MASK;

	for (; num_msgs > 0; num_msgs--, msgs++) {
		SYS_LOG_DBG("msg: len=%d flags=%x\n", msgs->len, dev_data->msg.flags);
		dev_data->msg.buf = msgs->buf;
		dev_data->msg.len = msgs->len;
		dev_data->msg.idx = 0;
		dev_data->msg.sam0_sts = 0;
		dev_data->msg.flags = msgs->flags;

		if ((msgs->flags & I2C_MSG_RW_MASK) == I2C_MSG_READ) {
			i2c_sam0_msg_start(i2c, addr, true);
		} else {
			i2c_sam0_msg_start(i2c, addr, false);
		}

		/* Wait for the transfer to complete */
		k_sem_take(&dev_data->sem, K_FOREVER);

		if (dev_data->msg.sam0_sts) {
			/* Error during transmission */
			return -EIO;
		}
	}

	return 0;
}

static int i2c_sam0_init(struct device *dev) {
	const struct i2c_sam0_dev_cfg *const cfg = DEV_CFG(dev);
	struct i2c_sam0_data *data = dev->driver_data;
	SercomI2cm *const i2c = cfg->regs;

	/* Initialize semaphore */
	k_sem_init(&data->sem, 0, 1);

	/* Enable the GCLK */
	GCLK->CLKCTRL.reg = cfg->gclk_clkctrl_id | GCLK_CLKCTRL_GEN_GCLK0 |
			    GCLK_CLKCTRL_CLKEN;

	/* Enable SERCOM clock in PM */
	PM->APBCMASK.reg |= cfg->pm_apbcmask;

	/* Reset I2C */
	i2c->CTRLA.bit.SWRST = 1;
	wait_synchronization(i2c);

	/* Disable all I2C interrupts */
	i2c->INTENCLR.reg = SERCOM_I2CS_INTENCLR_MASK;
	wait_synchronization(i2c);

	/* Configure IRQ */
	cfg->irq_config_func(dev);

	return 0;
}

static const struct i2c_driver_api i2c_sam0_driver_api = {
	.configure = i2c_sam0_configure,
	.transfer = i2c_sam0_transfer,
};

#define I2C_SAM0_IRQ_HANDLER_DECL(n)					\
static void i2c_sam0_irq_config_##n(struct device *dev)
#define I2C_SAM0_IRQ_HANDLER_FUNC(n)					\
.irq_config_func = i2c_sam0_irq_config_##n,
#define I2C_SAM0_IRQ_HANDLER(n)					\
static void i2c_sam0_irq_config_##n(struct device *dev)		\
{									\
	IRQ_CONNECT(CONFIG_I2C_SAM0_SERCOM##n##_IRQ,			\
		    CONFIG_I2C_SAM0_SERCOM##n##_IRQ_PRIORITY,		\
		    i2c_sam0_isr, DEVICE_GET(i2c_sam0_##n),		\
		    0);							\
    irq_enable(CONFIG_I2C_SAM0_SERCOM##n##_IRQ);			\
}

#define I2C_SAM0_DEFINE_CONFIG(n)                                            \
		static const struct i2c_sam0_dev_cfg i2c_sam0_config_##n = {          \
				.regs = (SercomI2cm *)CONFIG_I2C_SAM0_SERCOM##n##_BASE_ADDRESS, \
				.pm_apbcmask = PM_APBCMASK_SERCOM##n,                        \
				.gclk_clkctrl_id = GCLK_CLKCTRL_ID_SERCOM##n##_CORE,         \
				I2C_SAM0_IRQ_HANDLER_FUNC(n)					 			 \
	}

#define I2C_SAM0_DEVICE_INIT(n)                                              \
		I2C_SAM0_IRQ_HANDLER_DECL(n);						       			 \
		I2C_SAM0_DEFINE_CONFIG(n);                                           \
		static struct i2c_sam0_data i2c_sam0_dev_data_##n;                   \
		DEVICE_AND_API_INIT(i2c_sam0_##n, \
				CONFIG_I2C_SAM0_SERCOM##n##_LABEL,               \
				&i2c_sam0_init, &i2c_sam0_dev_data_##n,          \
				&i2c_sam0_config_##n, POST_KERNEL,               \
				CONFIG_I2C_INIT_PRIORITY, &i2c_sam0_driver_api); \
		I2C_SAM0_IRQ_HANDLER(n)

#if CONFIG_I2C_SAM0_SERCOM0_BASE_ADDRESS
I2C_SAM0_DEVICE_INIT(0)
#endif

#if CONFIG_I2C_SAM0_SERCOM1_BASE_ADDRESS
I2C_SAM0_DEVICE_INIT(1)
#endif

#if CONFIG_I2C_SAM0_SERCOM2_BASE_ADDRESS
I2C_SAM0_DEVICE_INIT(2)
#endif

#if CONFIG_I2C_SAM0_SERCOM3_BASE_ADDRESS
I2C_SAM0_DEVICE_INIT(3)
#endif

#if CONFIG_I2C_SAM0_SERCOM4_BASE_ADDRESS
I2C_SAM0_DEVICE_INIT(4)
#endif

#if CONFIG_I2C_SAM0_SERCOM5_BASE_ADDRESS
I2C_SAM0_DEVICE_INIT(5)
#endif
