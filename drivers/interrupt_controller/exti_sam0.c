/*
 * Copyright (c) 2018 Léonard Bise
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <interrupt_controller/exti_sam0.h>

#include <board.h>
#include <device.h>

#define SYS_LOG_DOMAIN "dev/exti_sam0"
#define SYS_LOG_LEVEL CONFIG_SYS_LOG_EXTI_SAM0_LEVEL
#include <logging/sys_log.h>

/**
 * @brief Driver for External interrupt/event controller in SAM0 MCUs
 *
 * Based on reference manuals:
 * 	SAM D21 Family Data Sheet DS40001882C
 *
 * Chapter 21: EIC - External Interrupt Controller
 *
 */

#define EXTI_LINES 16

struct exti_sam0_cb {
	sam0_exti_callback_t cb;
	void *data;
};

struct sam0_exti_data {
	/* per-line callbacks */
	struct exti_sam0_cb cb[EXTI_LINES];
};

static volatile Eic *regs = (Eic *)CONFIG_EIC_SAM0_BASE_ADDRESS;

static void sam0_exti_connect_irqs(struct device *dev);

static inline int sam0_exti_is_pending(int line)
{
	return (regs->INTFLAG.reg & (1 << line));
}

static inline void sam0_exti_clear_pending(int line)
{
	regs->INTFLAG.reg |= (1 << line);
}

static inline void sam0_exti_wait_synch(void)
{
	/* Wait for synchronization */
	while (regs->STATUS.bit.SYNCBUSY == 1);
}

static void sam0_exti_device_enable(void)
{
	regs->CTRL.bit.ENABLE = 1;

	sam0_exti_wait_synch();
}

static void sam0_exti_device_disable(void)
{
	regs->CTRL.bit.ENABLE = 0;

	sam0_exti_wait_synch();
}

int sam0_exti_enable(int line)
{
	if (line > (EXTI_LINES - 1)) {
		SYS_LOG_ERR("Invalid line");
		return -EINVAL;
	}

	SYS_LOG_DBG("Enable external interrupt: line=%d reg=%x", line, 1 << line);

	sam0_exti_device_disable();

	sam0_exti_clear_pending(line);

	regs->INTENSET.reg |= (1 << line);

	/* Enable wake-up on external interrupt */
	regs->WAKEUP.reg |= (1 << line);

	sam0_exti_device_enable();

	return 0;
}

int sam0_exti_disable(int line)
{
	if (line > (EXTI_LINES - 1)) {
		SYS_LOG_ERR("Invalid line");
		return -EINVAL;
	}

	SYS_LOG_DBG("Disable external interrupt: line=%d reg=%x", line, (1 << line));

	sam0_exti_device_disable();

	regs->INTENSET.reg &= ~(1 << line);

	regs->WAKEUP.reg &= ~(1 << line);

	sam0_exti_device_enable();

	return 0;
}

int sam0_exti_trigger(int line, enum sam0_exti_trigger trigger)
{
	int sense_x = (line % 8);
	int conf_n;

	if (line > (EXTI_LINES - 1)) {
		SYS_LOG_ERR("Invalid line");
		return -EINVAL;
	}

	if (trigger > SAM0_EXTI_TRIG_LOW_LEVEL) {
		SYS_LOG_ERR("Invalid trigger");
		return -EINVAL;
	}

	conf_n = (line > 8) ? 1 : 0;

	SYS_LOG_DBG("Set trigger: line=%d trigger=%d sense_x=%d conf_n=%d reg=%x", line, trigger, sense_x, conf_n, trigger << (sense_x * 4));

	sam0_exti_device_disable();

	regs->CONFIG[conf_n].reg |= (trigger << (sense_x * 4));

	sam0_exti_device_enable();

	return 0;
}

static void sam0_eic_isr(void *arg)
{
	struct device *dev = arg;
	struct sam0_exti_data *data = dev->driver_data;
	int line;

	for (line = 0; line < EXTI_LINES; line++) {
		if (sam0_exti_is_pending(line)) {
			sam0_exti_clear_pending(line);

			/* run callback only if one is registered */
			if (!data->cb[line].cb) {
				continue;
			}

			data->cb[line].cb(line, data->cb[line].data);
		}
	}
}

static int sam0_exti_init(struct device *dev)
{
	SYS_LOG_INF("Initialization SAM0 Exti");

	/* Enable EIC clock in PM */
	PM->APBAMASK.reg |= PM_APBAMASK_EIC;

	/* Enable the GCLK */
	GCLK->CLKCTRL.reg = GCLK_CLKCTRL_ID_EIC | GCLK_CLKCTRL_GEN_GCLK0 |
				    GCLK_CLKCTRL_CLKEN;

	sam0_exti_connect_irqs(dev);

	return 0;
}

static struct sam0_exti_data exti_data;

DEVICE_INIT(exti_sam0, SAM0_EXTI_NAME, sam0_exti_init,
	    &exti_data, NULL,
	    POST_KERNEL, 90);
/* Ensure this module is initialized after most of the others for some reason? */
//CONFIG_KERNEL_INIT_PRIORITY_DEVICE

int sam0_exti_set_callback(int line, sam0_exti_callback_t cb, void *arg)
{
	struct device *dev = DEVICE_GET(exti_sam0);
	struct sam0_exti_data *data = dev->driver_data;

	if (line > (EXTI_LINES - 1)) {
		SYS_LOG_ERR("Invalid line");
		return -EINVAL;
	}

	if (data->cb[line].cb) {
		SYS_LOG_ERR("Callback already registered for this line");
		return -EINVAL;
	}

	data->cb[line].cb = cb;
	data->cb[line].data = arg;

	return 0;
}

int sam0_exti_unset_callback(int line)
{
	struct device *dev = DEVICE_GET(exti_sam0);
	struct sam0_exti_data *data = dev->driver_data;

	if (line > (EXTI_LINES - 1)) {
		SYS_LOG_ERR("Invalid line");
		return -EINVAL;
	}

	data->cb[line].cb = NULL;
	data->cb[line].data = NULL;

	return 0;
}

static void sam0_exti_connect_irqs(struct device *dev)
{
	ARG_UNUSED(dev);

	IRQ_CONNECT(CONFIG_EIC_SAM0_IRQ,
		    CONFIG_EIC_SAM0_IRQ_PRIORITY,
		    sam0_eic_isr, DEVICE_GET(exti_sam0),
		    0);

	irq_enable(CONFIG_EIC_SAM0_IRQ);
}
