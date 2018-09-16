/*
 * Copyright (c) 2017 Google LLC.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>
#include <device.h>
#include <gpio.h>
#include <soc.h>
#include <kernel.h>
#include <interrupt_controller/exti_sam0.h>
#include "gpio_utils.h"

#define SYS_LOG_DOMAIN "dev/gpio_sam0"
#define SYS_LOG_LEVEL CONFIG_SYS_LOG_GPIO_SAM0_LEVEL
#include <logging/sys_log.h>

/* FIXME Get from exti driver */
#define NB_EXTI_LINES 16
#define PIN_TO_LINE(pin) (pin % NB_EXTI_LINES)

struct gpio_sam0_config {
	PortGroup *regs;
};

struct gpio_sam0_data {
	u32_t cb_pins;
	sys_slist_t cb;
};

#define DEV_CFG(dev) \
	((const struct gpio_sam0_config *const)(dev)->config->config_info)

static void gpio_sam0_isr(int line, void *arg)
{
	struct device *dev = arg;
	struct gpio_sam0_data *data = dev->driver_data;

	if (BIT(line) & data->cb_pins) {
		_gpio_fire_callbacks(&data->cb, dev, BIT(line));
	}
}

static int gpio_sam0_config(struct device *dev, int access_op, u32_t pin,
			    int flags)
{
	const struct gpio_sam0_config *config = DEV_CFG(dev);
	PortGroup *regs = config->regs;
	u32_t mask = 1 << pin;
	bool is_out = (flags & GPIO_DIR_MASK) == GPIO_DIR_OUT;
	int pud = flags & GPIO_PUD_MASK;
	PORT_PINCFG_Type pincfg;
	int trigger = -1;

	if (access_op != GPIO_ACCESS_BY_PIN) {
		return -ENOTSUP;
	}

	/* Builds the configuration and writes it in one go */
	pincfg.reg = 0;
	pincfg.bit.INEN = 1;

	/* Direction */
	if (is_out) {
		regs->DIRSET.bit.DIRSET = mask;
	} else {
		regs->DIRCLR.bit.DIRCLR = mask;
	}

	/* Pull up / pull down */
	if (is_out && pud != GPIO_PUD_NORMAL) {
		return -ENOTSUP;
	}

	switch (pud) {
	case GPIO_PUD_NORMAL:
		break;
	case GPIO_PUD_PULL_UP:
		pincfg.bit.PULLEN = 1;
		regs->OUTSET.reg = mask;
		break;
	case GPIO_PUD_PULL_DOWN:
		pincfg.bit.PULLEN = 1;
		regs->OUTCLR.reg = mask;
		break;
	default:
		return -ENOTSUP;
	}

	/* Write the now-built pin configuration */
	regs->PINCFG[pin] = pincfg;

	if ((flags & GPIO_INT) != 0) {
		SYS_LOG_DBG("pin=%d -> line=%d", pin, PIN_TO_LINE(pin));
		sam0_exti_set_callback(PIN_TO_LINE(pin), gpio_sam0_isr, dev);

		if (flags & GPIO_INT_EDGE) {
			if (flags & GPIO_INT_DOUBLE_EDGE) {
				trigger = SAM0_EXTI_TRIG_BOTH;
				SYS_LOG_INF("Trigger both edge");
			} else if (flags & (GPIO_INT_EDGE | GPIO_INT_ACTIVE_HIGH)) {
				trigger = SAM0_EXTI_TRIG_RISING;
				SYS_LOG_INF("Trigger rising edge");
			} else {
				trigger = SAM0_EXTI_TRIG_FALLING;
				SYS_LOG_INF("Trigger falling edge");
			}
		} else if (flags & GPIO_INT_LEVEL) {
			if (flags & GPIO_INT_ACTIVE_HIGH) {
				trigger = SAM0_EXTI_TRIG_HIGH_LEVEL;
				SYS_LOG_INF("Trigger high level");
			} else {
				trigger = SAM0_EXTI_TRIG_LOW_LEVEL;
				SYS_LOG_INF("Trigger low level");
			}
		}

		if (trigger > 0) {
			sam0_exti_trigger(PIN_TO_LINE(pin), trigger);
		}

		sam0_exti_enable(PIN_TO_LINE(pin));
	}

	if ((flags & GPIO_POL_MASK) != GPIO_POL_NORMAL) {
		return -ENOTSUP;
	}

	return 0;
}

static int gpio_sam0_write(struct device *dev, int access_op, u32_t pin,
			   u32_t value)
{
	const struct gpio_sam0_config *config = DEV_CFG(dev);
	u32_t mask = 1 << pin;

	if (access_op != GPIO_ACCESS_BY_PIN) {
		/* TODO(mlhx): support GPIO_ACCESS_BY_PORT */
		return -ENOTSUP;
	}

	if (value != 0) {
		config->regs->OUTSET.bit.OUTSET = mask;
	} else {
		config->regs->OUTCLR.bit.OUTCLR = mask;
	}

	return 0;
}

static int gpio_sam0_read(struct device *dev, int access_op, u32_t pin,
			  u32_t *value)
{
	const struct gpio_sam0_config *config = DEV_CFG(dev);
	u32_t bits;

	if (access_op != GPIO_ACCESS_BY_PIN) {
		/* TODO(mlhx): support GPIO_ACCESS_BY_PORT */
		return -ENOTSUP;
	}

	bits = config->regs->IN.bit.IN;
	*value = (bits >> pin) & 1;

	return 0;
}

static int gpio_sam0_manage_callback(struct device *dev,
				      struct gpio_callback *callback,
				      bool set)
{
	struct gpio_sam0_data *data = dev->driver_data;

	_gpio_manage_callback(&data->cb, callback, set);

	return 0;
}

static int gpio_sam0_enable_callback(struct device *dev,
				      int access_op, u32_t pin)
{
	struct gpio_sam0_data *data = dev->driver_data;

	if (access_op != GPIO_ACCESS_BY_PIN) {
		return -ENOTSUP;
	}

	data->cb_pins |= BIT(pin);

	return 0;
}

static int gpio_sam0_disable_callback(struct device *dev,
				       int access_op, u32_t pin)
{
	struct gpio_sam0_data *data = dev->driver_data;

	if (access_op != GPIO_ACCESS_BY_PIN) {
		return -ENOTSUP;
	}

	data->cb_pins &= ~BIT(pin);

	return 0;
}

static const struct gpio_driver_api gpio_sam0_api = {
	.config = gpio_sam0_config,
	.write = gpio_sam0_write,
	.read = gpio_sam0_read,
	.manage_callback = gpio_sam0_manage_callback,
	.enable_callback = gpio_sam0_enable_callback,
	.disable_callback = gpio_sam0_disable_callback,
};

static int gpio_sam0_init(struct device *dev)
{
	return 0;
}

/* Port A */
#ifdef CONFIG_GPIO_SAM0_PORTA_BASE_ADDRESS

static const struct gpio_sam0_config gpio_sam0_config_0 = {
	.regs = (PortGroup *)CONFIG_GPIO_SAM0_PORTA_BASE_ADDRESS,
};

static struct gpio_sam0_data gpio_sam0_data_0;

DEVICE_AND_API_INIT(gpio_sam0_0, CONFIG_GPIO_SAM0_PORTA_LABEL, gpio_sam0_init,
		    &gpio_sam0_data_0, &gpio_sam0_config_0, POST_KERNEL,
		    CONFIG_KERNEL_INIT_PRIORITY_DEVICE, &gpio_sam0_api);
#endif

/* Port B */
#ifdef CONFIG_GPIO_SAM0_PORTB_BASE_ADDRESS

static const struct gpio_sam0_config gpio_sam0_config_1 = {
	.regs = (PortGroup *)CONFIG_GPIO_SAM0_PORTB_BASE_ADDRESS,
};

static struct gpio_sam0_data gpio_sam0_data_1;

DEVICE_AND_API_INIT(gpio_sam0_1, CONFIG_GPIO_SAM0_PORTB_LABEL, gpio_sam0_init,
		    &gpio_sam0_data_1, &gpio_sam0_config_1, POST_KERNEL,
		    CONFIG_KERNEL_INIT_PRIORITY_DEVICE, &gpio_sam0_api);
#endif
