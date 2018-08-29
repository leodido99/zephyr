/*
 * Copyright (c) 2018 Léonard Bise
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef _SAM0_EXTI_H_
#define _SAM0_EXTI_H_
#include <zephyr/types.h>

/* device name */
#define SAM0_EXTI_NAME "sam0-exti"

typedef void (*sam0_exti_callback_t) (int line, void *user);

/**
 * @brief enable EXTI interrupt for specific line
 *
 * @param line EXTI# line
 * @return 0 on success, negative error code otherwise
 */
int sam0_exti_enable(int line);

/**
 * @brief disable EXTI interrupt for specific line
 *
 * @param line EXTI# line
 * @return 0 on success, negative error code otherwise
 */
int sam0_exti_disable(int line);

/**
 * @brief EXTI trigger flags
 */
enum sam0_exti_trigger {
	/* trigger on rising edge */
	SAM0_EXTI_TRIG_RISING  = 0x1,
	/* trigger on falling edge */
	SAM0_EXTI_TRIG_FALLING = 0x2,
	/* trigger on both-edges */
	SAM0_EXTI_TRIG_BOTH = 0x3,
	/* trigger high-level detection */
	SAM0_EXTI_TRIG_HIGH_LEVEL = 0x4,
	/* trigger low-level detection */
	SAM0_EXTI_TRIG_LOW_LEVEL = 0x5,
};

/**
 * @brief set EXTI interrupt line triggers
 *
 * @param line EXTI# line
 * @param trg  OR'ed stm32_exti_trigger flags
 * @return 0 on success, negative error code otherwise
 */
int sam0_exti_trigger(int line, enum sam0_exti_trigger trg);

/**
 * @brief set EXTI interrupt callback
 * @param line EXI# line
 * @param cb   user callback
 * @param arg  user arg
 * @return 0 on success, negative error code otherwise
 */
int sam0_exti_set_callback(int line, sam0_exti_callback_t cb, void *data);

/**
 * @brief unset EXTI interrupt callback
 *
 * @param line EXI# line
 * @return 0 on success, negative error code otherwise
 */
int sam0_exti_unset_callback(int line);

#endif /* _SAM0_EXTI_H_ */
