/*-
 * Copyright (c) 2023 Ruslan Bukin <br@bsdpad.com>
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 */

#ifndef _CYW43_CONFIGPORT_H
#define _CYW43_CONFIGPORT_H

#include <sys/cdefs.h>
#include <sys/param.h>
#include <sys/systm.h>
#include <dev/gpio/gpio.h>
#include <assert.h>

extern struct mdx_device dev_gpio;

#ifndef CYW43_HOST_NAME
#define CYW43_HOST_NAME "Pico-W"
#endif

#ifndef CYW43_LWIP
#define CYW43_LWIP	0
#endif

#ifndef	CYW43_GPIO
#define	CYW43_GPIO	1
#endif

#ifndef	CYW43_LOGIC_DEBUG
#define	CYW43_LOGIC_DEBUG	0
#endif

#ifndef	CYW43_USE_OTP_MAC
#define	CYW43_USE_OTP_MAC	1
#endif

#ifndef	CYW43_NO_NETUTILS
#define	CYW43_NO_NETUTILS	1
#endif

#ifndef	CYW43_IOCTL_TIMEOUT_US
#define	CYW43_IOCTL_TIMEOUT_US	1000000
#endif

#ifndef	CYW43_USE_STATS
#define	CYW43_USE_STATS		0
#endif

#ifndef	CYW43_HAL_MAC_WLAN0
#define	CYW43_HAL_MAC_WLAN0	0
#endif

#ifndef	STATIC
#define	STATIC	static
#endif

#ifndef	CYW43_USE_SPI
#define	CYW43_USE_SPI	1
#endif

#ifndef	CYW43_SPI_PIO
#define	CYW43_SPI_PIO	1
#endif

#define	CYW43_EPERM		-1
#define	CYW43_EIO		-2
#define	CYW43_EINVAL		-3
#define	CYW43_ETIMEDOUT		-4

#define	CYW43_NUM_GPIOS		3

#define	CYW43_THREAD_ENTER	critical_enter()
#define	CYW43_THREAD_EXIT	critical_exit()
#define	CYW43_THREAD_LOCK_CHECK

#define CYW43_SDPCM_SEND_COMMON_WAIT cyw43_await_background_or_timeout_us(1000);
#define CYW43_DO_IOCTL_WAIT	cyw43_await_background_or_timeout_us(1000);
#define CYW43_POST_POLL_HOOK	cyw43_post_poll_hook();
#define CYW43_ARRAY_SIZE(x)	(sizeof(x) / sizeof((x)[0]))

#define	CYW43_PIN_WL_REG_ON	23
#define CYW43_PIN_WL_HOST_WAKE	24

#define CYW43_HAL_PIN_MODE_INPUT	(1)
#define CYW43_HAL_PIN_MODE_OUTPUT	(2)
#define CYW43_HAL_PIN_PULL_NONE		(0)
#define CYW43_HAL_PIN_PULL_UP		(1)
#define CYW43_HAL_PIN_PULL_DOWN		(2)

#define cyw43_hal_pin_obj_t uint32_t
typedef uint32_t uint;

void cyw43_schedule_internal_poll_dispatch(void (*func)(void));
void cyw43_post_poll_hook(void);
void cyw43_delay_ms(uint32_t ms);
void cyw43_delay_us(uint32_t us);
void cyw43_hal_get_mac(int idx, uint8_t buf[6]);
void cyw43_hal_generate_laa_mac(int idx, uint8_t buf[6]);
void cyw43_await_background_or_timeout_us(uint32_t timeout_us);

static inline uint32_t
cyw43_hal_ticks_us(void)
{

	//printf("%s\n", __func__);

	return 0;
}

static inline uint32_t
cyw43_hal_ticks_ms(void)
{

	//printf("%s\n", __func__);

	return 0;
}

static inline int
cyw43_hal_pin_read(cyw43_hal_pin_obj_t pin)
{
	uint32_t val;

	val = mdx_gpio_get(&dev_gpio, pin);

	printf("%s: pin %d, val %d\n", __func__, pin, val);

	return (val);
}

static inline void
cyw43_hal_pin_low(cyw43_hal_pin_obj_t pin)
{

	printf("%s: %d\n", __func__, pin);

	mdx_gpio_set(&dev_gpio, pin, 0);
}

static inline void
cyw43_hal_pin_high(cyw43_hal_pin_obj_t pin)
{

	printf("%s: %d\n", __func__, pin);

	mdx_gpio_set(&dev_gpio, pin, 1);
}

static inline void
cyw43_hal_pin_config(cyw43_hal_pin_obj_t pin, uint32_t mode, uint32_t pull,
    uint32_t alt __unused)
{

	printf("%s: pin %d mode %d pull %d alt %d\n", __func__,
	    pin, mode, pull, alt);

	if (mode == CYW43_HAL_PIN_MODE_INPUT)
		mdx_gpio_set_dir(&dev_gpio, pin, 0);
	else
		mdx_gpio_set_dir(&dev_gpio, pin, 1);

#if 0
	if (pull == CYW43_HAL_PIN_PULL_UP)
#endif
}

static inline uint32_t
make_cmd(bool write, bool inc, uint32_t fn, uint32_t addr, uint32_t sz)
{

	return write << 31 | inc << 30 | fn << 28 | (addr & 0x1ffff) << 11 | sz;
}

#endif
