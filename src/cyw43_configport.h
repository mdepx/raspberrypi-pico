
#ifndef _CYW43_CONFIGPORT_H
#define _CYW43_CONFIGPORT_H

#include <sys/cdefs.h>
#include <sys/param.h>
#include <sys/systm.h>
#include <assert.h>

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

#define	CYW43_NUM_GPIOS		4

#define	CYW43_THREAD_ENTER	critical_enter()
#define	CYW43_THREAD_EXIT	critical_exit()
#define	CYW43_THREAD_LOCK_CHECK

#define CYW43_SDPCM_SEND_COMMON_WAIT cyw43_await_background_or_timeout_us(1000);
#define CYW43_DO_IOCTL_WAIT	cyw43_await_background_or_timeout_us(1000);
#define CYW43_POST_POLL_HOOK	cyw43_post_poll_hook();
#define CYW43_ARRAY_SIZE(x)	(sizeof(x) / sizeof((x)[0]))

#define CYW43_PIN_WL_HOST_WAKE	24
#define	CYW43_PIN_WL_SDIO_1	2
#define	CYW43_PIN_WL_REG_ON	2
#define	CYW43_PIN_WL_SDIO	2

#define CYW43_HAL_PIN_MODE_INPUT	(1)
#define CYW43_HAL_PIN_MODE_OUTPUT	(2)
#define CYW43_HAL_PIN_PULL_NONE		(0)
#define CYW43_HAL_PIN_PULL_UP		(1)
#define CYW43_HAL_PIN_PULL_DOWN		(2)

#define cyw43_hal_pin_obj_t u_int
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

	printf("%s\n", __func__);

	return 0;
}

static inline uint32_t
cyw43_hal_ticks_ms(void)
{

	printf("%s\n", __func__);

	return 0;
}

static inline int
cyw43_hal_pin_read(cyw43_hal_pin_obj_t pin)
{

	printf("%s\n", __func__);

	return 0;
}

static inline void
cyw43_hal_pin_low(cyw43_hal_pin_obj_t pin)
{

	printf("%s\n", __func__);
}

static inline void
cyw43_hal_pin_high(cyw43_hal_pin_obj_t pin)
{

	printf("%s\n", __func__);
}

static inline void
cyw43_hal_pin_config(cyw43_hal_pin_obj_t pin, uint32_t mode, uint32_t pull,
    __unused uint32_t alt)
{

	printf("%s\n", __func__);
}

#endif
