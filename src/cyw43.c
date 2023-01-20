/*-
 * Copyright (c) 2018-2022 Ruslan Bukin <br@bsdpad.com>
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

#include <lib/cyw43-driver/src/cyw43.h>
#include <lib/cyw43-driver/src/cyw43_internal.h>

void
cyw43_schedule_internal_poll_dispatch(void (*func)(void))
{

	printf("%s\n", __func__);
}

void
cyw43_post_poll_hook(void)
{

	printf("%s\n", __func__);
}

void
cyw43_cb_tcpip_init(cyw43_t *self, int itf)
{

	printf("%s\n", __func__);
}

void
cyw43_cb_tcpip_deinit(cyw43_t *self, int itf)
{

	printf("%s\n", __func__);
}

void
cyw43_delay_ms(uint32_t ms)
{

	udelay(ms * 1000);
}

void
cyw43_delay_us(uint32_t us)
{

	udelay(us);
}

void
cyw43_hal_get_mac(int idx, uint8_t buf[6])
{

	printf("%s\n", __func__);
}

void
cyw43_hal_generate_laa_mac(int idx, uint8_t buf[6])
{

	printf("%s\n", __func__);
}

void
cyw43_await_background_or_timeout_us(uint32_t timeout_us)
{

	printf("%s\n", __func__);
}

void
cyw43_cb_tcpip_set_link_up(cyw43_t *self, int itf)
{

	printf("%s\n", __func__);
}

void
cyw43_cb_tcpip_set_link_down(cyw43_t *self, int itf)
{

	printf("%s\n", __func__);
}

void
cyw43_cb_process_ethernet(void *cb_data, int itf, size_t len,
    const uint8_t *buf)
{

	printf("%s\n", __func__);
}

int
cyw43_read_bytes(cyw43_int_t *self, uint32_t fn, uint32_t addr, size_t len,
    uint8_t *buf)
{

	printf("%s\n", __func__);

	return (0);
}

int
cyw43_write_bytes(cyw43_int_t *self, uint32_t fn, uint32_t addr, size_t len,
    const uint8_t *src)
{

	printf("%s\n", __func__);

	return (0);
}

struct pbuf;

uint16_t
pbuf_copy_partial(const struct pbuf *p, void *dataptr, uint16_t len,
    uint16_t offset);

uint16_t
pbuf_copy_partial(const struct pbuf *p, void *dataptr, uint16_t len,
    uint16_t offset)
{

	printf("%s\n", __func__);

	return (0);
}
