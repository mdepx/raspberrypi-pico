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
#include <lib/cyw43-driver/src/cyw43_spi.h>

#include <cyw43_spi.h>

#define	SPI_STATUS_REGISTER	((uint32_t)0x0008)
// SPI_STATUS_REGISTER bits
#define STATUS_DATA_NOT_AVAILABLE       ((uint32_t)0x00000001)
#define STATUS_UNDERFLOW                ((uint32_t)0x00000002)
#define STATUS_OVERFLOW                 ((uint32_t)0x00000004)
#define STATUS_F2_INTR                  ((uint32_t)0x00000008)
#define STATUS_F3_INTR                  ((uint32_t)0x00000010)
#define STATUS_F2_RX_READY              ((uint32_t)0x00000020)
#define STATUS_F3_RX_READY              ((uint32_t)0x00000040)
#define STATUS_HOST_CMD_DATA_ERR        ((uint32_t)0x00000080)
#define STATUS_F2_PKT_AVAILABLE         ((uint32_t)0x00000100)
#define STATUS_F2_PKT_LEN_MASK          ((uint32_t)0x000FFE00)
#define STATUS_F2_PKT_LEN_SHIFT         ((uint32_t)9)
#define STATUS_F3_PKT_AVAILABLE         ((uint32_t)0x00100000)
#define STATUS_F3_PKT_LEN_MASK          ((uint32_t)0xFFE00000)
#define STATUS_F3_PKT_LEN_SHIFT         ((uint32_t)21)

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
	udelay(timeout_us);
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
	size_t aligned_len;
	uint32_t padding;
	int ret;

	//printf("%s\n", __func__);

	padding = (fn == BACKPLANE_FUNCTION) ? 4 : 0;
	aligned_len = (len + 3) & ~3;
	self->spi_header[padding > 0 ? 0 : 1] = make_cmd(false, true, fn, addr,
	    len + padding);

	if (padding > 0) {
		ret = cyw43_spi_transfer(self, (uint8_t *)&self->spi_header[0],
		    4, (uint8_t *)&self->spi_header[1], aligned_len + padding);
	} else {
		ret = cyw43_spi_transfer(self, (uint8_t *)&self->spi_header[1],
		    4, (uint8_t *)&self->spid_buf[0], aligned_len);
	}

	if (ret != 0) {
		printf("cyw43_read_bytes error %d", ret);
		return (ret);
	}

	if (buf != self->spid_buf)
		memcpy(buf, self->spid_buf, len);

	return (0);
}

int
cyw43_write_bytes(cyw43_int_t *self, uint32_t fn, uint32_t addr, size_t len,
    const uint8_t *src)
{
	int f2_ready_attempts;
	uint32_t bus_status;
	size_t aligned_len;
	int res;

	aligned_len = (len + 3) & ~3u;

	//printf("%s\n", __func__);

	if (fn == WLAN_FUNCTION) {
		f2_ready_attempts = 1000;
		while (f2_ready_attempts-- > 0) {
			bus_status = cyw43_read_reg_u32(self, BUS_FUNCTION,
			    SPI_STATUS_REGISTER);
			if (bus_status & STATUS_F2_RX_READY)
				break;
		}
		if (f2_ready_attempts <= 0) {
			printf("F2 not ready\n");
			panic("vvvv");
			return CYW43_FAIL_FAST_CHECK(-CYW43_EIO);
		}
	}

	if (src == self->spid_buf) {
		self->spi_header[1] = make_cmd(true, true, fn, addr, len);
		//printf("%s: spid_buf[0] %x\n", __func__, self->spid_buf[0]);
		res = cyw43_spi_transfer2(self, (uint8_t *)&self->spi_header[1],
		    aligned_len + 4, NULL, 0);
		return (res);
	} else {
		self->spi_header[1] = make_cmd(true, true, fn, addr, len);
		memcpy(self->spid_buf, src, len);
		//printf("%s: spid_buf[0] %x\n", __func__, self->spid_buf[0]);
		res = cyw43_spi_transfer2(self, (uint8_t *)&self->spi_header[1],
		    aligned_len + 4, NULL, 0);
		return (res);
	}

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
