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
