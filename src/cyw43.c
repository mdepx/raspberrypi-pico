#include <lib/cyw43-driver/src/cyw43.h>
#include <lib/cyw43-driver/src/cyw43_internal.h>
#include <lib/cyw43-driver/src/cyw43_spi.h>

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

uint32_t
read_reg_u32_swap(cyw43_int_t *self, uint32_t fn, uint32_t reg)
{

	printf("%s: fn %x reg %x\n", __func__, fn, reg);

	return (0);
}

int
cyw43_spi_init(cyw43_int_t *self)
{

	printf("%s\n", __func__);

	return (0);
}

void cyw43_spi_deinit(cyw43_int_t *self)
{

	printf("%s\n", __func__);
}

void
cyw43_spi_reset(void)
{

	printf("%s\n", __func__);
}

void
cyw43_await_background_or_timeout_us(uint32_t timeout_us)
{

	printf("%s\n", __func__);
}

static uint32_t
_cyw43_read_reg(cyw43_int_t *self, uint32_t fn, uint32_t reg, uint size)
{

	printf("%s\n", __func__);

	return (0);
}

uint32_t
cyw43_read_reg_u32(cyw43_int_t *self, uint32_t fn, uint32_t reg)
{

	printf("%s\n", __func__);

	return _cyw43_read_reg(self, fn, reg, 4);
}

int
cyw43_read_reg_u16(cyw43_int_t *self, uint32_t fn, uint32_t reg)
{

	printf("%s\n", __func__);

	return _cyw43_read_reg(self, fn, reg, 2);
}

int
cyw43_read_reg_u8(cyw43_int_t *self, uint32_t fn, uint32_t reg)
{

	printf("%s\n", __func__);

	return _cyw43_read_reg(self, fn, reg, 1);
}

static inline int
_cyw43_write_reg(cyw43_int_t *self, uint32_t fn, uint32_t reg, uint32_t val,
    uint size)
{

	printf("%s\n", __func__);

	return (0);
}

int
cyw43_write_reg_u32(cyw43_int_t *self, uint32_t fn, uint32_t reg, uint32_t val)
{

    return _cyw43_write_reg(self, fn, reg, val, 4);
}

int
cyw43_write_reg_u16(cyw43_int_t *self, uint32_t fn, uint32_t reg, uint16_t val)
{

    return _cyw43_write_reg(self, fn, reg, val, 2);
}

int
cyw43_write_reg_u8(cyw43_int_t *self, uint32_t fn, uint32_t reg, uint32_t val)
{

    return _cyw43_write_reg(self, fn, reg, val, 1);
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

void
cyw43_spi_gpio_setup(void)
{

	printf("%s\n", __func__);
}

int
write_reg_u32_swap(cyw43_int_t *self, uint32_t fn, uint32_t reg, uint32_t val)
{

	printf("%s\n", __func__);

	return (0);
}
