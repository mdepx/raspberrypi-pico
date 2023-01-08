#include <lib/cyw43-driver/src/cyw43.h>
#include <lib/cyw43-driver/src/cyw43_internal.h>
#include <lib/cyw43-driver/src/cyw43_spi.h>

#include <arch/arm/raspberrypi/rp2040_pio.h>

extern struct mdx_device dev_pio;

/*
 * spi_gap01_sample0 program;
 * see pico-sdk/src/rp2_common/cyw43_driver/cyw43_bus_pio_spi.pio.
 */

static const uint16_t
cyw_program_instructions[] =
{
	0x6001, /* 0: out  pins, 1     side 0 */
	0x1040, /* 1: jmp  x--, 0      side 1 */
	0xe080, /* 2: set  pindirs, 0  side 0 */
	0xb042, /* 3: nop              side 1 */
	0x4001, /* 4: in   pins, 1     side 0 */
	0x1084, /* 5: jmp  y--, 4      side 1 */
};

static const rp2040_pio_program_t pio_program = {
	.instructions	= cyw_program_instructions,
	.length		= 6,
	.origin		= -1,
};

uint32_t
read_reg_u32_swap(cyw43_int_t *self, uint32_t fn, uint32_t reg)
{

	printf("%s: fn %x reg %x\n", __func__, fn, reg);

	return (0);
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

int
cyw43_spi_init(cyw43_int_t *self)
{
	int pio_offset;
	int error;

	error = pio_can_add_program(&dev_pio, &pio_program);

	printf("%s: error %d\n", __func__, error);

	pio_offset = pio_add_program(&dev_pio, &pio_program);

	printf("%s: program offset %x\n", __func__, pio_offset);

	return (0);
}

void
cyw43_spi_deinit(cyw43_int_t *self)
{

	printf("%s\n", __func__);
}

void
cyw43_spi_reset(void)
{

	printf("%s\n", __func__);
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
