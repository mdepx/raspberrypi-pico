#include <lib/cyw43-driver/src/cyw43.h>
#include <lib/cyw43-driver/src/cyw43_internal.h>
#include <lib/cyw43-driver/src/cyw43_spi.h>

#include <arch/arm/raspberrypi/rp2040_pio.h>
#include <arch/arm/raspberrypi/rp2040_io_bank0.h>
#include <arch/arm/raspberrypi/rp2040_pio_regs.h>

#include <dev/gpio/gpio.h>

extern struct mdx_device dev_pio;
extern struct rp2040_io_bank0_softc io_bank0_sc;

#define	WL_REG_ON	23
#define	DATA_OUT_PIN	24u
#define	DATA_IN_PIN	24u
#define	IRQ_PIN		24u
#define	CLOCK_PIN	29u
#define	CS_PIN		25u

/* Based on 125MHz cpu freq. */
#define	CLOCK_DIV	2
#define	CLOCK_DIV_MINOR	0

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
	struct rp2040_pio_sm_config config;
	int pio_offset;
	int error;
	int sm;

	error = pio_can_add_program(&dev_pio, &pio_program);

	printf("%s: error %d\n", __func__, error);

	pio_offset = pio_add_program(&dev_pio, &pio_program);

	printf("%s: program offset %x\n", __func__, pio_offset);

	memset(&config, 0, sizeof(struct rp2040_pio_sm_config));

	rp2040_pio_get_default_sm_config(&dev_pio, &config);
	rp2040_sm_config_set_out_pins(&config, DATA_OUT_PIN, 1);
	rp2040_sm_config_set_in_pins(&config, DATA_IN_PIN);
	rp2040_sm_config_set_set_pins(&config, DATA_OUT_PIN, 1);
	rp2040_sm_config_set_sideset(&config, 1, false, false);
	rp2040_sm_config_set_sideset_pins(&config, CLOCK_PIN);
	rp2040_sm_config_set_in_shift(&config, false, true, 32);
	rp2040_sm_config_set_out_shift(&config, false, true, 32);
	rp2040_sm_config_set_clkdiv_int_frac(&config, CLOCK_DIV,
	    CLOCK_DIV_MINOR);

	sm = 1;

	rp2040_pio_sm_init(&dev_pio, sm, pio_offset, &config);

	return (0);
}

void
cyw43_spi_deinit(cyw43_int_t *self)
{

	printf("%s\n", __func__);
}

static void
cyw43_gpio_init(int pin)
{

	mdx_gpio_configure(&dev_gpio, pin, MDX_GPIO_INPUT);
	mdx_gpio_set(&dev_gpio, pin, 0);
	rp2040_io_bank0_funcsel(&io_bank0_sc, pin, GPIO_FUNC_SIO);
}

void
cyw43_spi_gpio_setup(void)
{

	printf("%s\n", __func__);

	cyw43_gpio_init(WL_REG_ON);
	mdx_gpio_configure(&dev_gpio, WL_REG_ON,
	    MDX_GPIO_OUTPUT | MDX_GPIO_PULL_UP);

	cyw43_gpio_init(DATA_OUT_PIN);
	mdx_gpio_configure(&dev_gpio, DATA_OUT_PIN, MDX_GPIO_OUTPUT);
	mdx_gpio_set(&dev_gpio, DATA_OUT_PIN, 0);

	cyw43_gpio_init(CS_PIN);
	mdx_gpio_configure(&dev_gpio, CS_PIN, MDX_GPIO_OUTPUT);
	mdx_gpio_set(&dev_gpio, CS_PIN, 1);
}

void
cyw43_spi_reset(void)
{

	printf("%s\n", __func__);

	mdx_gpio_set(&dev_gpio, WL_REG_ON, 0);
	udelay(20000);

	mdx_gpio_set(&dev_gpio, WL_REG_ON, 1);
	udelay(250000);

	cyw43_gpio_init(IRQ_PIN);
	mdx_gpio_configure(&dev_gpio, IRQ_PIN, MDX_GPIO_INPUT);
}

int
write_reg_u32_swap(cyw43_int_t *self, uint32_t fn, uint32_t reg, uint32_t val)
{

	printf("%s\n", __func__);

	return (0);
}
