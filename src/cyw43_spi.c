#include <sys/endian.h>

#include <lib/cyw43-driver/src/cyw43.h>
#include <lib/cyw43-driver/src/cyw43_internal.h>
#include <lib/cyw43-driver/src/cyw43_spi.h>

#include <arch/arm/raspberrypi/rp2040.h>
#include <arch/arm/raspberrypi/rp2040_dma.h>
#include <arch/arm/raspberrypi/rp2040_pio.h>
#include <arch/arm/raspberrypi/rp2040_io_bank0.h>
#include <arch/arm/raspberrypi/rp2040_pio_regs.h>
#include <arch/arm/raspberrypi/rp2040_pio_instructions.h>

#include <dev/gpio/gpio.h>

extern struct mdx_device dev_pio;
extern struct rp2040_io_bank0_softc io_bank0_sc;
extern struct mdx_device dev_dma;

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

typedef struct {
	mdx_device_t pio;
	uint8_t pio_func_sel;
	int8_t pio_offset;
	int8_t pio_sm;
	int8_t dma_out;
	int8_t dma_in;
} bus_data_t;

static bus_data_t bus_data;

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


//#define SWAP32(A) ((((A) & 0xff000000U) >> 8) | (((A) & 0xff0000U) << 8) | (((A) & 0xff00U) >> 8) | (((A) & 0xffU) << 8))

static uint32_t
__swap16x2(uint32_t a) {
    __asm ("rev16 %0, %0" : "+l" (a) : : );
    return a;
}

#define SWAP32(a) __swap16x2(a)

#if 0
#define	SWAP32	bswap32
#endif

static inline uint32_t
make_cmd(bool write, bool inc, uint32_t fn, uint32_t addr, uint32_t sz)
{

	return write << 31 | inc << 30 | fn << 28 | (addr & 0x1ffff) << 11 | sz;
}

static void
start_spi_comms(void)
{

	/* Drive CS low. */
	mdx_gpio_set(&dev_gpio, CS_PIN, 0);
	rp2040_io_bank0_funcsel(&io_bank0_sc, DATA_OUT_PIN,
	    bus_data.pio_func_sel);
}

int
cyw43_spi_transfer(cyw43_int_t *self, const uint8_t *tx, size_t tx_length,
    uint8_t *rx, size_t rx_length)
{
	uint32_t reg;

	printf("%s\n", __func__);

	start_spi_comms();

	rp2040_pio_sm_set_enabled(bus_data.pio, bus_data.pio_sm, false);
	rp2040_pio_sm_set_wrap(bus_data.pio, bus_data.pio_sm,
	    bus_data.pio_offset, bus_data.pio_offset + 6 - 1);
	rp2040_pio_sm_clear_fifos(bus_data.pio, bus_data.pio_sm);
	rp2040_pio_sm_set_pindirs_with_mask(bus_data.pio, bus_data.pio_sm,
	    1u << DATA_OUT_PIN, 1u << DATA_OUT_PIN);
	rp2040_pio_sm_restart(bus_data.pio, bus_data.pio_sm);
	rp2040_pio_sm_clkdiv_restart(bus_data.pio, bus_data.pio_sm);

	printf("%s\n", __func__);
	rp2040_pio_sm_put(bus_data.pio, bus_data.pio_sm,
	    tx_length * 8 - 1);
	rp2040_pio_sm_exec(bus_data.pio, bus_data.pio_sm,
	    pio_encode_out(pio_x, 32));
	rp2040_pio_sm_put(bus_data.pio, bus_data.pio_sm,
	    rx_length * 8 - 1);
	rp2040_pio_sm_exec(bus_data.pio, bus_data.pio_sm,
	    pio_encode_out(pio_y, 32));
	rp2040_pio_sm_exec(bus_data.pio, bus_data.pio_sm,
	    pio_encode_jmp(bus_data.pio_offset));

	printf("%s 3\n", __func__);
	rp2040_dma_channel_abort(&dev_dma, bus_data.dma_out);
	rp2040_dma_channel_abort(&dev_dma, bus_data.dma_in);

	struct rp2040_dma_channel_config out_config;
	struct rp2040_dma_channel_config in_config;

	printf("%s 4\n", __func__);
	memset(&out_config, 0, sizeof(struct rp2040_dma_channel_config));
	out_config.src_addr = (uint32_t)tx;
	out_config.src_incr = true;
	out_config.dst_addr = RP2040_PIO0_BASE +
	    RP2040_PIO_TXF_OFFSET(bus_data.pio_sm);
	out_config.dst_incr = false;
	out_config.size = 2; // 32 bit
	out_config.count = tx_length; // 4;
	out_config.dreq = rp2040_pio_get_dreq_offset(&dev_pio,
	    bus_data.pio_sm, true);
	out_config.dreq = 0; // DREQ_PIO0_TX0

	printf("%s\n", __func__);
	memset(&in_config, 0, sizeof(struct rp2040_dma_channel_config));
	in_config.src_addr = RP2040_PIO0_BASE +
	    RP2040_PIO_RXF_OFFSET(bus_data.pio_sm);
	in_config.src_incr = false;
	in_config.dst_addr = (uint32_t)rx;
	in_config.dst_incr = true;
	in_config.size = 2; // 32 bit
	in_config.count = rx_length; // 4;
	in_config.dreq = rp2040_pio_get_dreq_offset(&dev_pio,
	    bus_data.pio_sm, false);
	in_config.dreq = 4; // DREQ_PIO0_RX0

	printf("%s\n", __func__);
	rp2040_dma_configure(&dev_dma, bus_data.dma_out, &out_config);

	printf("%s: set enabled true\n", __func__);
	rp2040_pio_sm_set_enabled(bus_data.pio, bus_data.pio_sm, true);

	reg = rp2040_pio_read_reg(&dev_pio, RP2040_PIO_FDEBUG_OFFSET);
	printf("%s: fdebug %x\n", __func__, reg);
	reg = rp2040_pio_read_reg(&dev_pio, RP2040_PIO_FSTAT_OFFSET);
	printf("%s: fstat %x\n", __func__, reg);
	reg = rp2040_pio_read_reg(&dev_pio, RP2040_PIO_FLEVEL_OFFSET);
	printf("%s: flevel %x\n", __func__, reg);

	rp2040_dma_configure(&dev_dma, bus_data.dma_in, &in_config);

	rp2040_dma_channel_is_busy(&dev_dma, bus_data.dma_out);
	udelay(20000);
	rp2040_dma_channel_is_busy(&dev_dma, bus_data.dma_out);
	udelay(20000);
	rp2040_dma_channel_is_busy(&dev_dma, bus_data.dma_out);
	udelay(20000);
	rp2040_dma_channel_is_busy(&dev_dma, bus_data.dma_out);
	udelay(20000);
	rp2040_dma_channel_is_busy(&dev_dma, bus_data.dma_out);
	udelay(20000);

printf("%s: chann 1\n", __func__);
	rp2040_dma_channel_is_busy(&dev_dma, bus_data.dma_in);
	udelay(20000);
	rp2040_dma_channel_is_busy(&dev_dma, bus_data.dma_in);
	udelay(20000);
	rp2040_dma_channel_is_busy(&dev_dma, bus_data.dma_in);
	udelay(20000);

	reg = rp2040_pio_read_reg(&dev_pio, RP2040_PIO_FDEBUG_OFFSET);
	printf("%s: fdebug %x\n", __func__, reg);
	reg = rp2040_pio_read_reg(&dev_pio, RP2040_PIO_FSTAT_OFFSET);
	printf("%s: fstat %x\n", __func__, reg);
	reg = rp2040_pio_read_reg(&dev_pio, RP2040_PIO_FLEVEL_OFFSET);
	printf("%s: flevel %x\n", __func__, reg);

#if 0
	dma_channel_get_default_config(&dev_dma, bus_data.dma_out, &config);
#endif

	return (0);
}

uint32_t
read_reg_u32_swap(cyw43_int_t *self, uint32_t fn, uint32_t reg)
{
	uint32_t buf[3];
	int error;

	assert(fn != BACKPLANE_FUNCTION);

	printf("%s: fn %x reg %x\n", __func__, fn, reg);

	buf[0] = SWAP32(make_cmd(false, true, fn, reg, 4));
	buf[1] = 0;

	error = cyw43_spi_transfer(self, (uint8_t *)&buf[0], 4,
	    (uint8_t *)&buf[1], 8);
	if (error != 0)
		return (error);

	printf("%s: res %x\n", __func__, buf[1]);
	printf("%s: res %x\n", __func__, buf[2]);

	while (1)
		udelay(10000);

	return (SWAP32(buf[1]));
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

	printf("%s\n", __func__);

	return _cyw43_write_reg(self, fn, reg, val, 4);
}

int
cyw43_write_reg_u16(cyw43_int_t *self, uint32_t fn, uint32_t reg, uint16_t val)
{

	printf("%s\n", __func__);

	return _cyw43_write_reg(self, fn, reg, val, 2);
}

int
cyw43_write_reg_u8(cyw43_int_t *self, uint32_t fn, uint32_t reg, uint32_t val)
{

	printf("%s\n", __func__);

	return _cyw43_write_reg(self, fn, reg, val, 1);
}

int
cyw43_spi_init(cyw43_int_t *self)
{
	struct rp2040_pio_sm_config config;
	int error;

	error = pio_can_add_program(&dev_pio, &pio_program);

	printf("%s: error %d\n", __func__, error);

	bus_data.pio = &dev_pio;
	bus_data.pio_offset = pio_add_program(&dev_pio, &pio_program);
	bus_data.pio_sm = 0;
	bus_data.dma_out = 0;
	bus_data.dma_in = 1;
	bus_data.pio_func_sel = GPIO_FUNC_PIO0;

	printf("%s: program offset %x\n", __func__, bus_data.pio_offset);

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

	mdx_gpio_configure(&dev_gpio, CLOCK_PIN,
	    MDX_GPIO_SPEED_HIGH | MDX_GPIO_SLEW_FAST);

	rp2040_io_bank0_funcsel(&io_bank0_sc, CLOCK_PIN,
	    bus_data.pio_func_sel);
	rp2040_io_bank0_funcsel(&io_bank0_sc, DATA_OUT_PIN,
	    bus_data.pio_func_sel);

	rp2040_pio_sm_set_consecutive_pindirs(&dev_pio, bus_data.pio_sm,
	    CLOCK_PIN, 1, true);

	mdx_gpio_configure(&dev_gpio, DATA_IN_PIN,
	    MDX_GPIO_PULL_DOWN | MDX_GPIO_HYSTERESIS_EN);

	rp2040_pio_sm_init(&dev_pio, bus_data.pio_sm, bus_data.pio_offset,
	    &config);
	rp2040_pio_set_input_sync_bypass(&dev_pio, DATA_IN_PIN, true);

	rp2040_pio_sm_exec(bus_data.pio, bus_data.pio_sm,
	    pio_encode_set(pio_pins, 1));

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

	printf("%s\n", __func__);

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
	udelay(25000);

	cyw43_gpio_init(IRQ_PIN);
	mdx_gpio_configure(&dev_gpio, IRQ_PIN, MDX_GPIO_INPUT);
}

int
write_reg_u32_swap(cyw43_int_t *self, uint32_t fn, uint32_t reg, uint32_t val)
{

	printf("%s\n", __func__);

	return (0);
}
