/*
 * Example to show using I2C and I2C with libopencm3.
 * This is intended for the STM32F411-Discovery demoboard.
 * It has a CS42L22 audio DAC onboard.
 *
 * The device plays a 1kHz sine through the audio jack.
 */


#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/i2c.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/spi.h>


/* Test audio: 8 points of a sine.
 * At Fs=8kHz, this means a 1kHz audio sine
 * (the other channel is mute)
 */
#define VOL 0x0020
#define D16(x) ((int16_t)(x*VOL) )
//+ 0x7fff)
int16_t audio[16]=
{ D16(0),       0,
  D16(0.70711), 0,
  D16(1),       0,
  D16(0.70711), 0,
  D16(0),       0,
  D16(-0.70711),0,
  D16(-0.9999), 0,
  D16(-0.707),  0 };


static void write_i2c_to_audiochip( uint8_t reg, uint8_t contents)
{
	uint8_t packet[2];
	packet[0] = reg;
	packet[1] = contents;
	/* STM32F411 gives device address with R/W bit,
	 * libopencm wants it without it */
	uint8_t address = (0x94)>>1;

	i2c_transfer7(I2C1, address, packet, 2, NULL, 0);
}

int main(void)
{
	/* Set device clocks from opencm3 provided preset.*/
	const struct rcc_clock_scale *clocks = &rcc_hsi_configs[RCC_CLOCK_3V3_84MHZ];
	rcc_clock_setup_pll( clocks );

	rcc_periph_clock_enable(RCC_GPIOA);
	rcc_periph_clock_enable(RCC_GPIOB);
	rcc_periph_clock_enable(RCC_GPIOC);
	rcc_periph_clock_enable(RCC_GPIOD);

	/* Initialize "heartbeat" LED GPIO */
	gpio_mode_setup(GPIOD, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO13);

	/* I2C GPIO pins
	 * PB6 - SCL (I2C clock)
	 * PB9 - SDA (I2C data)
	 * The board does not have pullups on the I2C lines, so
	 * we use the chip internal pullups.
	 * Also the pins must be open drain, as per I2C specification.
	 * STM32F411 datasheet "Alternate Functions table" tells that
	 * I2C is AlternateFucntion 4 for both pins.
	 */
	gpio_mode_setup(GPIOB, GPIO_MODE_AF, GPIO_PUPD_PULLUP, GPIO6);
	gpio_mode_setup(GPIOB, GPIO_MODE_AF, GPIO_PUPD_PULLUP, GPIO9);
	gpio_set_output_options(GPIOB, GPIO_OTYPE_OD, GPIO_OSPEED_25MHZ, GPIO6);
	gpio_set_output_options(GPIOB, GPIO_OTYPE_OD, GPIO_OSPEED_25MHZ, GPIO9);
	gpio_set_af(GPIOB, GPIO_AF4, GPIO6);
	gpio_set_af(GPIOB, GPIO_AF4, GPIO9);

	/* Initialize the I2C itself.
	 * Since we are master, we would not need to initialize slave
	 * address, but this is the only libopencm3 API call that sets
	 * the 'bit14 of CCR' - a bit in the I2C that is HW reset to 0,
	 * but manual says 'must be 1' */
	rcc_periph_clock_enable(RCC_I2C1);
	i2c_peripheral_disable(I2C1);
	i2c_set_speed(I2C1, i2c_speed_sm_100k, clocks->apb1_frequency/1000000);
	i2c_set_own_7bit_slave_address(I2C1, 0);
	i2c_peripheral_enable(I2C1);

	/* Initialize I2S.
	 * I2S is implemented as a HW mode of the SPI peripheral.
	 * Since this is a STM32F411, there is a separate I2S PLL
	 * that needs to be enabled.
	 */
	rcc_osc_on(RCC_PLLI2S);
	rcc_periph_clock_enable(RCC_SPI3);
	i2s_disable(SPI3);
	i2s_set_standard(SPI3, i2s_standard_philips);
	i2s_set_dataformat(SPI3, i2s_dataframe_ch16_data16);
	i2s_set_mode(SPI3, i2s_mode_master_transmit);
	i2s_masterclock_enable(SPI3);
	/* RCC_PLLI2SCFGR configured values are:
	 * 0x24003010 i.e.
	 * PLLR = 2
	 * PLLI2SN = 192
	 * PLLI2SM = 16
	 * And since the input is PLL source (i.e. HSI = 16MHz)
	 * The I2S clock = 16 / 16 * 192 / 2 = 96MHz
	 * Calculate sampling frequency from equation given in
	 * STM32F411 reference manual:
	 * Fs = I2Sclk/ (32*2 * ((2*I2SDIV)+ODD)*4)
	 * I2SDIV = I2Sclk/(512*Fs)
	 * I2SDIV=24 => 23,4 so 23 + ODD bit set
	 */
	i2s_set_clockdiv(SPI3, 23, 1);
	i2s_enable(SPI3);


	/* I2S pins:
	 * Master clock: PC7
	 * Bit clock: PC10
	 * Data: PC12
	 * L/R clock: PA4
	 */
	gpio_mode_setup(GPIOC, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO7);
	gpio_set_af(GPIOC, GPIO_AF6, GPIO7);
	gpio_mode_setup(GPIOC, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO10);
	gpio_set_af(GPIOC, GPIO_AF6, GPIO10);
	gpio_mode_setup(GPIOC, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO12);
	gpio_set_af(GPIOC, GPIO_AF6, GPIO12);
	gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO4);
	gpio_set_af(GPIOA, GPIO_AF6, GPIO4);


	/* Initialize the Audio DAC, as per its datasheet.
	 * CS43L22 /RESET is connected to PD4, first release it. Then write
	 * minimum set of needed settings. */
	gpio_mode_setup(GPIOD, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO4);
	gpio_set(GPIOD, GPIO4);
	write_i2c_to_audiochip(0x06, 0x04); // interface control 1: set I2S dataformat
	write_i2c_to_audiochip(0x02, 0x9e); // power control 1: Magic value to power up the chip


	while(1) {
		/* Blink the heartbeat LED */
		static int blinkslowdown=0;
		if( ++blinkslowdown == 8000) {
			gpio_toggle(GPIOD, GPIO13);
			blinkslowdown=0;
		}

		for( unsigned i=0; i < sizeof(audio)/sizeof(audio[0]); i++)
			spi_send(SPI3, audio[i]);
	}
	return 0;
}

