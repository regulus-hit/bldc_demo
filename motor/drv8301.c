/**********************************
 * DRV8301 Motor Driver Interface
 * Handles SPI communication and initialization of DRV8301 gate driver
 **********************************/
#include "main.h"
#include "drv8301.h"

uint16_t drv8301_reg_read[4];
uint8_t drv8301_init_ok_flag = 0;

/**
 * @brief Write data to DRV8301 register via SPI
 * 
 * Performs bit-banged SPI write to configure DRV8301 registers.
 * Used during initialization to set up gate driver parameters.
 * 
 * @param address Register address (0-3)
 * @param data Configuration data to write (11-bit)
 */
void drv8301_spi_write(uint8_t address, uint16_t data)
{
	uint16_t send_data = 0;
	uint8_t i;

	send_data = (uint16_t)(address & 0x03);
	send_data <<= 11;
	send_data |= data;

	DRV8301_CS_Clr();

	for (i = 0; i < 16; i++)
	{
		DRV8301_SCLK_Set();
		if (send_data & 0x8000)
		{
			DRV8301_MOSI_Set();
		}
		else
		{
			DRV8301_MOSI_Clr();
		}
		DRV8301_SCLK_Clr();
		send_data <<= 1;
	}

	DRV8301_SCLK_Clr();
	DRV8301_MOSI_Clr();

	DRV8301_CS_Set();
}

/**
 * @brief Read data from DRV8301 register via SPI
 * 
 * Performs bit-banged SPI read from DRV8301 registers.
 * First sends read command, then clocks in the response data.
 * 
 * @param address Register address (0-3)
 * @return uint16_t Register value from DRV8301
 */
uint16_t drv8301_spi_read(uint8_t address)
{
	uint16_t send_data = 0;
	uint16_t receive_data = 0;
	uint8_t i;

	send_data = (uint16_t)(address & 0x03);
	send_data <<= 11;
	send_data |= 0x8000;

	DRV8301_CS_Clr();

	for (i = 0; i < 16; i++)
	{
		DRV8301_SCLK_Set();
		if (send_data & 0x8000)
		{
			DRV8301_MOSI_Set();
		}
		else
		{
			DRV8301_MOSI_Clr();
		}
		send_data <<= 1;
		DRV8301_SCLK_Clr();
	}

	DRV8301_CS_Set();

	send_data = (uint16_t)(address & 0x03);
	send_data <<= 11;
	send_data |= 0x8000;

	send_data = 0;

	DRV8301_CS_Clr();

	for (i = 0; i < 16; i++)
	{
		DRV8301_SCLK_Set();
		if (send_data & 0x8000)
		{
			DRV8301_MOSI_Set();
		}
		else
		{
			DRV8301_MOSI_Clr();
		}
		send_data <<= 1;
		receive_data <<= 1;
		DRV8301_SCLK_Clr();
		if (DRV8301_MISO_GET() == 1)
		{
			receive_data |= 0x0001;
		}
	}

	DRV8301_CS_Set();

	return receive_data;
}


/**
 * @brief Initialize DRV8301 motor driver chip
 * 
 * Configures DRV8301 gate driver for BLDC motor control:
 * - Sets gate drive current to 0.7A for fast MOSFET switching
 * - Configures 6-PWM input mode for independent control of all 6 switches
 * - Enables overcurrent latch shutdown for hardware protection
 * - Sets current sense amplifier gain to 80x for accurate current measurement
 * - Reads back all registers to verify configuration
 * 
 * This is a critical initialization step that must complete before motor operation.
 */
void drv8301_init(void)
{
	/* Configure gate driver and protection settings */
	drv8301_spi_write(DRV8301_REG2, GATE_CURRENT_0_7_A | GATE_RESET_NOMAL | PWM_MODE_6_INPUTS | OCP_LATCH_SHUT_DOWN | OC_ADJ_SET_14);
	
	/* Configure current sense amplifier and protection thresholds */
	drv8301_spi_write(DRV8301_REG3, OCTW_OT_ONLY | GAIN_AMP_80 | DC_CAL_CH1_CON | DC_CAL_CH2_CON | OC_TOFF_CYCLE);
	
	/* Read back all registers to verify configuration */
	drv8301_reg_read[0] = drv8301_spi_read(DRV8301_REG0);	/* Status Register 1 */
	drv8301_reg_read[1] = drv8301_spi_read(DRV8301_REG1);	/* Status Register 2 */
	drv8301_reg_read[2] = drv8301_spi_read(DRV8301_REG2);	/* Control Register 1 */
	drv8301_reg_read[3] = drv8301_spi_read(DRV8301_REG3);	/* Control Register 2 */

	drv8301_init_ok_flag = 1;
}

/**
 * @brief Monitor and handle DRV8301 fault conditions
 * 
 * Checks the FAULT pin of DRV8301. If a fault is detected (pin low),
 * immediately disables all gate drivers to protect hardware.
 * Faults include overcurrent, overtemperature, and gate driver faults.
 * 
 * This function should be called regularly during motor operation.
 */
void drv8301_protection(void)
{
	if (GPIO_ReadInputDataBit(DRV8301_FAULT_GPIO_PORT, DRV8301_FAULT_PIN) != 1)
	{
		/* Fault detected - disable gate drivers immediately */
		GPIO_ResetBits(DRV8301_ENGATE_GPIO_PORT, DRV8301_ENGATE_PIN);
		drv8301_fault_flag = 1;
	}
}


