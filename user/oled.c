/**********************************
 * OLED Display Low-Level Driver
 * SH1106-based 128x64 monochrome OLED display control
 **********************************/
#include "main.h"
#include "oled.h"

uint8_t display_buff[8][128] = {0};     /* Display buffer: 8 pages x 128 columns */
int8_t display_data_buff[128];          /* Waveform display buffer */
uint8_t display_data_buff_cnt = 0;
uint8_t display_data_flag = 0;

/**
 * @brief Write One Byte to OLED via Bit-Banged SPI
 * 
 * Sends 8 bits of data/command to the OLED controller using software SPI.
 * MSB is transmitted first. Clock idles low, data is latched on rising edge.
 * 
 * @param dat Byte to transmit (data or command)
 */
void OLED_WR_Byte(uint8_t dat)
{
	uint8_t i;
	for (i = 0; i < 8; i++)
	{
		OLED_SCLK_Clr();
		if (dat & 0x80)
		{
			OLED_SDIN_Set();
		}
		else
		{
			OLED_SDIN_Clr();
		}
		dat <<= 1;
		OLED_SCLK_Set();
	}
}

/**
 * @brief Clear OLED Display
 * 
 * Clears all 8 pages (64 rows) of the display by writing zeros to all pixels.
 * The display is organized as 8 pages of 8 rows each, 128 columns wide.
 */
void OLED_Clear(void)
{
	uint8_t i, n;
	OLED_DC_Clr();  /* Command mode */
	for (i = 0; i < 8; i++)
	{
		OLED_WR_Byte(0xB0 + i);  /* Set page address */
		OLED_WR_Byte(0x00);      /* Set lower column address */
		OLED_WR_Byte(0x10);      /* Set higher column address */
		OLED_DC_Set();           /* Data mode */
		for (n = 0; n < 128; n++)
		{
			OLED_WR_Byte(0);
		}
		OLED_DC_Clr();           /* Back to command mode */
	}
}

/**
 * @brief Initialize OLED Display Controller
 * 
 * Performs hardware reset and sends initialization sequence to SH1106:
 * - Display off during configuration
 * - Set contrast to maximum
 * - Configure scan direction and segment mapping
 * - Set multiplex ratio (1:64)
 * - Configure display clock and timing
 * - Enable charge pump for internal voltage generation
 * - Turn display on
 * 
 * Must be called before any display operations.
 */
void OLED_Init(void)
{
	
	/* Hardware reset sequence */
	OLED_RST_Set();
//	Delay(100);
	OLED_RST_Clr();
	Delay(10);
	OLED_RST_Set();

	/* Initialization command sequence */
	OLED_WR_Byte(0xAE);  /* Display off */
	OLED_WR_Byte(0x02);  /* Set low column address */
	OLED_WR_Byte(0x10);  /* Set high column address */
	OLED_WR_Byte(0x40);  /* Set start line address (0x00-0x3F) */
	OLED_WR_Byte(0x81);  /* Set contrast control */
	OLED_WR_Byte(0xFF);  /* Maximum contrast */
	OLED_WR_Byte(0xA1);  /* Set segment remap (A0=normal, A1=reverse) */
	OLED_WR_Byte(0xC8);  /* Set COM output scan direction (C0=normal, C8=reverse) */
	OLED_WR_Byte(0xA6);  /* Normal display (not inverted) */
	OLED_WR_Byte(0xA8);  /* Set multiplex ratio */
	OLED_WR_Byte(0x3F);  /* 1/64 duty (64 rows) */
	OLED_WR_Byte(0xD3);  /* Set display offset */
	OLED_WR_Byte(0x00);  /* No offset */
	OLED_WR_Byte(0xD5);  /* Set display clock divide ratio/oscillator frequency */
	OLED_WR_Byte(0x80);  /* Default divide ratio, 100 frames/sec */
	OLED_WR_Byte(0xD9);  /* Set pre-charge period */
	OLED_WR_Byte(0xF1);  /* Pre-charge: 15 clocks, discharge: 1 clock */
	OLED_WR_Byte(0xDA);  /* Set COM pins hardware configuration */
	OLED_WR_Byte(0x12);  /* Alternative COM pin config, disable remap */
	OLED_WR_Byte(0xDB);  /* Set VCOMH deselect level */
	OLED_WR_Byte(0x40);  /* ~0.77 x VCC */
	OLED_WR_Byte(0x20);  /* Set memory addressing mode */
	OLED_WR_Byte(0x02);  /* Page addressing mode */
	OLED_WR_Byte(0x8D);  /* Charge pump setting */
	OLED_WR_Byte(0x14);  /* Enable charge pump */
	OLED_WR_Byte(0xA4);  /* Normal display (not all pixels on) */
	OLED_WR_Byte(0xA6);  /* Normal display (not inverted) */
	OLED_WR_Byte(0xAF);  /* Display on */

	OLED_Clear();

	/* Initialize display flags */
	usb_open_display_flag = 1;
	data_upload_display_flag = 0;
	motor_run_display_flag = 0;
	display_static_flag = 1;
	init_dispaly_flag = 1;
}

/**
 * @brief Display Waveform Data on OLED Screen
 * 
 * Renders a waveform oscilloscope view on the OLED display by converting
 * signed position data into a bitmap representation. Used for real-time
 * visualization of motor parameters (e.g., rotor position, speed).
 * 
 * The display uses vertical bars to represent signal amplitude:
 * - Positive values: Bar extends upward from center (pages 0-3)
 * - Negative values: Bar extends downward from center (pages 4-7)
 * - Each of 128 columns represents one time sample
 * 
 * Data flow:
 * 1. Read position data from display_data_buff[]
 * 2. Convert to vertical bar pattern in display_buff[][]
 * 3. Transfer entire buffer to OLED via SPI
 * 
 * This function is called from main loop when display_data_flag is set,
 * indicating a complete waveform buffer is ready for display.
 */
void oled_display(void)
{
	uint32_t display_temp;
	uint8_t i, j;
	int8_t pos;
	if (display_data_flag == 1)
	{
		/* Convert waveform data to display buffer */
		for (i = 0; i < 127; i++)
		{
			pos = (display_data_buff[i] / 4);  /* Scale position to pixel rows */
			if (pos > 0)  /* Positive values: bar extends upward */
			{
				/* Create vertical bar extending upward from center */
				display_temp = 0x80000000;
				display_temp = display_temp >> pos;
				display_buff[3][i] = (uint8_t)((display_temp & 0xFF000000) >> 24);
				display_buff[2][i] = (uint8_t)((display_temp & 0x00FF0000) >> 16);
				display_buff[1][i] = (uint8_t)((display_temp & 0x0000FF00) >> 8);
				display_buff[0][i] = (uint8_t)(display_temp & 0x000000FF);
				display_buff[4][i] = 0;  /* Clear lower half */
				display_buff[5][i] = 0;
				display_buff[6][i] = 0;
				display_buff[7][i] = 0;
			}
			else  /* Negative values: bar extends downward */
			{
				uint8_t temp;
				temp = -pos;
				display_temp = 1;
				display_temp = display_temp << temp;
				/* Create vertical bar extending downward from center */
				display_buff[7][i] = (uint8_t)((display_temp & 0xFF000000) >> 24);
				display_buff[6][i] = (uint8_t)((display_temp & 0x00FF0000) >> 16);
				display_buff[5][i] = (uint8_t)((display_temp & 0x0000FF00) >> 8);
				display_buff[4][i] = (uint8_t)(display_temp & 0x000000FF);
				display_buff[0][i] = 0;  /* Clear upper half */
				display_buff[1][i] = 0;
				display_buff[2][i] = 0;
				display_buff[3][i] = 0;
			}
		}
		
		/* Transfer entire display buffer to OLED */
		for (i = 0; i < 8; i++)
		{
			OLED_WR_Byte(0xB0 + i);  /* Set page address */
			OLED_WR_Byte(0x02);      /* Set column start (offset by 2 for SH1106) */
			OLED_WR_Byte(0x10);
			OLED_DC_Set();  /* Data mode */
			for (j = 0; j < 128; j++)
			{
				OLED_WR_Byte(display_buff[i][j]);
			}
			OLED_DC_Clr();  /* Back to command mode */
		}
		display_data_flag = 0;  /* Clear flag to indicate buffer processed */
	}
}

/**
 * @brief Set OLED Cursor Position
 * 
 * Sets the OLED display cursor to a specific column (x) and page (y) position
 * for subsequent write operations. The SH1106 controller requires column
 * address offset of +2 pixels.
 * 
 * Page addressing mode:
 * - Pages: 8 horizontal bands (0-7), each 8 pixels tall
 * - Columns: 128 vertical lines (0-127)
 * 
 * @param x Column position (0-127)
 * @param y Page position (0-7)
 */
void OLED_Set_Pos(unsigned char x, unsigned char y)
{
	OLED_WR_Byte(0xB0 + y);  /* Set page address (0xB0-0xB7) */
	OLED_WR_Byte((((x + 2) & 0xF0) >> 4) | 0x10);  /* Set column address high nibble */
	OLED_WR_Byte(((x + 2) & 0x0F));  /* Set column address low nibble */
}

/**
 * @brief Draw Bitmap Image on OLED
 * 
 * Displays a bitmap image on the OLED at the specified rectangular area.
 * Used for showing logos, icons, or pre-rendered graphics.
 * 
 * The bitmap data should be organized in vertical byte columns matching
 * the OLED's page structure (each byte represents 8 vertical pixels).
 * 
 * @param x0 Starting column (0-127)
 * @param y0 Starting page (0-7)
 * @param x1 Ending column (0-127)
 * @param y1 Ending page row in pixels (will be converted to pages)
 * @param BMP Pointer to bitmap data array (column-major, vertical bytes)
 */
void OLED_DrawBMP(unsigned char x0, unsigned char y0, unsigned char x1, unsigned char y1, unsigned char BMP[])
{
	unsigned int j = 0;
	unsigned char x, y;

	/* Calculate number of pages needed */
	if (y1 % 8 == 0)
	{
		y = y1 / 8;
	}
	else
	{
		y = y1 / 8 + 1;
	}
	
	/* Draw bitmap page by page */
	for (y = y0; y < y1; y++)
	{
		OLED_Set_Pos(x0, y);
		OLED_DC_Set();  /* Data mode */
		for (x = x0; x < x1; x++)
		{
			OLED_WR_Byte(BMP[j++]);
		}
		OLED_DC_Clr();  /* Command mode */
	}
}

/**
 * @brief Display 16x16 Chinese Character
 * 
 * Renders a 16x16 pixel Chinese character from the Hzk font table.
 * Each character occupies 2 pages (16 pixels tall) and 16 columns wide.
 * 
 * The character data is stored in the Hzk[][] array where:
 * - First 16 bytes: Top half of character (page y)
 * - Next 16 bytes: Bottom half of character (page y+1)
 * 
 * @param x Starting column position (0-127)
 * @param y Starting page position (0-7)
 * @param no Character index in Hzk font table
 */
void OLED_ShowCHinese(uint8_t x, uint8_t y, uint8_t no)
{
	uint8_t t, adder = 0;
	
	/* Draw top half of character (page y) */
	OLED_Set_Pos(x, y);
	OLED_DC_Set();  /* Data mode */
	for (t = 0; t < 16; t++)
	{
		OLED_WR_Byte(Hzk[2 * no][t]);
		adder += 1;
	}
	OLED_DC_Clr();  /* Command mode */
	
	/* Draw bottom half of character (page y+1) */
	OLED_Set_Pos(x, y + 1);
	OLED_DC_Set();  /* Data mode */
	for (t = 0; t < 16; t++)
	{
		OLED_WR_Byte(Hzk[2 * no + 1][t]);
		adder += 1;
	}
	OLED_DC_Clr();  /* Command mode */
}

/**
 * @brief Display 32x32 Chinese Character
 * 
 * Renders a larger 32x32 pixel Chinese character from the Hzk_32 font table.
 * Each character occupies 4 pages (32 pixels tall) and 32 columns wide.
 * Provides better readability for important text or headings.
 * 
 * The character data is split across 4 rows:
 * - Row 0: Bytes 0-31 (top quarter)
 * - Row 1: Bytes 32-63 (second quarter)
 * - Row 2: Bytes 64-95 (third quarter)
 * - Row 3: Bytes 96-127 (bottom quarter)
 * 
 * @param x Starting column position (0-95, leaves room for 32-pixel width)
 * @param y Starting page position (0-4, leaves room for 4 pages)
 * @param no Character index in Hzk_32 font table
 */
void OLED_ShowCHinese_32(uint8_t x, uint8_t y, uint8_t no)
{
	uint8_t t;
	
	/* Draw first quarter (page y) */
	OLED_Set_Pos(x, y);
	OLED_DC_Set();
	for (t = 0; t < 32; t++)
	{
		OLED_WR_Byte(Hzk_32[4 * no][t]);
	}
	OLED_DC_Clr();
	
	/* Draw second quarter (page y+1) */
	OLED_Set_Pos(x, y + 1);
	OLED_DC_Set();
	for (t = 0; t < 32; t++)
	{
		OLED_WR_Byte(Hzk_32[4 * no + 1][t]);
	}
	OLED_DC_Clr();
	
	/* Draw third quarter (page y+2) */
	OLED_Set_Pos(x, y + 2);
	OLED_DC_Set();
	for (t = 0; t < 32; t++)
	{
		OLED_WR_Byte(Hzk_32[4 * no + 2][t]);
	}
	OLED_DC_Clr();
	
	/* Draw fourth quarter (page y+3) */
	OLED_Set_Pos(x, y + 3);
	OLED_DC_Set();
	for (t = 0; t < 32; t++)
	{
		OLED_WR_Byte(Hzk_32[4 * no + 3][t]);
	}
	OLED_DC_Clr();
}

/**
 * @brief Display Large 32-Pixel Numeric Character
 * 
 * Renders a large numeric digit (0-9) using 32-pixel height from F8X16 font.
 * Each character occupies 4 pages and is 16 columns wide.
 * Used for displaying important numeric values with high visibility.
 * 
 * @param x Starting column position (0-111)
 * @param y Starting page position (0-4)
 * @param chr Character to display ('0'-'9')
 */
void OLED_ShowChar_32(uint8_t x, uint8_t y, uint8_t chr)
{
	unsigned char c = 0, i = 0;
	c = chr - '0';  /* Get offset for numeric character */
	
	/* Draw first quarter (page y) */
	OLED_Set_Pos(x, y);
	OLED_DC_Set();
	for (i = 0; i < 16; i++)
	{
		OLED_WR_Byte(F8X16[c * 64 + i]);
	}
	OLED_DC_Clr();
	
	/* Draw second quarter (page y+1) */
	OLED_Set_Pos(x, y + 1);
	OLED_DC_Set();
	for (i = 0; i < 16; i++)
	{
		OLED_WR_Byte(F8X16[c * 64 + i + 16]);
	}
	OLED_DC_Clr();

	/* Draw third quarter (page y+2) */
	OLED_Set_Pos(x, y + 2);
	OLED_DC_Set();
	for (i = 0; i < 16; i++)
	{
		OLED_WR_Byte(F8X16[c * 64 + i + 32]);
	}
	OLED_DC_Clr();
	
	/* Draw fourth quarter (page y+3) */
	OLED_Set_Pos(x, y + 3);
	OLED_DC_Set();
	for (i = 0; i < 16; i++)
	{
		OLED_WR_Byte(F8X16[c * 64 + i + 48]);
	}
	OLED_DC_Clr();
}

/**
 * @brief Display Single ASCII Character
 * 
 * Displays one ASCII character at the specified position using either:
 * - 8x16 font (SIZE=16): 2 pages tall, 8 columns wide
 * - 6x8 font (SIZE=8): 1 page tall, 6 columns wide
 * 
 * Supports printable ASCII characters (space through tilde).
 * Automatically wraps to next line if character exceeds display width.
 * 
 * @param x Column position (0-127)
 * @param y Page position (0-7 for 8x16, 0-6 for 6x8)
 * @param chr ASCII character to display (space to tilde)
 */
void OLED_ShowChar(uint8_t x, uint8_t y, uint8_t chr)
{
	unsigned char c = 0, i = 0;
	c = chr - ' ';  /* Get offset from space character */
	
	/* Wrap to next line if beyond display width */
	if (x > Max_Column - 1)
	{
		x = 0;
		y = y + 2;
	}
	
	if (SIZE == 16)  /* 8x16 font (2 pages tall) */
	{
		/* Draw top half of character */
		OLED_Set_Pos(x, y);
		OLED_DC_Set();
		for (i = 0; i < 8; i++)
		{
			OLED_WR_Byte(F8X16[c * 16 + i]);
		}
		OLED_DC_Clr();
		
		/* Draw bottom half of character */
		OLED_Set_Pos(x, y + 1);
		OLED_DC_Set();
		for (i = 0; i < 8; i++)
		{
			OLED_WR_Byte(F8X16[c * 16 + i + 8]);
		}
		OLED_DC_Clr();
	}
	else  /* 6x8 font (1 page tall) */
	{
		OLED_Set_Pos(x, y + 1);
		OLED_DC_Set();
		for (i = 0; i < 6; i++)
		{
			OLED_WR_Byte(F6x8[c][i]);
		}
		OLED_DC_Clr();
	}
}

/**
 * @brief Display ASCII String
 * 
 * Renders a null-terminated ASCII string starting at the specified position.
 * Characters are drawn horizontally with automatic line wrapping at display edge.
 * 
 * String rendering:
 * - Each character advances x by 8 pixels (for 8x16 font)
 * - Wraps to next line (y+2) when x exceeds 120
 * - Stops when null terminator '\0' is encountered
 * 
 * @param x Starting column position (0-127)
 * @param y Starting page position (0-7)
 * @param chr Pointer to null-terminated string
 */
void OLED_ShowString(uint8_t x, uint8_t y, uint8_t *chr)
{
	unsigned char j = 0;
	while (chr[j] != '\0')
	{
		OLED_ShowChar(x, y, chr[j]);
		x += 8;  /* Advance to next character position */
		if (x > 120)  /* Wrap to next line if at edge */
		{
			x = 0;
			y += 2;
		}
		j++;
	}
}

/**
 * @brief Calculate Power Function (m^n)
 * 
 * Computes integer power m raised to the power n using iterative multiplication.
 * Used internally for decimal digit position calculations in number display.
 * 
 * Example: oled_pow(10, 3) = 1000 (for extracting thousands digit)
 * 
 * @param m Base value
 * @param n Exponent value
 * @return uint32_t Result of m^n
 */
uint32_t oled_pow(uint8_t m, uint8_t n)
{
	uint32_t result = 1;
	uint8_t i;
	for (i = n; i > 0; i--)
	{
		result *= m;
	}
	return result;
}

/**
 * @brief Display Decimal Number
 * 
 * Renders an unsigned integer number with specified number of digits.
 * Features:
 * - Leading zero suppression (blanks instead of leading zeros)
 * - Fixed width display (maintains digit positions)
 * - Supports numbers from 0 to 4,294,967,295 (32-bit)
 * 
 * Example: OLED_ShowNum(0, 0, 42, 5, 16) displays "   42" (3 blanks, then 42)
 * 
 * @param x Starting column position (0-127)
 * @param y Starting page position (0-7)
 * @param num Number to display (0 to 4,294,967,295)
 * @param len Number of digits to display (1-10)
 * @param size Font size (8 for 6x8, 16 for 8x16)
 */
void OLED_ShowNum(uint8_t x, uint8_t y, uint32_t num, uint8_t len, uint8_t size)
{
	uint8_t t, temp;
	uint8_t enshow = 0;  /* Flag to enable display after first non-zero digit */
	
	for (t = 0; t < len; t++)
	{
		/* Extract digit at position (len-t-1) */
		temp = (num / oled_pow(10, len - t - 1)) % 10;
		
		/* Leading zero suppression */
		if (enshow == 0 && t < (len - 1))
		{
			if (temp == 0)
			{
				OLED_ShowChar(x + (size / 2) * t, y, ' ');  /* Display space for leading zero */
				continue;
			}
			else
			{
				enshow = 1;  /* Start displaying digits after first non-zero */
			}
		}
		
		/* Display digit */
		OLED_ShowChar(x + (size / 2) * t, y, temp + '0');
	}
}


