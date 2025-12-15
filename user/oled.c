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
		OLED_WR_Byte(0xb0 + i);  /* Set page address */
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
	OLED_RST_Clr();
	Delay(10);
	OLED_RST_Set();

	/* Initialization command sequence */
	OLED_WR_Byte(0xAE);  /* Display off */
	OLED_WR_Byte(0x02);  /* Set low column address */
	OLED_WR_Byte(0x10);  /* Set high column address */
	OLED_WR_Byte(0x40);  /* Set start line address (0x00-0x3F) */
	OLED_WR_Byte(0x81);  /* Set contrast control */
	OLED_WR_Byte(0xff);  /* Maximum contrast */
	OLED_WR_Byte(0xA1);  /* Set segment remap (A0=normal, A1=reverse) */
	OLED_WR_Byte(0xC8);  /* Set COM output scan direction (C0=normal, C8=reverse) */
	OLED_WR_Byte(0xA6);  /* Normal display (not inverted) */
	OLED_WR_Byte(0xA8);  /* Set multiplex ratio */
	OLED_WR_Byte(0x3f);  /* 1/64 duty (64 rows) */
	OLED_WR_Byte(0xD3);  /* Set display offset */
	OLED_WR_Byte(0x00);  /* No offset */
	OLED_WR_Byte(0xd5);  /* Set display clock divide ratio/oscillator frequency */
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

void oled_display(void)
{
	uint32_t display_temp;
	uint8_t i,j;
	int8_t pos;
	if(display_data_flag==1)
	{
		for(i = 0;i<127;i++)
		{
			pos = (display_data_buff[i]/4);
			if(pos>0)
			{
				display_temp = 0x80000000;
				display_temp = display_temp >> pos;
				display_buff[3][i] = (uint8_t)((display_temp&0xff000000)>>24);
				display_buff[2][i] = (uint8_t)((display_temp&0x00ff0000)>>16);
				display_buff[1][i] = (uint8_t)((display_temp&0x0000ff00)>>8);
				display_buff[0][i] = (uint8_t)(display_temp&0x000000ff);
				display_buff[4][i] = 0;
				display_buff[5][i] = 0;
				display_buff[6][i] = 0;
				display_buff[7][i] = 0;
			}
			else
			{
				uint8_t temp;
				temp = -pos;
				display_temp = 1;
				display_temp = display_temp << temp;
				display_buff[7][i] = (uint8_t)((display_temp&0xff000000)>>24);
				display_buff[6][i] = (uint8_t)((display_temp&0x00ff0000)>>16);
				display_buff[5][i] = (uint8_t)((display_temp&0x0000ff00)>>8);
				display_buff[4][i] = (uint8_t)(display_temp&0x000000ff);
				display_buff[0][i] = 0;
				display_buff[1][i] = 0;
				display_buff[2][i] = 0;
				display_buff[3][i] = 0;
			}
		}
		for(i=0;i<8;i++)  
		{  
			OLED_WR_Byte (0xb0+i);    
			OLED_WR_Byte (0x02);      
			OLED_WR_Byte (0x10);     
			OLED_DC_Set();
			for(j=0;j<128;j++)OLED_WR_Byte(display_buff[i][j]); 
			OLED_DC_Clr();
		} 
		display_data_flag=0;
	}
}

void OLED_Set_Pos(unsigned char x, unsigned char y) 
{ 
	OLED_WR_Byte(0xb0+y);
	OLED_WR_Byte((((x+2)&0xf0)>>4)|0x10);
	OLED_WR_Byte(((x+2)&0x0f)); 
} 
/***********功能描述：显示显示BMP图片128×64起始点坐标(x,y),x的范围0～127，y为页的范围0～7*****************/
void OLED_DrawBMP(unsigned char x0, unsigned char y0,unsigned char x1, unsigned char y1,unsigned char BMP[])
{ 	
	unsigned int j=0;
	unsigned char x,y;
	
	if(y1%8==0) y=y1/8;      
	else y=y1/8+1;
	for(y=y0;y<y1;y++)
	{
		OLED_Set_Pos(x0,y);
		OLED_DC_Set();
		for(x=x0;x<x1;x++)
		{      
			OLED_WR_Byte(BMP[j++]);	    	
		}
		OLED_DC_Clr();
	}
} 
//显示汉字
void OLED_ShowCHinese(uint8_t x,uint8_t y,uint8_t no)
{      			    
	uint8_t t,adder=0;
	OLED_Set_Pos(x,y);
	OLED_DC_Set();
	for(t=0;t<16;t++)
	{
		OLED_WR_Byte(Hzk[2*no][t]);
		adder+=1;
	}
	OLED_DC_Clr();
	OLED_Set_Pos(x,y+1);
	OLED_DC_Set();
	for(t=0;t<16;t++)
	{	
		OLED_WR_Byte(Hzk[2*no+1][t]);
		adder+=1;
	}
	OLED_DC_Clr();
}

void OLED_ShowCHinese_32(uint8_t x,uint8_t y,uint8_t no)
{
	uint8_t t;
	OLED_Set_Pos(x,y);
	OLED_DC_Set();
	for(t=0;t<32;t++)
	{
		OLED_WR_Byte(Hzk_32[4*no][t]);
	}
	OLED_DC_Clr();
	OLED_Set_Pos(x,y+1);
	OLED_DC_Set();
	for(t=0;t<32;t++)
	{
		OLED_WR_Byte(Hzk_32[4*no+1][t]);
	}
	OLED_DC_Clr();
	OLED_Set_Pos(x,y+2);
	OLED_DC_Set();
	for(t=0;t<32;t++)
	{
		OLED_WR_Byte(Hzk_32[4*no+2][t]);
	}
	OLED_DC_Clr();
	OLED_Set_Pos(x,y+3);
	OLED_DC_Set();
	for(t=0;t<32;t++)
	{
		OLED_WR_Byte(Hzk_32[4*no+3][t]);
	}
	OLED_DC_Clr();
}

void OLED_ShowChar_32(uint8_t x,uint8_t y,uint8_t chr)
{
	unsigned char c=0,i=0;
	c=chr-'0';//得到偏移；测试是显示几个数字
	OLED_Set_Pos(x,y);
	OLED_DC_Set();
	for(i=0;i<16;i++)
		OLED_WR_Byte(F8X16[c*64+i]);
	OLED_DC_Clr();
	OLED_Set_Pos(x,y+1);
	OLED_DC_Set();
	for(i=0;i<16;i++)
		OLED_WR_Byte(F8X16[c*64+i+16]);
	OLED_DC_Clr();
	
	OLED_Set_Pos(x,y+2);
	OLED_DC_Set();
	for(i=0;i<16;i++)
		OLED_WR_Byte(F8X16[c*64+i+32]);
	OLED_DC_Clr();
	OLED_Set_Pos(x,y+3);
	OLED_DC_Set();
	for(i=0;i<16;i++)
		OLED_WR_Byte(F8X16[c*64+i+48]);
	OLED_DC_Clr();
	
}
//在指定位置显示一个字符,包括部分字符
//x:0~127
//y:0~63
//mode:0,反白显示;1,正常显示				 
//size:选择字体 16/12 
void OLED_ShowChar(uint8_t x,uint8_t y,uint8_t chr)
{      	
	unsigned char c=0,i=0;	
	c=chr-' ';//得到偏移后的值			
	if(x>Max_Column-1){x=0;y=y+2;}
	if(SIZE ==16)
	{
		OLED_Set_Pos(x,y);	
		OLED_DC_Set();
		for(i=0;i<8;i++)
			OLED_WR_Byte(F8X16[c*16+i]);
		OLED_DC_Clr();
		OLED_Set_Pos(x,y+1);
		OLED_DC_Set();
		for(i=0;i<8;i++)
			OLED_WR_Byte(F8X16[c*16+i+8]);
		OLED_DC_Clr();
	}
	else {	
		OLED_Set_Pos(x,y+1);
		OLED_DC_Set();
		for(i=0;i<6;i++)
			OLED_WR_Byte(F6x8[c][i]);
		OLED_DC_Clr();
		
	}
}
void OLED_ShowString(uint8_t x,uint8_t y,uint8_t *chr)
{
	unsigned char j=0;
	while (chr[j]!='\0')
	{		OLED_ShowChar(x,y,chr[j]);
	x+=8;
	if(x>120){x=0;y+=2;}
	j++;
	}
}
//m^n函数
uint32_t oled_pow(uint8_t m,uint8_t n)
{
	uint32_t result=1;	
	uint8_t i;
	for(i=n;i>0;i--)
	{
		result*=m; 
	}
	//while(n--)result*=m;    
	return result;
}		
//显示2个数字
//x,y :起点坐标	 
//len :数字的位数
//size:字体大小
//mode:模式	0,填充模式;1,叠加模式
//num:数值(0~4294967295);	 		  
void OLED_ShowNum(uint8_t x,uint8_t y,uint32_t num,uint8_t len,uint8_t size)
{         	
	uint8_t t,temp;
	uint8_t enshow=0;						   
	for(t=0;t<len;t++)
	{
		temp=(num/oled_pow(10,len-t-1))%10;
		if(enshow==0&&t<(len-1))
		{
			if(temp==0)
			{
				OLED_ShowChar(x+(size/2)*t,y,' ');
				continue;
			}else enshow=1; 
			
		}
		OLED_ShowChar(x+(size/2)*t,y,temp+'0'); 
	}
} 


