#ifndef __OLED_H_
#include <stdint.h>
#define __OLED_H_

#define OLED_SCLK_Clr() GPIO_ResetBits(OLED_SPIx_SCK_GPIO_PORT,OLED_SPIx_SCK_PIN)//CLK
#define OLED_SCLK_Set() GPIO_SetBits(OLED_SPIx_SCK_GPIO_PORT,OLED_SPIx_SCK_PIN)

#define OLED_SDIN_Clr() GPIO_ResetBits(OLED_SPIx_MOSI_GPIO_PORT,OLED_SPIx_MOSI_PIN)//DIN
#define OLED_SDIN_Set() GPIO_SetBits(OLED_SPIx_MOSI_GPIO_PORT,OLED_SPIx_MOSI_PIN)

#define OLED_RST_Clr() GPIO_ResetBits(OLED_RESET_GPIO_PORT,OLED_RESET_PIN)//RES
#define OLED_RST_Set() GPIO_SetBits(OLED_RESET_GPIO_PORT,OLED_RESET_PIN)

#define OLED_DC_Clr() GPIO_ResetBits(OLED_DC_GPIO_PORT,OLED_DC_PIN)//DC
#define OLED_DC_Set() GPIO_SetBits(OLED_DC_GPIO_PORT,OLED_DC_PIN)
 		     
#define OLED_CMD  0	//写命令
#define OLED_DATA 1	//写数据
#define OLED_FONTSIZE 8
#define Max_Column	128
#define Max_Row		64

extern int8_t display_data_buff[128];
extern uint8_t display_data_buff_cnt;
extern uint8_t display_data_flag;
extern void OLED_Init(void);
extern void OLED_Clear(void);
extern void oled_display(void);
extern void OLED_DrawBMP(unsigned char x0, unsigned char y0,unsigned char x1, unsigned char y1,unsigned char BMP[]);
extern void OLED_ShowCHinese(uint8_t x,uint8_t y,uint8_t no);
extern void OLED_ShowCHinese_32(uint8_t x,uint8_t y,uint8_t no);
extern void OLED_ShowChar_32(uint8_t x,uint8_t y,uint8_t chr);
extern void OLED_ShowString(uint8_t x,uint8_t y,uint8_t *chr);
extern void OLED_ShowNum(uint8_t x,uint8_t y,uint32_t num,uint8_t len,uint8_t size);

#endif
