#ifndef __OLED_H
#define __OLED_H

#include "main.h"
#include "i2c.h"
#include <stdint.h>

/* OLED basic parameters */
#define OLED_WIDTH              128
#define OLED_HEIGHT             64
#define OLED_PAGES              8

/* SSD1306 I2C address: usually 0x3C or 0x3D */
#define OLED_I2C_ADDR           (0x3C << 1)

/* OLED pixel color */
#define OLED_COLOR_BLACK        0
#define OLED_COLOR_WHITE        1

/* API */
void OLED_Init(void);
void OLED_Clear(void);
void OLED_Fill(uint8_t data);
void OLED_Refresh(void);

void OLED_DrawPoint(uint8_t x, uint8_t y, uint8_t color);

void OLED_ShowChar(uint8_t x, uint8_t y, char chr);
void OLED_ShowString(uint8_t x, uint8_t y, char *str);

#endif
