#ifndef __LCD_H
#define __LCD_H

#include <stdint.h>
#include <stdio.h>
#include <drv_glcd.h>

extern FontType_t Terminal_9_12_6;

#define lcdWrite printf

void lcdInit(void);
void lcdSetTextPos(uint8_t x, uint8_t y);

void lcdDrawPixel(uint32_t x, uint32_t y, uint32_t colour);
void lcdDrawLine(uint32_t x1, uint32_t y1, uint32_t x2, uint32_t y2, uint32_t colour);
void lcdDrawRectangle(uint32_t x1, uint32_t y1, uint32_t x2, uint32_t y2, uint32_t colour);
void lcdDrawFilledRectangle(uint32_t x1, uint32_t y1, uint32_t x2, uint32_t y2, uint32_t colour);
void lcdDrawCircle(uint32_t x, uint32_t y, uint32_t radius, uint32_t colour);
void lcdDrawFilledCircle(uint32_t x, uint32_t y, uint32_t radius, uint32_t colour);

#endif