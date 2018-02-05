#include <stdint.h>
#include <stdarg.h>
#include <delay.h>
#include <lcd.h>

void lcdInit(void) {
   LCDPowerUpInit((uint8_t *)0);
   LCDSetBacklight(BACKLIGHT_ON);
   dly100us(5000);
   LCDSetFont( &Terminal_9_12_6, BLACK, WHITE );
   LCDSetWindow( 0, 0, 131, 131 );
   printf( "\f" );              /* Clear screen */
}

void lcdSetTextPos(uint8_t x, uint8_t y) {
  LCDTextSetPos(x, y);
}

void lcdDrawPixel(uint32_t x, uint32_t y, uint32_t colour) {
  LCDDrawPixel(x, y, colour);
}

void lcdDrawLine(uint32_t x1, uint32_t y1, uint32_t x2, uint32_t y2, uint32_t colour) {
  LCDDrawLine(x1,y1,x2,y2,colour);
}

void lcdDrawRectangle(uint32_t x1, uint32_t y1, uint32_t x2, uint32_t y2, uint32_t colour) {
  LCDDrawRectangle(x1,y1,x2,y2,colour);
}

void lcdDrawFilledRectangle(uint32_t x1, uint32_t y1, uint32_t x2, uint32_t y2, uint32_t colour) {
  LCDDrawFilledRectangle(x1,y1,x2,y2,colour);
}

void lcdDrawCircle(uint32_t x, uint32_t y, uint32_t radius, uint32_t colour) {
  LCDDrawCircle(x, y, radius, colour);
}

void lcdDrawFilledCircle(uint32_t x, uint32_t y, uint32_t radius, uint32_t colour) {
  LCDDrawFilledCircle(x, y, radius, colour);
}
