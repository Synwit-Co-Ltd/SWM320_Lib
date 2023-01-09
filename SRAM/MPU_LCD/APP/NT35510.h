#ifndef __NT35510_H__
#define __NT35510_H__


void TFTInit(void);
void TFTSetCursor(uint16_t x, uint16_t y);
void TFTDrawPoint(uint16_t x, uint16_t y, uint16_t rgb);
uint16_t TFTReadPoint(uint16_t x, uint16_t y);
void TFTClear(uint16_t rgb);

void LCD_I80_WriteReg(uint16_t reg, uint16_t data);
uint16_t LCD_I80_ReadReg(uint16_t reg);


#endif //__NT35510_H__
