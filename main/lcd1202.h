/**
 * Nokia 1202/1203/1280 LCD display library
 *
 * (CC) 2011 Ivan A-R <ivan@tuxotronic.org>
 */
// modified HN


// 96 x 68 pixels, effectively used as 96x64


#ifndef LCD1202_H_
#define LCD1202_H_

#define LEFT      0
#define RIGHT     9999
#define CENTER    9998

#define LCD_COMMAND  0
#define LCD_DATA     1


typedef struct FONT_ {
	const uint8_t* font;
	uint8_t width;
	uint8_t height;
	uint8_t offset;
	uint8_t numchars;
	uint8_t inverted;
} FONT;

void lcd_reset(void);
void lcd_writeByte(unsigned char data, unsigned char dc);
void lcd_setRow(unsigned char row);
void lcd_setCol(unsigned char col);

void	lcd_print(char *st, int x, int y);
void	lcd_setFont(const uint8_t* font);
void	lcd_printChar(unsigned char c, int x, int row);

void lcd_printf(int x, int y, char* format, ...);
void lcd_init(void);
void lcd_clear(void);
void lcd_home(void);
void lcd_startLine(unsigned char line);

void lcd_reverse(unsigned char inv);

#endif



