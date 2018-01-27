#include "common.h"
#include "config.h"
#include "lcdspi.h"
#include "lcd1202.h"

FONT	cfont;

void lcd_init(void){
   lcdspi_config();
   lcd_reset();
   lcd_writeByte(0xE2, LCD_COMMAND); // Reset
   delayMs(20);
   lcd_writeByte(0xA4, LCD_COMMAND); // Power saver off
   lcd_writeByte(0x2F, LCD_COMMAND); // Power control set
   lcd_writeByte(0xAF, LCD_COMMAND); // LCD display on
   //lcd_reverse(1);
   lcd_clear();
   }


void lcd_clear(void){
    int i;
    lcd_home();
    for(i = 0; i < 96*9; i++){
        lcd_writeByte(0x00, LCD_DATA);
        }
    }


void lcd_reverse(unsigned char inv){
    if(inv)
        lcd_writeByte(0xA7,LCD_COMMAND); // reverse display
    else
        lcd_writeByte(0xA6, LCD_COMMAND); // normal display
    }


void lcd_startLine(unsigned char line){
    lcd_writeByte(0x40 | (line & 0x3F), LCD_COMMAND);
    }


void lcd_home(void){
	lcd_writeByte(0xB0, LCD_COMMAND); // Page address set
   lcd_writeByte(0x10, LCD_COMMAND); // Sets the DDRAM column address - upper 3-bit
   lcd_writeByte(0x00, LCD_COMMAND); // lower 4-bit
   }


void lcd_setRow(unsigned char row){
	lcd_writeByte(0xB0 | (row & 0x0F), LCD_COMMAND); // Page address set
    }


void lcd_setCol(unsigned char col){
    lcd_writeByte(0x10 | (col>>4), LCD_COMMAND); // Sets the DDRAM column address - upper 3-bit
    lcd_writeByte(0x00 | (col & 0x0F), LCD_COMMAND); // lower 4-bit
    }


void lcd_reset(void){
    LCDRST_LO();
    delayMs(20);
    LCDRST_HI();
    delayMs(10);
    }

// most spi interface lcds use separate data/command pin
// 1202 expects 9bit spi transmission (data/command bit + 8bits data)
void lcd_writeByte(unsigned char data, unsigned char dc){
    uint16_t val = (uint16_t)data |  (((uint16_t)dc)<<8);
    val <<= 7; // spi module expects left justified bits
    lcdspi_write9(val);
    }


void lcd_print(char *st, int x, int y){
	int stl, row;
    stl = strlen(st);
    if (x == RIGHT)
        x = 96-(stl*cfont.width);
    else
    if (x == CENTER)
        x = (96-(stl*cfont.width))/2;
    else
    if (x < 0)
        x = 0;
    row = y / 8;
    int cnt;
    for (cnt=0; cnt<stl; cnt++)
        lcd_printChar(*st++, x + (cnt*(cfont.width)), row);
    }


void lcd_printf(int x, int y, char* format, ...)    {
	char szbuf[80];
   va_list args;
   va_start(args,format);
   vsprintf(szbuf,format,args);
   va_end(args);
   lcd_print(szbuf, x, y);
   }	



void lcd_printChar(unsigned char c, int x, int row){
	if (((x+cfont.width)<=96) && (row+(cfont.height/8)<= 8)){
        int rowcnt;
        for (rowcnt=0; rowcnt<(cfont.height/8); rowcnt++){
            lcd_setRow(row+rowcnt);
            lcd_setCol(x);
            int font_idx = ((c - cfont.offset)*(cfont.width*(cfont.height/8)))+4;
            int cnt;
            for(cnt=0; cnt<cfont.width; cnt++)	{
                if (cfont.inverted==0)
                    lcd_writeByte(cfont.font[font_idx+cnt+(rowcnt*cfont.width)], LCD_DATA);
                else
                    lcd_writeByte(~(cfont.font[font_idx+cnt+(rowcnt*cfont.width)]), LCD_DATA);
                }
            }
        }
    }


void lcd_setFont(const uint8_t* font){
	cfont.font = font;
	cfont.width = font[0];
	cfont.height = font[1];
	cfont.offset = font[2];
	cfont.numchars = font[3];
	cfont.inverted = 0;
   }


