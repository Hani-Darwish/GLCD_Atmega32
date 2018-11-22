/*

  main.c 
  
  Graphics Test for ATMEGA Controller

  Universal 8bit Graphics Library
  
  Copyright (c) 2012, olikraus@gmail.com
  All rights reserved.

  Redistribution and use in source and binary forms, with or without modification, 
  are permitted provided that the following conditions are met:

  * Redistributions of source code must retain the above copyright notice, this list 
    of conditions and the following disclaimer.
    
  * Redistributions in binary form must reproduce the above copyright notice, this 
    list of conditions and the following disclaimer in the documentation and/or other 
    materials provided with the distribution.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND 
  CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, 
  INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF 
  MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE 
  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR 
  CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, 
  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT 
  NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; 
  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER 
  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, 
  STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) 
  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF 
  ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.  
  
*/

#include "u8g.h"

#if defined(__AVR__)
#include <avr/interrupt.h>
#include <avr/io.h>
#endif
#include "util/delay.h"

#include "MCAL/EF_UART.h"



#define KEY_NONE 0
#define KEY_PREV 1
#define KEY_NEXT 2
#define KEY_SELECT 3
#define KEY_BACK 4




static char tools_28_bits[] U8G_PROGMEM = {
  0x00, 0x00, 0x0E, 0x00, 0x00, 0x80, 0x0F, 0x00, 0x04, 0xC0, 0x0F, 0x00,
  0x0E, 0xE0, 0x07, 0x00, 0x1F, 0xE0, 0x03, 0x00, 0x3F, 0xF0, 0x03, 0x00,
  0x3E, 0xF0, 0x03, 0x00, 0x7C, 0xF0, 0x03, 0x00, 0xF0, 0xF0, 0x03, 0x0E,
  0xE0, 0xF1, 0x07, 0x0F, 0xC0, 0xF3, 0xFF, 0x0F, 0x80, 0xFF, 0xFF, 0x07,
  0x00, 0xFF, 0xFF, 0x07, 0x00, 0xFE, 0xFF, 0x03, 0x00, 0xFF, 0xFF, 0x01,
  0x80, 0xFF, 0x7F, 0x00, 0xC0, 0xFF, 0x01, 0x00, 0xE0, 0xFF, 0x01, 0x00,
  0xF0, 0xFF, 0x03, 0x00, 0xF8, 0xBF, 0x0F, 0x00, 0xFC, 0x1F, 0x3F, 0x00,
  0xFE, 0x0F, 0x7F, 0x00, 0xCF, 0x07, 0xFF, 0x00, 0xC7, 0x03, 0xFE, 0x01,
  0xE7, 0x01, 0xFE, 0x01, 0xFF, 0x00, 0xFC, 0x01, 0x7E, 0x00, 0xF8, 0x01,
0x3C, 0x00, 0xE0, 0x00, };

static char clock_28_bits[] U8G_PROGMEM = {
  0x00, 0x00, 0x00, 0x00, 0x00, 0xFE, 0x07, 0x00, 0x80, 0xFF, 0x1F, 0x00,
  0xC0, 0xFF, 0x7F, 0x00, 0xE0, 0x1F, 0xFF, 0x00, 0xF0, 0x1F, 0xFF, 0x01,
  0xF8, 0xFF, 0xFF, 0x03, 0xFC, 0xFF, 0xCF, 0x03, 0xFC, 0xFF, 0xC7, 0x07,
  0xFE, 0xFC, 0xE3, 0x07, 0xFE, 0xF8, 0xF1, 0x07, 0xFE, 0xF1, 0xF8, 0x0F,
  0xFE, 0x63, 0xFC, 0x0F, 0xCE, 0x07, 0x7E, 0x0E, 0xCE, 0x0F, 0x7F, 0x0E,
  0xCE, 0x9F, 0x7F, 0x0E, 0xFE, 0xFF, 0xFF, 0x0F, 0xFE, 0xFF, 0xFF, 0x0F,
  0xFE, 0xFF, 0xFF, 0x07, 0xFC, 0xFF, 0xFF, 0x07, 0xFC, 0xFF, 0xFF, 0x03,
  0xF8, 0xFF, 0xFF, 0x03, 0xF8, 0xFF, 0xFF, 0x01, 0xF0, 0x1F, 0xFF, 0x00,
  0xE0, 0x1F, 0x7F, 0x00, 0xC0, 0xFF, 0x3F, 0x00, 0x00, 0xFF, 0x0F, 0x00,
0x00, 0xF8, 0x03, 0x00, };

static char memory_card_28_bits[] U8G_PROGMEM = {
  0xF8, 0xFF, 0x3F, 0x00, 0xFC, 0xFF, 0x7F, 0x00, 0xFC, 0xFF, 0xFF, 0x00,
  0x3C, 0xE7, 0xFC, 0x01, 0x3C, 0xE7, 0xFC, 0x03, 0x3C, 0xE7, 0xFC, 0x03,
  0x3C, 0xE7, 0xFC, 0x03, 0x3C, 0xE7, 0xFC, 0x03, 0x3C, 0xE7, 0xFC, 0x03,
  0x3C, 0xE7, 0xFC, 0x03, 0xFC, 0xFF, 0xFF, 0x03, 0xFC, 0xFF, 0xFF, 0x01,
  0xFC, 0xFF, 0xFF, 0x01, 0xFC, 0xFF, 0xFF, 0x01, 0xFC, 0xFF, 0xFF, 0x01,
  0xFC, 0xFF, 0xFF, 0x01, 0xFC, 0xFF, 0xFF, 0x03, 0xFC, 0xFF, 0xFF, 0x03,
  0xFC, 0xFF, 0xFF, 0x03, 0x3C, 0x00, 0xE0, 0x03, 0x1C, 0x00, 0xC0, 0x03,
  0x1C, 0x00, 0xC0, 0x03, 0x1C, 0x00, 0xC0, 0x03, 0x1C, 0x00, 0xC0, 0x03,
  0x3C, 0x00, 0xE0, 0x03, 0xFC, 0xFF, 0xFF, 0x03, 0xFC, 0xFF, 0xFF, 0x03,
0xF8, 0xFF, 0xFF, 0x01, };


/* 
  Software SPI:
  uint8_t u8g_InitSPI(u8g_t *u8g, u8g_dev_t *dev, uint8_t sck, uint8_t mosi, uint8_t cs, uint8_t a0, uint8_t reset); 

  Hardware SPI:
  uint8_t u8g_InitHWSPI(u8g_t *u8g, u8g_dev_t *dev, uint8_t cs, uint8_t a0, uint8_t reset);

  Parallel Interface:
  uint8_t u8g_Init8Bit(u8g_t *u8g, u8g_dev_t *dev, uint8_t d0, uint8_t d1, uint8_t d2, uint8_t d3, uint8_t d4, uint8_t d5, uint8_t d6, uint8_t d7, 
    uint8_t en, uint8_t cs1, uint8_t cs2, uint8_t di, uint8_t rw, uint8_t reset);

  Visit 
    http://code.google.com/p/u8glib/wiki/device 
  for a list of valid devices (second argument of the constructor).

  The following examples will use the dogm132 device: u8g_dev_st7565_dogm132_sw_spi

  Note: The device must match the setup: For example, do not use a sw_spi device with u8g_InitHWSPI().
*/

u8g_t u8g;
uint8_t draw_state = 0;



uint8_t sys_key_first = KEY_NONE;
uint8_t sys_key_second = KEY_NONE;
uint8_t sys_key_code = KEY_NONE;


static UART_cfg_str uart_cfg1 = {9600, 8, ONE_STOP_BIT, NO_PARITY, FALSE, FALSE, TRUE, TRUE};


void u8g_setup(void)
{  
  /*
    Test Envionment 1, ATMEGA and DOGM132 
    CS: PORTB, Bit 2
    A0: PORTB, Bit 1
    SCK: PORTB, Bit 5
    MOSI: PORTB, Bit 3
  */
  // u8g_InitSPI(&u8g, &u8g_dev_st7565_dogm132_sw_spi, PN(1, 5), PN(1, 3), PN(1, 2), PN(1, 1), U8G_PIN_NONE);

  /*
    Test Envionment 2, ATMEGA and Seeedstudio 96x96 OLED via I2C
    SCL: Port C, Bit 5
    SDA: Port C, Bit 4
  */
  
  /* activate pull-up, set ports to output, init U8glib */  
  /*
  u8g_SetPinInput(PN(2,5)); u8g_SetPinLevel(PN(2,5), 1); u8g_SetPinOutput(PN(2,5));
  u8g_SetPinInput(PN(2,4)); u8g_SetPinLevel(PN(2,4), 1); u8g_SetPinOutput(PN(2,4));
  u8g_InitI2C(&u8g, &u8g_dev_ssd1327_96x96_gr_i2c, U8G_I2C_OPT_NONE);
  */

  /*
    Test Envionment 3, ATMEGA and NHD 192x32 ST7920 special SPI
    R/W, MOSI, Red: 	Port C, Bit 5
    RS, CS, Yellow: 	Port C, Bit 4
    EN, SCK, Green:	Port C, Bit 3
    Arguments for u8g_InitSPI are: SCK, MOSI, CS, A0, Reset
      A0 and Reset are not used.
  */
  //u8g_InitSPI(&u8g, &u8g_dev_st7920_192x32_sw_spi, PN(2, 3), PN(2, 5), PN(2, 4), U8G_PIN_NONE, U8G_PIN_NONE);

  /*
    Test Envionment 4, ATMEGA and NHD 192x32 ST7920 special SPI
    R/W, MOSI, Red: 	Port B, Bit 3
    RS, CS, Yellow: 	Port C, Bit 4
    EN, SCK, Green:	Port B, Bit 5
    Arguments for u8g_InitHWSPI are: CS, A0, Reset
      A0 and Reset are not used.
  */
//  u8g_InitHWSPI(&u8g, &u8g_dev_st7920_192x32_hw_spi, PN(2, 4), U8G_PIN_NONE, U8G_PIN_NONE);


	  u8g_InitSPI(&u8g, &u8g_dev_st7920_128x64_hw_spi, PN(1, 7), PN(1, 5), PN(1, 4), U8G_PIN_NONE, U8G_PIN_NONE);
	  u8g_InitHWSPI(&u8g, &u8g_dev_st7920_128x64_hw_spi, PN(1, 4), U8G_PIN_NONE, U8G_PIN_NONE);




	  EF_void_UART_Init(&uart_cfg1);

  /* flip screen, if required */
//  u8g_SetRot180(&u8g);

  /* assign default color value */
  if ( u8g_GetMode(&u8g) == U8G_MODE_R3G3B2 )
    u8g_SetColorIndex(&u8g, 255);     /* white */
  else if ( u8g_GetMode(&u8g) == U8G_MODE_GRAY2BIT )
    u8g_SetColorIndex(&u8g, 3);         /* max intensity */
  else if ( u8g_GetMode(&u8g) == U8G_MODE_BW )
    u8g_SetColorIndex(&u8g, 1);         /* pixel on */
}

void sys_init(void)
{
#if defined(__AVR__)
  /* select minimal prescaler (max system speed) */
//  CLKPR = 0x80;
//  CLKPR = 0x00;
#endif
}


void sys_setup_keys(void)
{
#if defined(__AVR__)
  /* configure buttons (inputs with pullup) */
  DDRD &= ~(1<<5);
  PORTD |= (1<<5);
  DDRD &= ~(1<<6);
  PORTD |= (1<<6);
  DDRD &= ~(1<<7);
  PORTD |= (1<<7);
//  DDRB &= ~(1<<7);
//  PORTB |= (1<<7);
#endif
}



void u8g_prepare(void) {

  sys_setup_keys();
  u8g_SetFont(&u8g, u8g_font_5x8r);
  u8g_SetFontRefHeightExtendedText(&u8g);
  u8g_SetDefaultForegroundColor(&u8g);
  u8g_SetFontPosTop(&u8g);
}

void u8g_box_frame(uint8_t a) {
  u8g_DrawStr(&u8g, 0, 0, "drawBox");
  u8g_DrawBox(&u8g, 5,10,20,10);
  u8g_DrawBox(&u8g, 10+a,15,30,7);
  u8g_DrawStr(&u8g, 0, 30, "drawFrame");
  u8g_DrawFrame(&u8g, 5,10+30,20,10);
  u8g_DrawFrame(&u8g, 10+a,15+30,30,7);
}

void u8g_string(uint8_t a) {
  u8g_DrawStr(&u8g, 30+a,31, " 0");
  u8g_DrawStr90(&u8g, 30,31+a, " 90");
  u8g_DrawStr180(&u8g, 30-a,31, " 180");
  u8g_DrawStr270(&u8g, 30,31-a, " 270");
}

void u8g_line(uint8_t a) {
  u8g_DrawStr(&u8g, 0, 0, "drawLine");
  u8g_DrawLine(&u8g, 7+a, 10, 40, 55);
  u8g_DrawLine(&u8g, 7+a*2, 10, 60, 55);
  u8g_DrawLine(&u8g, 7+a*3, 10, 80, 55);
  u8g_DrawLine(&u8g, 7+a*4, 10, 100, 55);
}

void u8g_ascii_1(void) {
  char s[2] = " ";
  uint8_t x, y;
  u8g_DrawStr(&u8g, 0, 0, "ASCII page 1");
  for( y = 0; y < 6; y++ ) {
    for( x = 0; x < 16; x++ ) {
      s[0] = y*16 + x + 32;
      u8g_DrawStr(&u8g, x*7, y*10+10, s);
    }
  }
}

void u8g_ascii_2(void) {
  char s[2] = " ";
  uint8_t x, y;
  u8g_DrawStr(&u8g, 0, 0, "ASCII page 2");
  for( y = 0; y < 6; y++ ) {
    for( x = 0; x < 16; x++ ) {
      s[0] = y*16 + x + 160;
      u8g_DrawStr(&u8g, x*7, y*10+10, s);
    }
  }
}


void U8g_welcom(void)
{

	  u8g_DrawStr(&u8g,50,15, "Welcome");
	  u8g_DrawStr(&u8g,40,31, "Embeddedfab");

	  u8g_DrawXBMP(&u8g,10,15,28,28,tools_28_bits);

//	  _delay_ms(150);


}

uint8_t sys_get_key(void)
{
  uint8_t volatile result = KEY_NONE;

//  cli();

#if defined(__AVR__)
  if ( (PIND & (1<<5)) == 0 )
	  result = KEY_NEXT;

  if ( (PIND & (1<<6)) == 0 )
	  result = KEY_PREV;

  if ( (PIND & (1<<7)) == 0 )
    result |= KEY_SELECT;
//  if ( (PINB & (1<<7)) == 0 )
//    result |= KEY_BACK;
#endif

//  EF_void_UART_SendString("the key pressed ");
//   EF_void_UART_Send_Integer(result);
//   EF_void_UART_SendString("\n");



  return result;



}



void sys_debounce_key(void)
{
  sys_key_second = sys_key_first;
  sys_key_first = sys_get_key();

  if ( sys_key_second == sys_key_first )
    sys_key_code = sys_key_first;
  else
    sys_key_code = KEY_NONE;
}


#define MENU_ITEMS 4
char *menu_strings[MENU_ITEMS] = { "1) Enter Machine Number", "2) Enter User Name ", "3) Enter Your Password", "4) Home Menu" };

volatile uint8_t menu_current = 0;
volatile uint8_t menu_redraw_required = 0;
uint8_t last_key_code = KEY_NONE;

void draw_menu(void)
{
  uint8_t i, h;
  u8g_uint_t w, d;

//  u8g_SetFont(&u8g, u8g_font_5x7);
  u8g_SetFontRefHeightText(&u8g);
  u8g_SetFontPosTop(&u8g);


  u8g_DrawStr(&u8g, 15, 3,"Setup Menu");


  h = u8g_GetFontAscent(&u8g)-u8g_GetFontDescent(&u8g);
  w = u8g_GetWidth(&u8g);
  for( i = 0; i < MENU_ITEMS; i++ )
  {
//    d = (w-u8g_GetStrWidth(&u8g, menu_strings[i]))/2;
    u8g_SetDefaultForegroundColor(&u8g);
    if ( i == menu_current )
    {
      u8g_DrawBox(&u8g, 0, (i+2)*h+1, w, h);
      u8g_SetDefaultBackgroundColor(&u8g);

    }
//    u8g_DrawStr(&u8g, d, i*h, menu_strings[i]);
    u8g_DrawStr(&u8g, 0, ((i+2)*h), menu_strings[i]);

  }

}




void draw(void) {


//	EF_void_UART_SendString("draw state ");
//	EF_void_UART_Send_Integer(draw_state >> 3);
//	EF_void_UART_SendString("\n");


//  switch(draw_state >> 3)
  switch(draw_state)
  {


//  case 0: draw_menu();break;


      case 0: U8g_welcom(); break;
  //    case 1: u8g_box_frame(draw_state&7); break;
  //    case 2: u8g_string(draw_state&7); break;
  //    case 3: u8g_line(draw_state&7); break;
      case 1: draw_menu();break;
  //    case 3: u8g_ascii_1(); break;
  //    case 4: u8g_ascii_2(); break;
  }
}




void update_menu(void)
{

	if ( sys_key_code != KEY_NONE && last_key_code == sys_key_code )
	{
		return;
	}
	last_key_code = sys_key_code;

	switch ( sys_key_code )
	{
	case KEY_NEXT:
		menu_current++;
		if ( menu_current >= MENU_ITEMS )
			menu_current = 0;
		menu_redraw_required = 1;
		break;
	case KEY_PREV:
		if ( menu_current == 0 )
			menu_current = MENU_ITEMS;
		menu_current--;
		menu_redraw_required = 1;
		break;

	case KEY_SELECT:

		switch(menu_current)
		{
		case 0:
			u8g_FirstPage(&u8g);
			do{
				u8g_DrawStr(&u8g,30,5, "Memory Card");
				u8g_DrawXBMP(&u8g,50,25,28,28,memory_card_28_bits);


			}while ( u8g_NextPage(&u8g) );
			break;

		case 1:

			u8g_FirstPage(&u8g);
			do{
				  u8g_DrawStr(&u8g,45,5,"Tool");
				  u8g_DrawXBMP(&u8g,45,25,28,28,tools_28_bits);


			}while ( u8g_NextPage(&u8g) );

			break;

		case 3:

			EF_void_UART_SendString("select the fourth item\n");

			u8g_prepare();



			u8g_FirstPage(&u8g);
			do{

				  u8g_DrawStr(&u8g,35,15,"Thank You");
				  u8g_DrawStr(&u8g,55,30,":)");


//				u8g_DrawXBMP(&u8g,10,15,28,28,clock_28_bits);

			}while ( u8g_NextPage(&u8g) );
//			_delay_ms(50);
			break;


		case 2:
			u8g_FirstPage(&u8g);
			do{
				 u8g_DrawStr(&u8g,40,5,"Clock");
				 u8g_DrawXBMP(&u8g,40,25,28,28,clock_28_bits);
			}while ( u8g_NextPage(&u8g) );
			break;


			//    	default:
			//    		break;
		}/* end inner SW  */

//		break;
	}//outer SW
}




int main(void)
{
	sys_init();
	u8g_setup();
	menu_redraw_required = 1;
	u8g_prepare();

	 u8g_FirstPage(&u8g);
	 do{
		 U8g_welcom();
	 }while ( u8g_NextPage(&u8g) );
	  _delay_ms(1000);


	for(;;)
	{
		sys_debounce_key();

		if(menu_redraw_required != 0 )
		{
			u8g_FirstPage(&u8g);
			do
			{
				draw_menu();

			} while ( u8g_NextPage(&u8g) );

			menu_redraw_required = 0;
		}

		update_menu();

//		EF_void_UART_SendString("update menu redraw ");
//		EF_void_UART_Send_Integer(menu_redraw_required);
//		EF_void_UART_SendString("\n");



//		draw_state++;
//		if ( draw_state >= 2 )
//			draw_state = 0;

		u8g_Delay(100);
	}
}
