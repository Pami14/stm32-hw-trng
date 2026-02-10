/*
	Source: ST Community
	STM32 i2c LCD Display 
	Communications Source code 
	Downloadlink: 
	https://community.st.com/servlet/JiveServlet/download/2001-4-11593/Nucleo_L4_I2C_LCD_HD44780_v2.zip 

    Modified & corrected an error in cursor positioning:
    Herbert Paulis, FH Campus Wien University of Applied Sciences, 2020, 2022
*/

// Adopt according to STM32 version used (l0, f4, f7...):
#include "stm32f4xx_hal.h"

// Slave address is commonly (0x27 << 1) or (0x3F << 1) for 2004 type I2C displays
// I2C must be defined with STM32 Cube IDE/MX or manually by user (with HAL)
void lcd_init (I2C_HandleTypeDef * i2cHandleAddress, uint16_t SlaveAddress);

void lcd_disp_on(void);
void lcd_disp_off(void);
void lcd_home(void);
void lcd_clear_display(void);
void lcd_shift_left(void);
void lcd_shift_right(void);
void lcd_locate(uint8_t row, uint8_t column);

void lcd_printchar(unsigned char ascode);
void lcd_print_string (char *str);  // print string to the LCD
void lcd_printf(const char *fmt, ...);
