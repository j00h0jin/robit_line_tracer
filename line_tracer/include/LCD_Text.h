#define F_CPU 16000000UL

#ifndef LCD_TEXT_H_
#define LCD_TEXT_H_

#include <avr/io.h>
#include <util/delay.h>
#include <stdint.h>
#include <stdio.h>


//==============================
// Software I2C Pin Definition
//==============================
#define I2C_DDR     DDRD
#define I2C_PORT    PORTD
#define I2C_PIN     PIND

#define SDA         PD1
#define SCL         PD0


//==============================
// PCF8574T Address
//==============================
#define LCD_ADDR    0x27


//==============================
// LCD Function
//==============================
void lcdInit(void);
void lcdClear(void);

void lcdString(uint8_t row, uint8_t col, char *str);
void lcdNumber(uint8_t row, uint8_t col, int number);
void lcdFloat(uint8_t row,uint8_t col,float number, uint8_t decimals);


#endif