#include "include/LCD_Text.h"

// PCF8574T
#define LCD_RS         0x01
#define LCD_RW         0x02
#define LCD_ENABLE     0x04
#define LCD_BACKLIGHT  0x08



// Software I2C Delay
static void I2C_Delay(void)
{
	_delay_us(5);
}


static void SDA_HIGH(void)
{
	I2C_DDR &= ~(1<<SDA);
	I2C_PORT |= (1<<SDA);
}


static void SDA_LOW(void)
{
	I2C_DDR |= (1<<SDA);
	I2C_PORT &= ~(1<<SDA);
}


static void SCL_HIGH(void)
{
	I2C_DDR &= ~(1<<SCL);
	I2C_PORT |= (1<<SCL);
}


static void SCL_LOW(void)
{
	I2C_DDR |= (1<<SCL);
	I2C_PORT &= ~(1<<SCL);
}



// I2C
static void I2C_Start(void)
{
	SDA_HIGH();
	SCL_HIGH();

	I2C_Delay();

	SDA_LOW();

	I2C_Delay();

	SCL_LOW();
}

static void I2C_Stop(void)
{
	SDA_LOW();

	SCL_HIGH();

	I2C_Delay();

	SDA_HIGH();

	I2C_Delay();
}

static void I2C_Write(uint8_t data)
{
	uint8_t i;


	for(i=0;i<8;i++)
	{
		if(data & 0x80)
		SDA_HIGH();
		else
		SDA_LOW();


		SCL_HIGH();

		I2C_Delay();

		SCL_LOW();

		data <<= 1;
	}


	// ACK
	SDA_HIGH();

	SCL_HIGH();

	I2C_Delay();

	SCL_LOW();
}

static void PCF8574_Write(uint8_t data)
{
	I2C_Start();

	I2C_Write((LCD_ADDR<<1)|0);

	I2C_Write(data);

	I2C_Stop();
}

// LCD 
static void LCD_Enable(uint8_t data)
{
	PCF8574_Write(data | LCD_ENABLE);

	_delay_us(1);

	PCF8574_Write(data & ~LCD_ENABLE);

	_delay_us(50);
}

static void LCD_Send4(uint8_t data)
{
	data |= LCD_BACKLIGHT;

	LCD_Enable(data);
}

static void LCD_Command(uint8_t cmd)
{
	uint8_t high;
	uint8_t low;


	high = cmd & 0xF0;
	low  = (cmd<<4)&0xF0;


	LCD_Send4(high);
	LCD_Send4(low);

	_delay_ms(2);
}

static void LCD_Data(uint8_t data)
{
	uint8_t high;
	uint8_t low;


	high = data & 0xF0;
	low  = (data<<4)&0xF0;


	LCD_Send4(high | LCD_RS);
	LCD_Send4(low  | LCD_RS);

	_delay_us(50);
}


// custom
void lcdInit(void)
{
	// SDA/SCL release
	SDA_HIGH();
	SCL_HIGH();


	_delay_ms(50);


	// 4bit mode init

	LCD_Send4(0x30);
	_delay_ms(5);

	LCD_Send4(0x30);
	_delay_us(150);

	LCD_Send4(0x30);

	LCD_Send4(0x20);


	LCD_Command(0x28); // 4bit, 2line

	LCD_Command(0x0C); // Display ON

	LCD_Command(0x06); // Entry mode

	LCD_Command(0x01); // Clear


	_delay_ms(5);
}



void lcdClear(void)
{
	LCD_Command(0x01);

	_delay_ms(2);
}


static void lcdGoto(uint8_t row,uint8_t col)
{
	uint8_t addr;


	if(row==0)
	addr=0x00;
	else
	addr=0x40;


	addr += col;


	LCD_Command(0x80 | addr);
}


void lcdString(uint8_t row,uint8_t col,char *str)
{
	lcdGoto(row,col);


	while(*str)
	{
		LCD_Data(*str);

		str++;
	}
}


void lcdNumber(uint8_t row,uint8_t col,int number)
{
	char buf[16];


	lcdGoto(row,col);


	sprintf(buf, "%2d", number);


	lcdString(row,col,buf);
}

void lcdFloat(uint8_t row,uint8_t col,float number, uint8_t decimals)
{
	char buf[16];


	lcdGoto(row,col);

	dtostrf(number, 1, decimals, buf);

	lcdString(row,col,buf);
}