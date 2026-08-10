/*
 * line_tracer.c
 *
 * Created: 2026-08-10 오후 4:33:32
 * Author : hojin
 */ 

#define F_CPU 16000000UL
#include <avr/io.h>
#include <util/delay.h>

// PB0, 1 모터 1 방향 제어
// PB2, 3 모터 2 방향 제어
// PB5, 6 모터 1 속도, 모터 2 속도
#define motor1Forward()  {PORTB &= ~(1<<PB0); PORTB |= (1<<PB1);}
#define motor1Stop() {PORTB &= ~((1<<PB0) | (1<<PB1)); }
#define motor2Forward()  {PORTB |= (1<<PB2); PORTB &= ~(1<<PB3);}
#define motor2Stop() {PORTB &= ~((1<<PB2) | (1<<PB3));}

void set_speed(unsigned char speed_1, unsigned char speed_2);

int main(void)
{
	DDRB = 0x6F; // 0110 1111 (PB6, 5, 3, 2, 1, 0)
	// non-inverting mode A B, Fast PWM, 8-bit mode(WGM), 분주비 64(CS)
	TCCR1A = (1 << COM1A1) | (1 << COM1B1) | (1 << WGM10);
	TCCR1B = (1 << WGM12) | (1 << CS11) | (1 << CS10);

	while (1)
	{
		// 전진
		motor1Forward();
		motor2Forward();
		set_speed(200, 200);
		_delay_ms(4000);
		// 정지
		motor1Stop();
		motor2Stop();
		set_speed(0, 0);
		_delay_ms(1000);
	}
	return 0;
}

void set_speed(unsigned char speed_1, unsigned char speed_2)
{
	OCR1A = speed_1;
	OCR1B = speed_2;
	// timer 1번 사용했으므로 OCR1에 속도 지정
	// 모터 속도 (0 ~ 255)
}


