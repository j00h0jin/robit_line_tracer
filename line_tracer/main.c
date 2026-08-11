/*
 * line_tracer.c
 *
 * Created: 2026-08-10 오후 4:33:32
 * Author : hojin
 */ 

#define F_CPU 16000000UL

#define arrSize 3
#define indexIR 6

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>

// PB0, 1 모터1 방향 제어
// PB2, 3 모터2 방향 제어
// PB5, 6 모터1 속도, 모터2 속도
#define motor1Forward()  {PORTB &= ~(1<<PB0); PORTB |= (1<<PB1);}
#define motor1Backward() {PORTB &= ~(1<<PB1); PORTB |= (1<<PB0);}
#define motor1Stop()     {PORTB &= ~((1<<PB0)|(1<<PB1));}
#define motor2Forward()  {PORTB |= (1<<PB2); PORTB &= ~(1<<PB3);}
#define motor2Backward() {PORTB |= (1<<PB3); PORTB &= ~(1<<PB2);}
#define motor2Stop()     {PORTB &= ~((1<<PB2)|(1<<PB3));}

void set_speed(unsigned char speed_1, unsigned char speed_2);

volatile unsigned int adc_value[6];
volatile int current_idx = 0;

ISR(ADC_vect)
{
	adc_value[current_idx] = ADC;
	current_idx = (current_idx + 1) % 6;
	ADMUX = (ADMUX & 0xE0) | ((current_idx + 2) & 0x1F);

	ADCSRA |= (1 << ADSC);
}

int main(void)
{
	ADMUX = 0x40; // 0100 0000
	// ADEN(활성화), ADIE(인터럽트 허용), 분주비 128
	ADCSRA = (1 << ADEN) | (1 << ADIE) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);

	sei(); _delay_ms(10);// 전역 인터럽트 활성화
	

	// 첫 변환 시작
	ADCSRA |= (1 << ADSC);
	
	DDRB = 0x6F; // 0110 1111 (PB6, 5, 3, 2, 1, 0)
	// non-inverting mode A B, Fast PWM, 8-bit mode(WGM), 분주비 64(CS)
	TCCR1A = (1 << COM1A1) | (1 << COM1B1) | (1 << WGM10);
	TCCR1B = (1 << WGM12) | (1 << CS11) | (1 << CS10);
	
	int moveAvgArr[indexIR][arrSize]; // 이동 평균 필터에 사용할 값 저장
	int moveAvgFilterValue[indexIR]; // 필터링 값
	
	int minMax[indexIR][2]; // 정규화에 필요한 포트별 min, max 값 저장
	
	float normalization[indexIR] = {0}; // 정규화 값
		
	int sum = 0;

	for (int i = 0; i < indexIR; i++) // min, max 초기값 설정
	{
		minMax[i][0] = 1024;
		minMax[i][1] = -1;
	}
	
	for (int i = 0; i < indexIR; i++) // while문 전 평균값을 낼 데이터들을 채움
	{
		for (int j = 0; j < arrSize; j++)
		{
			moveAvgArr[i][j] = adc_value[i];
			// min, max 판별
			if(moveAvgArr[i][j] < minMax[i][0])
			minMax[i][0] = moveAvgArr[i][j];
			if(moveAvgArr[i][j] > minMax[i][1])
			minMax[i][1] = moveAvgArr[i][j];
			
		}
	}
	
	_delay_ms(10);

	while (1)
	{
		for (int i = 0; i < indexIR; i++)
		{
			sum = 0;
			for(int j = arrSize - 1; j > 0; j--)
			{
				// 한 칸씩 밀기 a, b, c => a, a, b
				moveAvgArr[i][j] = moveAvgArr[i][j-1];
			}
			// New_value, a, b
			moveAvgArr[i][0] = adc_value[i];
			
			// min, max 판별
			if(moveAvgArr[i][0] < minMax[i][0])
				minMax[i][0] = moveAvgArr[i][0];
			else minMax[i][0]++;
			if(moveAvgArr[i][0] > minMax[i][1])
				minMax[i][1] = moveAvgArr[i][0];
			else minMax[i][1]--;
			
			// sum 구한 후 avg에 넣기
			for (int j = 0; j < arrSize; j++)
			{
				sum += moveAvgArr[i][j];
			}
			moveAvgFilterValue[i] = sum / arrSize;
		}
		
		for (int i = 0; i < indexIR; i++)
		{
			float temp;
			temp = minMax[i][1] - minMax[i][0]; // max - min 저장
			if(temp == 0) // 초기에 max - min이 0인 경우 0으로 나눌 수 없으므로 정규화 값 0으로 설정
			{
				normalization[i] = 0;
				continue;
			}
			normalization[i] = (float)(moveAvgFilterValue[i]-minMax[i][0]) / temp;
		}
		
		if(normalization[2] >= 0.8 && normalization[3] >= 0.8)
		{
			motor1Forward();
			motor2Forward();
			set_speed(100, 100);	
		}
		else if(normalization[2] >= 0.8 && normalization[3] <= 0.8)
		{
			motor1Forward();
			motor2Forward();
			set_speed(60, 120);
		}
		else if(normalization[2] <= 0.8 && normalization[3] >= 0.8)
		{
			motor1Forward();
			motor2Forward();
			set_speed(120, 60);
		}
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


