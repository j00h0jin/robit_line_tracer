/*
 * line_tracer.c
 *
 * Created: 2026-08-10 오후 4:33:32
 * Author : hojin
 */ 

#define F_CPU 16000000UL

#define arrSize 2
#define indexIR 6

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include "include/LCD_Text.h"

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

volatile unsigned int adc_PSD_value[2]; // [1] 사용
volatile unsigned int adc_value[6];
volatile int current_idx = 0;

int full_count = 0;
int prev_is_Full = 0;

volatile unsigned int ms_count = 0;
volatile unsigned char print_flag = 0;
ISR(TIMER0_OVF_vect) // timer0 interrupt
{
	// 클럭 / 분주비 = 250KHz (16MHz / 64)
	// 1주기 = 4us ( 1 / 250K )
	// 4us * 250 = 1ms
	TCNT0 = 256 - 250; // 250번 count ((256 - 250) ~ 256)
	ms_count++;
	
	if (ms_count >= 333-5) // 주기
	{
		ms_count = 0;
		print_flag = 1;
	}
}

ISR(ADC_vect)
{
	if(current_idx < 2){ adc_PSD_value[current_idx] = ADC; }
	else{ adc_value[current_idx - 2] = ADC; }
	
	current_idx = (current_idx + 1) % 8;
	ADMUX = (ADMUX & 0xE0) | (current_idx & 0x07);

	ADCSRA |= (1 << ADSC);
}

int main(void)
{
	ADMUX = 0x40; // 0100 0000
	// ADEN(활성화), ADIE(인터럽트 허용), 분주비 128
	ADCSRA = (1 << ADEN) | (1 << ADIE) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);
	
	TCCR0 = (1 << CS02); // 분주비 64
	TCNT0 = 256 - 250;
	TIMSK |= (1 << TOIE0); // timer0 interrupt 활성화


	sei(); _delay_ms(10);// 전역 인터럽트 활성화
	
	// 첫 변환 시작
	ADCSRA |= (1 << ADSC);
	// 0110 1111 (PB6, 5, 3, 2, 1, 0)
	DDRB = 0x6F;
	DDRE = 0x02; // E0 입력, E1 출력
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
	
	for (int i = 0; i < indexIR; i++) // 루프 전 평균값을 낼 데이터들을 채움
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
	
	// PD 제어
	float Kp = 45.0f; // 비례
	float Kd = 35.0f; // 적분
	int base_speed = 95;
	float last_error = 0.0f; // PD control prev error
	int last_turn_dir = 0; // -1: 좌회전, 1: 우회전
	
	int missing_count = 0; // 000000일 때 count됨
	int basic_mode = 1; 
	
	int is_8 = 0;
	int cross_state = 0; // 중앙 센서 최소 하나 포함 검은색 3개 이상
	int is_cross_pass = 0; // 역회전
	int is_8_flag = 0; // 8 코스 종료
	
	int is_line_course = 0; // 차선 코스 시작
	int is_line_course_end = 0; // 차선 코스 끝 부분 도달
	int is_line_flag = 0; // 차선 코스 종료
	
	int is_stop_bar = 0; // 차단바 구간
	int is_stop_bar_flag = 0;
	
	int is_parking = 0; // 주차 구간
	int turn_left = 0; // 좌회전
	int is_parking_end = 0;
	int is_parking_flag = 0;
	
	// PSD
	float voltage[2] = {0};
	
	lcdInit();
	lcdClear();
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
			//else minMax[i][0]++;
			if(moveAvgArr[i][0] > minMax[i][1])
				minMax[i][1] = moveAvgArr[i][0];
			//else minMax[i][1]--;
			
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
			normalization[i] = 1.0f - (float)(moveAvgFilterValue[i]-minMax[i][0]) / temp;
		}
		
		// PSD
		for (int i = 0; i < 2; i++)
		{
			voltage[i] = ((float)adc_PSD_value[i] * 5) / 1023;
		}
		
		// 라인 트레이싱 알고리즘
		
		int detect_count = 0;
		for (int i = 0; i < indexIR; i++) {
			if (normalization[i] >= 0.3f) detect_count++;
		}

		// 센서 이진화 (0.4 이상 1, 미만 0)
		int bin[6] = {0};
		for (int i = 0; i < 6; i++) {
			if (normalization[i] >= 0.4f) bin[i] = 1;
		}

		// 변수
		int is_center = (bin[2] || bin[3]); // 중앙 센서 반응
		int is_fork = (bin[0] || bin[1] || bin[4] || bin[5]) && (detect_count >= 3);
		int is_full = bin[0] && bin[1] && bin[2] && bin[3] && bin[4] && bin[5];
		int is_left = (normalization[0] >= 0.7f && normalization[1] >= 0.7f)||(normalization[1] >= 0.7f && normalization[2] >= 0.7f);
		int is_right = (normalization[4] >= 0.7f && normalization[5] >= 0.7f) || (normalization[3] >= 0.7f && normalization[4] >= 0.7f);
		
		if(is_full && !prev_is_Full) full_count++;
		prev_is_Full = is_full;
		
		if(full_count >= 3) is_8 = 1;
		
		// 8자
		if(is_8 && !is_cross_pass)
		{
			if (is_center && detect_count >= 3 ) {cross_state = 1;}
			if (cross_state == 1 && is_fork)
			{
				cross_state = 0;
				motor1Forward(); motor2Forward();
				if (last_turn_dir == -1) {set_speed(80, 130);}
				else  {set_speed(130, 80);}
				_delay_ms(250);
				// 8자 다음원으로 넘어가기 위해 반대 방향으로 걸어줌
				if (last_turn_dir == -1) {
					motor1Forward(); motor2Forward();
					set_speed(150, 100);
					last_turn_dir = 1;
					} else {
					motor1Forward(); motor2Forward();
					set_speed(100, 150); 
					last_turn_dir = -1;
				}
				_delay_ms(150);
				last_error = 0.0f;
				missing_count = 0;

				is_cross_pass = 1;
			
				continue;
			}
			if (is_center && detect_count <= 2) {cross_state = 0;}
		} 
		
		
		if(is_cross_pass == 1) is_8_flag = 1;
		// 차선 코스
		if(is_8_flag && is_full)
		{
			is_line_course = 1;
			motor1Forward(); motor2Forward();
			set_speed(160, 80);
			_delay_ms(600);
		}
		if(is_line_course && !is_line_flag)
		{
			basic_mode = 0;
			
			if(bin[0] && (is_line_course_end < 3))
			{
				motor1Stop(); motor2Stop();
				set_speed(0, 0);
				_delay_ms(50);
				motor1Backward(); motor2Backward();
				set_speed(100, 100);
				_delay_ms(100);
				motor1Stop(); motor2Backward();
				set_speed(0, 100);
				_delay_ms(400);
			}
			else if(bin[5]) {
				is_line_course_end ++;

				if(is_line_course_end == 3)
				{
					motor1Stop(); motor2Stop();
					set_speed(0, 0);
					_delay_ms(50);
					motor1Backward(); motor2Backward();
					set_speed(100, 100);
					_delay_ms(250);
					motor1Backward(); motor2Backward();
					set_speed(150, 0);
					_delay_ms(400);
				}
				else
				{
					if(is_full && (is_line_course_end >= 3))
					{
						last_error = 0;
						last_turn_dir = 0;
						is_line_flag = 1;
						basic_mode = 1;
						is_stop_bar = 1;
						continue;
					}
					motor1Stop(); motor2Stop();
					set_speed(0, 0);
					_delay_ms(50);
					motor1Backward(); motor2Backward();
					set_speed(100, 100);
					_delay_ms(125);
					motor1Backward(); motor2Backward();
					set_speed(100, 0);
					_delay_ms(125);
				}
			}
			motor1Forward(); motor2Forward();
			set_speed(100, 100);
		}
		
		// 차단바 구간
		if(is_stop_bar && is_line_flag)
		{
			if(voltage[1] > 2.8)
			{
				while(voltage[1] > 1.7)
				{
					voltage[1] = ((float)adc_PSD_value[1] * 5) / 1023;
					motor1Forward(); motor2Forward();
					set_speed(90, 90);
				}
				
				motor1Stop(); motor2Stop();
				set_speed(0, 0);
				
				while(voltage[1] > 0.7)
				{
					voltage[1] = ((float)adc_PSD_value[1] * 5) / 1023;
				}
				_delay_ms(1500);
				is_stop_bar_flag = 1;
				is_parking = 1;
			}
		}
		
		/*
		if(is_parking && is_stop_bar_flag)
		{
			
		}
		*/
		
		// 기본
		if(basic_mode)
		{
			// 좌측 급커브
			if (is_left)
			{
				set_speed(0, 0);
				motor1Backward();
				motor2Forward();
				set_speed(200, 100);
				last_turn_dir = -1;
				_delay_ms(28);
			}
			// 우측 급커브
			else if (is_right)
			{
				set_speed(0, 0);
				motor1Forward();
				motor2Backward();
				set_speed(100, 200);
				last_turn_dir = 1;
				_delay_ms(28);
			}
			else if (detect_count == 0)
			{
				missing_count++;
					
				// 000000 -> 관성대로 회전
				if (missing_count > 30)
				{
					if (last_turn_dir == -1) { motor1Forward(); motor2Forward(); set_speed(0, 100); }
					else{ motor1Forward(); motor2Forward(); set_speed(100, 0); }
				}
			}
			else
			{
				missing_count = 0;
					
				float error = (-5.0f * normalization[0]) +
				(-2.5f * normalization[1]) +
				(-0.5f * normalization[2]) +
				( 0.5f * normalization[3]) +
				( 2.5f * normalization[4]) +
				( 5.0f * normalization[5]);

				if (error < -0.9f) last_turn_dir = -1;
				else if (error > 0.9f) last_turn_dir = 1;

				float control = (Kp * error) + (Kd * (error - last_error));
				last_error = error;

				int left_speed = base_speed + (int)control;
				int right_speed = base_speed - (int)control;

				// 속도 범위 제한
				if(left_speed > 0) { motor1Forward(); }
				else { motor1Backward(); }
				if(right_speed > 0) { motor2Forward(); }
				else { motor2Backward(); }
					
				if (left_speed > 220) left_speed = 220;
				if (left_speed < 0)
				{
					if(left_speed < -220){left_speed = 220;}
					else{left_speed = -left_speed;}
				}
				if (right_speed > 220) right_speed = 220;
				if (right_speed < 0)
				{
					if(right_speed < -220){right_speed = 220;}
					else{right_speed = -right_speed;}
				}
					
				set_speed((unsigned char)left_speed, (unsigned char)right_speed);
			}
		}
		
		
		

		if(print_flag) // LCD 출력
		{
			print_flag = 0;
			
			lcdClear();
			_delay_ms(3);
			
			
			lcdNumber(0,0,is_cross_pass);
			lcdNumber(0,5,full_count);
			lcdNumber(0,10,is_8);

			// lcdNumber(1,5,bin[5]);
			/*
			for (int i = 0; i < 6; i++)
			{
				if(i < 3)
				lcdFloat(0, i*5, normalization[i], 2);
				// lcdNumber(0, i*5, normalization[i]); // 0 5 10
				else
				lcdFloat(1, (i-3)*5, normalization[i], 2);
				// lcdNumber(1, (i-3)*5, normalization[i]); // 0 5 10
			}
			*/
		
		}
		_delay_ms(2);
	}
	return 0;
}

void set_speed(unsigned char speed_1, unsigned char speed_2)
{
	OCR1A = speed_1;
	OCR1B = speed_2;
	// 모터 속도 (0 ~ 255)
}


