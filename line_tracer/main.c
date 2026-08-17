/*
 * line_tracer.c
 *
 * Created: 2026-08-10 오후 4:33:32
 * Author : hojin
 */

#define F_CPU 16000000UL

#define arrSize 2
#define indexIR 6

#include "include/LCD_Text.h"
#include <avr/interrupt.h>
#include <avr/io.h>
#include <util/delay.h>

// PB0, 1 모터1 방향 제어
// PB2, 3 모터2 방향 제어
// PB5, 6 모터1 속도, 모터2 속도
#define motor1Forward()                                                                                                \
    {                                                                                                                  \
        PORTB &= ~(1 << PB0);                                                                                          \
        PORTB |= (1 << PB1);                                                                                           \
    }
#define motor1Backward()                                                                                               \
    {                                                                                                                  \
        PORTB &= ~(1 << PB1);                                                                                          \
        PORTB |= (1 << PB0);                                                                                           \
    }
#define motor1Stop()                                                                                                   \
    {                                                                                                                  \
        PORTB &= ~((1 << PB0) | (1 << PB1));                                                                           \
    }
#define motor2Forward()                                                                                                \
    {                                                                                                                  \
        PORTB |= (1 << PB2);                                                                                           \
        PORTB &= ~(1 << PB3);                                                                                          \
    }
#define motor2Backward()                                                                                               \
    {                                                                                                                  \
        PORTB |= (1 << PB3);                                                                                           \
        PORTB &= ~(1 << PB2);                                                                                          \
    }
#define motor2Stop()                                                                                                   \
    {                                                                                                                  \
        PORTB &= ~((1 << PB2) | (1 << PB3));                                                                           \
    }

void set_speed(unsigned char speed_1, unsigned char speed_2);

volatile unsigned int adc_PSD_value[2]; // [1] 사용, [0] 연결 안함
volatile unsigned int adc_value[6];     // IR
volatile int current_idx = 0;           // IR for문용

int full_count = 0;   // IR 전부 인식 시 count 111111
int prev_is_Full = 0; // full count 중복 카운트 방지

volatile int minMax[indexIR][2]; // 정규화에 필요한 포트별 min, max 값 저장

volatile float normalization[indexIR] = {0}; // 정규화 값
volatile float voltage[2] = {0};             // PSD V 변환
volatile float prev_voltage[2] = {0};
volatile int bin[6] = {0};           // ADC 정규화 값을 특정값과 비교하여 1 또는 0 (바이너리)
volatile int detect_count = 0;       // 감지된 값 개수(0~6)
volatile int detect_count_force = 0; // 감지된 값 개수(0~6) 기준 더 강화
volatile int is_dark = 0;            // 흑색 맵 들어갔는지

void _delay_ms_minMax_update(int ms); // 딜레이 동안 min max 업데이트(흑색 진입시 사용)
void is_bin();                        // bin값을 while문 내에서 또 호출할 때

volatile unsigned int ms_count = 0;
volatile unsigned char print_flag = 0; // timer flag(LCD 과연산 방지)

unsigned int ir_time[6] = {0, 0, 0, 0, 0, 0};
unsigned char ir_flag[6] = {0, 0, 0, 0, 0, 0};

volatile int is_turn = 0;
volatile int is_turn_end = 0;

volatile int is_overlap_end = 0;
volatile unsigned int ms_count_is_full = 0;
volatile unsigned int ms_count_is_turn = 0;

volatile int is_stop_bar_end = 0;
volatile int straight_true = 0;
volatile unsigned int ms_count_is_straight = 0;

ISR(TIMER0_OVF_vect) // timer0 interrupt
{
    // 클럭 / 분주비 = 250KHz (16MHz / 64)
    // 1주기 = 4us ( 1 / 250K )
    // 4us * 250 = 1ms
    TCNT0 = 256 - 250; // 250번 count ((256 - 250) ~ 256)
    ms_count++;
    if (is_overlap_end == 1)
    {
        ms_count_is_full++;
    }

    if (is_turn && !is_turn_end)
    {
        ms_count_is_turn++;
    }

    if (is_stop_bar_end && !straight_true)
    {
        ms_count_is_straight++;
    }

    if (ms_count >= 250 - 5) // 주기
    {
        ms_count = 0;
        print_flag = 1;
    }
}

ISR(ADC_vect)
{
    if (current_idx < 2)
    {
        adc_PSD_value[current_idx] = ADC;
    }
    else
    {
        adc_value[current_idx - 2] = ADC;
    }

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

    sei(); // 전역 인터럽트 활성화
    _delay_ms(10);

    // 첫 변환 시작
    ADCSRA |= (1 << ADSC);
    // 0110 1111 (PB6, 5, 3, 2, 1, 0)
    DDRB = 0x6F;
    DDRE = 0x02; // E0 입력, E1 출력
    // non-inverting mode A B, Fast PWM, 8-bit mode(WGM), 분주비 64(CS)
    TCCR1A = (1 << COM1A1) | (1 << COM1B1) | (1 << WGM10);
    TCCR1B = (1 << WGM12) | (1 << CS11) | (1 << CS10);

    int moveAvgArr[indexIR][arrSize]; // 이동 평균 필터에 사용할 값 저장
    int moveAvgFilterValue[indexIR];  // 필터링 값

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
            if (moveAvgArr[i][j] < minMax[i][0])
                minMax[i][0] = moveAvgArr[i][j];
            if (moveAvgArr[i][j] > minMax[i][1])
                minMax[i][1] = moveAvgArr[i][j];
        }
    }

    // PD 제어
    float Kp = 45.0f; // 비례
    float Kd = 35.0f; // 적분
    int base_speed = 90;
    float last_error = 0.0f; // PD control prev error
    int last_turn_dir = 0;   // -1: 좌회전, 1: 우회전 (마지막 회전 방향 기억)

    int missing_count = 0; // 000000일 때 count됨
    int basic_mode = 1;    // line tracing (basic)

    int is_8 = 0;      // 8 진입
    int is_8_flag = 0; // 8 코스 종료

    int is_line_course = 0;     // 차선 코스 시작
    int is_line_course_end = 0; // 차선 코스 끝 부분 도달
    int is_line_flag = 0;       // 차선 코스 종료

    int is_recognize = 0;     // psd 인지
    int is_stop_bar_flag = 0; // 차단바 구간 종료

    int is_parking_step = 0;
    int is_parking_flag = 0; // 주차 구간 종료

    int is_overlap_flag = 0; // 반전 종료

    int is_dark_flag = 0; // 숫자에 따라 동작(is dark되면 count)

    // int high_count = 0;
    int low_count = 0;
    // int decrease_count = 0;

    float max_voltage = 0;
    int has_peak = 0;

    // test(차단바)
    // is_8_flag = 1;
    // is_line_course = 1;
    // is_line_course_end = 1;
    // is_line_flag = 1;
    // is_stop_bar_flag = 1;
    // is_parking_flag = 1;

    // test (차선)
    // is_8 = 1;
    // is_8_flag = 1;
    // is_line_course = 1;

    _delay_ms(2000);

    lcdInit();
    lcdClear();
    _delay_ms(10);

    while (1)
    {
        for (int i = 0; i < indexIR; i++)
        {
            sum = 0;
            for (int j = arrSize - 1; j > 0; j--)
            {
                // 한 칸씩 밀기 a, b, c => a, a, b
                moveAvgArr[i][j] = moveAvgArr[i][j - 1];
            }
            // New_value, a, b
            moveAvgArr[i][0] = adc_value[i];

            // min, max 판별
            if (moveAvgArr[i][0] < minMax[i][0])
                minMax[i][0] = moveAvgArr[i][0];
            // else minMax[i][0]++;
            if (moveAvgArr[i][0] > minMax[i][1])
                minMax[i][1] = moveAvgArr[i][0];
            // else minMax[i][1]--;

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
            if (temp == 0) // 초기에 max - min이 0인 경우 0으로 나눌 수 없으므로 정규화 값 0으로 설정
            {
                normalization[i] = 0;
                continue;
            }
            if (is_dark)
                normalization[i] = (float)(moveAvgFilterValue[i] - minMax[i][0]) / temp;
            else
                normalization[i] = 1.0f - (float)(moveAvgFilterValue[i] - minMax[i][0]) / temp;
        }

        // PSD 볼트 변환
        for (int i = 0; i < 2; i++)
        {
            voltage[i] = ((float)adc_PSD_value[i] * 5) / 1023;
        }

        // 라인 트레이싱 알고리즘

        float detect_std = is_dark ? 0.55f : 0.3f;
        detect_count = 0;
        detect_count_force = 0;
        for (int i = 0; i < indexIR; i++)
        {
            if (normalization[i] >= detect_std)
                detect_count++;
        }

        for (int i = 0; i < indexIR; i++)
        {
            if (normalization[i] >= detect_std + 0.06f)
                detect_count_force++;
        }

        // 센서 이진화 (0.4 이상 1, 미만 0)
        float bin_std = 0.4f;
        for (int i = 0; i < 6; i++)
        {
            if (normalization[i] >= bin_std)
                bin[i] = 1;
            else
                bin[i] = 0;
        }

        // 변수
        float dir_std = 0.55f;

        int is_center = (bin[2] || bin[3]);                                     // 중앙 센서 반응
        int is_full = bin[0] && bin[1] && bin[2] && bin[3] && bin[4] && bin[5]; // 111111
        int is_left = (normalization[0] >= dir_std && normalization[1] >= dir_std) ||
                      (normalization[1] >= dir_std && normalization[2] >= dir_std); // 01 or 12
        int is_right = (normalization[4] >= dir_std && normalization[5] >= dir_std) ||
                       (normalization[3] >= dir_std && normalization[4] >= dir_std); // 34 or 45

        if (is_full && !prev_is_Full) // 라이징 엣지인 경우만 읽어오도록 함
            full_count++;
        prev_is_Full = is_full;

        // --------
        // LCD 출력
        if (print_flag)
        {
            print_flag = 0;

            lcdClear();
            _delay_ms(3);

            // if (!is_dark)
            //{
            lcdNumber(0, 0, OCR1A);
            lcdNumber(0, 5, OCR1B);
            lcdNumber(0, 10, full_count);

            lcdFloat(1, 0, voltage[1], 1);
            lcdNumber(1, 5, bin[2]);
            lcdNumber(1, 10, is_overlap_end);

            //}
            /*else
            {
                for (int i = 0; i < 6; i++)
                {
                    if (i < 3)
                        lcdFloat(0, i * 5, normalization[i], 2);
                    else
                        lcdFloat(1, (i - 3) * 5, normalization[i], 2);
                }
            }*/
        }
        // LCD 출력
        // --------

        //-----
        // 기본
        if (basic_mode)
        {

            // 좌측 급커브
            if (is_left)
            {
                set_speed(0, 0);
                motor1Backward();
                motor2Forward();
                set_speed(200, 100);
                last_turn_dir = -1;
                _delay_ms(20);
            }
            // 우측 급커브
            else if (is_right)
            {
                set_speed(0, 0);
                motor1Forward();
                motor2Backward();
                set_speed(100, 0);
                last_turn_dir = 1;
                _delay_ms(20);
            }
            else if (detect_count == 0)
            {
                missing_count++;

                // 000000 -> 관성대로 회전
                if (missing_count > 30)
                {
                    if (last_turn_dir == -1)
                    {

                        motor2Forward();
                        motor1Forward();
                        set_speed(0, 100);
                    }
                    else
                    {
                        motor1Forward();
                        motor2Forward();
                        set_speed(100, 0);
                    }
                }
            }
            else
            {
                missing_count = 0;

                float error = (-5.0f * normalization[0]) + (-2.5f * normalization[1]) + (-0.5f * normalization[2]) +
                              (0.5f * normalization[3]) + (2.5f * normalization[4]) + (5.0f * normalization[5]);

                if (error < -0.4f)
                    last_turn_dir = -1;
                else if (error > 0.4f)
                    last_turn_dir = 1;

                float control = (Kp * error) + (Kd * (error - last_error));
                last_error = error;

                int left_speed = base_speed + (int)control + 4;
                int right_speed = base_speed - (int)control;

                // 속도 범위 제한
                if (left_speed > 0)
                {
                    motor1Forward();
                }
                else
                {
                    motor1Backward();
                }
                if (right_speed > 0)
                {
                    motor2Forward();
                }
                else
                {
                    motor2Backward();
                }

                if (left_speed > 220)
                    left_speed = 220;
                if (left_speed < 0)
                {
                    if (left_speed < -220)
                    {
                        left_speed = 220;
                    }
                    else
                    {
                        left_speed = -left_speed;
                    }
                }
                if (right_speed > 220)
                    right_speed = 220;
                if (right_speed < 0)
                {
                    if (right_speed < -220)
                    {
                        right_speed = 220;
                    }
                    else
                    {
                        right_speed = -right_speed;
                    }
                }

                set_speed((unsigned char)left_speed, (unsigned char)right_speed);
            }
        }
        // 기본
        //-----

        // ---
        // 8자
        if (full_count >= 3 && !is_8)
        {
            is_8 = 1;
        }

        if (is_8 && !is_8_flag)
        {
            motor1Forward();
            motor2Forward();
            if (last_turn_dir == -1)
            {
                set_speed(40, 140);
            }
            else
            {
                set_speed(140, 40);
            }

            _delay_ms(700); // 안되면 해당 딜레이, 밑에 있는 딜레이 2개 조정해보기
            // 8자 다음원으로 넘어가기 위해 반대 방향으로 걸어줌?인데 같은 방향으로 돌아감
            if (last_turn_dir == -1)
            {
                motor1Forward();
                motor2Forward();
                set_speed(140, 40);
                last_turn_dir = 1;
            }
            else
            {
                motor1Forward();
                motor2Forward();
                set_speed(40, 140);
                last_turn_dir = -1;
            }
            _delay_ms(300);
            last_error = 0.0f;
            missing_count = 0;

            is_8_flag = 1;
        }
        //  8자
        //  ---

        // --------
        // 차선 코스
        if (is_8_flag && full_count >= 4 && !is_line_course)
        {
            is_line_course = 1;
            motor1Forward();
            motor2Forward();
            set_speed(150, 90);
            _delay_ms(750);
        }

        if (is_line_course && !is_line_flag)
        {
            basic_mode = 0;

            if (bin[0] && (is_line_course_end < 3))
            {
                motor1Stop();
                motor2Stop();
                set_speed(0, 0);
                _delay_ms(50);
                motor1Backward();
                motor2Backward();
                set_speed(100, 100);
                _delay_ms(125);
                motor1Stop();
                motor2Backward();
                set_speed(0, 100);
                _delay_ms(400);
            }
            else if (bin[5])
            {
                is_line_course_end++;

                if (is_line_course_end == 3)
                {
                    motor1Stop();
                    motor2Stop();
                    set_speed(0, 0);
                    _delay_ms(50);
                    motor1Backward();
                    motor2Backward();
                    set_speed(100, 100);
                    _delay_ms(350);
                    motor1Backward();
                    motor2Backward();
                    set_speed(150, 0);
                    _delay_ms(525);
                }
                else
                {
                    // int is_5 = bin[1] && bin[2] && bin[3] && bin[4] && bin[5];
                    if (is_line_course_end >= 3) // detect_count >= 5 && // is_full
                    {
                        is_line_flag = 1;
                        last_error = 0;
                        last_turn_dir = 0;
                        motor1Forward();
                        motor2Forward();
                        set_speed(40, 140);
                        _delay_ms(400);
                        basic_mode = 1;
                        continue;
                    }
                    motor1Stop();
                    motor2Stop();
                    set_speed(0, 0);
                    _delay_ms(50);
                    motor1Backward();
                    motor2Backward();
                    set_speed(100, 100);
                    _delay_ms(100);
                    motor1Backward();
                    motor2Backward();
                    set_speed(100, 0);
                    _delay_ms(100);
                }
            }
            motor1Forward();
            motor2Forward();
            set_speed(100, 100);
        }
        // 차선 코스
        // --------

        // ----------
        // 차단바 코스
        if (is_line_flag && !is_stop_bar_flag)
        {
            if (voltage[1] > 2.8) // test 필요
            {
                is_recognize = 1;
            }

            if (is_recognize == 1) // TODO test
            {
                if (voltage[1] < 1.6)
                {
                    low_count++;
                    _delay_ms(2);
                }
                else
                {
                    low_count = 0;
                    continue;
                }

                if (low_count >= 2)
                {
                    motor1Stop();
                    motor2Stop();
                    set_speed(0, 0);
                    low_count = 0;
                }
                else
                    continue;

                while (low_count < 5) // TODO 테스트 필요 // 2회 확인
                {
                    voltage[1] = ((float)adc_PSD_value[1] * 5) / 1023;
                    if (voltage[1] < 0.7)
                    {
                        low_count++;
                        _delay_ms(2);
                    }
                    else
                        low_count = 0;
                }
                _delay_ms(1500);
                is_stop_bar_flag = 1;
                is_stop_bar_end = 1;
                is_recognize = 0;
                last_error = 0;
                last_turn_dir = 0;
                full_count = 0;
                low_count = 0;
            }
        }
        // 차단바 코스
        // ----------

        // --------
        // 주차 코스
        if (is_stop_bar_flag && !is_parking_flag)
        {
            if (ms_count_is_straight >= 1500 && !straight_true) // 1.5초 뒤에 직진 모드
            {
                basic_mode = 0;
                straight_true = 1;
            }

            if (bin[0] && is_parking_step == 0)
            {
                motor1Forward();
                motor2Forward();
                set_speed(75, 75);
                _delay_ms(200);

                motor1Backward();
                motor2Forward();
                set_speed(75, 75);
                _delay_ms(650);
                last_turn_dir = -1;
                is_parking_step = 1;
            }

            if (detect_count_force >= 5 && is_parking_step == 1)
            {
                motor1Stop();
                motor2Stop();
                set_speed(0, 0);
                _delay_ms(2000);
                is_parking_step = 2;
            }

            if (is_parking_step == 2) // bin[0] &&
            {
                motor1Forward();
                motor2Forward();
                set_speed(80, 80);
                _delay_ms(350);

                motor1Backward();
                motor2Forward();
                set_speed(75, 75);
                _delay_ms(1500);
                last_turn_dir = -1;
                is_parking_step = 3;
            }

            if (!is_center)
            {
                if (last_turn_dir == -1)
                {
                    motor1Backward();
                    motor2Forward();
                    set_speed(75, 75);
                }
                else
                {
                    motor1Forward();
                    motor2Backward();
                    set_speed(75, 75);
                }
            }
            else if (bin[0] && detect_count >= 2) // T자 구간에서 좌회전 우선 지정 // && is_center
            {
                motor1Backward();
                motor2Forward();
                set_speed(75, 75);
            }
            else
            {
                float overlap_error = (-0.75f * normalization[1]) + (-0.75f * normalization[2]) +
                                      (0.75f * normalization[3]) + (0.75f * normalization[4]);

                float error = (-5.0f * normalization[0]) + (-2.5f * normalization[1]) + (-0.5f * normalization[2]) +
                              (0.5f * normalization[3]) + (2.0f * normalization[4]) +
                              (4.0f * normalization[5]); // 좌회전 가중치

                if (error < -0.25f)
                    last_turn_dir = -1;
                else if (error > 0.25f)
                    last_turn_dir = 1;

                float overlap_Kp = 15.0f;
                float overlap_Kd = 10.0f;

                float control = (overlap_Kp * overlap_error) + (overlap_Kd * (overlap_error - last_error));
                last_error = overlap_error;

                int left_speed = base_speed + (int)control;
                int right_speed = base_speed - (int)control;

                // 직진 속도 보장
                if (left_speed < 70)
                    left_speed = 70;
                if (right_speed < 70)
                    right_speed = 70;
                if (left_speed > 130)
                    left_speed = 130;
                if (right_speed > 130)
                    right_speed = 130;

                motor1Forward();
                motor2Forward();
                set_speed((unsigned char)left_speed, (unsigned char)right_speed);
            }

            if (full_count >= 1)
            {
                is_parking_flag = 1;
            }
        }
        // 주차 코스
        // --------

        // --------------------
        // 반전 구간 + 중첩 구간
        if (is_parking_flag && !is_overlap_flag)
        {
            basic_mode = 0;
            float error = 0;
            if (normalization[5] >= 0.75f) //
            {
                is_dark = 1;
                last_error = 0;
                if (is_dark_flag == 0)
                {
                    is_recognize = 0;
                    last_turn_dir = 1;
                    motor1Forward();
                    motor2Forward();
                    set_speed(90, 100);
                    _delay_ms(600);

                    motor1Stop();
                    motor2Stop();
                    set_speed(0, 0);
                    _delay_ms(500);

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
                            if (moveAvgArr[i][j] < minMax[i][0])
                                minMax[i][0] = moveAvgArr[i][j];
                            if (moveAvgArr[i][j] > minMax[i][1])
                                minMax[i][1] = moveAvgArr[i][j];
                        }
                    }
                    is_dark_flag++;
                    _delay_ms(10);
                }
            }
            else if (is_dark_flag == 1)
            {
                motor1Stop();
                motor2Stop();
                set_speed(0, 0);
                _delay_ms_minMax_update(100);

                motor1Forward();
                motor2Backward();
                set_speed(100, 100);
                _delay_ms_minMax_update(300);

                motor1Stop();
                motor2Stop();
                set_speed(0, 0);
                _delay_ms_minMax_update(100);

                motor1Backward();
                motor2Forward();
                set_speed(100, 100);
                _delay_ms_minMax_update(600);

                motor1Stop();
                motor2Stop();
                set_speed(0, 0);
                _delay_ms_minMax_update(100);

                motor1Forward();
                motor2Backward();
                set_speed(75, 75);
                _delay_ms_minMax_update(400); // TODO: test해보기(작동 여부)
                last_turn_dir = 1;

                is_dark_flag++;
                full_count = 0;
                _delay_ms(10);
                continue;
            }

            for (int i = 0; i < 2; i++)
            {
                voltage[i] = ((float)adc_PSD_value[i] * 5) / 1023;
            }

            // -----------
            // 벽 인식 구간
            if (voltage[1] > 2.3 && is_overlap_end == 0) // TODO test 필요 // 작동 2회 확인
            {
                is_overlap_end = 1;
                for (int i = 0; i < 6; i++)
                {
                    ir_time[i] = 0;
                    ir_flag[i] = 0;
                }
                ms_count_is_full = 0;
            }

            if (is_overlap_end == 1)
            {
                for (int i = 0; i < 6; i++)
                {
                    if (bin[i] && !ir_flag[i])
                    {
                        ir_flag[i] = 1;                // 중복 기록 방지 플래그 ON
                        ir_time[i] = ms_count_is_full; // 현재 측정된 시간(ms) 저장
                    }
                }

                int all_on = 1; // 0 1 2 3 4만 떠도 all_on으로 판정(가끔 5가 안 걸림)
                for (int i = 0; i < 5; i++)
                {
                    if (ir_flag[i] == 0)
                    {
                        all_on = 0;
                        break;
                    }
                }

                if (all_on)
                {
                    unsigned int min_time = ir_time[0];
                    unsigned int max_time = ir_time[0];

                    for (int i = 1; i < 6; i++)
                    {
                        if (ir_time[i] < min_time)
                            min_time = ir_time[i];
                        if (ir_time[i] > max_time)
                            max_time = ir_time[i];
                    }

                    unsigned int time_gap = max_time - min_time; // 첫 센서와 마지막 센서의 시간 간격

                    // 허용 오차 범위 내에 모두 들어왔다면 정지선 확정
                    if (time_gap <= 5000)
                    {
                        motor1Stop();
                        motor2Stop();
                        set_speed(0, 0);
                        _delay_ms(500);
                        while (1)
                        {
                            for (int i = 0; i < indexIR; i++)
                            {
                                sum = 0;
                                for (int j = arrSize - 1; j > 0; j--)
                                {
                                    // 한 칸씩 밀기 a, b, c => a, a, b
                                    moveAvgArr[i][j] = moveAvgArr[i][j - 1];
                                }
                                // New_value, a, b
                                moveAvgArr[i][0] = adc_value[i];

                                // min, max 판별
                                if (moveAvgArr[i][0] < minMax[i][0])
                                    minMax[i][0] = moveAvgArr[i][0];
                                if (moveAvgArr[i][0] > minMax[i][1])
                                    minMax[i][1] = moveAvgArr[i][0];

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
                                if (temp ==
                                    0) // 초기에 max - min이 0인 경우 0으로 나눌 수 없으므로 정규화 값 0으로 설정
                                {
                                    normalization[i] = 0;
                                    continue;
                                }
                                if (is_dark)
                                    normalization[i] = (float)(moveAvgFilterValue[i] - minMax[i][0]) / temp;
                                else
                                    normalization[i] = 1.0f - (float)(moveAvgFilterValue[i] - minMax[i][0]) / temp;
                            }

                            // PSD 볼트 변환
                            for (int i = 0; i < 2; i++)
                            {
                                voltage[i] = ((float)adc_PSD_value[i] * 5) / 1023;
                            }

                            // 라인 트레이싱 알고리즘

                            float detect_std = is_dark ? 0.55f : 0.3f;
                            detect_count = 0;
                            for (int i = 0; i < indexIR; i++)
                            {
                                if (normalization[i] >= detect_std)
                                    detect_count++;
                            }

                            // 센서 이진화 (0.4 이상 1, 미만 0)
                            float bin_std = 0.4f;
                            for (int i = 0; i < 6; i++)
                            {
                                if (normalization[i] >= bin_std)
                                    bin[i] = 1;
                                else
                                    bin[i] = 0;
                            }
                            if (!bin[0])
                            {
                                motor1Backward();
                                motor2Stop();
                                set_speed(75, 0);
                            }
                            else
                            {
                                is_overlap_end = 2;
                                break;
                            }
                        }
                        continue;
                    }
                }
            }

            if (is_overlap_end == 2)
            {
                motor1Forward();
                motor2Forward();
                set_speed(85, 77);

                for (int i = 0; i < 2; i++)
                {
                    voltage[i] = ((float)adc_PSD_value[i] * 5) / 1023;
                }

                if (voltage[1] > max_voltage)
                {
                    max_voltage = voltage[1];
                }

                if (max_voltage >= 2.4f)
                {
                    has_peak = 1;
                }

                if (voltage[1] < 1.6)
                {
                    low_count++;
                    _delay_ms(2);
                }
                else
                {
                    low_count = 0;
                    continue;
                }

                if (has_peak && low_count >= 2)
                {
                    motor1Stop();
                    motor2Stop();
                    set_speed(0, 0);
                    _delay_ms(250);
                    is_overlap_end = 3;
                }
                else
                {
                    continue;
                }
            }

            if (is_overlap_end == 3)
            {
                motor1Backward();
                motor2Forward();
                set_speed(85, 85);
                _delay_ms(900);

                motor1Stop();
                motor2Stop();
                set_speed(0, 0);
                _delay_ms(100);

                motor1Forward();
                motor2Forward();
                set_speed(85, 77);
                is_overlap_end = 4;
            }

            if (is_overlap_end == 4)
            {
                if (detect_count == 0)
                {
                    motor1Forward();
                    motor2Forward();
                    set_speed(85, 75);
                    continue;
                }
                else
                {
                    is_overlap_end = 5;
                    full_count = 0;
                }
            }

            if (is_overlap_end == 5 && full_count >= 2 && detect_count == 0) // TODO test 해보기, overlap end 6도 같이
            {
                motor1Forward();
                motor2Forward();
                set_speed(85, 77);
                if (detect_count == 0)
                    continue;
                else
                {
                    is_overlap_end = 6;
                }
            }

            if (is_overlap_end == 6 && is_full)
            {
                motor1Stop();
                motor2Stop();
                set_speed(0, 0);
                while (1)
                    ;
            }

            if (is_center)
            {

                if (is_dark_flag == 2)
                {
                    is_dark_flag++;
                }
                else if (is_dark_flag >= 3)
                {
                    motor1Forward();
                    motor2Forward();
                    if (last_turn_dir == -1)
                        set_speed(75, 78);
                    else
                        set_speed(82, 75);
                    _delay_ms(125);
                    motor1Stop();
                    motor2Stop();
                    is_bin();
                    if (!(bin[2] || bin[3]))
                    {
                        if (last_turn_dir == -1)
                        {
                            motor1Backward();
                            motor2Forward();
                            set_speed(100, 100);
                        }
                        else
                        {
                            motor1Forward();
                            motor2Backward();
                            set_speed(100, 100);
                        }
                        is_dark_flag++;
                    }
                    if (is_full)
                    {
                        motor1Forward();
                        motor2Forward();
                        set_speed(75, 75);
                    }
                }

                // 벽 인식 구간
                // -----------

                if (is_overlap_end == 5 && full_count >= 2 && detect_count >= 3) // 세갈래 정렬
                {
                    float diff_0_5 = normalization[5] - normalization[0];
                    float diff_1_4 =
                        normalization[4] - normalization[1]; // 왼쪽으로 치우쳐있으면 -, 오른쪽으로 치우쳐있으면 +

                    if (diff_0_5 > 0.1f || diff_1_4 > 0.1f)
                    {
                        motor1Backward();
                        motor2Stop();
                        set_speed(80, 0);
                        _delay_ms(20);
                        continue;
                    }
                    else if (diff_0_5 < -0.1f || diff_1_4 < -0.1f)
                    {
                        motor1Stop();
                        motor2Backward();
                        set_speed(0, 80);
                        _delay_ms(20);
                        continue;
                    }
                }
                else if (detect_count >= 3) // 직진 우선
                {
                    motor1Forward();
                    motor2Forward();
                    set_speed(81, 75);
                    error = (-5.0f * normalization[0]) + (-2.5f * normalization[1]) + (-0.5f * normalization[2]) +
                            (0.5f * normalization[3]) + (2.5f * normalization[4]) + (5.0f * normalization[5]);

                    if (error < -0.5f)
                        last_turn_dir = -1;
                    else if (error > 0.5f)
                        last_turn_dir = 1;

                    continue;
                }

                float overlap_error = (-0.75f * normalization[2]) + (0.75f * normalization[3]);

                error = (-5.0f * normalization[0]) + (-2.5f * normalization[1]) + (-0.5f * normalization[2]) +
                        (0.5f * normalization[3]) + (2.5f * normalization[4]) + (5.0f * normalization[5]);

                if (error < -0.25f)
                    last_turn_dir = -1;
                else if (error > 0.25f)
                    last_turn_dir = 1;

                float overlap_Kp = 15.0f;
                float overlap_Kd = 10.0f;

                float control = (overlap_Kp * overlap_error) + (overlap_Kd * (overlap_error - last_error));
                last_error = overlap_error;

                int left_speed = base_speed + (int)control + 10; // 바퀴 정렬 보정(왼쪽으로 살짝 휨)
                int right_speed = base_speed - (int)control;

                // 직진 속도 보장
                if (left_speed < 70)
                    left_speed = 70;
                if (right_speed < 70)
                    right_speed = 70;
                if (left_speed > 130)
                    left_speed = 130;
                if (right_speed > 130)
                    right_speed = 130;

                motor1Forward();
                motor2Forward();
                set_speed((unsigned char)left_speed, (unsigned char)right_speed);
            }
            else // center 00
            {
                error = (-5.0f * normalization[0]) + (-2.5f * normalization[1]) + (-0.5f * normalization[2]) +
                        (0.5f * normalization[3]) + (2.5f * normalization[4]) + (5.0f * normalization[5]);

                if (error < -0.25f)
                    last_turn_dir = -1;
                else if (error > 0.25f)
                    last_turn_dir = 1;

                if (last_turn_dir == -1)
                {
                    motor1Backward();
                    motor2Forward();
                    set_speed(75, 75);
                }
                else
                {
                    motor1Forward();
                    motor2Backward();
                    set_speed(75, 75);
                }
            }
        }
        // 반전 구간 + 중첩 구간
        // --------------------

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

void _delay_ms_minMax_update(int ms)
{
    for (int i = 0; i < ms; i++)
    {
        for (int j = 0; j < indexIR; j++)
        {
            if (adc_value[j] < minMax[j][0])
                minMax[j][0] = adc_value[j];
            if (adc_value[j] > minMax[j][1])
                minMax[j][1] = adc_value[j];
        }
        _delay_ms(1);
    }
}

void is_bin()
{
    // PSD
    for (int i = 0; i < 2; i++)
    {
        voltage[i] = ((float)adc_PSD_value[i] * 5) / 1023;
    }

    // 라인 트레이싱 알고리즘

    float detect_std = is_dark ? 0.55f : 0.3f;
    detect_count = 0;
    for (int i = 0; i < indexIR; i++)
    {
        if (normalization[i] >= detect_std)
            detect_count++;
    }

    // 센서 이진화 (0.4 이상 1, 미만 0)
    float bin_std = 0.4f;
    for (int i = 0; i < 6; i++)
    {
        if (normalization[i] >= bin_std)
            bin[i] = 1;
        else
            bin[i] = 0;
    }
}
