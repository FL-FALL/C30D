#include "arm_servo.h"
#include "math.h"
#include "FreeRTOS.h"
#include "system.h"
/* TIM8 clock is 168 MHz with the current 168 MHz system clock.
 * Prescaler 167 gives 1 us/tick; ARR 19999 gives a 20 ms period (50 Hz).
 * Servo supply must be separate and share GND with the C30D board.
 */
#include "arm_position_config.h"
static const uint16_t zero_us[4]={ARM_PC6_ZERO_US,ARM_PC7_ZERO_US,ARM_PC8_ZERO_US,ARM_PC9_ZERO_US};
static const int8_t direction[4]={ARM_PC6_DIRECTION,ARM_PC7_DIRECTION,ARM_PC8_DIRECTION,ARM_PC9_DIRECTION};

void ArmServo_Init(void)
{
    GPIO_InitTypeDef gpio;
    TIM_TimeBaseInitTypeDef timer;
    TIM_OCInitTypeDef oc;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM8, ENABLE);
    /* Release former UART4 TX PC10; all servo signals are PWM again. */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART4, ENABLE);
    USART_Cmd(UART4, DISABLE);
    GPIO_StructInit(&gpio);
    gpio.GPIO_Pin=GPIO_Pin_10;gpio.GPIO_Mode=GPIO_Mode_IN;gpio.GPIO_PuPd=GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOC,&gpio);

    gpio.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9;
    gpio.GPIO_Mode = GPIO_Mode_AF;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    gpio.GPIO_OType = GPIO_OType_PP;
    gpio.GPIO_PuPd = GPIO_PuPd_DOWN;
    GPIO_Init(GPIOC, &gpio);
    GPIO_PinAFConfig(GPIOC, GPIO_PinSource6, GPIO_AF_TIM8);
    GPIO_PinAFConfig(GPIOC, GPIO_PinSource7, GPIO_AF_TIM8);
    GPIO_PinAFConfig(GPIOC, GPIO_PinSource8, GPIO_AF_TIM8);
    GPIO_PinAFConfig(GPIOC, GPIO_PinSource9, GPIO_AF_TIM8);

    TIM_TimeBaseStructInit(&timer);
    timer.TIM_Prescaler = 167U;
    timer.TIM_Period = 19999U;
    timer.TIM_ClockDivision = TIM_CKD_DIV1;
    timer.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM8, &timer);

    TIM_OCStructInit(&oc);
    oc.TIM_OCMode = TIM_OCMode_PWM1;
    oc.TIM_OutputState = TIM_OutputState_Enable;
    oc.TIM_OCPolarity = TIM_OCPolarity_High;
    oc.TIM_OCIdleState = TIM_OCIdleState_Reset;
    oc.TIM_Pulse = 0U; /* No motion command on power-up. */
    TIM_OC1Init(TIM8, &oc);
    TIM_OC2Init(TIM8, &oc);
    TIM_OC3Init(TIM8, &oc);
    TIM_OC4Init(TIM8, &oc);
    TIM_OC1PreloadConfig(TIM8, TIM_OCPreload_Enable);
    TIM_OC2PreloadConfig(TIM8, TIM_OCPreload_Enable);
    TIM_OC3PreloadConfig(TIM8, TIM_OCPreload_Enable);
    TIM_OC4PreloadConfig(TIM8, TIM_OCPreload_Enable);
    TIM_ARRPreloadConfig(TIM8, ENABLE);
    TIM_CtrlPWMOutputs(TIM8, ENABLE);
    TIM_Cmd(TIM8, ENABLE);
}

void ArmServo_SetPulseUs(uint8_t index, uint16_t pulse_us)
{
    if (index > 3U) return;
    if(index==3U && (pulse_us<500U || pulse_us>2500U)) return;
    /* Reject invalid PWM widths; do not clamp to a servo travel range. */
    if(pulse_us==0U || pulse_us>=(TIM8->ARR+1U)) return;
    switch (index) {
    case 0U: TIM8->CCR1 = pulse_us; break;
    case 1U: TIM8->CCR2 = pulse_us; break;
    case 2U: TIM8->CCR3 = pulse_us; break;
    default: TIM8->CCR4 = pulse_us; break;
    }
}

/* Absolute offset from configured zero, not a relative increment or speed.
 * Return 0 for invalid input; keep the previous command in that case.
 */
int Arm_SetAngle(uint8_t index, int16_t angle_deg)
{
    int32_t offset,pulse;
    if(index>3U) return 0;
    if(direction[index]!=1 && direction[index]!=-1) return 0;
    offset=(int32_t)angle_deg*ARM_POSITION_SPAN_US/ARM_POSITION_SPAN_DEG;
    pulse=(int32_t)zero_us[index]+direction[index]*offset;
    /* Only numeric/PWM-period validity remains; no mechanical angle limit. */
    if(pulse<=0 || pulse>=20000L) return 0;
    if(index==3U && (pulse<500L || pulse>2500L)) return 0;
    ArmServo_SetPulseUs(index,(uint16_t)pulse);
    return 1;
}

void ArmServo_Disable(uint8_t index)
{
    switch (index) {
    case 0U: TIM8->CCR1 = 0U; break;
    case 1U: TIM8->CCR2 = 0U; break;
    case 2U: TIM8->CCR3 = 0U; break;
    case 3U: TIM8->CCR4 = 0U; break;
    default: break;
    }
}

void ArmServo_DisableAll(void)
{
    TIM8->CCR1 = 0U;
    TIM8->CCR2 = 0U;
    TIM8->CCR3 = 0U;
    TIM8->CCR4 = 0U;
}

/* TD-8115MG label: 270 degrees, PWM 500..2500 us.
 * Absolute nominal position, not a relative turn. 135 = midpoint.
 * PWM sends a target continuously; motion duration is not programmable here.
 */
int Arm3_SetPwmPositionDeg(uint16_t degrees)
{
    uint32_t pulse;
    if(degrees>270U) return 0;
    pulse=500U+((uint32_t)degrees*2000U+135U)/270U;
    ArmServo_SetPulseUs(3U,(uint16_t)pulse);
    return 1;
}
