#include "system.h"
#include "vofa_uart.h"
#include "vision_uart.h"
#include "chassis_auto_route.h"
#include "heading_control.h"
#include "chassis_odom.h"
#include <stdio.h>

static int vofa_ready;
volatile unsigned int VOFA_BytesSent;
/* Same whole-frame queue approach as the gimbal, adapted to STM32F4 SPL. */
#define VOFA_TX_SIZE 1024U
static unsigned char tx_buffer[VOFA_TX_SIZE];
static volatile unsigned int tx_head,tx_tail;
volatile unsigned int VOFA_TxQueueFull;
void VOFA_Init(void)
{
    /* Existing Bluetooth socket: PD5 TX, PD6 RX. Standalone is transmit-only. */
    GPIO_InitTypeDef gpio;
    USART_InitTypeDef uart;
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD,ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2,ENABLE);
    GPIO_PinAFConfig(GPIOD,GPIO_PinSource5,GPIO_AF_USART2);
    GPIO_StructInit(&gpio);
    gpio.GPIO_Pin=GPIO_Pin_5;
    gpio.GPIO_Mode=GPIO_Mode_AF;
    gpio.GPIO_OType=GPIO_OType_PP;
    gpio.GPIO_Speed=GPIO_Speed_50MHz;
    gpio.GPIO_PuPd=GPIO_PuPd_UP;
    GPIO_Init(GPIOD,&gpio);
    USART_StructInit(&uart);
    uart.USART_BaudRate=VOFA_UART_BAUD;
    uart.USART_Mode=USART_Mode_Tx;
    USART_Init(USART2,&uart);
    USART_ITConfig(USART2,USART_IT_RXNE,DISABLE);
    NVIC_DisableIRQ(USART2_IRQn);
    USART_Cmd(USART2,ENABLE);
    tx_head=tx_tail=0U;
    USART_ITConfig(USART2,USART_IT_TXE,DISABLE);
    NVIC_ClearPendingIRQ(USART2_IRQn);
    {
        NVIC_InitTypeDef irq;
        irq.NVIC_IRQChannel=USART2_IRQn;
        irq.NVIC_IRQChannelPreemptionPriority=6;
        irq.NVIC_IRQChannelSubPriority=0;
        irq.NVIC_IRQChannelCmd=ENABLE;
        NVIC_Init(&irq);
    }
    vofa_ready=1;
}

/* Task-context API: copy a complete frame or reject it without a partial send. */
int VOFA_Write(const char *data,unsigned int len)
{
    unsigned int used,i;
    if(!vofa_ready || !data || !len || len>=VOFA_TX_SIZE) return 0;
    taskENTER_CRITICAL();
    used=(tx_head+VOFA_TX_SIZE-tx_tail)%VOFA_TX_SIZE;
    if(len>VOFA_TX_SIZE-1U-used) {
        ++VOFA_TxQueueFull;
        taskEXIT_CRITICAL();
        return 0;
    }
    for(i=0;i<len;i++) {
        tx_buffer[tx_head]=(unsigned char)data[i];
        tx_head=(tx_head+1U)%VOFA_TX_SIZE;
    }
    USART_ITConfig(USART2,USART_IT_TXE,ENABLE);
    taskEXIT_CRITICAL();
    return 1;
}

#if CHASSIS_STANDALONE
/* Match the existing project's interrupt declaration in usartx.h. */
int USART2_IRQHandler(void)
{
    if(USART_GetITStatus(USART2,USART_IT_TXE)!=RESET) {
        if(tx_tail!=tx_head) {
            USART_SendData(USART2,tx_buffer[tx_tail]);
            tx_tail=(tx_tail+1U)%VOFA_TX_SIZE;
            ++VOFA_BytesSent;
        }
        if(tx_tail==tx_head) USART_ITConfig(USART2,USART_IT_TXE,DISABLE);
    }
    return 0;
}
#endif

/* Existing fputc in SYSTEM/usart/usart.c redirects printf here. */
int VOFA_PutChar(int ch)
{
    char byte=(char)ch;
    return VOFA_Write(&byte,1U) ? (unsigned char)ch : EOF;
}

/* FireWater channels: X metres, Y metres, yaw degrees. */
void VOFA_SendYawOffset(void)
{
    ChassisPose pose=Chassis_OdomGetPose();
    float values[3];
    char line[96];
    unsigned int i,used=5U,value;
    int n;
    values[0]=pose.x_m; values[1]=pose.y_m;
    values[2]=HeadingControl_GetYawOffsetDeg();
    line[0]='p';line[1]='o';line[2]='s';line[3]='e';line[4]=':';
    for(i=0;i<3U;i++) {
        float v=values[i];
        const char *sign=v<0.0f ? "-" : "";
        if(!(v>=-1000.0f && v<=1000.0f)) return;
        if(v<0.0f) v=-v;
        value=(unsigned int)(v*1000.0f+0.5f);
        if(value==0U) sign="";
        n=snprintf(line+used,sizeof(line)-used,"%s%s%u.%03u",i?",":"",sign,value/1000U,value%1000U);
        if(n<0 || (unsigned int)n>=sizeof(line)-used) return;
        used+=(unsigned int)n;
    }
    if(used+2U>sizeof(line)) return;
    line[used++]='\r';line[used++]='\n';
    (void)VOFA_Write(line,used);
}

/* Route API retained; event text is disabled for yaw-only telemetry.
 * The independent UART5 departure reply to MaixCAM remains active. */
void VOFA_NotifyRouteStarted(void) { }
void VOFA_NotifyQrRecognized(void) { }

void VOFA_Task(void *argument)
{
    TickType_t last=xTaskGetTickCount();
    TickType_t stats_tick=last;
    TickType_t xy_tick=last;
    (void)argument;
    for(;;) {
#if !CHASSIS_ARM_ONLY_TEST
        VOFA_SendYawOffset();
        if((TickType_t)(xTaskGetTickCount()-xy_tick)>=pdMS_TO_TICKS(100U)) {
            VisionBallSample sample;
            char line[144];
            int n;
            xy_tick=xTaskGetTickCount();
            if(VisionUart_GetBall(&sample)) {
                unsigned long age=(unsigned long)((TickType_t)(xTaskGetTickCount()-sample.received_tick))*portTICK_PERIOD_MS;
                n=snprintf(line,sizeof(line),"[XY] x=%u y=%u w=%u h=%u slot=%u valid=%u age=%lums packets=%lu\r\n",
                    (unsigned int)sample.x,(unsigned int)sample.y,(unsigned int)sample.width,(unsigned int)sample.height,
                    (unsigned int)sample.slot,(unsigned int)sample.valid,age,(unsigned long)sample.generation);
            } else {
                n=snprintf(line,sizeof(line),"[XY] no decoded packet\r\n");
            }
            if(n>0 && n<(int)sizeof(line)) (void)VOFA_Write(line,(unsigned int)n);
        }
        if((TickType_t)(xTaskGetTickCount()-stats_tick)>=pdMS_TO_TICKS(1000U)) {
            char line[112];
            unsigned long rx,err,byte,qr;
            int n;
            stats_tick=xTaskGetTickCount();
            taskENTER_CRITICAL();
            rx=VisionUart_RxCount; err=VisionUart_ErrorCount;
            byte=VisionUart_LastRx; qr=VisionUart_QrEventCount();
            taskEXIT_CRITICAL();
            n=snprintf(line,sizeof(line),"[VISION_RX_V1] rx=%lu err=%lu last=%02lX qr=%lu\r\n",rx,err,byte,qr);
            if(n>0 && n<(int)sizeof(line)) (void)VOFA_Write(line,(unsigned int)n);
            n=snprintf(line,sizeof(line),"[C5] phase=%u fail=%u tx6=%lu tx5=%lu ack=%lu armed=%d imu=%d\r\n",
                Chassis_RoutePhase(),Route_FailureReason,(unsigned long)VisionUart_SearchTxCount,
                (unsigned long)VisionUart_StoppedTxCount,(unsigned long)VisionUart_BallsReadyCount(),Chassis_IsArmed(),HeadingControl_Ready());
            if(n>0 && n<(int)sizeof(line)) (void)VOFA_Write(line,(unsigned int)n);
        }
#endif
        vTaskDelayUntil(&last,pdMS_TO_TICKS(VOFA_PERIOD_MS));
    }
}
