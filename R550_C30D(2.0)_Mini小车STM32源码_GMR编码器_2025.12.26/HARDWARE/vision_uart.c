#include "system.h"
#include "vision_uart.h"

#if CHASSIS_STANDALONE
/* Four complete lines are retained. An overflowing line is discarded. */
#define VISION_DEPTH 4U
static volatile char lines[VISION_DEPTH][VISION_UART_LINE_MAX];
static volatile uint8_t lengths[VISION_DEPTH];
static volatile uint8_t head, tail, count, building, dropping;
static volatile uint8_t qr_ready;
static volatile uint32_t qr_event_count;
static volatile uint32_t ball_stop_event_count;
static volatile uint32_t balls_ready_count;
volatile uint32_t VisionUart_SearchTxCount,VisionUart_StoppedTxCount;
volatile uint32_t VisionUart_RxCount, VisionUart_ErrorCount, VisionUart_LastRx;
static volatile char partial[VISION_UART_LINE_MAX];

static volatile VisionBallSample ball_sample;
static volatile VisionTargetSample target_sample;
static volatile uint32_t target_stop_count;
static volatile VisionTargetSample rescue_sample;
static volatile uint32_t rescue_stop_count;
static uint8_t ball_packet[15],ball_bytes;
static TickType_t ball_byte_tick;

int VisionUart_GetBall(VisionBallSample *out)
{
    if(!out) return 0;
    taskENTER_CRITICAL();
    *out=ball_sample;
    taskEXIT_CRITICAL();
    return out->generation!=0U;
}

/* Fixed length frame; payload bytes never become legacy single-byte events. */
static int BallPacketByte(uint8_t c)
{
    TickType_t now=xTaskGetTickCountFromISR();
    uint8_t i,j,crc=0;
    uint16_t x,y,w,h;
    if(ball_bytes && (TickType_t)(now-ball_byte_tick)>pdMS_TO_TICKS(50U)) ball_bytes=0;
    ball_byte_tick=now;
    if(!ball_bytes) {
        if(c!=0xAAU) return 0;
        ball_packet[ball_bytes++]=c; return 1;
    }
    if(ball_bytes==1 && c!=0x55U && c!=0x56U && c!=0x57U && c!=0x58U) {
        ball_bytes=(c==0xAAU)?1U:0U;
        return 1;
    }
    ball_packet[ball_bytes++]=c;
    /* 新打靶/救援帧：AA 57/58 位次 X低 X高 宽低 宽高 CRC8。 */
    if(ball_packet[1]==0x57U || ball_packet[1]==0x58U) {
        volatile VisionTargetSample *sample;
        if(ball_bytes<8U) return 1;
        ball_bytes=0U;
        for(i=2;i<7;i++) {
            crc^=ball_packet[i];
            for(j=0;j<8;j++) crc=(uint8_t)((crc&0x80U)?(crc<<1)^0x07U:crc<<1);
        }
        if(crc!=ball_packet[7]) return 1;
        x=ball_packet[3]|((uint16_t)ball_packet[4]<<8);
        w=ball_packet[5]|((uint16_t)ball_packet[6]<<8);
        if(ball_packet[2]<1U || ball_packet[2]>3U || w<2U || w>4096U ||
           (x!=65535U && x>=w)) return 1;
        sample=(ball_packet[1]==0x57U)?&target_sample:&rescue_sample;
        sample->x=x;sample->width=w;sample->slot=ball_packet[2];
        sample->valid=(x!=65535U);sample->received_tick=now;++sample->generation;
        return 1;
    }
    if(ball_packet[1]==0x56U) {
        if(ball_bytes<7U) return 1;
        ball_bytes=0U;
        for(i=2;i<6;i++) {
            crc^=ball_packet[i];
            for(j=0;j<8;j++) crc=(uint8_t)((crc&0x80U)?(crc<<1)^0x07U:crc<<1);
        }
        if(crc!=ball_packet[6]) return 1;
        x=ball_packet[2]|((uint16_t)ball_packet[3]<<8);
        w=ball_packet[4]|((uint16_t)ball_packet[5]<<8);
        if(w<2U || w>4096U || (x!=65535U && x>=w)) return 1;
        target_sample.slot=0U;target_sample.x=x;target_sample.width=w;target_sample.valid=(x!=65535U);
        target_sample.received_tick=now;++target_sample.generation;
        return 1;
    }
    if(ball_bytes<15U) return 1;
    ball_bytes=0;
    for(i=2;i<14;i++) {
        crc^=ball_packet[i];
        for(j=0;j<8;j++) crc=(uint8_t)((crc&0x80U)?(crc<<1)^0x07U:crc<<1);
    }
    if(crc!=ball_packet[14]) return 1;
    x=ball_packet[4]|((uint16_t)ball_packet[5]<<8);
    y=ball_packet[6]|((uint16_t)ball_packet[7]<<8);
    w=ball_packet[8]|((uint16_t)ball_packet[9]<<8);
    h=ball_packet[10]|((uint16_t)ball_packet[11]<<8);
    if(ball_packet[2]>1 || (ball_packet[3]&15U)<1 || (ball_packet[3]&15U)>3 || (ball_packet[3]>>4)>3 ||
       w<2 || h<2 || w>4096 || h>4096 || x>=w || y>=h) return 1;
    ball_sample.x=x; ball_sample.y=y; ball_sample.width=w; ball_sample.height=h;
    ball_sample.valid=ball_packet[2]; ball_sample.color=ball_packet[3]&15U; ball_sample.slot=ball_packet[3]>>4;
    ball_sample.sequence=ball_packet[12]|((uint16_t)ball_packet[13]<<8);
    ball_sample.received_tick=now;
    ++ball_sample.generation;
    return 1;
}

void VisionUart_Init(uint32_t baud)
{
    GPIO_InitTypeDef gpio;
    USART_InitTypeDef serial;
    NVIC_InitTypeDef irq;
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC | RCC_AHB1Periph_GPIOD, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART5, ENABLE);
    NVIC_DisableIRQ(UART5_IRQn);
    USART_Cmd(UART5, DISABLE);
    USART_ITConfig(UART5, USART_IT_RXNE, DISABLE);
    ball_bytes=0U; ball_sample.generation=0U;
    target_stop_count=0U;target_sample.generation=0U;target_sample.slot=0U;
    rescue_stop_count=0U;rescue_sample.generation=0U;rescue_sample.slot=0U;
    qr_ready=0U;
    qr_event_count=0U;
    ball_stop_event_count=0U;
    balls_ready_count=0U;
    head=tail=count=building=dropping=0U;
    VisionUart_RxCount=0U; VisionUart_ErrorCount=0U;
    GPIO_PinAFConfig(GPIOC, GPIO_PinSource12, GPIO_AF_UART5);
    GPIO_PinAFConfig(GPIOD, GPIO_PinSource2, GPIO_AF_UART5);
    gpio.GPIO_Pin = GPIO_Pin_12;
    gpio.GPIO_Mode = GPIO_Mode_AF;
    gpio.GPIO_OType = GPIO_OType_PP;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    gpio.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOC, &gpio);
    gpio.GPIO_Pin = GPIO_Pin_2;
    GPIO_Init(GPIOD, &gpio);
    USART_StructInit(&serial);
    serial.USART_BaudRate = baud;
    serial.USART_WordLength = USART_WordLength_8b;
    serial.USART_StopBits = USART_StopBits_1;
    serial.USART_Parity = USART_Parity_No;
    serial.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    serial.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(UART5, &serial);
    irq.NVIC_IRQChannel = UART5_IRQn;
    irq.NVIC_IRQChannelPreemptionPriority = 6;
    irq.NVIC_IRQChannelSubPriority = 0;
    irq.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&irq);
    USART_ITConfig(UART5, USART_IT_RXNE, ENABLE);
    USART_Cmd(UART5, ENABLE);
}

uint32_t VisionUart_QrEventCount(void)
{
    return qr_event_count;
}

uint32_t VisionUart_BallStopEventCount(void)
{
    return ball_stop_event_count;
}

uint32_t VisionUart_BallsReadyCount(void)
{
    return balls_ready_count;
}

int VisionUart_SendStandbyMode(void)
{
    if(USART_GetFlagStatus(UART5,USART_FLAG_TXE)==RESET) return 0;
    USART_SendData(UART5,0x30U); /* ASCII 0: preview only at balls resolution. */
    return 1;
}

int VisionUart_SendQrMode(void)
{
    if(USART_GetFlagStatus(UART5,USART_FLAG_TXE)==RESET) return 0;
    USART_SendData(UART5,0x31U); /* ASCII 1: start QR recognition. */
    return 1;
}

int VisionUart_SendBallsMode(void)
{
    /* Single-byte reply; only the route task owns UART5 TX. No busy wait. */
    if(USART_GetFlagStatus(UART5,USART_FLAG_TXE)==RESET) return 0;
    USART_SendData(UART5,0x32U);
    return 1;
}

int VisionUart_SendBallsReset(void)
{
    if(USART_GetFlagStatus(UART5,USART_FLAG_TXE)==RESET) return 0;
    USART_SendData(UART5,0x36U);
    ++VisionUart_SearchTxCount;
    return 1;
}

int VisionUart_SendChassisStopped(void)
{
    if(USART_GetFlagStatus(UART5,USART_FLAG_TXE)==RESET) return 0;
    USART_SendData(UART5,0x35U); /* ASCII 5: wheels stopped, start XY stream. */
    ++VisionUart_StoppedTxCount;
    return 1;
}

int VisionUart_QrReady(void)
{
    return qr_ready != 0U;
}

int VisionUart_ReadLine(char *out, uint8_t capacity)
{
    uint8_t i, n;
    if (out == 0 || capacity == 0U) return 0;
    taskENTER_CRITICAL();
    if (count == 0U) {
        taskEXIT_CRITICAL();
        return 0;
    }
    n = lengths[tail];
    if (n >= capacity) n = capacity - 1U;
    for (i = 0U; i < n; ++i) out[i] = lines[tail][i];
    out[n] = '\0';
    tail = (uint8_t)((tail + 1U) % VISION_DEPTH);
    --count;
    taskEXIT_CRITICAL();
    return 1;
}

int UART5_IRQHandler(void)
{
    uint32_t status = UART5->SR;
    if (status & (USART_SR_RXNE | USART_SR_ORE | USART_SR_FE | USART_SR_NE | USART_SR_PE)) {
        char c = (char)UART5->DR; /* Reading SR then DR clears RX/error flags. */
        VisionUart_LastRx=(uint8_t)c;
        if (status & (USART_SR_ORE | USART_SR_FE | USART_SR_NE | USART_SR_PE)) {
            ++VisionUart_ErrorCount;
            ball_bytes=0U;
            building = 0U;
            dropping = 1U;
            return 0;
        }
        ++VisionUart_RxCount;
        if(BallPacketByte((uint8_t)c)) return 0;
        /* Binary event 0x30; no newline and no ISR motor operations. */
        if ((uint8_t)c == 0x54U) ++target_stop_count;
        if ((uint8_t)c == 0x64U) ++rescue_stop_count; /* 救援目标已进入视野。 */
        if ((uint8_t)c == 0x30U) { qr_ready=1U; ++qr_event_count; }
        /* Camera STOP event, target point 1..3; not the 0x11..0x13 discovery event. */
        if ((uint8_t)c==0x42U || (uint8_t)c==0x46U) ++balls_ready_count; /* Fresh-session ACK only. */
        if ((uint8_t)c>=0x51U && (uint8_t)c<=0x53U) ++ball_stop_event_count; /* Camera sends STOP only for saved QR target. */
        if (c == '\r') return 0;
        if (c == '\n') {
            uint8_t i;
            if (!dropping && building != 0U && count < VISION_DEPTH) {
                for (i = 0U; i < building; ++i) lines[head][i] = partial[i];
                lengths[head] = building;
                head = (uint8_t)((head + 1U) % VISION_DEPTH);
                ++count;
            }
            building = 0U;
            dropping = 0U;
        } else if (!dropping) {
            if (building < VISION_UART_LINE_MAX - 1U) partial[building++] = c;
            else dropping = 1U;
        }
    }
    return 0;
}
int VisionUart_GetTarget(VisionTargetSample *out)
{
    if(!out) return 0;
    taskENTER_CRITICAL();*out=target_sample;taskEXIT_CRITICAL();
    return out->generation!=0U;
}
uint32_t VisionUart_TargetStopCount(void) {return target_stop_count;}
int VisionUart_SendTargetMode(void)
{
    if(USART_GetFlagStatus(UART5,USART_FLAG_TXE)==RESET) return 0;
    USART_SendData(UART5,0x33U);return 1;
}
int VisionUart_GetRescue(VisionTargetSample *out)
{
    if(!out) return 0;
    taskENTER_CRITICAL();*out=rescue_sample;taskEXIT_CRITICAL();
    return out->generation!=0U;
}
uint32_t VisionUart_RescueStopCount(void) {return rescue_stop_count;}
int VisionUart_SendRescueMode(void)
{
    if(USART_GetFlagStatus(UART5,USART_FLAG_TXE)==RESET) return 0;
    USART_SendData(UART5,0x34U);return 1;
}
#endif
