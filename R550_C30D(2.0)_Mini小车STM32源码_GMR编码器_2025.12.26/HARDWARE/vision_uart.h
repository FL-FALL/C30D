#ifndef VISION_UART_H
#define VISION_UART_H
#include "stm32f4xx.h"

/* UART5: PC12 TX, PD2 RX; 115200 baud, 8N1, 3.3 V TTL. */
#define VISION_UART_LINE_MAX 64U
typedef struct {
    uint16_t x,y,width,height,sequence;
    uint8_t valid,color,slot; /* Wire color: 1 red, 2 blue, 3 green. */
    uint32_t received_tick,generation;
} VisionBallSample;
/* Atomic latest sample, pixel origin top-left; generation 0 means no packet. */
int VisionUart_GetBall(VisionBallSample *out);
extern volatile uint32_t VisionUart_RxCount, VisionUart_ErrorCount, VisionUart_LastRx;
void VisionUart_Init(uint32_t baud);
/* Task context, nonblocking: ASCII 2 enters balls mode; repeat to reset the tracker without reopening the same-size camera. */
int VisionUart_SendQrMode(void);
int VisionUart_SendStandbyMode(void);
int VisionUart_SendBallsMode(void);
int VisionUart_SendBallsReset(void); /* ASCII 6, new corner-5 red-ball session. */
int VisionUart_SendChassisStopped(void);
uint32_t VisionUart_BallsReadyCount(void);
/* Latched QR_OK (0x30), cleared only by initialization/reset. */
int VisionUart_QrReady(void);
/* Monotonic event counter: require a fresh QR event at the scan stop. */
uint32_t VisionUart_QrEventCount(void);
/* V10: 0x61..0x63 RED STOP carries physical slot; XY frame is AA55 X16 Y16 CRC8. */
uint32_t VisionUart_BallStopEventCount(void);
/* Returns 1 for a complete LF-terminated line, otherwise 0. Task context only. */
int VisionUart_ReadLine(char *out, uint8_t capacity);

extern volatile uint32_t VisionUart_SearchTxCount,VisionUart_StoppedTxCount;
typedef struct {
    uint16_t x,width;
    uint8_t valid,slot; /* slot为经过顺序1、2、3，不是颜色或形状码。 */
    uint32_t received_tick,generation;
} VisionTargetSample;
int VisionUart_GetTarget(VisionTargetSample *out);
uint32_t VisionUart_TargetStopCount(void);
int VisionUart_SendTargetMode(void);
int VisionUart_GetRescue(VisionTargetSample *out);
uint32_t VisionUart_RescueStopCount(void);
int VisionUart_SendRescueMode(void);
#endif
