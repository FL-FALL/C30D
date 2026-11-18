#ifndef VOFA_UART_H
#define VOFA_UART_H
#define VOFA_UART_BAUD 115200U
#define VOFA_PERIOD_MS 20U
void VOFA_Init(void);
int VOFA_PutChar(int ch);
/* Task context; returns 1 if the complete frame was copied into the TX queue. */
int VOFA_Write(const char *data,unsigned int len);
void VOFA_SendYawOffset(void);
/* Task-context event; actual UART write is serialized by VOFA_Task. */
void VOFA_NotifyRouteStarted(void);
void VOFA_NotifyQrRecognized(void);
void VOFA_Task(void *argument);
#endif
