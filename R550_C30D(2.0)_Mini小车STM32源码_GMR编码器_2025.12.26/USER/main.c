/***********************************************
��˾����Ȥ�Ƽ�����ݸ�����޹�˾
Ʒ�ƣ�WHEELTEC
������wheeltec.net
�Ա����̣�shop114407458.taobao.com 
����ͨ: https://minibalance.aliexpress.com/store/4455017
�汾��V5.0
�޸�ʱ�䣺2022-05-05

Brand: WHEELTEC
Website: wheeltec.net
Taobao shop: shop114407458.taobao.com 
Aliexpress: https://minibalance.aliexpress.com/store/4455017
Version: V5.0
Update��2022-05-05

All rights reserved
***********************************************/
#include "system.h"
#include "arm_servo.h"
#include "arm_task.h"
#include "chassis_auto_route.h"
#include "vision_uart.h"
#include "vofa_uart.h"
//Task priority    //�������ȼ�
#define START_TASK_PRIO	1

//Task stack size //�����ջ���?
#define START_STK_SIZE 	256  

//Task handle     //������
TaskHandle_t StartTask_Handler;
volatile BaseType_t RouteTask_CreateResult;
volatile BaseType_t ArmOnlyTask_CreateResult;
volatile BaseType_t VofaTask_CreateResult;

//Task function   //������
void start_task(void *pvParameters);

//Main function
int main(void)
{
    GPIO_InitTypeDef pc11_gpio;
    systemInit();
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
    GPIO_ResetBits(GPIOC, GPIO_Pin_11); /* Default output LOW. */
    GPIO_StructInit(&pc11_gpio);
    pc11_gpio.GPIO_Pin = GPIO_Pin_11;
    pc11_gpio.GPIO_Mode = GPIO_Mode_OUT;
    pc11_gpio.GPIO_OType = GPIO_OType_PP;
    pc11_gpio.GPIO_PuPd = GPIO_PuPd_NOPULL;
    pc11_gpio.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_Init(GPIOC, &pc11_gpio);
    Chassis_LocalInit(); /* Keep chassis locked for this bench test. */
    ArmServo_Init();

    /* Optional power-up arm commands go here; corner-5 pose runs at corner 5. */
    VisionUart_Init(115200U);
    VOFA_Init();
    if(xTaskCreate(start_task, "start_task", START_STK_SIZE, NULL,
                   START_TASK_PRIO, &StartTask_Handler)!=pdPASS) for(;;) { }
    vTaskStartScheduler();
    for (;;) { }
}

//Start task task function //��ʼ����������
void start_task(void *pvParameters)
{
    taskENTER_CRITICAL(); //Enter the critical area //�����ٽ���
	
    //Create the task //��������
#if CHASSIS_STANDALONE && CHASSIS_ARM_ONLY_TEST
    xTaskCreate(Balance_task, "Balance_task", BALANCE_STK_SIZE, NULL, BALANCE_TASK_PRIO, NULL);
#else
	if(Check == 1)
		xTaskCreate(Check_task,  "Check_task",  CHECK_STK_SIZE,  NULL, CHECK_TASK_PRIO,  NULL);
	else if(Check == 0)
		xTaskCreate(Balance_task,  "Balance_task",  BALANCE_STK_SIZE,  NULL, BALANCE_TASK_PRIO,  NULL);	//Vehicle motion control task //С���˶���������
#endif
	if(SysVal.HardWare_Ver==V1_0) 	//IMU data read task //IMU���ݶ�ȡ����,���ݲ�ͬ��Ӳ���汾������ͬ������.
		xTaskCreate(MPU6050_task,  "IMU_task",  IMU_STK_SIZE,  NULL, IMU_TASK_PRIO,  NULL);
	else if( SysVal.HardWare_Ver==V1_1 )
		xTaskCreate(ICM20948_task,  "IMU_task",  IMU_STK_SIZE,  NULL, IMU_TASK_PRIO,  NULL);
	xTaskCreate(show_task,     "show_task",     SHOW_STK_SIZE,     NULL, SHOW_TASK_PRIO,     NULL); //The OLED display displays tasks //OLED��ʾ����ʾ����
	xTaskCreate(led_task,      "led_task",      LED_STK_SIZE,      NULL, LED_TASK_PRIO,      NULL);	//LED light flashing task //LED����˸����
#if CHASSIS_STANDALONE

#if CHASSIS_ARM_ONLY_TEST
    Chassis_Disarm();
    ArmOnlyTask_CreateResult=xTaskCreate(ArmMission_TestTask, "Arm_test", 256, NULL, 3, NULL);
#else
    if (Check == 0)
        RouteTask_CreateResult=xTaskCreate(Chassis_RouteTask, "Route_task", 384, NULL, 3, NULL);
    if(Check==0 && RouteTask_CreateResult!=pdPASS) Chassis_Disarm();
#endif
    VofaTask_CreateResult=xTaskCreate(VOFA_Task, "VOFA_task", 512, NULL, 2, NULL);
#endif
#if !CHASSIS_STANDALONE
	xTaskCreate(pstwo_task,    "PSTWO_task",    PS2_STK_SIZE,      NULL, PS2_TASK_PRIO,      NULL);	//Read the PS2 controller task //��ȡPS2�ֱ�����
#endif
#if !CHASSIS_STANDALONE
	xTaskCreate(data_task,     "DATA_task",     DATA_STK_SIZE,     NULL, DATA_TASK_PRIO,     NULL);	//Usartx3, Usartx1 and CAN send data task //����3������1��CAN������������
#endif
	
    taskEXIT_CRITICAL();
    vTaskDelete(NULL);
}






