#ifndef CHASSIS_AUTO_ROUTE_H
#define CHASSIS_AUTO_ROUTE_H
void Chassis_RouteTask(void *argument);
unsigned int Chassis_RoutePhase(void);
extern volatile unsigned int Route_FailureReason;
#endif
